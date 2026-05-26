/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <helper/log.h>
#include <libgen.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef __MINGW32__
	#include <sys/utsname.h>
#endif

#include <unistd.h>
#include "jsmn.h"
#include "ndsv5_ace.h"
#include "tiny-AES-c/aes.h"

/* #define DEBUG_DUMP */

#if defined(DEBUG_DUMP)
#define DUMP_PAIR(k, v, indent) dump_pair((k), (v), (indent))
#define DUMP_KV(kl, kn, vl, vn) \
	dump_key();                 \
	printf(".%.*s = %.*s\n", kl, kn, vl, vn);
#else
#define DUMP_PAIR(k, v, indent)
#define DUMP_KV(kl, kn, vl, vn)
#endif


typedef enum ace_pstate {
	ace_pst_null,
	ace_pst_root,
	ace_pst_info,
	ace_pst_acr,
	ace_pst_acr_one,
	ace_pst_acr_two,
} ace_pstate_t;

typedef struct Ace_Info {
	int acr_reg_count;
	int acr_type_count;
} Ace_Info_t;

typedef struct Acx_Type {
	void *next;
	int type;
} Acx_Type_t;

typedef struct Acx_Code {
	void *next;
	uint32_t insn;
} Acx_Code_t;

typedef struct Acx_Patch_Tuple {
	void *next;
	/* Patch format: (source_bit_offset, bit_length, target_bit_offset) */
	/* This structure describes how to extract and place register index bit-fields */
	/* within an instruction, as register indices may be split across multiple */
	/* non-contiguous bit positions in the instruction encoding. */
	uint8_t source_bit_offset;  /* Starting bit position to extract register index from */
	uint8_t bit_length;         /* Number of bits to extract (since a single register index may span
								multiple bit-fields) */
	uint8_t target_bit_offset;  /* Starting bit position in instruction where extracted bits should be placed */
} Acx_Patch_Tuple_t;

typedef struct Acx_Patch {
	void *next;
	Acx_Patch_Tuple_t *tuple;
} Acx_Patch_t;

typedef struct Acx_Access {
	Acx_Code_t *code;
	Acx_Patch_t *patch;
	size_t code_len;
	size_t patch_len;
} Acx_Access_t;

typedef struct Ace_Acx {
	void *next;
	const char *name;
	size_t len;
	uint32_t width;
	uint32_t number;
	Acx_Type_t *type;
	Acx_Access_t get;
	Acx_Access_t set;
} Ace_Acx_t;

typedef struct Ace_Context {
	Ace_Info_t info;
	Ace_Acx_t *acxs;
	uint8_t *gas_eca;
	size_t gas_eca_len;
	ace_pstate_t pst;
	/* runtime */
	Acx_Access_t *acc;
} Ace_Context_t;

typedef enum json_vtype {
	jsn_null,
	jsn_number,
	jsn_string,
} json_vtype_t;

#define SZ_KEY_STACK (8)
typedef struct Json_Context {
	const char *stream;
	jsmntok_t *start;
	jsmntok_t *value;
	const char *sstack[SZ_KEY_STACK];
	size_t lstack[SZ_KEY_STACK];
	size_t count;
	size_t idxstatck;
	json_vtype_t type;
} Json_Context_t;

unsigned *global_acr_reg_count_v5;
unsigned *global_acr_type_count_v5;
unsigned *ace_gas_eca_length_for_gdb_client;
const char *ace_gas_eca_for_gdb_client;
ACR_INFO_T_V5 *acr_info_list_v5;

INSN_CODE_T_V5 *(*gen_get_value_code)(char *name, unsigned index);
INSN_CODE_T_V5 *(*gen_set_value_code)(char *name, unsigned index);

/* Andes ACE JSON helpers.  */
static Ace_Context_t ace;
static Json_Context_t json;
static uint8_t ace_runtime_key[16];
static bool ace_runtime_key_set;

static int hexval(int c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

void ndsv5_ace_set_key_hex(const char *hex_key)
{
	memset(ace_runtime_key, 0, sizeof(ace_runtime_key));
	ace_runtime_key_set = false;

	if (!hex_key || !*hex_key)
		return;

	if (hex_key[0] == '0' && (hex_key[1] == 'x' || hex_key[1] == 'X'))
		hex_key += 2;

	size_t len = strlen(hex_key);
	if (len != 32) {
		LOG_ERROR("ACE key must be 32 hex chars (AES-128)");
		return;
	}

	for (size_t i = 0; i < 16; i++) {
		int hi = hexval(hex_key[i * 2]);
		int lo = hexval(hex_key[i * 2 + 1]);
		if (hi < 0 || lo < 0) {
			LOG_ERROR("ACE key contains non-hex characters");
			return;
		}
		ace_runtime_key[i] = (uint8_t)((hi << 4) | lo);
	}

	ace_runtime_key_set = true;
}

static int parse_object(jsmntok_t *t, size_t count);

static inline void *realloc_it(void *ptrmem, size_t size)
{
	void *p = realloc(ptrmem, size);
	if (!p) {
		free(ptrmem);
		fprintf(stderr, "realloc(): errno=%d\n", errno);
	}
	return p;
}

static const char *skip_ws(const char *p, const char *end)
{
	while (p < end && isspace((unsigned char)*p))
		p++;
	return p;
}

static int parse_long_token(const char **pp, const char *end, long *out,
							const char *what)
{
	const char *p = skip_ws(*pp, end);
	char *endptr;
	size_t preview_len;

	if (p >= end) {
		LOG_ERROR("Unexpected end while parsing %s", what);
		return ERROR_FAIL;
	}

	errno = 0;
	long val = strtol(p, &endptr, 0);
	if (endptr == p) {
		preview_len = (size_t)(end - p);
		if (preview_len > 24)
			preview_len = 24;
		LOG_ERROR("Invalid %s near '%.*s'", what, (int)preview_len, p);
		return ERROR_FAIL;
	}

	if (errno == ERANGE) {
		LOG_ERROR("%s out of range", what);
		return ERROR_FAIL;
	}

	*out = val;
	*pp = endptr;
	return ERROR_OK;
}

static int parse_gas_eca(const char *t, size_t count, uint8_t **out, size_t *bytes)
{
	const char *p = t;
	const char *end = t + count;
	uint8_t *buf = NULL;
	size_t cap = 0;
	size_t nr = 0;

	*out = NULL;
	*bytes = 0;
	while (p < end) {
		long val;

		p = skip_ws(p, end);
		if (p >= end)
			break;
		if (*p == ',') {
			p++;
			continue;
		}

		if (parse_long_token(&p, end, &val, "gas_eca byte") != ERROR_OK) {
			free(buf);
			*bytes = 0;
			return ERROR_FAIL;
		}
		if (val < 0 || val > 0xff) {
			LOG_ERROR("gas_eca byte out of range: %ld", val);
			free(buf);
			*bytes = 0;
			return ERROR_FAIL;
		}

		if (nr == cap) {
			size_t new_cap = cap ? cap * 2 : 16;
			uint8_t *tmp = realloc(buf, new_cap);
			if (!tmp) {
				LOG_ERROR("Out of memory");
				free(buf);
				*bytes = 0;
				return ERROR_FAIL;
			}
			buf = tmp;
			cap = new_cap;
		}

		buf[nr++] = (uint8_t)val;
		*bytes = nr;
		p = skip_ws(p, end);
		if (p < end && *p == ',')
			p++;
	}

	if (nr == 0) {
		free(buf);
		return ERROR_OK;
	}

	*out = buf;
	return ERROR_OK;
}

static int parse_acx_type(const char *t, size_t len)
{
	const char *p = t;
	const char *end = t + len;
	size_t parsed = 0;

	if (!ace.acxs) {
		LOG_ERROR("ACX object is missing before type list");
		return ERROR_FAIL;
	}

	while (p < end) {
		long val;

		p = skip_ws(p, end);
		if (p >= end)
			break;
		if (*p == ',') {
			p++;
			continue;
		}

		if (parse_long_token(&p, end, &val, "ACR type") != ERROR_OK)
			return ERROR_FAIL;

		Acx_Type_t *nu = calloc(1, sizeof(Acx_Type_t));
		if (!nu) {
			LOG_ERROR("Out of Memory!");
			return ERROR_FAIL;
		}
		nu->type = val;
		nu->next = ace.acxs->type;
		ace.acxs->type = nu;
		parsed++;

		p = skip_ws(p, end);
		if (p < end && *p == ',')
			p++;
	}

	if (parsed == 0) {
		LOG_ERROR("Empty ACR type list");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int parse_acx_code(const char *t, size_t len)
{
	const char *p = t;
	const char *end = t + len;
	size_t parsed = 0;

	if (!ace.acc) {
		LOG_ERROR("Access object is missing before code list");
		return ERROR_FAIL;
	}

	while (p < end) {
		long val;

		p = skip_ws(p, end);
		if (p >= end)
			break;
		if (*p == ',') {
			p++;
			continue;
		}

		if (parse_long_token(&p, end, &val, "ACR code") != ERROR_OK)
			return ERROR_FAIL;
		if (val < 0 || val > UINT32_MAX) {
			LOG_ERROR("ACR code out of range: %ld", val);
			return ERROR_FAIL;
		}

		Acx_Code_t *nu = calloc(1, sizeof(Acx_Code_t));
		if (!nu) {
			LOG_ERROR("Out of Memory!");
			return ERROR_FAIL;
		}
		nu->next = ace.acc->code;
		nu->insn = (uint32_t)val;
		ace.acc->code = nu;
		ace.acc->code_len++;
		parsed++;

		p = skip_ws(p, end);
		if (p < end && *p == ',')
			p++;
	}

	if (parsed == 0) {
		LOG_ERROR("Empty ACR code list");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int parse_acx_tupple(const char **pp, const char *end,
							Acx_Patch_Tuple_t **lst)
{
	const char *p = skip_ws(*pp, end);
	long a, b, c;
	Acx_Patch_Tuple_t *nu;

	if (p >= end || *p != '(')
		return ERROR_OK;

	p++;
	if (parse_long_token(&p, end, &a, "patch source bit offset") != ERROR_OK)
		return ERROR_FAIL;
	p = skip_ws(p, end);
	if (p >= end || *p != ',') {
		LOG_ERROR("Expected ',' in patch tuple");
		return ERROR_FAIL;
	}
	p++;
	if (parse_long_token(&p, end, &b, "patch bit length") != ERROR_OK)
		return ERROR_FAIL;
	p = skip_ws(p, end);
	if (p >= end || *p != ',') {
		LOG_ERROR("Expected ',' in patch tuple");
		return ERROR_FAIL;
	}
	p++;
	if (parse_long_token(&p, end, &c, "patch target bit offset") != ERROR_OK)
		return ERROR_FAIL;
	p = skip_ws(p, end);
	if (p >= end || *p != ')') {
		LOG_ERROR("Expected ')' in patch tuple");
		return ERROR_FAIL;
	}
	p++;

	nu = calloc(1, sizeof(Acx_Patch_Tuple_t));
	if (!nu) {
		LOG_ERROR("Out of Memory!");
		return ERROR_FAIL;
	}
	if (a < 0 || b < 0 || c < 0 ||
		a > UINT8_MAX || b > UINT8_MAX || c > UINT8_MAX || b > 63) {
		LOG_ERROR("Patch tuple value out of range");
		free(nu);
		return ERROR_FAIL;
	}
	nu->next = *lst;
	/* Patch format: (source_bit_offset, bit_length, target_bit_offset) */
	nu->source_bit_offset = (uint8_t)a;
	nu->bit_length = (uint8_t)b;
	nu->target_bit_offset = (uint8_t)c;
	*lst = nu;
	*pp = p;
	return 1;
}

static int parse_acx_patch(const char *t, size_t len)
{
	const char *p = skip_ws(t, t + len);
	const char *end = t + len;

	if (!ace.acc) {
		LOG_ERROR("Access object is missing before patch list");
		return ERROR_FAIL;
	}

	while (p < end && *p == ';') {
		p++;
		p = skip_ws(p, end);
	}

	if (p >= end)
		return ERROR_OK;

	if (*p != '(') {
		size_t preview_len = (size_t)(end - p);
		if (preview_len > 24)
			preview_len = 24;
		LOG_ERROR("Bad acx patch tuple near '%.*s'", (int)preview_len, p);
		return ERROR_FAIL;
	}

	while (p < end) {
		Acx_Patch_Tuple_t *lst = NULL;
		int tuple_count = 0;
		int expect_tuple = 1;

		p = skip_ws(p, end);
		if (p >= end)
			break;
		if (*p == ';') {
			p++;
			continue;
		}

		for (;;) {
			int rc = parse_acx_tupple(&p, end, &lst);
			if (rc < 0)
				return ERROR_FAIL;
			if (rc == 0) {
				if (tuple_count == 0)
					break;
				if (expect_tuple) {
					LOG_ERROR("Trailing comma in patch segment");
					return ERROR_FAIL;
				}
				break;
			}

			tuple_count++;
			expect_tuple = 0;
			p = skip_ws(p, end);
			if (p >= end || *p == ';')
				break;
			if (*p == ',') {
				p++;
				expect_tuple = 1;
				continue;
			}

			size_t preview_len = (size_t)(end - p);
			if (preview_len > 24)
				preview_len = 24;
			LOG_ERROR("Unexpected character in patch near '%.*s'", (int)preview_len, p);
			return ERROR_FAIL;
		}

		if (tuple_count == 0) {
			size_t preview_len = (size_t)(end - p);
			if (preview_len > 24)
				preview_len = 24;
			LOG_ERROR("Bad acx patch tuple near '%.*s'", (int)preview_len, p);
			return ERROR_FAIL;
		}

		Acx_Patch_t *nu = calloc(1, sizeof(Acx_Patch_t));
		if (!nu) {
			LOG_ERROR("Out of Memory!");
			return ERROR_FAIL;
		}
		nu->next = ace.acc->patch;
		nu->tuple = lst;
		ace.acc->patch = nu;
		ace.acc->patch_len++;

		p = skip_ws(p, end);
		if (p < end && *p == ';')
			p++;
		else if (p < end) {
			size_t preview_len = (size_t)(end - p);
			if (preview_len > 24)
				preview_len = 24;
			LOG_ERROR("Unexpected character in patch near '%.*s'", (int)preview_len, p);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static Ace_Acx_t *push_acx(const char *name, size_t len)
{
	Ace_Acx_t *p = calloc(1, sizeof(Ace_Acx_t));
	/* assert(p && "Out of Memory!"); */
	p->name = name;
	p->len = len;
	p->next = ace.acxs;
	ace.acxs = p;
	return p;
}

static Acx_Access_t *push_access(const char *name, size_t len)
{
	if (!ace.acxs) {
		LOG_ERROR("ACX object is missing before access type");
		ace.acc = NULL;
		return NULL;
	}

	if (0 == strncmp(name, "get", len))
		ace.acc = &ace.acxs->get;
	else if (0 == strncmp(name, "set", len))
		ace.acc = &ace.acxs->set;
	else {
		LOG_ERROR("Unknown access type: %.*s", (int)len, name);
		ace.acc = NULL;
	}
	return ace.acc;
}

static size_t push_key(const char *name, size_t len)
{
	size_t i = json.idxstatck;
	if (i >= SZ_KEY_STACK) {
		LOG_ERROR("JSON key stack overflow");
		return i;
	}
	json.sstack[i] = name;
	json.lstack[i] = len;
	return ++json.idxstatck;
}

static size_t pop_key(void)
{
	if (json.idxstatck == 0) {
		LOG_ERROR("JSON key stack underflow");
		return 0;
	}
	return --json.idxstatck;
}

#ifdef DEBUG_DUMP
static Ace_Acx_t *peek_acx(void) { return ace.acxs; }

static void dump_indent(int indent)
{
	int i;
	for (i = 0; i < indent; i++) {
		printf("  ");
	}
}

static void dump_jsmntok(jsmntok_t *t, int indent)
{
	const char *typename[] = {
		"JSMN_UNDEFINED", "JSMN_OBJECT",    "JSMN_ARRAY",
		"JSMN_STRING",    "JSMN_PRIMITIVE",
	};
	int i = indent;
	int j = indent + 1;
	dump_indent(i);
	printf("{\n");
	dump_indent(j);
	printf("  type: %s\n", typename[__builtin_ffs(t->type)]);
	dump_indent(j);
	printf("  text: %.*s\n", t->end - t->start, json.stream + t->start);
	dump_indent(j);
	printf("  size: %d\n", t->size);
	dump_indent(i);
	printf("}\n");
}

static void dump_key(void)
{
	size_t i;
	for (i = 0; i < json.idxstatck; ++i) {
		printf(".%.*s", (int)json.lstack[i], json.sstack[i]);
	}
}

static void dump_pair(jsmntok_t *k, jsmntok_t *v, int indent)
{
	int i = indent + 1;
	dump_indent(indent);
	printf("{\n");
	dump_jsmntok(k, i);
	dump_jsmntok(v, i);
	dump_indent(indent);
	printf("}\n");
}
#endif

static int parse_pair(jsmntok_t *t, size_t count)
{
	jsmntok_t *key, *value;
	char *keyname, *valtext;
	size_t klen, vlen;
	int j;
	ace_pstate_t prev_pst;

	j = 0; /* number of elements consumed */

	/* key */
	key = t;
		if (key->type != JSMN_STRING) {
			LOG_ERROR("Expected JSON string key");
			return ERROR_FAIL;
		}
		keyname = (char *)(json.stream + key->start);
		klen = key->end - key->start;
		++j;

	/* value */
	value = t + j;
	valtext = (char *)(json.stream + value->start);
	vlen = value->end - value->start;

	switch (value->type) {
	case JSMN_STRING:
	case JSMN_PRIMITIVE:
		DUMP_PAIR(key, value, 0);
		json.value = value;
		++j;
		break;
		case JSMN_OBJECT:
			prev_pst = ace.pst; /* save state */
			switch (ace.pst) {
			case ace_pst_acr: {
				if (!push_acx(keyname, klen))
					return ERROR_FAIL;
				ace.pst = ace_pst_acr_one;
				break;
			}
			case ace_pst_acr_one: {
				if (!push_access(keyname, klen))
					return ERROR_FAIL;
				ace.pst = ace_pst_acr_two;
				break;
			}
			case ace_pst_acr_two: {
				LOG_ERROR("Unexpected nested object under ACX access");
				return ERROR_FAIL;
			}
			default:
				if (0 == strncmp(keyname, "info", klen)) {
					ace.pst = ace_pst_info;
				} else if (0 == strncmp(keyname, "acr_acm", klen)) {
					ace.pst = ace_pst_acr;
				} else {
					LOG_ERROR("Nonsupported JSON object '%.*s'!", (int)klen,
						   keyname);
				}
				break;
			}
			push_key(keyname, klen);
			{
				int consumed = parse_object(value, count - j);
				if (consumed < 0)
					return ERROR_FAIL;
				j += consumed;
			}
			ace.pst = prev_pst; /* restore state.  */
			pop_key();
			break;
#if 0
	case JSMN_ARRAY:
		j += parse_array(value, count - j);
		break;
#endif
		default:
			LOG_ERROR("Nonsupported JSON token type!");
			return ERROR_FAIL;
		}

	/* apply */
	switch (ace.pst) {
	case ace_pst_root: {
		const char *value_text = json.stream + value->start;
		if (0 == strncmp(keyname, "gas_eca", klen)) {
			size_t bytes;
			uint8_t *p = NULL;
			if (parse_gas_eca(value_text, vlen, &p, &bytes) != ERROR_OK)
				return ERROR_FAIL;
			ace.gas_eca = p;
			ace.gas_eca_len = bytes;
#if defined(DEBUG_DUMP)
			if (p) {
				dump_key();
				printf(".%.*s[%zu] =\n", (int)klen, keyname, bytes);
				for (int i = 0; i < bytes; i += 4) {
					if (i > 64)
						break;
					if ((i % 16) == 0)
						printf("\n");
					printf("  0x%02x", p[i + 0]);
					if (i + 1 < bytes)
						printf(" 0x%02x", p[i + 1]);
					if (i + 2 < bytes)
						printf(" 0x%02x", p[i + 2]);
					if (i + 3 < bytes)
						printf(" 0x%02x", p[i + 3]);
				}
				printf("\n");
			}
#endif
		} else {
			if (value->type != JSMN_OBJECT && strncmp(keyname, "end", klen))
				LOG_ERROR("redundant pair: (%.*s, %.*s)", (int)klen, keyname,
					  (int)vlen, valtext);
		}
		break;
	}
	case ace_pst_info: {
		break;
	}
	case ace_pst_acr: {
		break;
	}
	case ace_pst_acr_one: {
		if (0 == strncmp(keyname, "width", klen)) {
			long width;
			const char *vp = valtext;
			if (parse_long_token(&vp, json.stream + value->end, &width, "ACR width") != ERROR_OK)
				return ERROR_FAIL;
			if (width < 0 || width > UINT32_MAX) {
				LOG_ERROR("ACR width out of range: %ld", width);
				return ERROR_FAIL;
			}
			ace.acxs->width = (uint32_t)width;
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "number", klen)) {
			long number;
			const char *vp = valtext;
			if (parse_long_token(&vp, json.stream + value->end, &number, "ACR number") != ERROR_OK)
				return ERROR_FAIL;
			if (number < 0 || number > UINT32_MAX) {
				LOG_ERROR("ACR number out of range: %ld", number);
				return ERROR_FAIL;
			}
			ace.acxs->number = (uint32_t)number;
			/* count the number of ACR and SRAM type ACM kinds */
			ace.info.acr_type_count++;
			/* count the number of total ACR entires + the number of
			   total SRAM type ACM entires */
			ace.info.acr_reg_count += ace.acxs->number;
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "type", klen)) {
			if (parse_acx_type(valtext, vlen) != ERROR_OK)
				return ERROR_FAIL;
			DUMP_KV(klen, keyname, vlen, valtext);
		} else {
			if (value->type != JSMN_OBJECT && strncmp(keyname, "end", klen))
				LOG_ERROR("redundant pair: (%.*s, %.*s)", (int)klen, keyname,
					  (int)vlen, valtext);
		}
		break;
	}
	case ace_pst_acr_two: {
		if (0 == strncmp(keyname, "code", klen)) {
			if (parse_acx_code(valtext, vlen) != ERROR_OK)
				return ERROR_FAIL;
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "patch", klen)) {
			if (parse_acx_patch(valtext, vlen) != ERROR_OK)
				return ERROR_FAIL;
			DUMP_KV(klen, keyname, vlen, valtext);
		}
		break;
	}
	default: {
		LOG_ERROR("Nonsupported ACE element class!");
		break;
	}
	}

	return j;
}

static int parse_object(jsmntok_t *t, size_t count)
{
	jsmntok_t *key;
	int i, j = 0;

	if (t->type != JSMN_OBJECT) {
		LOG_ERROR("Expected JSON object");
		return ERROR_FAIL;
	}

	while (count > 0) {
		for (i = 0, j = 1; i < t->size; i++) {
			key = t + j;
			if (key->type != JSMN_STRING) {
				LOG_ERROR("Expected JSON string key");
				return ERROR_FAIL;
			}
			{
				int consumed = parse_pair(key, count - j);
				if (consumed < 0)
					return ERROR_FAIL;
				j += consumed;
			}
		}
		count -= j;
		break;
	}

	return j;
}

static int process_json(const char *js, jsmntok_t *t, size_t count)
{
	memset(&ace, 0, sizeof(ace));
	memset(&json, 0, sizeof(json));

	json.stream = js;
	json.start = t;
	json.count = count;

	/* root object */
	ace.pst = ace_pst_root;
	return parse_object(t, count);
}

static int parse_json_content(char *json_content, const uint32_t json_content_length)
{
	jsmn_parser jsmn_p;
	jsmntok_t *tok;
	size_t tokcount = 2;

	/* Prepare parser */
	jsmn_init(&jsmn_p);

	/* Allocate some tokens as a start */
	tok = malloc(sizeof(*tok) * tokcount);
	if (!tok) {
		LOG_ERROR("malloc(): errno=%d", errno);
		return ERROR_FAIL;
	}

	/*
	 * Run JSON parser. It parses a JSON data string into an array of tokens,
	 * each describing a single JSON object.
	 */
	for (;;) {
		int r = jsmn_parse(&jsmn_p, json_content, json_content_length, tok, tokcount);
		if (r < 0) {
			if (r == JSMN_ERROR_NOMEM) {
				tokcount *= 2;
				tok = realloc_it(tok, sizeof(*tok) * tokcount);
				if (!tok)
					return ERROR_FAIL;
			} else {
				LOG_ERROR("JSON parser error: %d", r);
				return ERROR_FAIL;
			}
		} else {
			break;
		}
	}

	if (process_json(json_content, tok, jsmn_p.toknext) < 0) {
		free(tok);
		return ERROR_FAIL;
	}

	free(tok);
	return ERROR_OK;
}


static Ace_Acx_t *lookup_acr(char *name)
{
	Ace_Acx_t *acr = ace.acxs;
	size_t len = strlen(name);
	while (acr) {
		if (acr->len == len) {
			if (0 == strncmp(acr->name, name, len))
				break;
		}
		acr = acr->next;
	}
	if (!acr) {
		LOG_ERROR("not found ACR: %s", name);
	}
	return acr;
}

static INSN_CODE_T_V5 *ace_gen_access_code(Ace_Acx_t *acr, Acx_Access_t *acc,
										   unsigned index)
{
	INSN_CODE_T_V5 *insn_code =
		(INSN_CODE_T_V5 *)malloc(sizeof(INSN_CODE_T_V5));
	insn_code->num = acc->code_len;
	insn_code->code =
		(UTIL_INSN_T_V5 *)malloc(sizeof(UTIL_INSN_T_V5) * acc->code_len);

	Acx_Type_t *type = acr->type;
	Acx_Code_t *code = acc->code;
	Acx_Patch_t *patch = acc->patch;
	for (int i = acc->code_len - 1; i >= 0; --i) {
		unsigned int encoded_index = 0;
		if (patch) {
			Acx_Patch_Tuple_t *tuple = patch->tuple;
			while (tuple) {
				unsigned int mask = (1llu << tuple->bit_length) - 1;
				encoded_index |= (((index >> tuple->source_bit_offset) & mask) << tuple->target_bit_offset);
				tuple = tuple->next;
			}
			patch = patch->next;
		}
		insn_code->code[i].insn = code->insn | encoded_index;
		insn_code->code[i].version = type->type;
		type = type->next;
		code = code->next;
	}

	return insn_code;
}

static INSN_CODE_T_V5 *ace_gen_get_value_code(char *name, unsigned index)
{
	Ace_Acx_t *acr = lookup_acr(name);
	return acr ? ace_gen_access_code(acr, &acr->get, index) : NULL;
}

static INSN_CODE_T_V5 *ace_gen_set_value_code(char *name, unsigned index)
{
	Ace_Acx_t *acr = lookup_acr(name);
	return acr ? ace_gen_access_code(acr, &acr->set, index) : NULL;
}

static ACR_INFO_T_V5 *gen_acr_list(void)
{
	int count = 0;

	/* count acr */
	Ace_Acx_t *p = ace.acxs;
	while (p) {
		p = p->next;
		count++;
	}

	/* gen list */
	ACR_INFO_T_V5 *lst = calloc(count, sizeof(ACR_INFO_T_V5));
	if (lst) {
		Ace_Acx_t *acr = ace.acxs;
		for (int i = count - 1; i >= 0; --i) {
			snprintf(lst[i].name, sizeof(lst[i].name), "%.*s", (int)acr->len, acr->name);
			lst[i].num = acr->number;
			lst[i].width = acr->width;
			acr = acr->next;
		}
	}

	return lst;
}

static int32_t decrypt_ace_eca_file(const char *eca_file_name, char **plaintext, uint32_t *json_content_length)
{
	uint8_t iv[16];
	struct AES_ctx ctx;

#ifdef COPILOT_GAS_KEY
#define STRINGIFY(x) #x
#define TO_STRING(x) STRINGIFY(x)
	uint8_t hex[32] = TO_STRING(COPILOT_GAS_KEY), key[16];
	for (size_t i = 0; i < 16; i++)
		sscanf((char *)&hex[i * 2], "%2hhx", &key[i]);
#else
	uint8_t key[16] = { 0 };
#endif

	if (ace_runtime_key_set)
		memcpy(key, ace_runtime_key, sizeof(key));

	const char *suffix = ".eca";
	const size_t suffix_len = strlen(suffix);
	const size_t eca_file_name_len = strlen(eca_file_name);

	/* Check whether file extension is ".eca", if not, log error message and return -1 */
	if (suffix_len >= strlen(eca_file_name) || strcmp(eca_file_name + eca_file_name_len - suffix_len, suffix) != 0) {
		LOG_ERROR("The file extension must be .eca");
		return -1;
	}

	FILE *file = fopen(eca_file_name, "rb");
	if (file == NULL) {
		LOG_ERROR("Error opening ACE ECA file");
		return -1;
	}

	/* Seek to the end of the file to determine its size */
	fseek(file, 0, SEEK_END);
	long file_size = ftell(file);
	rewind(file); /* Go back to the start of the file */

	if (file_size < 0) {
		LOG_ERROR("Error determining file size");
		fclose(file);
		return -1;
	}

	/* Skip the first three lines, since the first three lines are used for comments */
	for (int i = 0; i < 3; i++) {
		char ch;
		while ((ch = fgetc(file)) != '\n' && ch != EOF) {
			/*
			*/
		}
	}

	/* Read IV */
	/* IV is always allocated at the first 16 chars (bytes) at line four */
	if (fread(iv, 1, sizeof(iv), file) != sizeof(iv)) {
		LOG_ERROR("Error reading file (iv)");
		fclose(file);
		return -1;
	}

	/* Read ciphertext */
	size_t data_size = file_size - ftell(file);
	uint8_t *data = (uint8_t *)malloc(data_size + 1);
	size_t bytes_read = fread(data, 1, data_size, file);
	if (bytes_read != data_size) {
		LOG_ERROR("Error reading file");
		free(data);
		fclose(file);
		return -1;
	}

	/* Decrypt the data to JSON format if the data is 16-bytes aligned */
	if (data_size % 16 != 0) {
		LOG_ERROR("Incorrect ECA format (content size does not align to 16 bytes)");
		free(data);
		fclose(file);
		return -1;
	}

	/* Decrypt file string */
	AES_init_ctx_iv(&ctx, key, iv);
	AES_CBC_decrypt_buffer(&ctx, data, data_size);

	/* Remove PKCS#7 padding */
	size_t padding = data[data_size - 1];
	if (padding > 16 || padding > data_size) {
		LOG_ERROR("Incorrect ECA format (cannot find padding size info)");
		free(data);
		fclose(file);
		return -1;
	}
	data[data_size - padding] = '\0';

	*plaintext = (char *) malloc(data_size - padding + 1);
	memcpy(*plaintext, data, data_size - padding + 1);
	*json_content_length = data_size - padding + 1;

	free(data);
	fclose(file);
	return 0;
}

#if defined(DEBUG_DUMP)

/* Function declarations */
static void print_ace_info(Ace_Info_t *info) {
    if (!info) return;
    printf("Ace_Info: { acr_reg_count: %d, acr_type_count: %d }\n", info->acr_reg_count, info->acr_type_count);
}

static void print_acx_type(Acx_Type_t *type) {
    while (type) {
        printf("Acx_Type: { type: %d }\n", type->type);
        type = (Acx_Type_t *)type->next;
    }
}

static void print_acx_code(Acx_Code_t *code) {
    while (code) {
        printf("Acx_Code: { insn: %x }\n", code->insn);
        code = (Acx_Code_t *)code->next;
    }
}

static void print_acx_patch_tupple(Acx_Patch_Tuple_t *tuple)
{
	while (tuple) {
		printf("Acx_Patch_Tupple: { target_bit_offset: %u, source_bit_offset: %u, bit_length: %u }\n",
				tuple->target_bit_offset, tuple->source_bit_offset, tuple->bit_length);
		tuple = (Acx_Patch_Tuple_t *)tuple->next;
	}
}

static void print_acx_patch(Acx_Patch_t *patch) {
	while (patch) {
        printf("Acx_Patch: {\n");
		print_acx_patch_tupple(patch->tuple);
		printf("}\n");
		patch = (Acx_Patch_t *)patch->next;
    }
}

static void print_acx_access(Acx_Access_t *access) {
	if (!access)
		return;
	printf("Acx_Access: {\n");
	printf("  Code Len: %llu\n", access->code_len);
	printf("  Code:\n");
	print_acx_code(access->code);
	printf("  Patch Len: %llu\n", access->patch_len);
	printf("  Patch:\n");
	print_acx_patch(access->patch);
	printf("}\n");
}

static void print_ace_acx(Ace_Acx_t *acx) {
	while (acx) {
		printf("Ace_Acx: { name: %.*s, len: %zu, width: %u, number: %u }\n", acx->len, acx->name,
				acx->len, acx->width, acx->number);
		printf("  Type:\n");
		print_acx_type(acx->type);
		printf("  Get:\n");
		print_acx_access(&acx->get);
		printf("  Set:\n");
		print_acx_access(&acx->set);
		acx = (Ace_Acx_t *)acx->next;
	}
}

static void print_ace_struct(Ace_Context_t *a) {
    if (!a) return;
    printf("Ace_Context: {\n");
    print_ace_info(&a->info);
    printf("  Acxs:\n");
    print_ace_acx(a->acxs);
    printf("  gas_eca_len: %zu\n", a->gas_eca_len);
    printf("  Acc:\n");
    print_acx_access(a->acc);
    printf("}\n");
}
#endif

static int32_t handle_ace_eca_file(const char *eac_file_path)
{
	/* decrypt ACE ECA file to become ACE JSON file */
	char *json_file_content;
	uint32_t json_content_length = 0;
	uint32_t decrypt_ret = decrypt_ace_eca_file(eac_file_path, &json_file_content, &json_content_length);
	if (0 != decrypt_ret)
		return decrypt_ret;

	/* parse JSON content */
	if (parse_json_content(json_file_content, json_content_length) != ERROR_OK)
		return ERROR_FAIL;

#if defined(DEBUG_DUMP)
	print_ace_struct(&ace);
#endif

	global_acr_reg_count_v5 = (unsigned *)&ace.info.acr_reg_count;
	LOG_DEBUG("global_acr_reg_count_v5 = %d", *global_acr_reg_count_v5);

	global_acr_type_count_v5 = (unsigned *)&ace.info.acr_type_count;
	LOG_DEBUG("global_acr_type_count_v5 = %d", *global_acr_type_count_v5);

	gen_get_value_code = ace_gen_get_value_code;
	gen_set_value_code = ace_gen_set_value_code;

	if (*global_acr_type_count_v5 != 0)
		acr_info_list_v5 = (ACR_INFO_T_V5 *)gen_acr_list();

	ace_gas_eca_length_for_gdb_client = (unsigned *)&ace.gas_eca_len;
	LOG_DEBUG("ace_gas_eca_length_for_gdb_client = %d", *ace_gas_eca_length_for_gdb_client);

	ace_gas_eca_for_gdb_client = (const char *)ace.gas_eca;

	LOG_DEBUG("end of handle_ace_eca_file");
	return 0;
}

#define MAX_ACECONF_STRING 2048
#define MAX_TEXT_STRING 256

int32_t nds32_ace_init_v5(const char *aceconf)
{
	global_acr_type_count_v5 = 0;
	global_acr_reg_count_v5 = 0;
	acr_info_list_v5 = NULL;
	ace_gas_eca_for_gdb_client = NULL;

	if (aceconf == NULL || strcmp(aceconf, "") == 0)
		return 0;

	/* Find the file extension of given file (including path) */
	char *ext;
	ext = strrchr(aceconf, '.');
	LOG_DEBUG("aceconf ext = %s", ext + 1);

	char *ext_eca = "eca";
	char *ext_conf = "conf";
	LOG_DEBUG("ext_so cmp = %d", strcmp(ext + 1, ext_eca));
	LOG_DEBUG("ext_conf cmp = %d", strcmp(ext + 1, ext_conf));

	if (strcmp(ext + 1, ext_eca) == 0) {
		int32_t ret = handle_ace_eca_file(aceconf);
		return ret == ERROR_OK ? ERROR_OK : -1;
	} else if (strcmp(ext + 1, ext_conf) == 0) {
		LOG_ERROR("The configuration input (e.g., ICEman.conf) is no longer supported. Please specify the path to libacedbg.eca.");
		return -1;
	}

	return 0;
}

int32_t get_ace_file_name_for_gdb_v5(const char *aceconf, const char *platform,
									 char **name)
{
	int32_t ret = 0;

	if (ace_gas_eca_for_gdb_client) {
		char *ace_gas_eca_file_name;
		ace_gas_eca_file_name = (char *) malloc(16); /* must be malloc because there was a free() afterwards */
		const char *str;

		LOG_DEBUG("platform: %s", platform);
		LOG_DEBUG("os.sysname: %s", platform);

		str = ace_gas_eca_for_gdb_client;

		/* use size as filename to do fopen */
		sprintf(ace_gas_eca_file_name, "%u", *ace_gas_eca_length_for_gdb_client);

		FILE *fd = fopen(ace_gas_eca_file_name, "wb");
		if (fd == NULL) {
			ret = -2;
		} else {
			if (fwrite(str, sizeof(char), *ace_gas_eca_length_for_gdb_client, fd) != *ace_gas_eca_length_for_gdb_client)
				ret = -3;
			else
				*name = ace_gas_eca_file_name;

			fclose(fd);
		}
	} else {
		*name = NULL;
		return -4;
	}

	return ret;
}
