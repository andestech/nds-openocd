/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
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

// #define DEBUG_DUMP

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

typedef struct Acx_Patch_Tupple {
	void *next;
	uint8_t right;
	uint8_t left;
	uint8_t len;
} Acx_Patch_tupple_t;

typedef struct Acx_Patch {
	void *next;
	Acx_Patch_tupple_t *tupple;
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

static uint8_t *parse_gas_eca(const char *t, size_t count, size_t *bytes)
{
	/* sizing */
	const char *p = t;
	size_t n = count;
	*bytes = 0;
	while (n-- > 0)
		if (*p++ == ',')
			(*bytes)++;
	if (t[count - 1] != ',')
		*bytes += 1;

	uint8_t *buf = calloc(*bytes, 1);
	if (buf == NULL)
		return buf;

	char *ep;
	int i = 0;
	n = *bytes;
	p = t;
	while (n-- > 0) {
		buf[i++] = (uint8_t)strtol(p, &ep, 0);
		p = ep + 1; /* skip ',' */
	}

	return buf;
}

static void parse_acx_type(const char *t, size_t len)
{
	const char *p = t;
	const char *end = t + len;
	char *endptr;
	while (p < end) {
		int val = strtol(p, &endptr, 0);

		Acx_Type_t *nu = calloc(1, sizeof(Acx_Type_t));
		nu->type = val;
		nu->next = ace.acxs->type;
		ace.acxs->type = nu;

		p = endptr;
		if (*p == ',')
			p++;
	}
}

static void parse_acx_code(const char *t, size_t len)
{
	const char *p = t;
	const char *end = t + len;
	char *endptr;
	while (p < end) {
		uint32_t insn = strtol(p, &endptr, 0);

		Acx_Code_t *nu = calloc(1, sizeof(Acx_Code_t));
		nu->next = ace.acc->code;
		nu->insn = insn;
		ace.acc->code = nu;
		ace.acc->code_len++;

		p = endptr;
		if (*p == ',')
			p++;
	}
}

static const char *parse_acx_tupple(const char *t, const char *end,
									Acx_Patch_tupple_t **lst)
{
	const char *p = t;
	char *endptr;
	int succ = 0;
	while (p < end) {
		int a, b, c;
		/* (a,b,c),... */
		if (*p++ != '(')
			break;
		a = strtol(p, &endptr, 0);
		p = endptr;
		if (*p++ != ',')
			break;
		b = strtol(p, &endptr, 0);
		p = endptr;
		if (*p++ != ',')
			break;
		c = strtol(p, &endptr, 0);
		p = endptr;
		if (*p++ != ')')
			break;

		Acx_Patch_tupple_t *nu = calloc(1, sizeof(Acx_Patch_tupple_t));
		nu->next = *lst;
		nu->right = a;
		nu->left = b;
		nu->len = c;
		*lst = nu;

		if (*p++ == ',')
			continue;

		succ = 1;
		break;
	}

	return succ ? p : t;
}

static void parse_acx_patch(const char *t, size_t len)
{
	const char *p = t;
	const char *end = t + len;
	while (p < end) {
		Acx_Patch_tupple_t *lst = NULL;
		p = parse_acx_tupple(p, end, &lst);
		assert(lst && "Bad acx patch tupple!");

		Acx_Patch_t *nu = calloc(1, sizeof(Acx_Patch_t));
		nu->next = ace.acc->patch;
		nu->tupple = lst;
		ace.acc->patch = nu;
		ace.acc->patch_len++;

		if (*p == ';')
			p++;
	}
}

static Ace_Acx_t *push_acx(const char *name, size_t len)
{
	Ace_Acx_t *p = calloc(1, sizeof(Ace_Acx_t));
	assert(p && "Out of Memory!");
	p->name = name;
	p->len = len;
	p->next = ace.acxs;
	ace.acxs = p;
	return p;
}

static Acx_Access_t *push_access(const char *name, size_t len)
{
	if (0 == strncmp(name, "get", len))
		ace.acc = &ace.acxs->get;
	else if (0 == strncmp(name, "set", len))
		ace.acc = &ace.acxs->set;
	else
		assert(0 && "Unknown access type!");
	return ace.acc;
}

static size_t push_key(const char *name, size_t len)
{
	size_t i = json.idxstatck;
	json.sstack[i] = name;
	json.lstack[i] = len;
	return ++json.idxstatck;
}

static size_t pop_key(void) { return --json.idxstatck; }

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
	char *keyname, *valtext, *endptr;
	size_t klen, vlen;
	int j;
	ace_pstate_t prev_pst;

	j = 0; /* number of elements consumed */

	/* key */
	key = t;
	assert(key->type == JSMN_STRING);
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
			push_acx(keyname, klen);
			ace.pst = ace_pst_acr_one;
			break;
		}
		case ace_pst_acr_one: {
			push_access(keyname, klen);
			ace.pst = ace_pst_acr_two;
			break;
		}
		case ace_pst_acr_two: {
			assert(0);
			break;
		}
		default:
			if (0 == strncmp(keyname, "info", klen)) {
				ace.pst = ace_pst_info;
			} else if (0 == strncmp(keyname, "acr_acm", klen)) {
				ace.pst = ace_pst_acr;
			} else {
				printf("Nonsupported JSON object '%.*s'!\n", (int)klen,
					   keyname);
			}
			break;
		}
		push_key(keyname, klen);
		j += parse_object(value, count - j);
		ace.pst = prev_pst; /* restore state.  */
		pop_key();
		break;
#if 0
	case JSMN_ARRAY:
		j += parse_array(value, count - j);
		break;
#endif
	default:
		assert(0 && "Nonsupported JSON token type!");
	}

	/* apply */
	switch (ace.pst) {
	case ace_pst_root: {
		const char *value_text = json.stream + value->start;
		if (0 == strncmp(keyname, "gas_eca", klen)) {
			size_t bytes;
			uint8_t *p = parse_gas_eca(value_text, vlen, &bytes);
			assert(ace.gas_eca == NULL);
			ace.gas_eca = p;
			ace.gas_eca_len = bytes;
#if defined(DEBUG_DUMP)
			if (p) {
				dump_key();
				printf(".%.*s[%d] = \n", klen, keyname, bytes);
				for (int i = 0; i < bytes; i += 4) {
					if (i > 64)
						break;
					if ((i % 16) == 0)
						printf("\n");
					printf("  0x%02x 0x%02x 0x%02x 0x%02x", p[i + 0], p[i + 1],
						   p[i + 2], p[i + 3]);
				}
				printf("\n");
			}
#endif
		} else {
			if (value->type != JSMN_OBJECT && strncmp(keyname, "end", klen))
				printf("redundant pair: (%.*s, %.*s)\n", (int)klen, keyname,
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
			ace.acxs->width = strtol(valtext, &endptr, 0);
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "number", klen)) {
			ace.acxs->number = strtol(valtext, &endptr, 0);
			/* count the number of ACR and SRAM type ACM kinds */
			ace.info.acr_type_count++;
			/* count the number of total ACR entires + the number of
			   total SRAM type ACM entires */
			ace.info.acr_reg_count += ace.acxs->number;
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "type", klen)) {
			parse_acx_type(valtext, vlen);
			DUMP_KV(klen, keyname, vlen, valtext);
		} else {
			if (value->type != JSMN_OBJECT && strncmp(keyname, "end", klen))
				printf("redundant pair: (%.*s, %.*s)\n", (int)klen, keyname,
					   (int)vlen, valtext);
		}
		break;
	}
	case ace_pst_acr_two: {
		if (0 == strncmp(keyname, "code", klen)) {
			parse_acx_code(valtext, vlen);
			DUMP_KV(klen, keyname, vlen, valtext);
		} else if (0 == strncmp(keyname, "patch", klen)) {
			parse_acx_patch(valtext, vlen);
			DUMP_KV(klen, keyname, vlen, valtext);
		}
		break;
	}
	default: {
		assert(0 && "Nonsupported ACE element class!");
		break;
	}
	}

	return j;
}

static int parse_object(jsmntok_t *t, size_t count)
{
	jsmntok_t *key;
	int i, j = 0;

	assert(t->type == JSMN_OBJECT);

	while (count > 0) {
		for (i = 0, j = 1; i < t->size; i++) {
			key = t + j;
			assert(key->type == JSMN_STRING);
			j += parse_pair(key, count - j);
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
		fprintf(stderr, "malloc(): errno=%d\n", errno);
		return 3;
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
					return 3;
			} else {
				return 1; /* Return error for other parsing issues */
			}
		} else {
			break;
		}
	}

	process_json(json_content, tok, jsmn_p.toknext);

	free(tok);
	return EXIT_SUCCESS;
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
		LOG_ERROR("%d : not found ACR : %.*s\n", (int)acr->len, (int)acr->len, acr->name);
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
	for (int i = acc->code_len - 1; i >= 0; --i) {
		Acx_Patch_t *patch = acc->patch;
		unsigned int encoded_index = 0;
		if (patch) {
			Acx_Patch_tupple_t *tupple = patch->tupple;
			while (tupple) {
				unsigned int mask = (1llu << tupple->len) - 1;
				encoded_index |= (((index >> tupple->left) << tupple->right) & mask);
				tupple = tupple->next;
			}
		}
		insn_code->code[i].insn = code->insn | encoded_index;
		insn_code->code[i].version = type->type;
		type = type->next;
		code = code->next;
		patch = patch->next;
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

	const char *suffix = ".eca";
	const size_t suffix_len = strlen(suffix);
	const size_t eca_file_name_len = strlen(eca_file_name);

	/* Check whether file extension is ".eca", if not, log error message and return -1 */
	if (suffix_len >= strlen(eca_file_name) || strcmp(eca_file_name + eca_file_name_len - suffix_len, suffix) != 0) {
		LOG_DEBUG("The file extension must be .eca");
		return -1;
	}

	FILE *file = fopen(eca_file_name, "r");
	if (file == NULL) {
		LOG_DEBUG("Error opening ACE ECA file");
		return -1;
	}

	/* Seek to the end of the file to determine its size */
	fseek(file, 0, SEEK_END);
	long file_size = ftell(file);
	rewind(file); /* Go back to the start of the file */

	if (file_size < 0) {
		LOG_DEBUG("Error determining file size");
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
	fread(iv, 1, sizeof(iv), file);

	/* Read ciphertext */
	size_t data_size = file_size - ftell(file);
	uint8_t *data = (uint8_t *)malloc(data_size + 1);
	size_t bytes_read = fread(data, 1, data_size, file);
	if (bytes_read != data_size) {
		LOG_DEBUG("Error reading file");
		free(data);
		fclose(file);
		return -1;
	}

	/* Decrypt the data to JSON format if the data is 16-bytes aligned */
	if (data_size % 16 != 0) {
		LOG_DEBUG("Incorrect ECA format (content size does not align to 16 bytes)");
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
		LOG_DEBUG("Incorrect ECA format (cannot find padding size info)");
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

// Function declarations
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

static void print_acx_patch_tupple(Acx_Patch_tupple_t *tupple) {
    while (tupple) {
        printf("Acx_Patch_Tupple: { right: %u, left: %u, len: %u }\n", tupple->right, tupple->left, tupple->len);
        tupple = (Acx_Patch_tupple_t *)tupple->next;
    }
}

static void print_acx_patch(Acx_Patch_t *patch) {
    while (patch) {
        printf("Acx_Patch: {\n");
        print_acx_patch_tupple(patch->tupple);
        printf("}\n");
        patch = (Acx_Patch_t *)patch->next;
    }
}

static void print_acx_access(Acx_Access_t *access) {
    if (!access) return;
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
        printf("Ace_Acx: { name: %.*s, len: %zu, width: %u, number: %u }\n", acx->len, acx->name, acx->len, acx->width, acx->number);
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
	parse_json_content(json_file_content, json_content_length);

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
		return handle_ace_eca_file(aceconf);
	} else if (strcmp(ext + 1, ext_conf) == 0) {
		LOG_DEBUG("The configuration input (e.g., ICEman.conf) is no longer supported. Please specify the path to libacedbg.eca.");
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

		FILE *fd = fopen(ace_gas_eca_file_name, "w");
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
