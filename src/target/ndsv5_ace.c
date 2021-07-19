/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <dlfcn.h>
#include <sys/utsname.h>
#include <libgen.h>
#include "ndsv5_ace.h"

unsigned *global_acr_reg_count_v5;
unsigned *global_acr_type_count_v5;
unsigned *ace_lib_for_gdb_len_v5;
const char *ace_lib_for_gdb_v5;
ACR_INFO_T_V5 *acr_info_list_v5;

INSN_CODE_T_V5* (*gen_get_value_code) (char *name, unsigned index);
INSN_CODE_T_V5* (*gen_set_value_code) (char *name, unsigned index);

static int32_t get_ace_file_name(const char *aceconf, const char *marker, char **name)
{
	if (aceconf == NULL || strcmp(aceconf, "") == 0)
		return -1;
	else {
		char *ptr = strstr(aceconf, marker);

		if (ptr != NULL) {
			char *ret;

			ptr += strlen(marker);
			ret = strchr(ptr, ',');
			if (ret == NULL) {
				ret = (char *) malloc(strlen(ptr) + 1);
				strcpy(ret, ptr);
			} else {
				int n = ret - ptr;
				ret = (char *) malloc(n + 1);
				strncpy(ret, ptr, n);
				ret[n] = '\0';
			}
			*name = ret;
		} else
			*name = NULL;

		return 0;
	}
}

#define LE(msg) do {                  \
	err_str = dlerror();          \
	if (err_str != NULL) {        \
		LOG_ERROR("%s", msg); \
		return -1;            \
	}                             \
} while (0)
void *handle;
/* dlopen a shared object named 'so_name'
 * dlsym certain global variables */
static int32_t loadSharedLib(const char *so_name)
{
	char *err_str = NULL;

	LOG_DEBUG("loadSharedLib from %s", so_name);

	/* dlopen a shared object named 'so_name' */
	handle = dlopen(so_name, RTLD_NOW | RTLD_LOCAL);
	LE("unable to dlopen the shared library.");

	assert(handle != NULL);

	global_acr_reg_count_v5 = (unsigned *) dlsym(handle, "acr_reg_count");
	LE("unable to load symbol acr_reg_count");
	LOG_DEBUG("global_acr_reg_count_v5 = %d", *global_acr_reg_count_v5);

	global_acr_type_count_v5 = (unsigned *) dlsym(handle, "acr_type_count");
	LE("unable to load symbol acr_type_count");
	LOG_DEBUG("global_acr_type_count_v5 = %d", *global_acr_type_count_v5);

	gen_get_value_code = dlsym(handle, "gen_get_value_code");
	LE("unable to load symbol gen_get_value_code");

	gen_set_value_code = dlsym(handle, "gen_set_value_code");
	LE("unable to load symbol gen_set_value_code");

	if (*global_acr_type_count_v5 != 0)
		acr_info_list_v5 = (ACR_INFO_T_V5 *) dlsym(handle, "acr_list");

	ace_lib_for_gdb_len_v5 = (unsigned *) dlsym(handle, "ace_lib_for_gdb_len");
	LE("unable to load symbol ace_lib_fog_gdb_len");
	LOG_DEBUG("ace_lib_for_gdb_len_v5 = %d", *ace_lib_for_gdb_len_v5);

	ace_lib_for_gdb_v5 = (const char *) dlsym(handle, "ace_lib_for_gdb");
	LE("unable to load symbol ace_lib_for_gdb");

	LOG_DEBUG("end of loadSharedLib");
	return 0;
}

char *so_name;
static int32_t init_core_ace_reg_list(const char *aceconf)
{
	int32_t ret;
	char str[8];

	strcpy(str, "ace=");
	if (get_ace_file_name(aceconf, str, &so_name) == -1)
		return -1;

	if (so_name == NULL)
		ret = 0;

	ret = loadSharedLib(so_name);
	LOG_DEBUG("end of loadSharedLib");
	return ret;
}

#define MAX_ACECONF_STRING	2048
#define MAX_TEXT_STRING		256
static int32_t parse_ace_conf(const char *aceconf, char *ace_opt)
{
	char *o_aceconf, *ace_dir, *ace_rel_fname;
	char line[MAX_TEXT_STRING];
	int32_t len = 0;

	o_aceconf = strdup(aceconf);
	ace_dir = dirname(o_aceconf);

	LOG_DEBUG("aceconf = %s", aceconf);

	FILE *fp = fopen(aceconf, "r");
	if (NULL == fp)
		return -1;

	unsigned first_entry = 1;
	ace_opt[0] = '\0';
	while (fgets(line, MAX_TEXT_STRING, fp)) {
		if ('#' == line[0])
			continue;
		else if (!first_entry)
			strcat(ace_opt, ",");
		else
			first_entry = 0;

		/* All keywords must start at column 1. */
		if (strncmp(line, "--ace=", 6) == 0) {
			ace_rel_fname = &line[6];
			strcat(ace_opt, "ace=");
		} else if (strncmp(line, "--cop", 5) == 0 &&
			   line[5] >= '0' &&
			   line[5] <= '0' + MAX_COP_COUNT &&
			   line[6] == '=') {
			char c = line[7];
			line[7] = '\0';
			strcat(ace_opt, &line[2]);
			line[7] = c;
			ace_rel_fname = &line[7];
		} else {
			fclose(fp);
			return -1;
		}

		/* trim the trailing newline */
		len = strlen(ace_rel_fname);
		if (ace_rel_fname[len - 1] == '\n')
			ace_rel_fname[len - 1] = '\0';

		/* append ACE file name to ACE conf path */
		strcat(ace_opt, ace_dir);
		strcat(ace_opt, "/");
		strcat(ace_opt, ace_rel_fname);

		LOG_DEBUG("ace_opt = %s", ace_opt);
	}
	fclose(fp);
	return 0;
}

int32_t nds32_ace_init_v5(const char *aceconf)
{
	char ace_opt[MAX_ACECONF_STRING];

	global_acr_type_count_v5 = 0;
	global_acr_reg_count_v5 = 0;
	acr_info_list_v5 = NULL;
	ace_lib_for_gdb_v5 = NULL;

	if (aceconf == NULL || strcmp(aceconf, "") == 0)
		return 0;

	/* Find the file extension of given file (including path) */
	char *ext;
	ext = strrchr(aceconf, '.');
	LOG_DEBUG("aceconf ext = %s", ext + 1);

	char *ext_so = "so";
	char *ext_conf = "conf";
	LOG_DEBUG("ext_so cmp = %d", strcmp(ext + 1, ext_so));
	LOG_DEBUG("ext_conf cmp = %d", strcmp(ext + 1, ext_conf));

	if (strcmp(ext + 1, ext_so) == 0) {
		return loadSharedLib(aceconf);
	} else if (strcmp(ext + 1, ext_conf) == 0) {
		/* Get all ACE/COP options for this core first */
		/* and then get HW register information.       */
		if (-1 == parse_ace_conf(aceconf, ace_opt) || -1 == init_core_ace_reg_list(ace_opt))
			return -1;
	}


	return 0;
}

int32_t get_ace_file_name_for_gdb_v5(const char *aceconf,
		const char *platform, char **name)
{
	int32_t ret = 0;

	if (ace_lib_for_gdb_v5) {
		struct utsname os;
		if (uname(&os) == 0) {
			char *soname;
			soname = (char *) malloc(16); /* must be malloc because there was a free() afterwards */
			const char *str;

			if (strcmp(platform, os.sysname) == 0) {
				/* Return binary share library.  */
				str = ace_lib_for_gdb_v5;
			} else
				return -1;

			/* Follow V3 to use byte size of "ace_lib_fog_gdb".
			   But the way to get the information is different. */
			sprintf(soname, "%u", *ace_lib_for_gdb_len_v5);	/* use size as filename to do fopen */

			FILE *fd = fopen(soname, "w");
			if (fd == NULL) {
				ret = -1;
			} else {
				if (fwrite(str, sizeof(char), *ace_lib_for_gdb_len_v5, fd) != *ace_lib_for_gdb_len_v5)
					ret = -1;
				else
					*name = soname;

				fclose(fd);
			}
		}
	} else {
		*name = NULL;
		return -1;
	}

	return ret;
}
