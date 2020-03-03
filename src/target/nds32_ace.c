/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <dlfcn.h>
#include <sys/utsname.h>
#include <libgen.h>
#include "nds32_reg.h"
#include "nds32.h"
#include "nds32_ace.h"

unsigned* global_acr_reg_count;
unsigned* global_acr_type_count;
const char *ace_lib_for_gdb;
const char *ace_xml_for_gdb;
ACR_INFO_T *acr_info_list;
/* Bitfield used in copilot_version is: major[23:16], minor[15:8], extension[7:0] */
static const uint32_t *sym_copilot_version = NULL;

uint32_t nds32_copilot_version(void)
{
	uint32_t ver = 0x40000;

	if (sym_copilot_version != NULL)
		ver = *sym_copilot_version;
	if (ver == 0x0)
		ver = 0x40000;
	return ver;
}

static int32_t get_ace_file_name (const char *aceconf, const char *marker, char **name)
{
	if (aceconf == NULL || strcmp (aceconf, "") == 0) {
		return -1;
	} else {
		char *ptr = strstr (aceconf, marker);

		if (ptr != NULL) {
			char *ret;

			ptr += strlen (marker);
			ret = strchr (ptr, ',');
			if (ret == NULL) {
				ret = (char*) malloc (strlen (ptr) + 1);
				strcpy (ret, ptr);
			} else {
				int n = ret - ptr;
				ret = (char*) malloc (n + 1);
				strncpy (ret, ptr, n);
				ret[n] = '\0';
			}
			*name = ret;
		} else {
			*name = NULL;
		}
		return 0;
	}
}

#define LE(msg) \
	err_str = dlerror (); \
	if (err_str != NULL) { \
		LOG_ERROR("%s", msg); \
		return -1; \
	}
void *handle;
// dlopen a shared object named 'so_name'
// dlsym certain global variables
static int loadSharedLib (const char *so_name)
{
	char *err_str = NULL;

	LOG_DEBUG("loadSharedLib");

	// dlopen a shared object named 'so_name'
	handle = dlopen (so_name, RTLD_NOW | RTLD_LOCAL);
	LE("unable to dlopen the shared library.")

	assert (handle != NULL);

	global_acr_reg_count = (unsigned*)dlsym (handle, "acr_reg_count");
	LE("unable to load symbol acr_reg_count")

	global_acr_type_count = (unsigned*)dlsym (handle, "acr_type_count");
	LE("unable to load symbol acr_type_count")

	ace_lib_for_gdb = (const char *)dlsym (handle, "ace_lib_for_gdb");
	LE("unable to load symbol ace_lib_for_gdb")

	//ace_xml_for_gdb = (const char *)dlsym (handle, "ace_xml_for_gdb");
	//LE("unable to load symbol ace_xml_for_gdb");

	sym_copilot_version = (const uint32_t *)dlsym (handle, "copilot_version");

	if (*global_acr_type_count != 0) {
		acr_info_list = (ACR_INFO_T *)dlsym (handle, "acr_list");
	}
	LOG_DEBUG("end of loadSharedLib");
	return 0;
}

char *so_name;
static int init_core_ace_reg_list (const char *aceconf)
{
	int ret;
	char str[8];
#if 0
	/* Coprocessor */
	strcpy(str, "cop0=");
	for (k = 0; k < MAX_COP_COUNT; k++) {
		if (get_ace_file_name(aceconf, str, &so_name) == -1)
			return -1;
		if (soname != NULL) {
			ret = loadSharedLib(so_name, k);
			if (ret == -1)
				return -1;
		}
		str[3] += 1;
	}
#endif

	/* ACE */
	strcpy(str, "ace=");
	if (get_ace_file_name(aceconf, str, &so_name) == -1) {
		LOG_ERROR("Get ace file name failed!!");
		return -1;
	}

	if (so_name == NULL) {
		LOG_ERROR("so_name empty");
		ret = 0;
	}

	LOG_DEBUG("so_name: %s", so_name);
	ret = loadSharedLib(so_name);
	LOG_DEBUG("end of loadSharedLib");
	return ret;
}

#define MAX_ACECONF_STRING	2048
#define MAX_TEXT_STRING		256
static int
parse_ace_conf(const char *aceconf, char *ace_opt)
{
	char *o_aceconf, *ace_dir, *ace_rel_fname;
	char line[MAX_TEXT_STRING];
	int len = 0;

	o_aceconf = strdup(aceconf);
	ace_dir = dirname(o_aceconf);
	LOG_DEBUG("ace_dir: %s", ace_dir);

	FILE *fp = fopen(aceconf, "r");
	if (NULL == fp) {
		LOG_ERROR("Error Open Failed!! aceconf: %s", aceconf);
		return -1;
	}

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
			LOG_DEBUG("--ace ace_rel_fname: %s", ace_rel_fname);
		} else if (strncmp(line, "--cop", 5) == 0 && line[5] >= '0' && line[5] <= '0' + MAX_COP_COUNT && line[6] == '=') {
			char c = line[7];
			line[7] = '\0';
			strcat(ace_opt, &line[2]);
			line[7] = c;
			ace_rel_fname = &line[7];
			LOG_DEBUG("--cop ace_rel_fname: %s", ace_rel_fname);
		} else {
			LOG_ERROR("invalid parameter: %s", line);
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
		LOG_DEBUG("ace_opt: %s", ace_opt);
	}
	fclose(fp);
	return 0;
}

int
nds32_ace_init(const char *aceconf,
	       unsigned *acr_type_count, unsigned *total_acr_reg_nums)
{
	LOG_DEBUG("aceconf: %s", aceconf);
	char ace_opt[MAX_ACECONF_STRING];

	global_acr_type_count = 0;
	global_acr_reg_count = 0;
	acr_info_list = NULL;
	ace_lib_for_gdb = NULL;
	ace_xml_for_gdb = NULL;

	if (aceconf == NULL || strcmp(aceconf, "") == 0) {
		LOG_DEBUG("aceconf does not specify!!");
		*acr_type_count = 0;
		*total_acr_reg_nums = 0;
		return 0;
	}

	*acr_type_count = 0;

	/* Get all ACE/COP options for this core first */
	/* and then get HW register information.       */
	if (-1 == parse_ace_conf(aceconf, ace_opt) || -1 == init_core_ace_reg_list(ace_opt))
		return -1;

	*acr_type_count = *global_acr_type_count;
	*total_acr_reg_nums = *global_acr_reg_count;
	return 0;
}

int32_t get_ace_file_name_for_gdb (const char *aceconf, const char *platform, char **name)
{
	int32_t ret = 0;
	if (ace_lib_for_gdb) {
		struct utsname os;
		if (uname (&os) == 0) {
			char *soname;
			const char *str;
			if (strcmp(platform, os.sysname) == 0) {
				/* Return binary share library.  */
				str = ace_lib_for_gdb;
			} else {
				/* Return XML text.  */
				str = ace_xml_for_gdb;
			}
			soname = strchr(str, ',');
			if (soname == NULL) {
				ret = -1; /* corrupted data */
			} else {
				unsigned len = soname - str;
				soname = (char*) malloc(len + 1);
				strncpy(soname, str, len);
				soname[len] = '\0';
				sscanf(&soname[1], "%u", &len);
				FILE *fd = fopen(soname, "w");
				if (fd == NULL) {
					ret = -1;
				} else {
					if (fwrite(str, len + strlen(soname) + 1, 1, fd) != 1) {
						ret = -1;
					} else {
						*name = soname;
					}
					fclose(fd);
				}
			}
		}
	} else {
		*name = NULL;
	}
	return ret;
}
