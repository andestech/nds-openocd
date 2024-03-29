/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#include "target.h"
#include <target/smp.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <helper/jim-nvp.h>
#include "register.h"
#include "riscv/riscv.h"
#include "riscv/encoding.h"
#include "riscv/debug_defines.h"
#include "riscv/ndsv5.h"
#include "riscv/ndsv5-013.h"
#include "breakpoints.h"
#include "ndsv5_ace.h"
#include "nds32_new/nds32_log.h"
#include <jtag/interfaces.h>
#include <jtag/adapter.h>

#include "semihosting_common.h"

extern char *user_algorithm_path;
extern bool algorithm_bin_read;
extern uint64_t g_value_milmb;
extern int update_trigger_config(struct target *target);
extern char *gpBitFieldFileName;
#define FILE_V5_BIT_FIELD      "ndsv5_tdesc_bitfield.xml"
char *gpBitFieldFileNameV5 = (char *)FILE_V5_BIT_FIELD;

extern int ndsv5_profile_init(struct target *target);
extern int ndsv5_profile_state(struct target *target);
extern int ndsv5_profile_post(struct target *target);
extern int ndsv5_burner_server_init(struct target *target);

/* Trace */
extern uint32_t ndsv5_tracer_all_cores_setting(void);
extern uint32_t ndsv5_tracer_disable(struct target *target);
extern int ndsv5_tracer_dumpfile(struct target *target, char *pFileName);
extern int ndsv5_tracer_decode_pktfile(char *pFileName);
extern int ndsv5_tracer_polling(struct target *target);
extern int ndsv5_tracer_capability_check(struct target *target);
extern uint32_t nds_tracer_action;
extern uint32_t nds_tracer_stop_on_wrap;
extern uint32_t nds_trTeSyncMax, nds_trTeInstMode;
extern uint32_t nds_teInhibitSrc;
extern uint64_t nds_tracer_active_id;
extern uint32_t nds_timestamp_on, nds_trTsControl;
extern uint32_t nds_trTeFilteriMatchInst;

/* global command context from openocd.c */
extern struct command_context *global_cmd_ctx;

#define NDS_TCL_REG_READ    "nds_reg_read.tcl"
#define NDS_TCL_REG_WRITE   "nds_reg_write.tcl"
#define NDS_TCL_DMI_READ    "nds_dmi_read.tcl"
#define NDS_TCL_DMI_WRITE   "nds_dmi_write.tcl"

FILE *nds_script_reg_get;
FILE *nds_script_reg_set;
FILE *nds_script_dmi_read;
FILE *nds_script_dmi_write;

uint64_t ndsv5_reg_misa_value;
uint32_t ndsv5_cur_script_status;
uint32_t ndsv5_cur_reg_number;
uint64_t ndsv5_cur_reg_value;
uint32_t ndsv5_cur_dmi_addr;
uint64_t ndsv5_cur_dmi_data;

uint32_t ena_hit_exception;
uint32_t nds_force_word_access;
uint32_t nds_force_aligned_access;
uint32_t nds_reg_symbolic_name;
uint32_t nds_scan_retry_times;
uint32_t nds_jtag_scans_optimize = 1;
uint32_t nds_jtag_max_scans = 32; /* 256 */
uint32_t nds_dis_condition_break;
uint32_t nds_dmi_access_mem;
uint32_t ndsv5_cur_target_xlen = 32;
uint32_t nds_no_reset_detect;
uint32_t nds_no_halt_detect;
uint32_t nds_ftdi_devices;
uint32_t nds_halt_on_reset = 1;
int nds_targetburn_targetnum;
int nds_targetburn_corenum;
char *nds_remotetargetburn_fpath;
char *nds_remotetargetburn_buffer;
int nds_remotetargetburn_fsize;
unsigned int nds_remotetargetburn_chksum;
bool collect_remotetargetburn_file;

extern char *log_output_path;
extern void nds_dump_detail_debug_info(void);
extern char *p_nds_bak_debug_buffer_cur;
extern char *p_nds_bak_debug_buffer_start;
extern char *p_nds_bak_debug_buffer_end;
extern uint32_t nds_bak_debug_file_nums;
uint32_t nds_bak_debug_buf_size;

struct nds32_v5 *gpnds32_v5;
extern unsigned int MaxLogFileSize;
char *ndsv5_dump_trace_folder;


static const char *const NDS_MEMORY_ACCESS_NAME[] = {
	"BUS",
	"CPU",
};

uint32_t v5_count_to_check_dm    = 3000;
uint32_t v5_dmi_busy_delay_count = 3;
bool     v5_stepie;

extern void nds32_do_log_callback(struct target *target);
unsigned int ndsv5_do_once;
static void ndsv5_do_once_time(struct target *target)
{
	if (ndsv5_do_once)
		return;

	nds32_do_log_callback(target);
	ndsv5_do_once = 1;
}

/** Convert target handle to generic Andes target state handle. */
struct nds32_v5 *target_to_nds32_v5(struct target *target)
{
	struct nds32_v5 *pnds32_v5;
	struct target *ttarget = target;

	if (target->smp) {
		/* Find SMP group header */
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			if (t->rtos) {
				ttarget = t;
				break;
			}
		}
	}

	for (pnds32_v5 = gpnds32_v5; pnds32_v5; pnds32_v5 = pnds32_v5->next) {
		if (pnds32_v5->target == ttarget)
			break;
	}

	return pnds32_v5;
}

bool is_ndsv5(struct target *target)
{
	struct nds32_v5 *ndsv5 = target_to_nds32_v5(target);
	if (ndsv5 == NULL)
		return false;

	return ndsv5->common_magic == NDSV5_COMMON_MAGIC;
}

static int ndsv5_init_arch_info(struct target *target, struct nds32_v5 *new_nds32)
{
	new_nds32->common_magic = NDSV5_COMMON_MAGIC;

	if (!target->smp || target->rtos)
		new_nds32->target = target;
	new_nds32->next = NULL;
	new_nds32->hit_syscall = false;
	new_nds32->active_syscall_id = NDS_EBREAK_UNDEFINED;
	new_nds32->virtual_hosting_errno = 0;
	new_nds32->virtual_hosting_ctrl_c = false;
	new_nds32->attached = false;
	new_nds32->target_burn_attached = false;
	new_nds32->aceconf = target->variant;
	new_nds32->boot_time = 1500;
	new_nds32->reset_time = 1000;
	new_nds32->reset_halt_as_examine = false;
	new_nds32->auto_convert_hw_bp = true;
	new_nds32->execute_register_init = false;
	target->fileio_info = malloc(sizeof(struct gdb_fileio_info));
	target->fileio_info->identifier = NULL;
	new_nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
	new_nds32->nds_do_fencei = false;

	if (gpnds32_v5) {
		for (struct nds32_v5 *pnds32_v5 = gpnds32_v5; pnds32_v5; pnds32_v5 = pnds32_v5->next) {
			if (pnds32_v5->next == NULL) {
				pnds32_v5->next = new_nds32;
				break;
			}
		}
	} else {
		gpnds32_v5 = new_nds32;
	}

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_query_target_command)
{
	command_print(CMD, "OCD");
	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_query_endian_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t big_endian = 0;

	if (big_endian)
		command_print(CMD, "%s: BE", target_name(target));
	else
		command_print(CMD, "%s: LE", target_name(target));

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_query_cpuid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct target *tmptarget;
	uint32_t number_of_core = 0;

	for (tmptarget = all_targets; tmptarget; tmptarget = tmptarget->next)
		number_of_core++;

	if (number_of_core == 0)
		command_print(CMD, " ");
	else if (number_of_core == 1)
		command_print(CMD, " ");
	else
		command_print(CMD, "%s", target_name(target));

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_query_capability_command)
{
	/* //struct target *target = get_current_target(CMD_CTX); */

	/* AndeSight query disbus value to decide bus mode icon exist or
	 * not(default value 0 mean bus mode icon exist) */
	uint32_t if_tracer = 0, if_profiling = 1, disable_busmode = 0;
	uint32_t hit_exception = 1, if_targetburn = 1;
	uint32_t if_pwr_sample = 0;
	uint32_t q_access_mode = 0;
	uint32_t no_cctl_idx = 0;
	uint32_t tlb_dump = 0;

	/* According to eticket 16199: system bus access incomplete support by hardware,
	 * so report 0 to AndeSight query(means:bus mode auto refresh not support),
	 * before hardware incomplete support system bus access, ndsv5_sys_bus_supported is 0 */
	uint32_t system_bus_access = nds_sys_bus_supported;

	struct target *target = get_current_target(CMD_CTX);
	if (riscv_debug_buffer_size(target) >= 7)
		q_access_mode = 1;

	/* Check L1  */
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *icache = NULL;
	struct nds32_v5_cache *dcache = NULL;
	if (nds32 != NULL && (ndsv5_init_cache(target) == ERROR_OK)) {
		icache = &(nds32->memory.icache);
		dcache = &(nds32->memory.dcache);
	} else {
		LOG_ERROR("gpnds32_v5 is NULL");
		icache = NULL;
		dcache = NULL;
	}

	/* check l2c before access */
	uint64_t tmp;
	ndsv5_check_l2cache_exist(target, &tmp);


	/* Bug-26232, set disbus to 1 when HW system bus access is not supported */
	disable_busmode = (system_bus_access)? 0 : 1;

	/* tracer capability checking */
	if (ndsv5_tracer_capability_check(target) == ERROR_OK)
		if_tracer = 1;
	else
		if_tracer = 0;

	/* check IX CCTL command support */
	if (ndsv5_mml_capability_check(target) != ERROR_OK)
		no_cctl_idx = 1;
	else
		no_cctl_idx = 0;

	if (ndsv5_tlb_dump_capability_check(target) == ERROR_OK)
		tlb_dump = 1;
	else
		tlb_dump = 0;

	command_print(CMD, "tracer:%d;"
			   "profiling:%d;"
			   "disbus:%d;"
			   "exception:%d;"
			   "targetburn:%d;"
			   "pwr:%d;"
			   "q_access_mode:%d;"
			   "sysbusaccess:%d;"
			   "l1i_support:%d;"
			   "l1d_support:%d;"
			   "l2c_support:%d;"
			   "no_cctl_idx:%d;"
			   "tlb_dump:%d;"
			   "btb_dump:%d;",
				if_tracer,
				if_profiling,
				disable_busmode,
				hit_exception,
				if_targetburn,
				if_pwr_sample,
				q_access_mode,
				system_bus_access,
				((icache != NULL && icache->line_size) ? 1 : 0),
				((dcache != NULL && dcache->line_size) ? 1 : 0),
				ndsv5_l2c_support,
				no_cctl_idx,
				tlb_dump,
				0);
	return ERROR_OK;
}

static unsigned long long string_to_ulong(const char *str)
{
	errno = 0;
	unsigned long long value = strtoull(str, NULL, 0);
	if ((errno == ERANGE && (value == ULONG_MAX)) || (errno != 0 && value == 0))
		perror("strtoul");

	return value;
}

extern const char *ndsv5_burner_port;
extern uint32_t ndsspi_write_bytes_per_dot;
__COMMAND_HANDLER(handle_ndsv5_configure_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	static int do_once;

	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}

	/* check if any property is given */
	if (0 >= CMD_ARGC) {
		command_print(CMD, "configure: no property given!");
		return ERROR_FAIL;
	}

	LOG_DEBUG("NDS configure on [%s] hart %d", target->tap->dotted_name, target->coreid);

	/* check if property is supported */
	if (strcmp(CMD_ARGV[0], "run_mode") == 0) {
		if (CMD_ARGC > 1) {
			if (strcmp(CMD_ARGV[1], "debug") == 0)
				nds32->gdb_run_mode = 0;
			else if (strcmp(CMD_ARGV[1], "profile") == 0)
				nds32->gdb_run_mode |= 0x01;
			else if (strcmp(CMD_ARGV[1], "pwr") == 0)
				nds32->gdb_run_mode |= 0x02;
		}
		command_print(CMD, "configure: %s = %s", CMD_ARGV[0], CMD_ARGV[1]);
	} else if (strcmp(CMD_ARGV[0], "disable_pwr") == 0) {
		nds32->gdb_run_mode &= ~0x02;
	} else if (strcmp(CMD_ARGV[0], "disable_profile") == 0) {
		nds32->gdb_run_mode &= ~0x01;
	} else if (strcmp(CMD_ARGV[0], "prof_addr_min") == 0) {
		if (CMD_ARGC > 1)
			nds32->prof_addr_min = string_to_ulong(CMD_ARGV[1]);

		command_print(CMD, "configure: %s = 0x%" PRIx64, CMD_ARGV[0], nds32->prof_addr_min);
	} else if (strcmp(CMD_ARGV[0], "prof_addr_max") == 0) {
		if (CMD_ARGC > 1)
			nds32->prof_addr_max = string_to_ulong(CMD_ARGV[1]);

		command_print(CMD, "configure: %s = 0x%" PRIx64, CMD_ARGV[0], nds32->prof_addr_max);
	} else if (strcmp(CMD_ARGV[0], "prof_with_range") == 0) {
		if (CMD_ARGC > 1)
			nds32->prof_with_range = (uint32_t)string_to_ulong(CMD_ARGV[1]);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32->prof_with_range);
	} else if (strcmp(CMD_ARGV[0], "can_profile") == 0) {
		/* can_profile no work but keep this command for ide */
	} else if (strcmp(CMD_ARGV[0], "ena_hit_exception") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ena_hit_exception);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], ena_hit_exception);
	} else if (strcmp(CMD_ARGV[0], "log_file_size") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], MaxLogFileSize);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], MaxLogFileSize);
	} else if (strcmp(CMD_ARGV[0], "desc") == 0) {
		command_print(CMD, "configure: %s %s", CMD_ARGV[0], CMD_ARGV[1]);
	} else if (strcmp(CMD_ARGV[0], "tdesc_bit") == 0) {
		gpBitFieldFileNameV5 = strdup(CMD_ARGV[1]);
		command_print(CMD, "configure: %s %s", CMD_ARGV[0], gpBitFieldFileNameV5);
	} else if (strcmp(CMD_ARGV[0], "burn_port") == 0) {
		ndsv5_burner_port = strdup(CMD_ARGV[1]);
		command_print(CMD, "configure: %s %s", CMD_ARGV[0], ndsv5_burner_port);

		if (do_once == 0) {
			ndsv5_burner_server_init(target);
			do_once = 1;
		}
	} else if (strcmp(CMD_ARGV[0], "word_access") == 0) {
		nds_force_word_access = 1;
		command_print(CMD, "configure: %s", CMD_ARGV[0]);
	} else if (strcmp(CMD_ARGV[0], "aligned_access") == 0) {
		nds_force_aligned_access = 1;
		command_print(CMD, "configure: %s", CMD_ARGV[0]);
	} else if (strcmp(CMD_ARGV[0], "reg_symbolic_name") == 0) {
		nds_reg_symbolic_name = 1;
		command_print(CMD, "configure: %s", CMD_ARGV[0]);
	} else if (strcmp(CMD_ARGV[0], "scan_retry_times") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_scan_retry_times);
		if (nds_scan_retry_times > 5)
			nds_scan_retry_times = 5;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_scan_retry_times);
	} else if (strcmp(CMD_ARGV[0], "jtag_scans_optimize") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_jtag_scans_optimize);
		else
			nds_jtag_scans_optimize = 1;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_jtag_scans_optimize);
	} else if (strcmp(CMD_ARGV[0], "redirect_mem_func") == 0) {
		/* ??? */
	} else if (strcmp(CMD_ARGV[0], "use_script") == 0) {
		FILE *script_fd = fopen(NDS_TCL_REG_READ, "r");
		if (script_fd)
			nds_script_reg_get = script_fd;
		script_fd = fopen(NDS_TCL_REG_WRITE, "r");
		if (script_fd)
			nds_script_reg_set = script_fd;
		script_fd = fopen(NDS_TCL_DMI_READ, "r");
		if (script_fd)
			nds_script_dmi_read = script_fd;
		script_fd = fopen(NDS_TCL_DMI_WRITE, "r");
		if (script_fd)
			nds_script_dmi_write = script_fd;
	} else if (strcmp(CMD_ARGV[0], "jtag_max_scans") == 0) {
		uint32_t tmp_jtag_max_scans;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], tmp_jtag_max_scans);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], tmp_jtag_max_scans);

		if ((nds_ftdi_devices == 1) && (tmp_jtag_max_scans >= nds_jtag_max_scans)) {
			LOG_INFO("nds_ftdi_devices=1 and user defined jtag_max_scans >= %d,"
					"no update nds_jtag_max_scans", nds_jtag_max_scans);
			LOG_INFO("nds_jtag_max_scans: %d", nds_jtag_max_scans);
		} else {
			LOG_INFO("Setting nds_jtag_max_scans(%d) to tmp_jtag_max_scans(%d)",
					nds_jtag_max_scans, tmp_jtag_max_scans);
			nds_jtag_max_scans = tmp_jtag_max_scans;
		}
	} else if (strcmp(CMD_ARGV[0], "dis_condition_break") == 0) {
		nds_dis_condition_break = 1;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_dis_condition_break);
	} else if (strcmp(CMD_ARGV[0], "base64enc_info") == 0) {
		collect_remotetargetburn_file = false;
		if (CMD_ARGC < 4) {
			LOG_ERROR("error: the count of arguments of base64enc info < 4");
			return ERROR_FAIL;
		}

		char *filename = strdup(CMD_ARGV[1]);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], nds_remotetargetburn_fsize);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], nds_remotetargetburn_chksum);
		LOG_DEBUG("file name: %s, size: %d, checksum: %d",
				filename, nds_remotetargetburn_fsize, nds_remotetargetburn_chksum);

		/* add iceman log path */
		size_t log_path_len = strlen(log_output_path);
		size_t filename_len = strlen(filename);
		nds_remotetargetburn_fpath = calloc(log_path_len + filename_len + 2, 1);
		if (log_path_len > 0) {
			char *c = strstr(log_output_path, "iceman_debug0.log");
			if (c) {
				*c = '\0';
				log_path_len = strlen(log_output_path);
			}
			if (log_path_len > 0) {
				strncpy(nds_remotetargetburn_fpath, log_output_path, log_path_len);
				if (log_output_path[log_path_len - 1] != '/') {
					/* strncat(nds_remotetargetburn_fpath, "/", 1); */
					nds_remotetargetburn_fpath[log_path_len] = '/';
					nds_remotetargetburn_fpath[log_path_len+1] = '\0';
				}
			}
		}
		strncat(nds_remotetargetburn_fpath, filename, filename_len);

		/* remove older file */
		remove(nds_remotetargetburn_fpath);
		nds_remotetargetburn_buffer = calloc(nds_remotetargetburn_fsize, 1);
		collect_remotetargetburn_file = true;
		command_print(CMD, "configure: %lu %s", strlen(nds_remotetargetburn_fpath), nds_remotetargetburn_fpath);
	} else if (strcmp(CMD_ARGV[0], "algorithm_bin") == 0) {
		if (user_algorithm_path)
			free(user_algorithm_path);
		user_algorithm_path = strdup(CMD_ARGV[1]);
		command_print(CMD, "configure: %s = %s", CMD_ARGV[0], user_algorithm_path);
		algorithm_bin_read = true;
		nds32->target_burn_attached = true;

		struct target *use_target = get_current_target(CMD_CTX);

		/* target_burn send targets, need set nds_targetburn_targetnum value */
		nds_targetburn_targetnum = use_target->target_number;

#if 0
		/*get enable coreid of user defined target(nds_targetburn_targetnum) for run_algorithm */
		int hartid = -1;
		for (int i = 0; i < riscv_count_harts(use_target); ++i) {
			if (use_target->smp) {
				LOG_DEBUG("nds_targetburn_corenum : %d", i);
				hartid = i;
				break;
			}
		}
		if (hartid < 0) {
			LOG_ERROR("hartid < 0, no hart enable");
			return ERROR_FAIL;
		}
		nds_targetburn_corenum = hartid;
#endif
		nds_targetburn_corenum = use_target->coreid;
		riscv_set_current_hartid(use_target, nds_targetburn_corenum);
		use_target->coreid = nds_targetburn_corenum;
		RISCV_INFO(r);
		if (target->smp)
			r->current_hartid = nds_targetburn_corenum;
		LOG_DEBUG("target->coreid: %d and r->rtos_hartid: %d", use_target->coreid, r->current_hartid);

		/* FOR AE350 : ENABLE ILM*/
		LOG_DEBUG("enable ilm for nds_targetburn_targetnum: %d and nds_targetburn_corenum: %d",
				nds_targetburn_targetnum, nds_targetburn_corenum);
		struct reg *reg_name = ndsv5_get_reg_by_CSR(use_target, CSR_MICM_CFG);
		uint64_t value = ndsv5_get_register_value(reg_name);
		NDS_INFO("micm_cfg = 0x%" PRIx64, value);
		/* micm_cfg.ILMB != 0 (bit 14-12) CSR_MILMB exist */
		if ((value & 0x7000) != 0) {
			NDS_INFO("Enabling ILM");
			struct reg *reg_milmb = ndsv5_get_reg_by_CSR(use_target, CSR_MILMB);
			if (reg_milmb == NULL) {
				LOG_ERROR("get reg_milmb ERROR");
				return ERROR_FAIL;
			}
			g_value_milmb = ndsv5_get_register_value(reg_milmb);
			if ((g_value_milmb & 0x1) == 0)
				ndsv5_set_register_value(reg_milmb, (g_value_milmb | 0x1));
		}
	} else if (strcmp(CMD_ARGV[0], "dmi_quick_access") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_dmi_quick_access);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_dmi_quick_access);

	} else if (strcmp(CMD_ARGV[0], "dmi_access_mem") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_dmi_access_mem);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_dmi_access_mem);

	} else if (strcmp(CMD_ARGV[0], "sys_bus_access") == 0) {
		/* According to eticket 16199, default no support system bus access,
		 * so ndsv5_system_bus_access default value is 0 */
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ndsv5_system_bus_access);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], ndsv5_system_bus_access);
	} else if (strcmp(CMD_ARGV[0], "halt_on_reset") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_halt_on_reset);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_halt_on_reset);


		if (target_was_examined(target))
			ndsv5_haltonreset(target, nds_halt_on_reset);

	} else if (strcmp(CMD_ARGV[0], "wip_bytes_per_dot") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ndsspi_write_bytes_per_dot);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], ndsspi_write_bytes_per_dot);
	} else if (strcmp(CMD_ARGV[0], "lm_base") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], LM_BASE);
		command_print(CMD, "configure: %s = 0x%" PRIx64, CMD_ARGV[0], LM_BASE);
	} else if (strcmp(CMD_ARGV[0], "targetburn_targetnum") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], nds_targetburn_targetnum);

		struct target *use_target;
		bool target_num_ok = false;

		/* target_burn_frontend no define targets name, only define target number */
		for (use_target = all_targets; use_target; use_target = use_target->next) {
			if (use_target->target_number == nds_targetburn_targetnum) {
				target_num_ok = true;
				break;
			}
		}
		if (!target_num_ok)
			LOG_ERROR("target number %d is not exist", nds_targetburn_targetnum);

		if (!use_target->tap->enabled)
			LOG_ERROR("Target: TAP %s is disabled, can't be the current target", use_target->tap->dotted_name);

		/* change to the wanted targets */
		CMD_CTX->current_target = use_target;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_targetburn_targetnum);
	} else if (strcmp(CMD_ARGV[0], "custom_srst_script") == 0) {
		if (CMD_ARGC > 1) {
			if (ndsv5_script_custom_reset) {
				free(ndsv5_script_custom_reset);
				ndsv5_script_custom_reset = NULL;
			}
			ndsv5_script_custom_reset = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s: %s", CMD_ARGV[0], ndsv5_script_custom_reset);
		}
	} else if (strcmp(CMD_ARGV[0], "custom_trst_script") == 0) {
		if (CMD_ARGC > 1) {
			if (ndsv5_script_custom_reset) {
				free(ndsv5_script_custom_reset);
				ndsv5_script_custom_reset = NULL;
			}
			ndsv5_script_custom_reset = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s: %s", CMD_ARGV[0], ndsv5_script_custom_reset);
		}
	} else if (strcmp(CMD_ARGV[0], "custom_restart_script") == 0) {
		if (CMD_ARGC > 1) {
			if (ndsv5_script_custom_reset_halt) {
				free(ndsv5_script_custom_reset_halt);
				ndsv5_script_custom_reset_halt = NULL;
			}
			ndsv5_script_custom_reset_halt = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s: %s", CMD_ARGV[0], ndsv5_script_custom_reset_halt);
		}
	} else if (strcmp(CMD_ARGV[0], "custom_initial_script") == 0) {
		if (CMD_ARGC > 1) {
			if (ndsv5_script_custom_initial) {
				free(ndsv5_script_custom_initial);
				ndsv5_script_custom_initial = NULL;
			}
			ndsv5_script_custom_initial = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s: %s", CMD_ARGV[0], ndsv5_script_custom_initial);
		}
	} else if (strcmp(CMD_ARGV[0], "dmi_busy_retry_times") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ndsv5_dmi_busy_retry_times);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], ndsv5_dmi_busy_retry_times);
	} else if (strcmp(CMD_ARGV[0], "bak_debug_buf") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_bak_debug_buf_size);
		if (nds_bak_debug_buf_size < 0x80000)
			nds_bak_debug_buf_size = 0x80000;
		else if (nds_bak_debug_buf_size >= 0x800000)
			nds_bak_debug_buf_size = 0x800000;
		if (CMD_ARGC > 2)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], nds_bak_debug_file_nums);
		if (nds_bak_debug_file_nums > 16)
			nds_bak_debug_file_nums = 16;
		else if (nds_bak_debug_file_nums < 2)
			nds_bak_debug_file_nums = 2;

		if (p_nds_bak_debug_buffer_start) {
			free(p_nds_bak_debug_buffer_start);
			p_nds_bak_debug_buffer_start = NULL;
		}
		p_nds_bak_debug_buffer_start = (char *)malloc(nds_bak_debug_buf_size);
		p_nds_bak_debug_buffer_end = p_nds_bak_debug_buffer_start + nds_bak_debug_buf_size;
		p_nds_bak_debug_buffer_cur = p_nds_bak_debug_buffer_start;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_bak_debug_buf_size);
		NDS_INFO("p_nds_bak_debug_buffer_start = 0x%lx, p_nds_bak_debug_buffer_end = 0x%lx",
			(long unsigned int)p_nds_bak_debug_buffer_start, (long unsigned int)p_nds_bak_debug_buffer_end);
	} else if (strcmp(CMD_ARGV[0], "dump_detail_debug_info") == 0) {
		nds_dump_detail_debug_info();
	} else if (strcmp(CMD_ARGV[0], "vector_length") == 0) {
		uint32_t vector_length = 0;
		if (CMD_ARGC > 1) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vector_length);
			nds32->nds_vector_length = vector_length;
		}
	} else if (strcmp(CMD_ARGV[0], "l2c_base") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], L2C_BASE);
		ndsv5_l2c_support = 1;
		command_print(CMD, "configure: %s = 0x%" PRIx64, CMD_ARGV[0], L2C_BASE);
	} else if (strcmp(CMD_ARGV[0], "suppressed_hsp_exception") == 0) {
		bool option = false;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_ON_OFF(CMD_ARGV[1], option);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], option);
		if (ndsv5_suppressed_hsp_exception(target, option) == ERROR_OK)
			nds32->suppressed_hsp_exception = option;
		else
			return ERROR_FAIL;
	} else if (strcmp(CMD_ARGV[0], "no-n22-workaround-imprecise-ldst") == 0) {
		bool option = false;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_ON_OFF(CMD_ARGV[1], option);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], option);
		nds32->no_n22_workaround_imprecise_ldst = option;
	} else if (strcmp(CMD_ARGV[0], "no-n22-workaround-imprecise-div") == 0) {
		bool option = false;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_ON_OFF(CMD_ARGV[1], option);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], option);
		nds32->no_n22_workaround_imprecise_div = option;
	} else if (strcmp(CMD_ARGV[0], "no-group") == 0) {
		bool option = false;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_ON_OFF(CMD_ARGV[1], option);

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], option);
		nds32->no_group = option;
	} else if (strcmp(CMD_ARGV[0], "dump-trace-folder") == 0) {
		if (CMD_ARGC > 1)
			ndsv5_dump_trace_folder = strdup(CMD_ARGV[1]);

		if (!ndsv5_dump_trace_folder) {
			LOG_ERROR("configure: %s failed", CMD_ARGV[0]);
			return ERROR_FAIL;
		}
		command_print(CMD, "configure: %s = %s", CMD_ARGV[0], ndsv5_dump_trace_folder);
	} else {
		command_print(CMD, "configure: property '%s' unknown!", CMD_ARGV[0]);
		NDS32_LOG("<-- configure: property '%s' unknown! -->", CMD_ARGV[0]);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_ace_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	NDS_INFO("handle_nds32_ace_command");

	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}

	/* Check if any option is given */
	if (CMD_ARGC <= 0) {
		command_print(CMD, "ace: no option is given!");
		return ERROR_FAIL;
	}

	/* This function handles two job
	 * (1) parse aceconf_path (and then load libacedbg.so) or
	 *     load libacedbg.so direclty to get all variables in
	 *     libacedbg.so
	 * (2) handle monitor command from GDB client to get the
	 *     file name of shared library used to disassemble
	 *     ACE instructions (this library is same with
	 *     libacetool.so)
	 */
	if (strcmp(CMD_ARGV[0], "aceconf_path") == 0) {
		/* Parse option from openocd.cfg.v5 */
		NDS_INFO("aceconf_path = %s", CMD_ARGV[1]);

		char *aceconf_path;
		aceconf_path = strdup(CMD_ARGV[1]);

		if (aceconf_path == NULL) {
			LOG_ERROR("aceconf_path is NULL");
			return ERROR_FAIL;
		} else {
			nds32->aceconf = aceconf_path;
			int32_t ret = nds32_ace_init_v5(nds32->aceconf);
			if (ret == -1) {
				LOG_ERROR("Fault to initialize ACE");
				return ERROR_FAIL;
			}
		}
	} else {
		/* Handle command from GDB client */
		NDS_INFO("CMD_ARGV[0] = %s", CMD_ARGV[0]);
		if (CMD_ARGC > 0) {
			char *ace_file_name = NULL;
			int32_t ret =
				get_ace_file_name_for_gdb_v5(nds32->aceconf, CMD_ARGV[0], &ace_file_name);

			NDS_INFO("Return of get_ace_file_name_for_gdb_v5() = %d", ret);
			if (ret == 0) {
				if (ace_file_name != NULL) {
					command_print(CMD, "%s", ace_file_name);
					free(ace_file_name);
				}
			}
		}
	}

	return ERROR_OK;
}

int ndsv5cmd_set_boot_time(struct target *target, uint32_t boot_time)
{
	struct nds32_v5 *ndsv5 = target_to_nds32_v5(target);
	ndsv5->boot_time = boot_time;
	LOG_DEBUG("ndsv5->boot_time = %d", ndsv5->boot_time);
	return ERROR_OK;
}

int ndsv5cmd_set_reset_time(struct target *target, uint32_t reset_time)
{
	struct nds32_v5 *ndsv5 = target_to_nds32_v5(target);
	ndsv5->reset_time = reset_time;
	LOG_DEBUG("ndsv5->reset_time = %d", ndsv5->reset_time);
	return ERROR_OK;
}

int ndsv5cmd_set_reset_halt_as_examine(struct target *target, bool reset_halt_as_examine)
{
	struct nds32_v5 *ndsv5 = target_to_nds32_v5(target);
	ndsv5->reset_halt_as_examine = reset_halt_as_examine;
	LOG_DEBUG("ndsv5->reset_halt_as_examine = %d", (int)ndsv5->reset_halt_as_examine);
	return ERROR_OK;
}

COMMAND_HANDLER(ndsv5_handle_playground)
{
	/* struct target *target = get_current_target(CMD_CTX); */

	/* Below is playground */
	/*
	if (CMD_ARGC > 0) {

	} else {
		LOG_ERROR("ARGC failed!");
		return ERROR_FAIL;
	}
	*/
	return ERROR_OK;
}

COMMAND_HANDLER(ndsv5_handle_l2c_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "dump") == 0) {
			if (ndsv5_l2c_support == 0) {
				command_print(CMD, "%s: No L2 cache", target_name(target));
				return ERROR_OK;
			}

			if (CMD_ARGC < 3) {
				LOG_ERROR("Usage: dump va <address>");
				return ERROR_FAIL;
			} else if (strcmp(CMD_ARGV[1], "va") == 0) {
				uint64_t va;
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[2], va);

				if (CMD_ARGC == 3)
					return ndsv5_dump_l2cache_va(target, va);

				uint64_t way;
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[3], way);
				return ndsv5_dump_l2cache_va_way(target, va, way);
			} else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: dump va <address>");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[0], "query") == 0) {
			if (ndsv5_l2c_support == 0) {
				command_print(CMD, "%s: No L2 cache", target_name(target));
				return ERROR_OK;
			}

			if (CMD_ARGC != 2) {
				LOG_ERROR("Usage: Usage: query config");
				return ERROR_FAIL;
			} else if (strcmp(CMD_ARGV[1], "config") == 0) {
				return ndsv5_query_l2cache_config(target);
			} else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: query config");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[0], "writeback_invalidate") == 0) {
			if (ndsv5_l2c_support == 0) {
				command_print(CMD, "%s: No L2 cache", target_name(target));
				return ERROR_OK;
			}

			return ndsv5_l2cache_wb_invalidate(target);
		} else {
			LOG_ERROR("No valid paramerter");
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(nds32_handle_count_to_check_dm_command)
{
	char c;
	char *count_to_check_dm;

	NDS_INFO("handle_nds32_handle_count_to_check_dm_command");
	if (CMD_ARGC != 0) {
		count_to_check_dm = strdup(CMD_ARGV[0]);
		sscanf(count_to_check_dm, " %u%c", &v5_count_to_check_dm, &c);
		if (c == 's' || c == 'S')
			v5_count_to_check_dm *= 1000;

		NDS_INFO("count_to_check_dm=%d", v5_count_to_check_dm);
	} else
		LOG_ERROR("expected exactly one argument to nds count_to_check_dm "
				"<count_of_checking>");

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_dmi_busy_delay_count) {

	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	v5_dmi_busy_delay_count = atoi(CMD_ARGV[0]);
	NDS_INFO("riscv_set_dmi_busy_delay_count: %d", v5_dmi_busy_delay_count);
	return ERROR_OK;
}

int ndsv5cmd_set_va_to_pa_off(struct target *target, uint32_t va_to_pa_off)
{
	LOG_DEBUG("NDS set va_to_pa off on [%s] hart %d", target->tap->dotted_name, target->coreid);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	nds32->nds_va_to_pa_off = va_to_pa_off;
	NDS_INFO("va_to_pa_off: %d", nds32->nds_va_to_pa_off);
	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_memory_access_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	LOG_DEBUG("NDS mem_access on [%s] hart %d", target->tap->dotted_name, target->coreid);

	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "bus") == 0) {
			if (nds_sys_bus_supported)
				memory->access_channel = NDS_MEMORY_ACC_BUS;
			else {
				LOG_ERROR("memory access channel: BUS, but sysbusaccess:%d", nds_sys_bus_supported);
				LOG_INFO("Change back to CPU");
				memory->access_channel = NDS_MEMORY_ACC_CPU;
			}
		} else if (strcmp(CMD_ARGV[0], "cpu") == 0) {
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		} else {
			/* default access channel is NDS_MEMORY_ACC_CPU */
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		}
		NDS_INFO("memory access channel is change to %s", NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	} else {
		command_print(CMD, "%s: memory access channel: %s",
				target_name(target), NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	}
	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_cache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *icache = &(nds32->memory.icache);
	struct nds32_v5_cache *dcache = &(nds32->memory.dcache);
	int result = ERROR_OK;
	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}
	LOG_DEBUG("NDS cache command on [%s] hart %d", target->tap->dotted_name, target->coreid);

	riscv_select_current_hart(target);

	if (!nds32->execute_register_init) {
		struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
		uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
		reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
		uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);

		/* micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6), CSR_MCACHE_CTL exist */
		if (((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			ndsv5_dis_cache_busmode = 0;
			return result;
		}
	} else {
		if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCACHE_CTL].exist == false) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			return result;
		}
	}

	if (CMD_ARGC > 0) {
		if (ndsv5_init_cache(target) != ERROR_OK)
			return ERROR_FAIL;

		if (icache->line_size == 0 && dcache->line_size == 0) {
			command_print(CMD, "%s: No instruction cache and data cache", target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			if (icache->line_size > 0)
				result = ndsv5_icache_invalidate(target);
			else
				command_print(CMD, "%s: No instruction cache", target_name(target));

			if (dcache->line_size > 0)
				result = ndsv5_dcache_invalidate(target);
			else
				command_print(CMD, "%s: No data cache", target_name(target));

		} else
			command_print(CMD, "%s: No valid parameter", target_name(target));
	}

	return result;
}

static int ndsv5_iord_cache_dump(struct target *target, unsigned int cache_type, const char **argv, int argc)
{
	if (argc < 2) {
		LOG_ERROR("Usage: dump all <filename>(optimal) / dump va <address>");
		return ERROR_FAIL;
	}

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	const char *file_name;
	int result = ERROR_OK;
	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}

	if ((cache_type != ICACHE) && (cache_type != DCACHE)) {
		LOG_ERROR("No supported cache type:%x", cache_type);
		return ERROR_FAIL;
	}

	if (!nds32->execute_register_init) {
		struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
		uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
		reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
		uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);
		reg_name = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
		uint64_t mmsc_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("mmsc_cfg = 0x%" PRIx64, mmsc_cfg_value);

		/* (micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0) & (mmsc_cfg.CCTLCSR == 1) */
		if ((((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) ||
		     ((mmsc_cfg_value & 0x10000) == 0)) {
			NDS_INFO("CSR_MCCTLBEGINADDR/COMMAND/DATA not exist");
			return result;
		}
	} else {
		/* if CSR_MCCTLBEGINADDR no exist => CSR_MCCTLCOMMAND/CSR_MCCTLDATA no exist */
		if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCTLBEGINADDR].exist == false) {
			NDS_INFO("CSR_MCCTLBEGINADDR/COMMAND/DATA not exist");
			return result;
		}
	}

	if (strcmp(argv[1], "all") == 0) {
		if (argc == 3)
			file_name = argv[2];
		else {
			if (cache_type == ICACHE)
				file_name = "icache.dump";
			else
				file_name = "dcache.dump";
		}
		NDS_INFO("dump all %ccache to file: %s", file_name[0], file_name);
		result = ndsv5_dump_cache(target, cache_type, file_name);
	} else if (strcmp(argv[1], "va") == 0) {
		if (argc < 3) {
			LOG_ERROR("Usage: dump va <address>");
			return ERROR_FAIL;
		}
		uint64_t va;
		result = parse_ullong(argv[2], (unsigned long long *)&(va));
		if (result != ERROR_OK) {
			LOG_ERROR("option value ('%s') is not valid", argv[2]);
			return result;
		}
		result = ndsv5_dump_cache_va(target, cache_type, va);
	} else {
		LOG_ERROR("%s: No valid parameter", target_name(target));
		LOG_ERROR("Usage: dump all <filename>(optimal) / dump va <address>");
	}

	return result;
}

__COMMAND_HANDLER(handle_ndsv5_icache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *icache = &(nds32->memory.icache);
	int result = ERROR_OK;
	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}
	LOG_DEBUG("NDS icache command on [%s] hart %d", target->tap->dotted_name, target->coreid);

	riscv_select_current_hart(target);
	if (!nds32->execute_register_init) {
		struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
		uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
		reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
		uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);

		/* micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6), CSR_MCACHE_CTL exist */
		if (((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			ndsv5_dis_cache_busmode = 0;
			return result;
		}
	} else {
		if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCACHE_CTL].exist == false) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			return result;
		}
	}

	if (CMD_ARGC > 0) {
		if (ndsv5_init_cache(target) != ERROR_OK)
			return ERROR_FAIL;
		if (icache->line_size == 0) {
			command_print(CMD, "%s: No instruction cache", target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0)
			result = ndsv5_icache_invalidate(target);
		else if ((strcmp(CMD_ARGV[0], "enable") == 0) || (strcmp(CMD_ARGV[0], "disable") == 0))
			result = ndsv5_enableornot_cache(target, ICACHE, CMD_ARGV[0]);
		else if (strcmp(CMD_ARGV[0], "dump") == 0)
			result = ndsv5_iord_cache_dump(target, ICACHE, CMD_ARGV, CMD_ARGC);
		else if (strcmp(CMD_ARGV[0], "query") == 0) {
			if (CMD_ARGC != 2) {
				LOG_ERROR("Usage: Usage: query config");
				return ERROR_FAIL;
			} else if (strcmp(CMD_ARGV[1], "config") == 0) {
				return ndsv5_query_l1cache_config(target, icache);
			} else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: query config");
				return ERROR_FAIL;
			}
		} else
			command_print(CMD, "%s: No valid parameter", target_name(target));
	}

	return result;
}

__COMMAND_HANDLER(handle_ndsv5_dcache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *dcache = &(nds32->memory.dcache);
	int result = ERROR_OK;

	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}
	LOG_DEBUG("NDS dcache command on [%s] hart %d", target->tap->dotted_name, target->coreid);

	riscv_select_current_hart(target);
	if (!nds32->execute_register_init) {
		struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
		uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
		reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
		uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
		NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);

		/* micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6), CSR_MCACHE_CTL exist */
		if (((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			ndsv5_dis_cache_busmode = 0;
			return result;
		}
	} else {
		if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCACHE_CTL].exist == false) {
			NDS_INFO("CSR_MCACHE_CTL not exist");
			return result;
		}
	}

	if (CMD_ARGC > 0) {
		if (ndsv5_init_cache(target) != ERROR_OK)
			return ERROR_FAIL;

		if (dcache->line_size == 0) {
			command_print(CMD, "%s: No data cache", target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0)
			result = ndsv5_dcache_invalidate(target);
		else if (strcmp(CMD_ARGV[0], "writeback") == 0)
			result = ndsv5_dcache_wb(target);
		else if (strcmp(CMD_ARGV[0], "writeback_invalidate") == 0)
			result = ndsv5_dcache_wb_invalidate(target);
		else if ((strcmp(CMD_ARGV[0], "enable") == 0) || (strcmp(CMD_ARGV[0], "disable") == 0))
			result = ndsv5_enableornot_cache(target, DCACHE, CMD_ARGV[0]);
		else if (strcmp(CMD_ARGV[0], "dump") == 0)
			result = ndsv5_iord_cache_dump(target, DCACHE, CMD_ARGV, CMD_ARGC);
		else if (strcmp(CMD_ARGV[0], "query") == 0) {
			if (CMD_ARGC != 2) {
				LOG_ERROR("Usage: Usage: query config");
				return ERROR_FAIL;
			} else if (strcmp(CMD_ARGV[1], "config") == 0) {
				return ndsv5_query_l1cache_config(target, dcache);
			} else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: query config");
				return ERROR_FAIL;
			}
		} else
			command_print(CMD, "%s: No valid parameter", target_name(target));
	}

	return result;
}

static COMMAND_HELPER(handle_ndsv5_tlb_command_helper, enum tlb_target type)
{
	struct target *target = get_current_target(CMD_CTX);
	int result;

	if (ndsv5_tlb_dump_capability_check(target) != ERROR_OK) {
		LOG_ERROR("TLB dump not support!");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "dump") == 0) {
			if (strcmp(CMD_ARGV[1], "all") == 0) {
				if (CMD_ARGC < 2) {
					command_print(CMD, "Usage: dump all [<filename>] / dump va <address> [<ASID>]");
					return ERROR_FAIL;
				}

				char *filename = NULL;
				if (CMD_ARGC == 3)
					filename = strdup(CMD_ARGV[2]);
				else
					filename = strdup("tlb.dump");

				LOG_DEBUG("dump all dcache to file: %s", filename);
				result = ndsv5_dump_tlb_all(target, filename, type);
				free(filename);
				return result;
			} else if (strcmp(CMD_ARGV[1], "va") == 0) {
				uint64_t va;
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[2], va);

				uint32_t asid = (uint32_t)-1;
				if (CMD_ARGC == 4)
					COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], asid);
				return ndsv5_dump_tlb_va(target, va, type, asid);
			} else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
				return ERROR_FAIL;
			}
		} else {
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_ndsv5_tlb_command)
{
	/* TLB = STLB (default) */
	return CALL_COMMAND_HANDLER(handle_ndsv5_tlb_command_helper, NDSV5_TLB_TARGET_STLB);
}

__COMMAND_HANDLER(handle_ndsv5_itlb_command)
{
	return CALL_COMMAND_HANDLER(handle_ndsv5_tlb_command_helper, NDSV5_TLB_TARGET_ITLB);
}

__COMMAND_HANDLER(handle_ndsv5_dtlb_command)
{
	return CALL_COMMAND_HANDLER(handle_ndsv5_tlb_command_helper, NDSV5_TLB_TARGET_DTLB);
}

__COMMAND_HANDLER(handle_ndsv5_stlb_command)
{
	return CALL_COMMAND_HANDLER(handle_ndsv5_tlb_command_helper, NDSV5_TLB_TARGET_STLB);
}

__COMMAND_HANDLER(handle_ndsv5_reset_and_hold)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target == NULL) {
		LOG_ERROR("target NULL");
		return ERROR_FAIL;
	}
	LOG_DEBUG("NDS reset_and_hold on [%s] hart %d", target->tap->dotted_name, target->coreid);
	int retval = ndsv5_reset_target(target, RESET_HALT);
	return retval;
}

static int ndsv5_reg_number(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	NDS_INFO("get ndsv5_reg_number = 0x%x", ndsv5_cur_reg_number);
	char *str = buf_to_hex_str(&ndsv5_cur_reg_number, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_reg_data(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc > 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	if (goi.argc == 1) {
		jim_wide puthere;
		int e = jim_getopt_wide(&goi, &puthere);
		if (e != JIM_OK)
			return e;
		ndsv5_cur_reg_value = (uint64_t)puthere;
		NDS_INFO("set ndsv5_reg_data = 0x%" PRIx64, ndsv5_cur_reg_value);
		return JIM_OK;
	}
	NDS_INFO("get ndsv5_reg_data = 0x%" PRIx64, ndsv5_cur_reg_value);
	char *str = buf_to_hex_str(&ndsv5_cur_reg_value, 64);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_script_status(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "wrong parameters");
		return JIM_ERR;
	}
	jim_wide puthere;
	int e = jim_getopt_wide(&goi, &puthere);
	if (e != JIM_OK)
		return e;
	ndsv5_cur_script_status = (uint32_t)puthere;
	NDS_INFO("set ndsv5_script_status = 0x%x", ndsv5_cur_script_status);
	return JIM_OK;
}

static int ndsv5_dmi_addr(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	NDS_INFO("get ndsv5_dmi_addr = 0x%x", ndsv5_cur_dmi_addr);
	char *str = buf_to_hex_str(&ndsv5_cur_dmi_addr, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_dmi_data(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc > 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	if (goi.argc == 1) {
		jim_wide puthere;
		int e = jim_getopt_wide(&goi, &puthere);
		if (e != JIM_OK)
			return e;
		ndsv5_cur_dmi_data = (uint64_t)puthere;
		NDS_INFO("set ndsv5_dmi_data = 0x%" PRIx64, ndsv5_cur_dmi_data);
		return JIM_OK;
	}
	NDS_INFO("get ndsv5_dmi_data = 0x%" PRIx64, ndsv5_cur_dmi_data);
	char *str = buf_to_hex_str(&ndsv5_cur_dmi_data, 64);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_target_xlen(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	struct command_context *cmd_ctx = global_cmd_ctx;
	struct target *target = get_current_target(cmd_ctx);
	ndsv5_cur_target_xlen = (uint32_t)riscv_xlen(target);
	NDS_INFO("get ndsv5_target_xlen = 0x%x", ndsv5_cur_target_xlen);
	char *str = buf_to_hex_str(&ndsv5_cur_target_xlen, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_target_count(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	struct target *tmptarget;
	uint32_t number_of_target = 0;
	for (tmptarget = all_targets; tmptarget; tmptarget = tmptarget->next)
		number_of_target++;

	NDS_INFO("get ndsv5_target_count = 0x%x", number_of_target);
	char *str = buf_to_hex_str(&number_of_target, 32);
	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_target_info(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 1) {
		/*
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "wrong parameters");
		return JIM_ERR;
		*/
		LOG_ERROR("wrong parameters");
		return JIM_OK;
	}
	long target_number = 0;
	Jim_GetLong(interp, argv[1], &target_number);
	NDS_INFO("target_number = 0x%x", (unsigned int)target_number);
	struct target *target = NULL;

	for (target = all_targets; target; target = target->next) {
		if (target->target_number == target_number)
			break;
	}
	if (target == NULL)
		return JIM_OK;

	char tmp_str[256];
	char *str = &tmp_str[0];
	uint32_t corenums = target->corenums;
	if (corenums == 0)
		corenums = 1;
	/*
	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	*/
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	sprintf(str, "%s %x %x",
			target_name(target), target->coreid, corenums);

	Jim_AppendStrings(interp, Jim_GetResult(interp), str, NULL);
	return JIM_OK;
}

static int ndsv5_adapter_khz(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	int retval;
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc > 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	if (goi.argc == 1) {
		long khz = 0;
		Jim_GetLong(interp, argv[1], &khz);
		NDS_INFO("set adapter_khz = 0x%x", (unsigned int)khz);
		retval = adapter_config_khz(khz);
		if (ERROR_OK != retval)
			return retval;
		return JIM_OK;
	}
	int cur_speed = adapter_get_speed_khz();
	retval = adapter_get_speed_readable(&cur_speed);
	if (ERROR_OK != retval)
		return retval;

	NDS_INFO("get adapter_khz = 0x%x", cur_speed);
	char *str = buf_to_hex_str(&cur_speed, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_jtag_tap_count(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc >= 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}

	int tap_count = jtag_tap_count();
	NDS_INFO("get tap_count = 0x%x", tap_count);
	char *str = buf_to_hex_str(&tap_count, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

static int ndsv5_jtag_cur_tap_name(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	struct command_context *cmd_ctx = global_cmd_ctx;
	struct target *target = get_current_target(cmd_ctx);
	Jim_ListAppendElement(goi.interp,
		Jim_GetResult(goi.interp),
		Jim_NewStringObj(goi.interp,
			target->tap->dotted_name, -1));
	return JIM_OK;
}

extern int ndsv5_get_delay_count(struct target *target);
static int ndsv5_get_dmi_delay(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	struct command_context *cmd_ctx = global_cmd_ctx;
	struct target *target = get_current_target(cmd_ctx);
	uint32_t ndsv5_cur_dmi_delay = (uint32_t)ndsv5_get_delay_count(target);
	NDS_INFO("get ndsv5_dmi_delay = 0x%x", ndsv5_cur_dmi_delay);
	char *str = buf_to_hex_str(&ndsv5_cur_dmi_delay, 32);

	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	Jim_ListAppendElement(goi.interp,
			Jim_GetResult(goi.interp),
			Jim_NewStringObj(goi.interp,
				str, -1));
	free(str);
	return JIM_OK;
}

int ndsv5cmd_set_auto_convert_hw_bp(struct target *target, bool auto_convert_hw_bp)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32 == NULL)
		return ERROR_FAIL;

	nds32->auto_convert_hw_bp = auto_convert_hw_bp;
	return ERROR_OK;
}

COMMAND_HANDLER(ndsv5_handle_stepie_handlers)
{

	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], v5_stepie);
	NDS_INFO("set stepie: %d", v5_stepie);
	return ERROR_OK;
}

COMMAND_HANDLER(ndsv5_get_smp_target_count)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t count = ndsv5_count_smp_target(target);

	NDS_INFO("smp_target_count: %d", count);
	command_print(CMD, "smp_target_count: %d", count);
	return ERROR_OK;
}

/* Copy from target.c */
static const struct jim_nvp nvp_target_endian[] = {
	{ .name = "big",    .value = TARGET_BIG_ENDIAN },
	{ .name = "little", .value = TARGET_LITTLE_ENDIAN },
	{ .name = "be",     .value = TARGET_BIG_ENDIAN },
	{ .name = "le",     .value = TARGET_LITTLE_ENDIAN },
	{ .name = NULL,     .value = -1 },
};
COMMAND_HANDLER(handle_ndsv5_targets_command)
{
	int retval = ERROR_OK;
	struct target *target = all_targets;
	command_print(CMD, "    TargetName         Type       Endian TapName            State         Coreid  ");
	command_print(CMD, "--  ------------------ ---------- ------ ------------------ ------------- --------");
	while (target) {
		const char *state;
		char marker = ' ';

		if (target->tap->enabled)
			state = target_state_name(target);
		else
			state = "tap-disabled";

		if (CMD_CTX->current_target == target)
			marker = '*';

		/* keep columns lined up to match the headers above */
		command_print(CMD,
				"%2d%c %-18s %-10s %-6s %-18s %-13s %4d",
				target->target_number,
				marker,
				target_name(target),
				target_type_name(target),
				jim_nvp_value2name_simple(nvp_target_endian,
					target->endianness)->name,
				target->tap->dotted_name,
				state,
				target->coreid);
		target = target->next;
	}

	return retval;
}

extern const struct command_registration riscv_exec_command_handlers[];
extern const struct command_registration nds32_exec_command_handlers[];
extern const struct command_registration semihosting_common_handlers[];
static const struct command_registration ndsv5_exec_command_handlers[] = {
	{
		.chain = nds32_exec_command_handlers,
	},
	{
		.name = "test",
		.handler = &ndsv5_handle_playground,
		.mode = COMMAND_ANY,
		.help = "playground command",
		.usage = "nds test <PARAM>",
	},
	{
		.name = "l2c",
		.handler = &ndsv5_handle_l2c_command,
		.mode = COMMAND_EXEC,
		.help = "['dump']",
		.usage = "l2 control",
	},
	{
		.name = "count_to_check_dm",
		.handler = &nds32_handle_count_to_check_dm_command,
		.mode = COMMAND_ANY,
		.help = "set retry times(counts or seconds) as checking $DMI status",
		.usage = "nds count_to_check_dm count_of_checking",
	},
	{
		.name = "dmi_busy_delay_count",
		.handler = riscv_set_dmi_busy_delay_count,
		.mode = COMMAND_ANY,
		.help = "Set dmi_busy_delay_count",
		.usage = "nds dmi_busy_delay_count [count]",
	},
	{
		.name = "stepie",
		.handler = ndsv5_handle_stepie_handlers,
		.mode = COMMAND_ANY,
		.help = "Set stepie feature",
		.usage = "nds stepie [on/off]",
	},
	{
		.name = "script_status",
		.jim_handler = ndsv5_script_status,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "reg_number",
		.jim_handler = ndsv5_reg_number,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "reg_data",
		.jim_handler = ndsv5_reg_data,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "dmi_addr",
		.jim_handler = ndsv5_dmi_addr,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "dmi_data",
		.jim_handler = ndsv5_dmi_data,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "target_xlen",
		.jim_handler = ndsv5_target_xlen,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "target_count",
		.jim_handler = ndsv5_target_count,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "target_info",
		.jim_handler = ndsv5_target_info,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "adapter_khz",
		.jim_handler = ndsv5_adapter_khz,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "jtag_tap_count",
		.jim_handler = ndsv5_jtag_tap_count,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "jtag_tap_name",
		.jim_handler = ndsv5_jtag_cur_tap_name,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "get_dmi_delay",
		.jim_handler = ndsv5_get_dmi_delay,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "smp_target_count",
		.handler = ndsv5_get_smp_target_count,
		.mode = COMMAND_EXEC,
		.help = "Retrun SMP target count",
		.usage = "nds smp_target_count",
	},
	{
		.name = "itlb",
		.handler = handle_ndsv5_itlb_command,
		.mode = COMMAND_EXEC,
		.usage = " ",
		.help = "tlb control",
	},
	{
		.name = "dtlb",
		.handler = handle_ndsv5_dtlb_command,
		.mode = COMMAND_EXEC,
		.usage = " ",
		.help = "tlb control",
	},
	{
		.name = "stlb",
		.handler = handle_ndsv5_stlb_command,
		.mode = COMMAND_EXEC,
		.usage = " ",
		.help = "tlb control",
	},
	{
		.name = "targets",
		.handler = handle_ndsv5_targets_command,
		.mode = COMMAND_EXEC,
		.usage = " ",
		.help = "[targets]",
	},
	{
		.chain = riscv_exec_command_handlers,
	},
	{
		.chain = semihosting_common_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration ndsv5_command_handlers[] = {
	{
		.name = "nds",
		.mode = COMMAND_ANY,
		.help = "Andes-V5 command group",
		.usage = "",
		.chain = ndsv5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int ndsv5_target_create(struct target *target, Jim_Interp *interp)
{
	NDS_INFO("%s", __func__);
	struct nds32_v5 *pnds32_v5 = NULL;


	if (target->smp && target->rtos == 0x0) {
		/* Find SMP group head's struct nds32_v5 */
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			if (t->rtos) {
				pnds32_v5 = target_to_nds32_v5(t);
				break;
			}
		}
	} else {
		/* allocate struct only for SMP head and AMP */
		pnds32_v5 = calloc(1, sizeof(struct nds32_v5));
	}

	ndsv5_init_arch_info(target, pnds32_v5);
	ndsv5_do_once_time(target);
	/*
	nds32_v3_common_register_callback(&nds32_v3_common_callback);
	nds32_v3_target_create_common(target, &(nds32_v3->nds32));
	*/

	return ERROR_OK;
}

enum target_state nds_print_cur_state;
static const char * const p_target_state[] = {
	"unknown",
	"running",
	"halted",
	"reset",
	"debug-running",
};

static const char * const target_debug_reason[] = {
	"debug-request", /* 0 */
	"breakpoint",
	"watchpoint",
	"watchpoint-and-breakpoint",
	"single-step",
	"target-not-halted", /* 5 */
	"program-exit",
	"exc_catch",
	"undefined",
	"buf-full",          /* 9  */
	"hit-user-watch",    /* 10 */
	"hit-exceptions",    /* 11 */
};

int ndsv5_handle_poll(struct target *target)
{
	if (nds_print_cur_state != target->state) {
		if (target->state == TARGET_HALTED)
			NDS_INFO("target->state = %s, target->debug_reason = %s",
				p_target_state[target->state], target_debug_reason[target->debug_reason]);
		else if (target->state <= TARGET_DEBUG_RUNNING)
			NDS_INFO("target->state = %s", p_target_state[target->state]);
				nds_print_cur_state = target->state;
	}
	/*
	if (target->state == TARGET_DEBUG_RUNNING) {
	} else if (target->state == TARGET_RUNNING) {
	} else if (target->state == TARGET_HALTED) {
	}
	*/

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if ((nds32->gdb_run_mode == RUN_MODE_PROFILE) &&
		(nds32->gdb_run_mode_acting == true)) {
		if (target->state == TARGET_HALTED)
			ndsv5_profile_post(target);
		else
			ndsv5_profile_state(target);
	}

	return ERROR_OK;
}

int ndsv5_handle_halt(struct target *target)
{
	if (target->state <= TARGET_DEBUG_RUNNING)
		NDS_INFO("target->state = %s", p_target_state[target->state]);

	if (target->state != TARGET_HALTED) {
		NDS_INFO("NOT TARGET_HALTED !!");
		return ERROR_FAIL;
	} else {
		if (target->debug_reason <= DBG_REASON_UNDEFINED)
			NDS_INFO("target->debug_reason = %s",
				target_debug_reason[target->debug_reason]);
	}

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if ((nds32->gdb_run_mode == RUN_MODE_PROFILE) &&
		(nds32->gdb_run_mode_acting == true)) {
		ndsv5_profile_post(target);
	}

	return ERROR_OK;
}

int ndsv5_handle_resume(struct target *target)
{
	if (target->state != TARGET_RUNNING) {
		NDS_INFO("NOT TARGET_RUNNING, target->state = %d", target->state);
		return ERROR_FAIL;
	}
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if (nds32->gdb_run_mode == RUN_MODE_PROFILE) {
		if (ndsv5_profile_init(target) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

#define MAX_SCRIPT_LENGTH 2100
int ndsv5_script_do_custom_reset(struct target *target, const char *script)
{
	char line_buffer[MAX_SCRIPT_LENGTH];
	struct command_context *cmd_ctx = global_cmd_ctx;
	uint32_t skip_execute_cmd = 0;
	int result;
	char *curr_str, *compare_str;
	char *cur_target_name;
	char tmp_buffer[MAX_SCRIPT_LENGTH];
	compare_str = "set_current_target";

	FILE *script_fd;
	script_fd = fopen(script, "r");
	if (script_fd == NULL)
		return ERROR_FAIL;

	nds_skip_dmi = 1;
	fseek(script_fd, 0, SEEK_SET);
	while (fgets(line_buffer, MAX_SCRIPT_LENGTH, script_fd) != NULL) {
		LOG_DEBUG("script: %s", line_buffer);
		if ((line_buffer[0] == '#') || (line_buffer[0] == '\r'))
			continue;

		curr_str = strstr(line_buffer, compare_str);
		if (curr_str != NULL) {
			result = sscanf(curr_str + strlen(compare_str), " %s", &tmp_buffer[0]);
			cur_target_name = (char *)&tmp_buffer[0];
			if (result == 1) {
				if (strcmp(target_name(target), cur_target_name) == 0)
					skip_execute_cmd = 0;
				else
					skip_execute_cmd = 1;

				LOG_DEBUG("set_current_target: %s, %s, skip_exe: %d",
						cur_target_name,
						target_name(target),
						skip_execute_cmd);
			}
		}
		if (skip_execute_cmd == 0) {
			NDS_INFO("cmd: %s", line_buffer);
			command_run_line(cmd_ctx, line_buffer);
		}
		LOG_DEBUG("script: DONE!");
	}
	nds_skip_dmi = 0;
	fclose(script_fd);
	return ERROR_OK;
}

int ndsv5_script_dmi_read(uint16_t address, uint64_t *dmi_read_data)
{
	char line_buffer[512];
	struct command_context *cmd_ctx = global_cmd_ctx;
	FILE *script_fd = nds_script_dmi_read;

	if (script_fd == NULL)
		return ERROR_FAIL;

	ndsv5_cur_dmi_addr = address;
	ndsv5_cur_script_status = 0;
	nds_skip_dmi = 1;
	fseek(script_fd, 0, SEEK_SET);
	NDS_INFO("ndsv5_script_dmi_read");
	while (fgets(line_buffer, 512, script_fd) != NULL)
		command_run_line(cmd_ctx, line_buffer);

	nds_skip_dmi = 0;
	if (ndsv5_cur_script_status != 0) {
		NDS_INFO("dmiread-0x%x ERROR", ndsv5_cur_dmi_addr);
		return ERROR_FAIL;
	}
	*dmi_read_data = ndsv5_cur_dmi_data;
	return ERROR_OK;
}

int ndsv5_script_dmi_write(uint16_t address, uint64_t value)
{
	char line_buffer[512];
	struct command_context *cmd_ctx = global_cmd_ctx;
	FILE *script_fd = nds_script_dmi_write;

	if (script_fd == NULL)
		return ERROR_FAIL;

	ndsv5_cur_dmi_addr = address;
	ndsv5_cur_dmi_data = value;
	ndsv5_cur_script_status = 0;
	nds_skip_dmi = 1;
	fseek(script_fd, 0, SEEK_SET);
	NDS_INFO("address-0x%x = 0x%" PRIx64, address, value);
	while (fgets(line_buffer, 512, script_fd) != NULL)
		command_run_line(cmd_ctx, line_buffer);

	nds_skip_dmi = 0;
	if (ndsv5_cur_script_status != 0) {
		NDS_INFO("dmiwrite-0x%x ERROR", ndsv5_cur_dmi_addr);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int ndsv5_script_reg_read(uint64_t *value, uint32_t number)
{
	char line_buffer[512];
	struct command_context *cmd_ctx = global_cmd_ctx;
	FILE *script_fd = nds_script_reg_get;

	if (script_fd == NULL)
		return ERROR_FAIL;

	ndsv5_cur_reg_number = number;
	ndsv5_cur_reg_value = 0;
	ndsv5_cur_script_status = 0;
	nds_skip_dmi = 1;
	fseek(script_fd, 0, SEEK_SET);
	while (fgets(line_buffer, 512, script_fd) != NULL)
		command_run_line(cmd_ctx, line_buffer);

	nds_skip_dmi = 0;
	if (ndsv5_cur_script_status != 0) {
		NDS_INFO("reg-0x%x ERROR", ndsv5_cur_reg_number);
		return ERROR_FAIL;
	}
	NDS_INFO("reg-0x%x = 0x%" PRIx64, ndsv5_cur_reg_number, ndsv5_cur_reg_value);
	*value = ndsv5_cur_reg_value;
	return ERROR_OK;
}

int ndsv5_script_reg_write(unsigned number, uint64_t value)
{
	char line_buffer[512];
	struct command_context *cmd_ctx = global_cmd_ctx;
	FILE *script_fd = nds_script_reg_set;

	if (script_fd == NULL)
		return ERROR_FAIL;


	ndsv5_cur_reg_number = number;
	ndsv5_cur_reg_value = value;
	ndsv5_cur_script_status = 0;
	nds_skip_dmi = 1;
	fseek(script_fd, 0, SEEK_SET);
	while (fgets(line_buffer, 512, script_fd) != NULL)
		command_run_line(cmd_ctx, line_buffer);

	nds_skip_dmi = 0;
	if (ndsv5_cur_script_status != 0) {
		NDS_INFO("reg-0x%x ERROR", ndsv5_cur_reg_number);
		return ERROR_FAIL;
	}
	NDS_INFO("reg-0x%x = 0x%" PRIx64, ndsv5_cur_reg_number, ndsv5_cur_reg_value);
	return ERROR_OK;
}

struct reg_arch_type *p_riscv_reg_arch_type;
static int ndsv5_register_get(struct reg *reg)
{
	int result;

	if (p_riscv_reg_arch_type == NULL)
		return ERROR_FAIL;

	result = p_riscv_reg_arch_type->get(reg);
	/*
	if (result != ERROR_OK) {
		//buf_set_u64(reg->value, 0, 8, 0xFFFFFFFF);
		buf_set_u32(reg->value, 0, 32, 0xFFFFFFFF);
		return ERROR_OK;
	}
	*/
	return result;
}

static int ndsv5_register_set(struct reg *reg, uint8_t *buf)
{
	int result;

	if (p_riscv_reg_arch_type == NULL)
		return ERROR_FAIL;

	result = p_riscv_reg_arch_type->set(reg, buf);
	return result;
}

struct reg_arch_type ndsv5_reg_arch_type = {
	.get = ndsv5_register_get,
	.set = ndsv5_register_set
};

char gNDSVectorRegBuf[32][1024/8];
static int ndsv5_register_vector_get(struct reg *reg)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	LOG_DEBUG("Get vectore reg[%s] on [%s] hart %d", reg->name, target->tap->dotted_name, target->coreid);

	char *pVectorData = (char *)&gNDSVectorRegBuf[reg->number - GDB_REGNO_V0][0];
	for (unsigned i = 0; i < (reg->size/8); i++)
		*pVectorData++ = 0xFF;

	pVectorData = (char *)&gNDSVectorRegBuf[reg->number - GDB_REGNO_V0][0];
	char *pRegValue = (char *)reg->value;
	LOG_DEBUG("reg->number=%d, reg->size=%d ", reg->number, reg->size);

	ndsv5_get_vector_register(target, reg->number, pVectorData);
	for (unsigned i = 0; i < (reg->size/8); i++) {
		LOG_DEBUG("*pVectorData = 0x%x ", *pVectorData);
		*pRegValue++ = *pVectorData++;
	}
	return ERROR_OK;
}

static int ndsv5_register_vector_set(struct reg *reg, uint8_t *buf)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	LOG_DEBUG("Set vectore reg[%s] on [%s] hart %d", reg->name, target->tap->dotted_name, target->coreid);
	char *pVectorData = (char *)&gNDSVectorRegBuf[reg->number - GDB_REGNO_V0][0];
	char *pRegValue = (char *)reg->value;
	char *pSrc = (char *)buf;
	LOG_DEBUG("reg->number=%d, reg->size=%d ", reg->number, reg->size);

	for (unsigned i = 0; i < (reg->size/8); i++) {
		LOG_DEBUG("*pSrc = 0x%x ", *pSrc);
		*pRegValue++ = *pSrc++;
	}

	pRegValue = (char *)reg->value;
	for (unsigned i = 0; i < (reg->size/8); i++) {
		LOG_DEBUG("*pRegValue = 0x%x ", *pRegValue);
		*pVectorData++ = *pRegValue++;
	}

	pVectorData = (char *)&gNDSVectorRegBuf[reg->number - GDB_REGNO_V0][0];
	ndsv5_set_vector_register(target, reg->number, pVectorData);

	return ERROR_OK;
}

struct reg_arch_type ndsv5_vector_reg_access_type = {
	.get = ndsv5_register_vector_get,
	.set = ndsv5_register_vector_set
};

char *ndsv5_get_CSR_name(struct target *target, uint32_t csr_id)
{
	if (csr_id > 4095) {
		NDS_INFO("get wrong csr_id: %d", csr_id);
		return NULL;
	}
	char *nds32_csr_name = (char *)target->reg_cache->reg_list[csr_id + GDB_REGNO_CSR0].name;
	if (nds32_csr_name == NULL) {
		NDS_INFO("get null csr_name: %d", csr_id);
		return NULL;
	}
	NDS_INFO("%s", nds32_csr_name);
	return (char *)nds32_csr_name;
}

/* ABI name */
char *reg_abi_name[] = {
	"zero",  "ra",   "sp",   "gp",  "tp",  "t0",   "t1",   "t2",
	"fp",  "s1",   "a0",   "a1",  "a2",  "a3",   "a4",   "a5",
	"a6",  "a7",   "s2",   "s3",  "s4",  "s5",   "s6",   "s7",
	"s8",  "s9",  "s10",  "s11",  "t3",  "t4",   "t5",   "t6",
	"pc",
	"ft0", "ft1",  "ft2",  "ft3", "ft4", "ft5",  "ft6",  "ft7",
	"fs0", "fs1",  "fa0",  "fa1", "fa2", "fa3",  "fa4",  "fa5",
	"fa6", "fa7",  "fs2",  "fs3", "fs4", "fs5",  "fs6",  "fs7",
	"fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11",
};

/* Architecture name */
char *reg_arch_name[] = {
	"x0",  "x1",  "x2",  "x3",  "x4",  "x5",  "x6",  "x7",
	"x8",  "x9", "x10", "x11", "x12", "x13", "x14", "x15",
	"x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
	"x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
	"pc",
	"f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
	"f8",  "f9", "f10", "f11", "f12", "f13", "f14", "f15",
	"f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
	"f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};

/* for gdb : default is abi name zero->x0 */
char **gpr_and_fpu_name = reg_abi_name;

void addtional_pm_counters(struct target *target, int start)
{
	if (start < 5)
		start = 4 - start;

	NDS_INFO("%d additional counters", start);
	for (int i = start; i <= 28; i++) {
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHPMCOUNTER3 + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHPMCOUNTER3H + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHPMEVENT3 + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_HPMCOUNTER3 + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_HPMCOUNTER3H + i].exist = false;
	}
}

static int ndsv5_init_option_reg(struct target *target)
{
	uint64_t reg_misa_value = 0, reg_mmsc_cfg_value = 0, reg_mmsc_cfg2_value = 0, reg_mmsc_cfg3_value = 0;
	uint64_t reg_micm_cfg_value = 0, reg_mdcm_cfg_value = 0;
	uint64_t reg_marchid_value = 0, reg_mimpid_value = 0;
	struct reg *p_cur_reg;
	char *reg_name;
	uint32_t i;

	if (riscv_xlen(target) == 64) {
		LOG_DEBUG("disable RV32-only registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCYCLEH].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MINSTRETH].exist = false;
		for (i = 0; i <= 28; i++)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHPMCOUNTER3H + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG1].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG3].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG5].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG7].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG9].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG11].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG13].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMPCFG15].exist = false;

		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMACFG1].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PMACFG3].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG2].exist = false;

		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSECCFGH].exist = false;
	}

	reg_name = ndsv5_get_CSR_name(target, CSR_MISA);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_misa_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);
	ndsv5_reg_misa_value = reg_misa_value;
	NDS_INFO("misa = 0x%" PRIx64, reg_misa_value);

	/* only rv32i exist */
	if (riscv_xlen(target) != 32 || ((reg_misa_value & 0x100) == 0)) {
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_CYCLEH].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_INSTRETH].exist = false;
		for (i = 0; i <= 28; i++)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_HPMCOUNTER3H + i].exist = false;
	}

	/* (misa[3]==1) | (misa[5]==1), enable FPU register */
	if ((reg_misa_value & 0x28) == 0) {
		NDS_INFO("disable FPU registers");
		for (i = GDB_REGNO_FPR0; i <= GDB_REGNO_FPR31; i++)
			target->reg_cache->reg_list[i].exist = false;

		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_FFLAGS].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_FRM].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_FCSR].exist = false;
	}

	/* (misa[18]==1) */
	if ((reg_misa_value & 0x40000) == 0) {
		LOG_DEBUG("disable CSR_SCOUNTEREN, CSR_SDCAUSE, CSR_SSTATUS, CSR_SIE, CSR_SIP, CSR_STVEC, "
			  "CSR_SEPC, CSR_SCAUSE, CSR_STVAL, CSR_SSCRATCH, CSR_SATP, CSR_MSLIDELEG, "
			  "CSR_SLIE, CSR_SLIP, CSR_SEDELEG, CSR_SIDELEG, CSR_SCOUNTOVF registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTEREN].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SDCAUSE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATUS].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SIE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SIP].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_STVEC].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SEPC].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCAUSE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_STVAL].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSCRATCH].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSLIDELEG].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SLIE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SLIP].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SEDELEG].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SIDELEG].exist = false;

		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTOVF].exist = false;
	}

	/* (misa[18]==1) | (misa[13]==1), enable CSR_MEDELEG, CSR_MIDELEG registers */
	if ((reg_misa_value & 0x42000) == 0) {
		NDS_INFO("disable CSR_MEDELEG, CSR_MIDELEG registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MEDELEG].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MIDELEG].exist = false;
	}

	/* (misa[20]==1) */
	if ((reg_misa_value & 0x100000) == 0) {
		NDS_INFO("disable CSR_MCOUNTEREN,CSR_CYCLE(H),CSR_INSTRET(H),CSR_HPMCOUNTER3(H) registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTEREN].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_CYCLE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_INSTRET].exist = false;
		for (i = 0; i <= 28; i++)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_HPMCOUNTER3 + i].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_CYCLEH].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_INSTRETH].exist = false;
		for (i = 0; i <= 28; i++)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_HPMCOUNTER3H + i].exist = false;
	}
	/* (misa[13]==1) */
	if ((reg_misa_value & 0x2000) == 0) {
		NDS_INFO("disable CSR_USTATUS,CSR_UIE, CSR_UIP, CSR_UTVEC, CSR_UEPC, "
			 "CSR_UCAUSE, CSR_UTVAL, CSR_USCRATCH, CSR_UDCAUSE, CSR_SIDELEG, CSR_SEDELEG registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_USTATUS].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UIE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UIP].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UTVEC].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UEPC].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UCAUSE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UTVAL].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_USCRATCH].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UDCAUSE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SIDELEG].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SEDELEG].exist = false;
	}

	/* ax45/nx45 and version < 0x100, the bitmap supports BF16,
	 * but accessing mclk_ctl will trigger an exception, set CSR_MCLK_CTL false */
	reg_name = ndsv5_get_CSR_name(target, CSR_MARCHID);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_marchid_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

	reg_name = ndsv5_get_CSR_name(target, CSR_MIMPID);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_mimpid_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

	if (((reg_marchid_value & 0xff) == 0x45) && (reg_mimpid_value < 0x100)) {
		NDS_INFO("disable CSR_MCLK_CTL register");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCLK_CTL].exist = false;
	}

	reg_name = ndsv5_get_CSR_name(target, CSR_MMSC_CFG);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_mmsc_cfg_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

	/* RV32 */
	/* MMSC_CFG2 : misa.MXL == 1 & mmsc_cfg[31] == 1
	 * if rv64 CSR_MMSC_CFG2 already not exist and use mmsc_cfg as CSR_MCRASH_STATESAVE,
	 * CSR_MSTATUS_CRASHSAVE and CSR_MCLK_CTL existence condition */
	if ((reg_misa_value & 0x40000000)) {
		if ((reg_mmsc_cfg_value & 0x80000000) == 0) {
			NDS_INFO("disable CSR_MMSC_CFG2");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG2].exist = false;

			/* if RV32 mmsc_cfg2.CRASHSAVE == 1 */
			NDS_INFO("disable CSR_MCRASH_STATESAVE, CSR_MSTATUS_CRASHSAVE registers");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCRASH_STATESAVE].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATUS_CRASHSAVE].exist = false;

			/* if mmsc_cfg.EXCSLVL > 0 | mmsc_cfg2.CRASHSAVE == 1*/
			if ((reg_mmsc_cfg_value & 0x300000) == 0) {
				NDS_INFO("disable CSR_MSAVEEPC1 CSR_MSAVECAUSE1 CSR_MSAVEDCAUSE1 registers");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEEPC1].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVECAUSE1].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEDCAUSE1].exist = false;
			}

			/* misa.V[21] == 1 && if RV32 mmsc_cfg2.veccfg == 1 */
			NDS_INFO("disable CSR_MVEC_CFG register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MVEC_CFG].exist = false;

			/* misa.V[21] == 1 | if RV32 mmsc_cfg2.BF16CVT == 1 */
			if ((reg_misa_value & 0x200000) == 0) {
				NDS_INFO("disable CSR_MCLK_CTL register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCLK_CTL].exist = false;
			}

			/* if RV32 mmsc_cfg2.CCACHEMP_CFG == 1 */
			NDS_INFO("disable CSR_MCCACHE_CTL_BASE register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCACHE_CTL_BASE].exist = false;

			/* if RV32 mmsc_cfg2.RVARCH == 1 */
			NDS_INFO("disable CSR_MRVARCH_CFG/CSR_MRVARCH_CFG2 register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG2].exist = false;


			/* mmsc_cfg2.XCSR == 1 */
			NDS_INFO("disable CSR_MNDSX_RDATA / CSR_MNDSX_WDATA");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_RDATA].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_WDATA].exist = false;


			/* if RV32 mmsc_cfg2.ALT_FP_FMT == 1 */
			NDS_INFO("disable CSR_UMISC_CTL");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UMISC_CTL].exist = false;

			/* mmsc_cfg2.MSC_EXT3 == 1 */
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG3].exist = false;
		} else {
			/* mmsc_cfg2 exist */
			reg_name = ndsv5_get_CSR_name(target, CSR_MMSC_CFG2);
			p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
			p_cur_reg->type->get(p_cur_reg);
			reg_mmsc_cfg2_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

			/* if RV32 mmsc_cfg2.CRASHSAVE == 1 */
			if ((reg_mmsc_cfg2_value & 0x8) == 0) {
				NDS_INFO("disable CSR_MCRASH_STATESAVE, CSR_MSTATUS_CRASHSAVE registers");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCRASH_STATESAVE].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATUS_CRASHSAVE].exist = false;
			}

			/* if mmsc_cfg.EXCSLVL > 0 | mmsc_cfg2.CRASHSAVE == 1*/
			if (((reg_mmsc_cfg_value & 0x300000) == 0) && ((reg_mmsc_cfg2_value & 0x8) == 0)) {
				NDS_INFO("disable CSR_MSAVEEPC1 CSR_MSAVECAUSE1 CSR_MSAVEDCAUSE1 registers");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEEPC1].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVECAUSE1].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEDCAUSE1].exist = false;
			}

			/* misa.V[21] == 1 | if RV32 mmsc_cfg2.BF16CVT == 1 */
			if (((reg_misa_value & 0x200000) == 0) && ((reg_mmsc_cfg2_value & 0x1) == 0)) {
				NDS_INFO("disable CSR_MCLK_CTL register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCLK_CTL].exist = false;
			}

			/* misa.V[21] == 1 && if RV32 mmsc_cfg2.veccfg == 1 */
			if ((reg_misa_value & 0x200000) == 0 || ((reg_mmsc_cfg2_value & 0x10) == 0)) {
				NDS_INFO("disable CSR_MVEC_CFG register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MVEC_CFG].exist = false;
			}

			/* if RV32 mmsc_cfg2.CCACHEMP_CFG == 1 */
			/* AndeStar_V5_SPA_UM164_V1.5.43-20220726 */
			if ((reg_mmsc_cfg2_value & 0x2000) == 0) {
				NDS_INFO("disable CSR_MCCACHE_CTL_BASE register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCACHE_CTL_BASE].exist = false;
			}

			/* if RV32 mmsc_cfg2.RVARCH == 1 */
			if ((reg_mmsc_cfg2_value & 0x100000) == 0) {
				NDS_INFO("disable CSR_MRVARCH_CFG/CSR_MRVARCH_CFG2 register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG2].exist = false;
			}

			/* if RV32 mmsc_cfg2.RVARCH2 == 1 */
			if ((reg_mmsc_cfg2_value & 0x10000000) == 0) {
				NDS_INFO("disable CSR_MRVARCH_CFG2 register");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG2].exist = false;
			}

			/* if RV32 mmsc_cfg2.XCSR[24] == 1 */
			if ((reg_mmsc_cfg2_value & 0x1000000) == 0) {
				NDS_INFO("disable CSR_MNDSX_RDATA / CSR_MNDSX_WDATA");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_RDATA].exist = false;
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_WDATA].exist = false;
			}

			/* if RV32 mmsc_cfg2.ALT_FP_FMT[25] == 1 */
			if ((reg_mmsc_cfg2_value & 0x2000000) == 0) {
				NDS_INFO("disable CSR_UMISC_CTL");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UMISC_CTL].exist = false;
			}

			/* if RV32 mmsc_cfg2.MSC_EXT3[31] */
			if ((reg_mmsc_cfg_value & 0x80000000) == 0) {
				NDS_INFO("disable CSR_MMSC_CFG3");
				target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG3].exist = false;
			}
		}
	}

	/* RV64 */
	if (riscv_xlen(target) == 64) {
		/* if RV64 mmsc_cfg.CRASHSAVE == 1 */
		if ((reg_mmsc_cfg_value & 0x800000000) == 0) {
			NDS_INFO("disable CSR_MCRASH_STATESAVE, CSR_MSTATUS_CRASHSAVE registers");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCRASH_STATESAVE].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATUS_CRASHSAVE].exist = false;
		}

		/* if mmsc_cfg.EXCSLVL > 0 | mmsc_cfg2.CRASHSAVE == 1*/
		if ((reg_mmsc_cfg_value & 0x300000) == 0 && ((reg_mmsc_cfg_value & 0x800000000) == 0)) {
			NDS_INFO("disable CSR_MSAVEEPC1 CSR_MSAVECAUSE1 CSR_MSAVEDCAUSE1 registers");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEEPC1].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVECAUSE1].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEDCAUSE1].exist = false;
		}

		/* misa.V[21] == 1 | if RV64 mmsc_cfg.BF16CVT == 1 */
		if ((reg_misa_value & 0x200000) == 0 && ((reg_mmsc_cfg_value & 0x100000000) == 0)) {
			NDS_INFO("disable CSR_MCLK_CTL register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCLK_CTL].exist = false;
		}

		/* misa.V[21] == 1 && if RV64 mmsc_cfg.veccfg == 1 */
		if ((reg_misa_value & 0x200000) == 0 || ((reg_mmsc_cfg_value & 0x1000000000) == 0)) {
			NDS_INFO("disable CSR_MVEC_CFG register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MVEC_CFG].exist = false;
		}


		/* if RV64 mmsc_cfg.CCACHEMP_CFG == 1 */
		/* AndeStar_V5_SPA_UM164_V1.5.43-20220726 */
		if ((reg_mmsc_cfg_value & 0x200000000000) == 0) {
			NDS_INFO("disable CSR_MCCACHE_CTL_BASE register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCACHE_CTL_BASE].exist = false;
		}

		/* if RV64 mmsc_cfg.RVARCH == 1 */
		if ((reg_mmsc_cfg_value & 0x10000000000000) == 0) {
			NDS_INFO("disable CSR_MRVARCH_CFG register");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG].exist = false;
		}

		/* if RV64 mmsc_cfg.XCSR[56] == 1 */
		if ((reg_mmsc_cfg_value & 0x100000000000000) == 0) {
			NDS_INFO("disable CSR_MNDSX_RDATA / CSR_MNDSX_WDATA");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_RDATA].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNDSX_WDATA].exist = false;
		}

		/* if RV64 mmsc_cfg.MSC_EXT3[63] == 1 */
		if ((reg_mmsc_cfg_value & 0x8000000000000000) == 0) {
			NDS_INFO("disable CSR_MMSC_CFG3");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG3].exist = false;
		}

		/* if RV64 mmsc_cfg.ALT_FP_FMT[57] == 1 */
		if ((reg_mmsc_cfg_value & 0x200000000000000) == 0) {
			NDS_INFO("disable CSR_UMISC_CTL");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UMISC_CTL].exist = false;
		}
	}

	/* MMSC_CFG3 */
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMSC_CFG3].exist) {
		reg_name = ndsv5_get_CSR_name(target, CSR_MMSC_CFG3);
		p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
		p_cur_reg->type->get(p_cur_reg);
		reg_mmsc_cfg3_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

		if ((reg_mmsc_cfg3_value & 0x10) == 0) {
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHVM_CFG].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHVMB].exist = false;
		}
	} else {
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHVM_CFG].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHVMB].exist = false;
	}


	/* mmsc_cfg.PMNDS == 1 */
	if ((reg_mmsc_cfg_value & 0x8000) == 0) {
		NDS_INFO("disable CSR_MCOUNTERINTEN, CSR_MCOUNTEROVF registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTERINTEN].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTEROVF].exist = false;
	}

	/* mmsc_cfg.PMNDS == 1 & misa[20] == 1 */
	if (((reg_mmsc_cfg_value & 0x8000) == 0) || ((reg_misa_value & 0x100000) == 0)) {
		NDS_INFO("disable CSR_MCOUNTERWEN, CSR_MCOUNTERMASK_M, CSR_MCOUNTERMASK_U registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTERWEN].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTERMASK_M].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTERMASK_U].exist = false;
	}

	/* mmsc_cfg.PMNDS == 1 & misa[18]==1 */
	if (((reg_mmsc_cfg_value & 0x8000) == 0) || ((reg_misa_value & 0x40000) == 0)) {
		NDS_INFO("disable CSR_MCOUNTERMASK_S, CSR_SCOUNTERMASK_M/S/U, CSR_SCOUNTERINTEN,"
			 "CSR_SCOUNTEROVF CSR_SHPMEVENT3~6, CSR_SCOUNTINHIBIT registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCOUNTERMASK_S].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTERMASK_M].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTERMASK_S].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTERMASK_U].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTERINTEN].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTEROVF].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SHPMEVENT3].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SHPMEVENT4].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SHPMEVENT5].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SHPMEVENT6].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTINHIBIT].exist = false;
	}

	/* misa[18] == 1 & mmsc_cfg.ACE == 1, enable CSR_SMISC_CTL register */
	if (((reg_misa_value & 0x40000) == 0) || ((reg_mmsc_cfg_value & 0x0040) == 0)) {
		NDS_INFO("disable CSR_SMISC_CTL register");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SMISC_CTL].exist = false;
	}

	/* mmsc_cfg.ECC == 1 */
	if ((reg_mmsc_cfg_value & 0x0001) == 0) {
		NDS_INFO("disable CSR_MECC_CODE registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MECC_CODE].exist = false;
	}

	/* mrvarch_cfg.sscofpmf==1 & misa[18]==1 */
	if (!target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MRVARCH_CFG].exist || 
	    !riscv_supports_extension(target, 'U')) {
		NDS_INFO("disable CSR_SCOUNTOVF");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTOVF].exist = false;

		NDS_INFO("disable CSR_MSTATEEN0~3 / CSR_MSTATEEN0H~3H / CSR_SSTATEEN0~3");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN0].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN1].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN2].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN3].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN0H].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN1H].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN2H].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN3H].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN0].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN1].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN2].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN3].exist = false;
	} else {
		reg_name = ndsv5_get_CSR_name(target, CSR_MRVARCH_CFG);
		p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
		p_cur_reg->type->get(p_cur_reg);
		uint64_t reg_mrvarch_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);

		if ((reg_mrvarch_value & 0x80) == 0) {
			NDS_INFO("disable CSR_MRVARCH_CFG");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCOUNTOVF].exist = false;
		}

		/*  mrvarch_cfg.Smstateen[6] == 1 */
		if ((reg_mrvarch_value & 0x40) == 0) {
			NDS_INFO("disable CSR_MSTATEEN0~3 / CSR_MSTATEEN0H~3H / CSR_SSTATEEN0~3");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN0].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN1].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN2].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN3].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN0H].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN1H].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN2H].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSTATEEN3H].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN0].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN1].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN2].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SSTATEEN3].exist = false;
		}
	}


	/* check pmpcfg0-15 exist */
	int retval;
	for (i = (GDB_REGNO_CSR0 + CSR_PMPCFG0); i <= (GDB_REGNO_CSR0 + CSR_PMPCFG15); i++) {
		/* 64bit skip CSR_PMPCFG1~15(odd) */
		if (riscv_xlen(target) == 64) {
			if (target->reg_cache->reg_list[i].exist == false) {
				NDS_INFO("%s register does not exist", target->reg_cache->reg_list[i].name);
				continue;
			}
		}
		/* if reg_list.exist is false, register_get_by_name function return null */
		if (target->reg_cache->reg_list[i].exist != false) {
			reg_name = ndsv5_get_CSR_name(target, (i - GDB_REGNO_CSR0));
			p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
			retval = p_cur_reg->type->get(p_cur_reg);
			if (retval != ERROR_OK) {
				NDS_INFO("disable %s register", target->reg_cache->reg_list[i].name);
				target->reg_cache->reg_list[i].exist = false;
			}
		}
	}

	/* check pmpaddr0-63 exist */
	for (i = (GDB_REGNO_CSR0 + CSR_PMPADDR0); i <= (GDB_REGNO_CSR0 + CSR_PMPADDR63); i++) {
		/* if reg_list.exist is false, register_get_by_name function return null */
		if (target->reg_cache->reg_list[i].exist != false) {
			reg_name = ndsv5_get_CSR_name(target, (i - GDB_REGNO_CSR0));
			p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
			retval = p_cur_reg->type->get(p_cur_reg);
			if (retval != ERROR_OK) {
				NDS_INFO("disable %s register", target->reg_cache->reg_list[i].name);
				target->reg_cache->reg_list[i].exist = false;
			}
		}
	}

	if (riscv_xlen(target) == 64) {
		/* mmsc_cfg.HSP == 1 || mmsc_cfg.HSPO == 1 */
		if ((reg_mmsc_cfg_value & 0x0020) == 0 && (reg_mmsc_cfg_value & 0x80000000000000) == 0) {
			NDS_INFO("disable CSR_MHSP_CTL, CSR_MSP_BOUND and CSR_MSP_BASE registers");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHSP_CTL].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSP_BOUND].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSP_BASE].exist = false;
		}
	} else {
		/* mmsc_cfg.HSP == 1 || mmsc_cfg2.HSPO == 1 */
		if ((reg_mmsc_cfg_value & 0x0020) == 0 && (reg_mmsc_cfg2_value & 0x800000) == 0) {
			NDS_INFO("disable CSR_MHSP_CTL, CSR_MSP_BOUND and CSR_MSP_BASE registers");
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MHSP_CTL].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSP_BOUND].exist = false;
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSP_BASE].exist = false;
		}
	}

	/* mmsc_cfg.PFT == 1 */
	if ((reg_mmsc_cfg_value & 0x0010) == 0) {
		NDS_INFO("disable CSR_MPFT_CTL registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MPFT_CTL].exist = false;
	}

	/* mmsc_cfg.ECD == 1 */
	if ((reg_mmsc_cfg_value & 0x0008) == 0) {
		NDS_INFO("disable CSR_UITB registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UITB].exist = false;
	}

	/* mmsc_cfg.AEC == 1 | .VPLIC == 1 | .ECD == 1 | .EV5 == 1 */
	if ((reg_mmsc_cfg_value & 0x3048) == 0) {
		NDS_INFO("disable CSR_MMISC_CTL registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MMISC_CTL].exist = false;
	}

	/* mmsc_cfg.EXCSLVL > 0 */
	if ((reg_mmsc_cfg_value & 0x300000) == 0) {
		NDS_INFO("disable CSR_MSAVESTATUS registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVESTATUS].exist = false;
	}

	/* mmsc_cfg.EXCSLVL > 1 */
	if (((reg_mmsc_cfg_value & 0x300000) >> 20) < 2) {
		NDS_INFO("disable CSR_MSAVEEPC2 CSR_MSAVECAUSE2 CSR_MSAVEDCAUSE2 registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEEPC2].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVECAUSE2].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSAVEDCAUSE2].exist = false;
	}

	/* mmsc_cfg.ESLEEP == 1 */
	if ((reg_mmsc_cfg_value & 0x1000000) == 0) {
		NDS_INFO("disable CSR_WFE CSR_SLEEPVALUE CSR_TXEVT registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_WFE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SLEEPVALUE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_TXEVT].exist = false;
	}

	/* mmsc_cfg.PPI == 1 */
	if ((reg_mmsc_cfg_value & 0x2000000) == 0) {
		NDS_INFO("disable CSR_MPPIB");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MPPIB].exist = false;
	}

	/* mmsc_cfg.FIO == 1 */
	if ((reg_mmsc_cfg_value & 0x4000000) == 0) {
		NDS_INFO("disable CSR_MFIOB");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MFIOB].exist = false;
	}

	/* mmsc_cfg.CLIC == 1 */
	if ((reg_mmsc_cfg_value & 0x8000000) == 0) {
		NDS_INFO("disable CSR_MTVT CSR_MNXTI CSR_MINTSTATUS CSR_MSCRATCHCSW CSR_MSCRATCHCSWL registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MTVT].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MNXTI].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MINTSTATUS].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSCRATCHCSW].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MSCRATCHCSWL].exist = false;
	}

	/* mmsc_cfg.EDSP == 1 */
	if ((reg_mmsc_cfg_value & 0x20000000) == 0) {
		NDS_INFO("disable CSR_UCODE register");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UCODE].exist = false;
	}

	/* mmsc_cfg.PPMA == 1 */
	if ((reg_mmsc_cfg_value & 0x40000000) == 0) {
		NDS_INFO("disable CSR_PMACFG0-3 CSR_PMAADDR0-15");
		for (i = (GDB_REGNO_CSR0 + CSR_PMACFG0); i <= (GDB_REGNO_CSR0 + CSR_PMACFG3); i++)
			target->reg_cache->reg_list[i].exist = false;
		for (i = (GDB_REGNO_CSR0 + CSR_PMAADDR0); i <= (GDB_REGNO_CSR0 + CSR_PMAADDR15); i++)
			target->reg_cache->reg_list[i].exist = false;
	}

	/* mmsc_cfg.ECLIC == 1 */
	if ((reg_mmsc_cfg_value & 0x10000000) == 0) {
		NDS_INFO("disable CSR_MIRQ_ENTRY registers");
		/* Bug-17785 comment9 :remove other csrs */
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MIRQ_ENTRY].exist = false;
		/*
		NDS_INFO("disable CSR_MIRQ_ENTRY CSR_MINTSEL_JAL CSR_PUSHMCAUSE "
			 "CSR_PUSHMEPC CSR_PUSHMXSTATUS registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MINTSEL_JAL].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PUSHMCAUSE].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PUSHMEPC].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_PUSHMXSTATUS].exist = false;
		*/
	}

	/* mmsc_cfg.ADDPMC ,check additional PM counters */
	addtional_pm_counters(target, ((reg_mmsc_cfg_value & 0xf80) >> 7));

	reg_name = ndsv5_get_CSR_name(target, CSR_MICM_CFG);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_micm_cfg_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);
	NDS_INFO("micm_cfg = 0x%" PRIx64, reg_micm_cfg_value);
	/* micm_cfg.ILMB != 0 (bit 14-12) */
	if ((reg_micm_cfg_value & 0x7000) == 0) {
		NDS_INFO("disable CSR_MILMB registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MILMB].exist = false;
	}

	reg_name = ndsv5_get_CSR_name(target, CSR_MDCM_CFG);
	p_cur_reg = register_get_by_name(target->reg_cache, reg_name, 1);
	p_cur_reg->type->get(p_cur_reg);
	reg_mdcm_cfg_value = buf_get_u64(p_cur_reg->value, 0, p_cur_reg->size);
	NDS_INFO("mdcm_cfg = 0x%" PRIx64, reg_mdcm_cfg_value);
	/* mdcm_cfg.DLMB != 0 (bit 14-12) */
	if ((reg_mdcm_cfg_value & 0x7000) == 0) {
		NDS_INFO("disable CSR_MDLMB registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MDLMB].exist = false;
	}

	/* micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6) */
	if (((reg_micm_cfg_value & 0x1c0) == 0) && ((reg_mdcm_cfg_value & 0x1c0) == 0)) {
		NDS_INFO("disable CSR_MCACHE_CTL register");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCACHE_CTL].exist = false;
		ndsv5_dis_cache_busmode = 0;
	}

	/* (micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0) & (mmsc_cfg.CCTLCSR == 1) */
	if ((((reg_micm_cfg_value & 0x1c0) == 0) && ((reg_mdcm_cfg_value & 0x1c0) == 0)) ||
	    ((reg_mmsc_cfg_value & 0x10000) == 0)) {
		NDS_INFO("disable CSR_MCCTLBEGINADDR,CSR_MCCTLCOMMAND,CSR_MCCTLDATA registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCTLBEGINADDR].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCTLCOMMAND].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCTLDATA].exist = false;
	}
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MILMB].exist == true) {
		struct reg *reg_milmb = ndsv5_get_reg_by_CSR(target, CSR_MILMB);
		if (reg_milmb != NULL) {
			uint64_t value_milmb = ndsv5_get_register_value(reg_milmb);
			if (value_milmb & 0x1) {
				ndsv5_ilm_bpa = value_milmb & ~0x3ff;
				ndsv5_ilm_ena = 1;
			} else {
				ndsv5_ilm_ena = 0;
			}
		}
	}
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MDLMB].exist == true) {
		struct reg *reg_mdlmb = ndsv5_get_reg_by_CSR(target, CSR_MDLMB);
		if (reg_mdlmb != NULL) {
			uint64_t value_mdlmb = ndsv5_get_register_value(reg_mdlmb);
			if (value_mdlmb & 0x1) {
				ndsv5_dlm_bpa = value_mdlmb & ~0x3ff;
				ndsv5_dlm_ena = 1;
			} else {
				ndsv5_dlm_ena = 0;
			}
		}
	}
	uint64_t lmsz;
	lmsz = (reg_micm_cfg_value & 0xf8000) >> 15;
	ndsv5_ilm_lmsz = ((1 << (lmsz - 1)) * 1024);
	lmsz = (reg_mdcm_cfg_value & 0xf8000) >> 15;
	ndsv5_dlm_lmsz = ((1 << (lmsz - 1)) * 1024);

	if ((reg_mmsc_cfg_value & 0x4000) == 0) {
		NDS_INFO("local memory slave port is not supported");
		ndsv5_local_memory_slave_port = 0;
	} else {
		ndsv5_local_memory_slave_port = 1;
	}
	ndsv5_check_idlm_capability_before = 1;
	NDS_INFO("ndsv5_ilm_bpa: 0x%" PRIx64 " ndsv5_dlm_bpa: 0x%" PRIx64, ndsv5_ilm_bpa, ndsv5_dlm_bpa);
	NDS_INFO("ndsv5_ilm_lmsz: 0x%" PRIx64 " ndsv5_dlm_lmsz: 0x%" PRIx64, ndsv5_ilm_lmsz, ndsv5_dlm_lmsz);
	NDS_INFO("ndsv5_local_memory_slave_port=0x%x", ndsv5_local_memory_slave_port);

	/* (misa[20] == 1) & (micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0) & (mmsc_cfg.CCTLCSR == 1) */
	if (((reg_misa_value & 0x100000) == 0) || (((reg_micm_cfg_value & 0x1c0) == 0) &&
	    ((reg_mdcm_cfg_value & 0x1c0) == 0)) || ((reg_mmsc_cfg_value & 0x10000) == 0)) {
		NDS_INFO("diasble CSR_UCCTLBEGINADDR,CSR_UCCTLCOMMAND registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UCCTLBEGINADDR].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_UCCTLCOMMAND].exist = false;
	}

	/* (misa[18] == 1) & (micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0) & (mmsc_cfg.CCTLCSR == 1) */
	if (((reg_misa_value & 0x40000) == 0) ||
	    (((reg_micm_cfg_value & 0x1c0) == 0) && ((reg_mdcm_cfg_value & 0x1c0) == 0)) ||
	    ((reg_mmsc_cfg_value & 0x10000) == 0)) {
		NDS_INFO("diasble CSR_SCCTLDATA register");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SCCTLDATA].exist = false;
	}

	if (ena_hit_exception) {
		/* enable Exception Redirection Register */
		char *dex2dbg_name = ndsv5_get_CSR_name(target, CSR_DEXC2DBG);
		if (dex2dbg_name == NULL) {
			NDS_INFO("get dex2dbg_name ERROR");
			return ERROR_FAIL;
		}
		struct reg *reg_dexc2dbg = register_get_by_name(target->reg_cache, dex2dbg_name, 1);
		reg_dexc2dbg->type->get(reg_dexc2dbg);
		uint64_t reg_dexc2dbg_value = buf_get_u64(reg_dexc2dbg->value, 0, reg_dexc2dbg->size);
		reg_dexc2dbg_value |= 0xFFF;
		reg_dexc2dbg->type->set(reg_dexc2dbg, (uint8_t *)&reg_dexc2dbg_value);
		NDS_INFO("reg_dexc2dbg: 0x%" PRIx64, reg_dexc2dbg_value);
	}
	/* flag for executing this function */
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	/* for vector registers */
	LOG_DEBUG("nds32->nds_vector_length: %d", nds32->nds_vector_length);
	if ((nds32->nds_vector_length == 0) && ((reg_misa_value & (0x01 << 21)) == 0)) {
		NDS_INFO("diasble vectors registers");
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VSTART].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VXSAT].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VXRM].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VL].exist = false;
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VTYPE].exist = false;
		for (i = GDB_REGNO_V0; i <= GDB_REGNO_V31; i++)
			target->reg_cache->reg_list[i].exist = false;
	}
	if ((reg_mmsc_cfg_value&0x1000000000)>>36 != 1)
		target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_VLENB].exist = false;

	nds32->execute_register_init = true;
	return ERROR_OK;
}

char BitFileName_64[512];
uint32_t nds32_redirect_64bitfilename;
static int ndsv5_init_reg(struct target *target)
{
	NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	uint32_t i;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	switch (info->dtm_version) {
		case 0:
			p_riscv_reg_arch_type = (struct reg_arch_type *)&riscv_reg_arch_type;
			break;
		case 1:
			p_riscv_reg_arch_type = (struct reg_arch_type *)&riscv_reg_arch_type;
			break;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return ERROR_FAIL;
	}

	/* disable FPU register */
	/*
	for (i = GDB_REGNO_FPR0; i <= GDB_REGNO_FPR31; i++) {
		target->reg_cache->reg_list[i].exist = false;
	}
	*/

	/* redirect all CSRs (r->name/r->exist) to NDS define */
	/*
	if (nds_reduce_csr)
		ndsv5_redefine_CSR_name(target);
	*/

	/* redirect reg_arch_type (reg_get/reg_set) */
	for (i = 0; i < GDB_REGNO_COUNT; i++) {
		struct reg *r = &target->reg_cache->reg_list[i];
		r->type = &ndsv5_reg_arch_type;
		if (i >= GDB_REGNO_V0 && i <= GDB_REGNO_V31)
			r->type = &ndsv5_vector_reg_access_type;
	}

	/* Support register alias name */
	LOG_DEBUG("Support Register alias name");
	struct reg *r_mccache_ctl_base = &target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MCCACHE_CTL_BASE];
	r_mccache_ctl_base->alias_name = strdup("mccache_ctl_base");
	LOG_DEBUG("Support Register alias name Done!");

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	gpBitFieldFileName = gpBitFieldFileNameV5;
	if (riscv_xlen(target) == 64) {
		nds32->gmon_64_bit = 1;
		if (nds32_redirect_64bitfilename == 0) {
			char *pBitFileName_64 = (char *)&BitFileName_64[0];
			char *pfilename = pBitFileName_64;
			strcpy(pfilename, gpBitFieldFileName);
			char *curr_str = strstr(pfilename, ".xml");
			strcpy(curr_str, "_64");
			curr_str += 3;
			strcpy(curr_str, ".xml");
			curr_str += 4;
			*curr_str = 0;
			gpBitFieldFileName = (char *)pBitFileName_64;
			gpBitFieldFileNameV5 = gpBitFieldFileName;
			nds32_redirect_64bitfilename = 1;
		}
		NDS_INFO("gpBitFieldFileName: %s", gpBitFieldFileName);
	} else
		nds32->gmon_64_bit = 0;
	NDS_INFO("nds32->gmon_64_bit: 0x%x", nds32->gmon_64_bit);

	return ERROR_OK;
}

extern int ndsv5_reexamine(struct target *target);
static int ndsv5_gdb_attach(struct target *target)
{
	NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if (!target_was_examined(target)) {
		NDS_INFO("target was NOT examined");
		if (ndsv5_reexamine(target) != ERROR_OK) {
			NDS_INFO("[%s] hart %d reexamine failed!!", target->tap->dotted_name, target->coreid);
			return ERROR_FAIL;
		}
	}

	if (nds32->attached == false) {
		if (target->smp) {
			struct target_list *tlist;
			foreach_smp_target(tlist, target->smp_targets) {
				struct target *t = tlist->target;
				struct nds32_v5 *t_nds32 = target_to_nds32_v5(t);

				ndsv5_init_reg(t);
				target_halt(t);
				ndsv5_init_option_reg(t);

				t_nds32->attached = true;
				t_nds32->hit_syscall = false;
				t_nds32->gdb_run_mode = RUN_MODE_DEBUG;
				t_nds32->is_program_exit = false;

				/* clear pending status */
				struct semihosting *semihosting = t->semihosting;
				if (semihosting)
					semihosting->hit_fileio = false;
			}
		} else {
			ndsv5_init_reg(target);
			target_halt(target);
			ndsv5_init_option_reg(target);

			nds32->attached = true;
			nds32->hit_syscall = false;


				/* clear pending status */
			struct semihosting *semihosting = target->semihosting;
			if (semihosting)
				semihosting->hit_fileio = false;

			/* reset to debug mode in case abnormal operations */
			nds32->gdb_run_mode = RUN_MODE_DEBUG;
			nds32->is_program_exit = false;
		}
	}

	return ERROR_OK;
}

static int ndsv5_gdb_detach(struct target *target)
{
	NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if (nds32->attached) {
		if (target->state != TARGET_HALTED) {
			/*
			nds32->attached = false;
			return ERROR_OK;
			*/
			target_halt(target);
		}
		/* clear all breakpoints & watchpoints */
		breakpoint_clear_target(nds32->target);
		watchpoint_clear_target(nds32->target);

		nds32->gdb_run_mode = RUN_MODE_DEBUG;

		/* turn-off suppressed hsp exception */
		if (nds32->suppressed_hsp_exception) {
			ndsv5_suppressed_hsp_exception(target, false);
			LOG_DEBUG("Disable suppressed hsp exception");
			nds32->suppressed_hsp_exception = false;
		}

		/* Set attached to false before resume */
		nds32->attached = false;

		/* free run in debug mode */
		target_resume(target, 1, 0, 0, 0);
	}

	return ERROR_OK;
}

static const char * const TARGET_EVENT_Name[] = {
	"GDB_HALT",
	"HALTED",
	"RESUMED",
	"RESUME_START",
	"RESUME_END",
	"STEP_START",
	"STEP_END",

	"GDB_START",
	"GDB_END",

	"RESET_START",
	"RESET_ASSERT_PRE",
	"RESET_ASSERT",
	"RESET_ASSERT_POST",
	"RESET_DEASSERT_PRE",
	"RESET_DEASSERT_POST",
	"RESET_INIT",
	"RESET_END",

	"DEBUG_HALTED",
	"DEBUG_RESUMED",

	"EXAMINE_START",
	"EXAMINE_FAIL",
	"EXAMINE_END",

	"GDB_ATTACH",
	"GDB_DETACH",

	"GDB_FLASH_ERASE_START",
	"GDB_FLASH_ERASE_END",
	"GDB_FLASH_WRITE_START",
	"GDB_FLASH_WRITE_END",

	"TRACE_CONFIG",
	"",
};

static int ndsv5_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	NDS_INFO("event = 0x%08x, %s", event, TARGET_EVENT_Name[event]);
	int retval = ERROR_OK;
	int target_number = *(int *)priv;
	LOG_DEBUG("target_number=%d, target->target_number=%d, event=0x%08x",
		target_number, target->target_number, event);
	if (target_number != target->target_number)
		return ERROR_OK;

	switch (event) {
		case TARGET_EVENT_GDB_ATTACH:
			retval = ndsv5_gdb_attach(target);
			break;
		case TARGET_EVENT_GDB_DETACH:
			retval = ndsv5_gdb_detach(target);
			break;
		default:
			break;
	}

	return retval;
}

int ndsv5_handle_examine(struct target *target)
{
	NDS_INFO("%s", __func__);
	/* register event callback */
	target_register_event_callback(ndsv5_callback_event_handler, &target->target_number);
	return ERROR_OK;
}

int ndsv5_poll_wo_announce(struct target *target)
{
	/* when polling target halted, do NOT announce gdb */
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	switch (info->dtm_version) {
		case 0:
			LOG_ERROR("Unsupported OLD DTM version: %d", info->dtm_version);
			return ERROR_FAIL;

		case 1:
			return riscv013_poll_wo_announce(target);

		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return ERROR_FAIL;
	}
	return ERROR_FAIL;
}

extern int semihosting_common_fileio_info(struct target *target,
		struct gdb_fileio_info *fileio_info);
int ndsv5_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	struct semihosting *semihosting = target->semihosting;
	if (semihosting && semihosting->hit_fileio)
		return semihosting_common_fileio_info(target, fileio_info);

	/* fill syscall parameters to file-I/O info */
	if (NULL == fileio_info) {
		LOG_ERROR("Target has not initial file-I/O data structure");
		return ERROR_FAIL;
	}

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->hit_syscall == false)
		return ERROR_FAIL;

	target = nds32->active_target;

	struct reg *reg_r0 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[10], 1); /* a0 */
	reg_r0->type->get(reg_r0);
	struct reg *reg_r1 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[11], 1); /* a1 */
	reg_r1->type->get(reg_r1);
	struct reg *reg_r2 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[12], 1); /* a2 */
	reg_r2->type->get(reg_r2);

	NDS_INFO("hit syscall ID: 0x%x\n", (uint32_t)nds32->active_syscall_id);

	/* free previous identifier storage */
	if (NULL != fileio_info->identifier) {
		free(fileio_info->identifier);
		fileio_info->identifier = NULL;
	}

	switch (nds32->active_syscall_id) {
		case NDS_EBREAK_EXIT:
			fileio_info->identifier = (char *)malloc(5);
			sprintf(fileio_info->identifier, "exit");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			/*
			target->is_program_exit = true;
			*/
			break;
		case NDS_EBREAK_OPEN:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(5);
				sprintf(fileio_info->identifier, "open");
				fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
				/* reserve fileio_info->param_2 for length of path */
				fileio_info->param_3 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
				fileio_info->param_4 = buf_get_u64(reg_r2->value, 0, reg_r2->size);

				target_read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS_EBREAK_CLOSE:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "close");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			break;
		case NDS_EBREAK_READ:
			fileio_info->identifier = (char *)malloc(5);
			sprintf(fileio_info->identifier, "read");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			fileio_info->param_2 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
			fileio_info->param_3 = buf_get_u64(reg_r2->value, 0, reg_r2->size);
			break;
		case NDS_EBREAK_WRITE:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "write");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			fileio_info->param_2 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
			fileio_info->param_3 = buf_get_u64(reg_r2->value, 0, reg_r2->size);
			break;
		case NDS_EBREAK_LSEEK:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "lseek");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			fileio_info->param_2 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
			fileio_info->param_3 = buf_get_u64(reg_r2->value, 0, reg_r2->size);
			break;
		case NDS_EBREAK_UNLINK:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "unlink");
				fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
				/* reserve fileio_info->param_2 for length of path */

				target_read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS_EBREAK_RENAME:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "rename");
				fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
				/* reserve fileio_info->param_2 for length of old path */
				fileio_info->param_3 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
				/* reserve fileio_info->param_4 for length of new path */

				target_read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;

				target_read_buffer(target, fileio_info->param_3,
						256, filename);
				fileio_info->param_4 = strlen((char *)filename) + 1;
			}
			break;
		case NDS_EBREAK_FSTAT:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fstat");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			fileio_info->param_2 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
			break;
		case NDS_EBREAK_STAT:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(5);
				sprintf(fileio_info->identifier, "stat");
				fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
				/* reserve fileio_info->param_2 for length of old path */
				fileio_info->param_3 = buf_get_u64(reg_r1->value, 0, reg_r1->size);

				target_read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS_EBREAK_GETTIMEOFDAY:
			fileio_info->identifier = (char *)malloc(13);
			sprintf(fileio_info->identifier, "gettimeofday");
			fileio_info->param_1 = buf_get_u64(reg_r0->value, 0, reg_r0->size);
			fileio_info->param_2 = buf_get_u64(reg_r1->value, 0, reg_r1->size);
			break;

		default:
			fileio_info->identifier = (char *)malloc(8);
			sprintf(fileio_info->identifier, "unknown");
			break;
	}

	return ERROR_OK;
}

extern int semihosting_common_fileio_end(struct target *target, int result,
		int fileio_errno, bool ctrl_c);
int ndsv5_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c)
{
	struct semihosting *semihosting = target->semihosting;
	if (semihosting && semihosting->hit_fileio)
		return semihosting_common_fileio_end(target, retcode, fileio_errno, ctrl_c);

	NDS_INFO("syscall return code: 0x%x, errno: 0x%x, ctrl_c: %s",
			retcode, fileio_errno, ctrl_c ? "true" : "false");

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	target = nds32->active_target;

	int64_t reg_r0_value = 0;
	struct reg *reg_r0 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[10], 1); /* a0 */

	LOG_DEBUG("riscv_select_current_hart...");
	riscv_select_current_hart(target);
	/* For virtual hosting, GDB always returns -1 as error.  OpenOCD should
	   fill correct return value to $r0 according to function calls.  */
	if (retcode == -1) {
		switch (nds32->active_syscall_id) {
			/* Return -1 if error.  */
			case	NDS_EBREAK_EXIT:
			case	NDS_EBREAK_OPEN:
			case	NDS_EBREAK_CLOSE:
			case	NDS_EBREAK_READ:
			case	NDS_EBREAK_WRITE:
			case	NDS_EBREAK_LSEEK:
			case	NDS_EBREAK_UNLINK:
			case	NDS_EBREAK_RENAME:
			case	NDS_EBREAK_FSTAT:
			case	NDS_EBREAK_STAT:
			case	NDS_EBREAK_GETTIMEOFDAY:
				/*
				reg_r0_value = -1;
				*/
				reg_r0_value = (0 - fileio_errno);
				reg_r0->type->set(reg_r0, (uint8_t *)&reg_r0_value);
				break;
		}
	} else {
		reg_r0_value = retcode;
		reg_r0->type->set(reg_r0, (uint8_t *)&reg_r0_value);
	}

	/*
	nds32->virtual_hosting_errno = nds32_convert_to_target_errno(fileio_errno);
	*/
	nds32->virtual_hosting_ctrl_c = ctrl_c;
	nds32->active_syscall_id = NDS_EBREAK_UNDEFINED;

	return ERROR_OK;
}

uint64_t nds_support_syscall_id[NDS_EBREAK_NUMS] = {
	NDS_EBREAK_EXIT,
	NDS_EBREAK_OPEN,
	NDS_EBREAK_CLOSE,
	NDS_EBREAK_READ,
	NDS_EBREAK_WRITE,
	NDS_EBREAK_LSEEK,
	NDS_EBREAK_UNLINK,
	NDS_EBREAK_RENAME,
	NDS_EBREAK_FSTAT,
	NDS_EBREAK_STAT,
	NDS_EBREAK_GETTIMEOFDAY,

	SEMIHOSTING_SYS_GET_CMDLINE,
};

extern char *p_nds_cmdline;
int ndsv5_virtual_hosting_check(struct target *target)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if (!target->smp && !nds32->hit_syscall) {
		nds32->hit_syscall = false;
		nds32->active_syscall_id = NDS_EBREAK_UNDEFINED;
		nds32->active_target = NULL;
	}

	char *ddcause_name = ndsv5_get_CSR_name(target, CSR_DDCAUSE);
	if (ddcause_name == NULL) {
		LOG_ERROR("get ddcause_name ERROR");
		return ERROR_FAIL;
	}
	struct reg *reg_ddcause = register_get_by_name(target->reg_cache, ddcause_name, 1);
	reg_ddcause->type->get(reg_ddcause);
	uint64_t reg_ddcause_value = buf_get_u64(reg_ddcause->value, 0, reg_ddcause->size);
	uint32_t reg_ddcause_maintype = (reg_ddcause_value & REG_DDCAUSE_MASK);
	NDS_INFO("reg_ddcause_value = 0x%" PRIx64, reg_ddcause_value);
	NDS_INFO("reg_ddcause_maintype = 0x%" PRIx32, reg_ddcause_maintype);
	if (reg_ddcause_maintype == REG_DDCAUSE_STACK_OVERFLOW && nds32->suppressed_hsp_exception) {
		/* current pc, addr = 0, do not handle breakpoints, not debugging */
		strict_step(target, false);

		/* Get $sp */
		struct reg *reg_sp = register_get_by_name(target->reg_cache, "sp", 1);
		reg_sp->type->get(reg_sp);
		uint64_t reg_sp_value = buf_get_u64(reg_sp->value, 0, reg_sp->size);
		LOG_DEBUG("strict_step (after) $sp: 0x%lx", (long unsigned int)reg_sp_value);

		/* Set $msp_bound */
		struct reg *msp_bound = ndsv5_get_reg_by_CSR(target, CSR_MSP_BOUND);
		uint64_t msp_value = ndsv5_get_register_value(msp_bound);
		LOG_DEBUG("msp_bound = 0x%" PRIx64, msp_value);
		ndsv5_set_register_value(msp_bound, reg_sp_value);
		LOG_DEBUG("set $msp_bound = 0x%" PRIx64, reg_sp_value);

		/* Enable OVF_EN again */
		struct reg *mhsp_ctl = ndsv5_get_reg_by_CSR(target, CSR_MHSP_CTL);
		ndsv5_set_register_value(mhsp_ctl, MHSP_CTL_OVF_EN | MHSP_CTL_U | MHSP_CTL_S | MHSP_CTL_M);

		return ERROR_FAIL;
	} else if (reg_ddcause_maintype != REG_DDCAUSE_EBREAK) {
		nds32->ddcause_maintype = reg_ddcause_maintype;
		target->debug_reason = DBG_REASON_HIT_EXCEPTIONS;
		return ERROR_OK;
	}

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		/* check if SW breakpoint */
		struct breakpoint *bp = target->breakpoints;
		struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
		reg_pc->type->get(reg_pc);
		uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
		if (bp) {
			for (bp = target->breakpoints; bp; bp = bp->next) {
				if ((bp->type == BKPT_SOFT) && ((target_addr_t)reg_pc_value == bp->address)) {
					NDS_INFO("SW breakpoint on ebreak");
					return ERROR_OK;
				}
			}
		}

		/*
		 * These instructions sequance trigger a virtual hosting call.
		 * The ebreak is always uncompressed, should look the following:
		 *
		 * .option norvc
		 * slli x0, x0, 0x1f		01f01013
		 * ebreak			00100073
		 * srai x0, x0, <protocol>	40505013(for RV32I/RV64I), 41505013(for RV32E)
		 *
		 */
		uint8_t tmp[12];
		struct reg *reg_syscall = NULL;
		uint64_t reg_syscall_value = 0;
		/* Read the current instruction, including the bracketing */
		if (target_read_memory(target, reg_pc_value - 4, 2, 6, tmp) == ERROR_OK) {
			uint32_t pre = target_buffer_get_u32(target, tmp);
			uint32_t ebreak = target_buffer_get_u32(target, tmp + 4);
			uint32_t post = target_buffer_get_u32(target, tmp + 8);
			NDS_INFO("check %08x %08x %08x from 0x%" PRIx64 "-4", pre, ebreak, post, reg_pc_value);

			if (pre == 0x01f01013 && ebreak == 0x00100073 && (post == 0x40505013 || post == 0x41505013)) {
				if (post == 0x40505013) {
					/* for RV32I/RV64I, use $a7 */
					reg_syscall = register_get_by_name(target->reg_cache, "a7", 1);
				} else {
					/* for RV32E, use $t0 */
					reg_syscall = register_get_by_name(target->reg_cache, "t0", 1);
				}
			}
		}

		if (reg_syscall != NULL) {
			reg_syscall->type->get(reg_syscall);
			reg_syscall_value = buf_get_u64(reg_syscall->value, 0, reg_syscall->size);

			/* Check  syscall id */
			uint32_t i;
			for (i = 0; i < NDS_EBREAK_NUMS; i++) {
				if ((reg_syscall_value&0xFFFF) == (SEMIHOSTING_SYS_GET_CMDLINE&0xFFFF)) {
					NDS_INFO("Virtual hosting get cmdline");

					if (!p_nds_cmdline) {
						LOG_ERROR("Unable to find any args, use set_args first!!");
						return ERROR_FAIL;
					}

					struct reg *reg_a0 = register_get_by_name(target->reg_cache, "a0", 1);
					reg_a0->type->get(reg_a0);
					uint64_t addr = buf_get_u64(reg_a0->value, 0, reg_a0->size);

					struct reg *reg_a1 = register_get_by_name(target->reg_cache, "a1", 1);
					reg_a1->type->get(reg_a1);
					uint64_t size = buf_get_u64(reg_a1->value, 0, reg_a1->size);

					uint32_t len = strlen(p_nds_cmdline) + 1;
					if (len > size) {
						LOG_ERROR("Unable to copy buffer, len(%d) > buffer size(%ld)", len, size);
						return ERROR_FAIL;
					}

					int retval = target_write_buffer(target, addr, len, (uint8_t *)p_nds_cmdline);
					if (retval != ERROR_OK) {
						LOG_ERROR("Write cmdline failed!!");
						return ERROR_FAIL;
					}

					riscv_reg_t pc;
					riscv_get_register(target, &pc, GDB_REGNO_PC);
					riscv_set_register(target, GDB_REGNO_PC, pc + 4);
					LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, pc+4);
					return ERROR_FAIL;
				} else if ((reg_syscall_value&0xFFFF) == (nds_support_syscall_id[i]&0xFFFF)) {
					nds32->active_syscall_id = (uint32_t)nds_support_syscall_id[i];
					nds32->hit_syscall = true;
					nds32->active_target = target;
					ndsv5_without_announce = 0;
					NDS_INFO("nds32->active_syscall_id = 0x%x", (uint32_t)nds32->active_syscall_id);

					/* Reset reg */
					reg_syscall_value = 0;
					reg_syscall->type->set(reg_syscall, (uint8_t *)&reg_syscall_value);
					NDS_INFO("reset reg_a7/t0_value");

					riscv_reg_t pc;
					riscv_get_register(target, &pc, GDB_REGNO_PC);
					riscv_set_register(target, GDB_REGNO_PC, pc + 4);
					LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, pc+4);
					return ERROR_OK;
				}
			}
		}

		/* NOT syscall_id (maybe target break insn. ) */
		NDS_INFO("target break");
		return ERROR_OK;
	}

	return ERROR_OK;
}

int ndsv5_hit_watchpoint_check(struct target *target)
{
	NDS_INFO("%s", target_debug_reason[target->debug_reason]);

	if (target->debug_reason == DBG_REASON_WPTANDBKPT) {
		/* check if hw-breakpoint, otherwise watchpoint */
		struct breakpoint *bp = target->breakpoints;
		if (bp) {
			struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
			reg_pc->type->get(reg_pc);
			uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);

			for (bp = target->breakpoints; bp; bp = bp->next) {
				if ((bp->type == BKPT_HARD) && ((target_addr_t)reg_pc_value == bp->address))
					return ERROR_FAIL;
			}
		}
		target->debug_reason = DBG_REASON_WATCHPOINT;
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

/*
Register-Based Loads and Stores
(C.LW/C.LD/..)
15-13    12-10   9-7   6-5  4-2  1-0
funct3    imm    rs1   imm  rd   op

(C.SW/C.SD/..)
15-13    12-10   9-7   6-5  4-2  1-0
funct3    imm    rs1   imm  rs2  op

Stack-Pointer-Based Loads and Stores
(C.LWSP/C.LDSP/..)
15-13    12   11-7  6-2  1-0
funct3   imm   rd   imm  op

(C.SWSP/C.SDSP/..)
15-13    12-7  6-2  1-0
funct3   imm   rs2  op
*/

#define MASK_C_LOAD_STORE   0xE003

#define MATCH_LBGP    0x000B
#define MATCH_LBUGP   0x200B
#define MATCH_LHGP    0x102B
#define MATCH_LHUGP   0x502B
#define MATCH_LWGP    0x202B
#define MATCH_LWUGP   0x602B
#define MATCH_LDGP    0x302B

#define MATCH_SBGP    0x300B
#define MATCH_SHGP    0x002B
#define MATCH_SWGP    0x402B
#define MATCH_SDGP    0x702B

/*
imm[11:0] rs1 000 rd 0000011 LB
imm[11:0] rs1 001 rd 0000011 LH
imm[11:0] rs1 010 rd 0000011 LW
imm[11:0] rs1 100 rd 0000011 LBU
imm[11:0] rs1 101 rd 0000011 LHU

imm[11:5] rs2 rs1 000 imm[4:0] 0100011 SB
imm[11:5] rs2 rs1 001 imm[4:0] 0100011 SH
imm[11:5] rs2 rs1 010 imm[4:0] 0100011 SW

8201a783:	lw	a5,-2016(gp)
		imm[11:0]        rs1(3->gp)       rd(15->a5)  opcode
		1000 0010 0000   00011       010  01111       0000011

00412c83:	lw	s9,4(sp)
		imm[11:0]        rs1(2->sp)       rd(25->s9)  opcode
		0000 0000 0100   00010       010  11001       0000011

02042823:	sw	zero,48(s0)
		imm[11:5]  rs2(0->zero) rs1(8->s0)       imm[4:0]   opcode
		0000001    00000        01000       010  10000      0100011


00f59623:	sh	a5,12(a1)
		imm[11:5]  rs2(15->a5)  rs1(11->a1)      imm[4:0]   opcode
		0000000    01111        01011       001  01100      0100011
*/
#define MATCH_ID_LOAD       0
#define MATCH_NUMS_LOAD_32  (10 + 7 + 2)

#define MATCH_ID_LOADGP     10
#define MATCH_ID_LBGP       (MATCH_ID_LOADGP + 0)
#define MATCH_ID_LBUGP      (MATCH_ID_LOADGP + 1)
#define MATCH_ID_LHGP       (MATCH_ID_LOADGP + 2)
#define MATCH_ID_LHUGP      (MATCH_ID_LOADGP + 3)
#define MATCH_ID_LWGP       (MATCH_ID_LOADGP + 4)
#define MATCH_ID_LWUGP      (MATCH_ID_LOADGP + 5)
#define MATCH_ID_LDGP       (MATCH_ID_LOADGP + 6)
#define MATCH_ID_LR         (MATCH_ID_LOADGP + 7)
#define MATCH_ID_LR_D       (MATCH_ID_LR + 0)
#define MATCH_ID_LR_W       (MATCH_ID_LR + 1)

#define MATCH_ID_C_LOAD     (MATCH_NUMS_LOAD_32)
#define MATCH_ID_C_LQ       (MATCH_ID_C_LOAD + 0)
#define MATCH_ID_C_LW       (MATCH_ID_C_LOAD + 1)
#define MATCH_ID_C_LD       (MATCH_ID_C_LOAD + 2)
#define MATCH_ID_C_FLW      (MATCH_ID_C_LOAD + 3)
#define MATCH_ID_C_FLD      (MATCH_ID_C_LOAD + 4)
#define MATCH_ID_C_LOADSP   (MATCH_ID_C_LOAD + 5)
#define MATCH_ID_C_LQSP     (MATCH_ID_C_LOADSP + 0)
#define MATCH_ID_C_LWSP     (MATCH_ID_C_LOADSP + 1)
#define MATCH_ID_C_LDSP     (MATCH_ID_C_LOADSP + 2)
#define MATCH_ID_C_FLWSP    (MATCH_ID_C_LOADSP + 3)
#define MATCH_ID_C_FLDSP    (MATCH_ID_C_LOADSP + 4)
#define MATCH_ID_LOAD_NUMS  (MATCH_ID_C_LOADSP + 5)

#define MATCH_ID_STORE      0
#define MATCH_NUMS_STORE_32 (7 + 4 + 2)

#define MATCH_ID_STOREGP    7
#define MATCH_ID_SBGP       (MATCH_ID_STOREGP + 0)
#define MATCH_ID_SHGP       (MATCH_ID_STOREGP + 1)
#define MATCH_ID_SWGP       (MATCH_ID_STOREGP + 2)
#define MATCH_ID_SDGP       (MATCH_ID_STOREGP + 3)
#define MATCH_ID_SC         (MATCH_ID_STOREGP + 4)
#define MATCH_ID_SC_D       (MATCH_ID_SC + 0)
#define MATCH_ID_SC_W       (MATCH_ID_SC + 1)

#define MATCH_ID_C_STORE    (MATCH_NUMS_STORE_32)
#define MATCH_ID_C_SQ       (MATCH_ID_C_STORE + 0)
#define MATCH_ID_C_SW       (MATCH_ID_C_STORE + 1)
#define MATCH_ID_C_SD       (MATCH_ID_C_STORE + 2)
#define MATCH_ID_C_FSW      (MATCH_ID_C_STORE + 3)
#define MATCH_ID_C_FSD      (MATCH_ID_C_STORE + 4)
#define MATCH_ID_C_STORESP  (MATCH_ID_C_STORE + 5)
#define MATCH_ID_C_SQSP     (MATCH_ID_C_STORESP + 0)
#define MATCH_ID_C_SWSP     (MATCH_ID_C_STORESP + 1)
#define MATCH_ID_C_SDSP     (MATCH_ID_C_STORESP + 2)
#define MATCH_ID_C_FSWSP    (MATCH_ID_C_STORESP + 3)
#define MATCH_ID_C_FSDSP    (MATCH_ID_C_STORESP + 4)
#define MATCH_ID_STORE_NUMS (MATCH_ID_C_STORESP + 5)

unsigned int g_insn_load_match[] = {
	MATCH_LB,
	MATCH_LH,
	MATCH_LW,
	MATCH_LD,
	MATCH_LBU,
	MATCH_LHU,
	MATCH_LWU,
	MATCH_FLW,
	MATCH_FLD,
	MATCH_FLQ,
	/* GP-implied load insn. */
	MATCH_LBGP,
	MATCH_LBUGP,
	MATCH_LHGP,
	MATCH_LHUGP,
	MATCH_LWGP,
	MATCH_LWUGP,
	MATCH_LDGP,
	/* Atomic Memory Operations */
	(MATCH_LR_D & MASK_LW),
	(MATCH_LR_W & MASK_LW),
	/* Compressed Instruction Formats */
	MATCH_C_FLD, /* MATCH_C_LQ, */
	MATCH_C_LW,
	MATCH_C_LD,
	MATCH_C_FLW,
	MATCH_C_FLD,
	MATCH_C_FLDSP, /* MATCH_C_LQSP, */
	MATCH_C_LWSP,
	MATCH_C_LDSP,
	MATCH_C_FLWSP,
	MATCH_C_FLDSP,
};

unsigned int g_insn_store_match[] = {
	MATCH_SB,
	MATCH_SH,
	MATCH_SW,
	MATCH_SD,
	MATCH_FSW,
	MATCH_FSD,
	MATCH_FSQ,
	/* GP-implied store insn. */
	MATCH_SBGP,
	MATCH_SHGP,
	MATCH_SWGP,
	MATCH_SDGP,
	/* Atomic Memory Operations */
	(MATCH_SC_D & MASK_SW),
	(MATCH_SC_W & MASK_SW),
	/* Compressed Instruction Formats */
	MATCH_C_FSD, /* MATCH_C_SQ, */
	MATCH_C_SW,
	MATCH_C_SD,
	MATCH_C_FSW,
	MATCH_C_FSD,
	MATCH_C_FSDSP, /* MATCH_C_SQSP, */
	MATCH_C_SWSP,
	MATCH_C_SDSP,
	MATCH_C_FSWSP,
	MATCH_C_FSDSP,
};

struct nds_insn_loadstore {
	const char *name;
	uint32_t length;
};

static struct nds_insn_loadstore nds_insn_load[] = {
	{ "lb", 1 },
	{ "lh", 2 },
	{ "lw", 4 },
	{ "ld", 8 },
	{ "lbu", 1 },
	{ "lhu", 2 },
	{ "lwu", 4 },
	{ "flw", 4 },
	{ "fld", 8 },
	{ "flq", 16 },
	{ "lbgp", 1 },
	{ "lbugp", 1 },
	{ "lhgp", 2 },
	{ "lhugp", 2 },
	{ "lwgp", 4 },
	{ "lwugp", 4 },
	{ "ldgp", 8 },
	{ "lr.d", 8 },
	{ "lr.w", 4 },

	{ "c.lq", 16 },
	{ "c.lw", 4 },
	{ "c.ld", 8 },
	{ "c.flw", 4 },
	{ "c.fld", 8 },
	{ "c.lqsp", 16 },
	{ "c.lwsp", 4 },
	{ "c.ldsp", 8 },
	{ "c.flwsp", 4 },
	{ "c.fldsp", 8 },
	{ "unknown", 0 },
};

static struct nds_insn_loadstore nds_insn_store[] = {
	{ "sb", 1 },
	{ "sh", 2 },
	{ "sw", 4 },
	{ "sd", 8 },
	{ "fsw", 4 },
	{ "fsd", 8 },
	{ "fsq", 16 },
	{ "sbgp", 1 },
	{ "shgp", 2 },
	{ "swgp", 4 },
	{ "sdgp", 8 },
	{ "sc.d", 8 },
	{ "sc.w", 4 },

	{ "c.sq", 16 },
	{ "c.sw", 4 },
	{ "c.sd", 8 },
	{ "c.fsw", 4 },
	{ "c.fsd", 8 },
	{ "c.sqsp", 16 },
	{ "c.swsp", 4 },
	{ "c.sdsp", 8 },
	{ "c.fswsp", 4 },
	{ "c.fsdsp", 8 },
	{ "unknown", 0 },
};

/*char *gp_rvc_reg_name[] = {
  REG_S0,
  REG_S1,
  REG_A0,
  REG_A1,
  REG_A2,
  REG_A3,
  REG_A4,
  REG_A5,
};*/

/*char *gp_reg_name[] = {
  REG_ZERO,  // "zero",// x0	zero	Zero
  REG_RA,    // "ra",  // x1	ra	Return address
  REG_SP,    // "sp",  // x2	sp	Stack pointer
  REG_GP,    // "gp",  // x3	gp	Global pointer
  REG_TP,    // "tp",  // x4	tp	Thread pointer
  REG_T0,    // "t0",  // x5-x7	t0-t2	Temporary registers
  REG_T1,    // "t1",
  REG_T2,    // "t2",
  REG_S0,    // "s0",  // x8-x9	s0-s1	Callee-saved registers
  REG_S1,    // "s1",
  REG_A0,    // "a0",  // x10-x17	a0-a7	Argument registers
  REG_A1,    // "a1",
  REG_A2,    // "a2",
  REG_A3,    // "a3",
  REG_A4,    // "a4",
  REG_A5,    // "a5",
  REG_A6,    // "a6",
  REG_A7,    // "a7",
  REG_S2,    // "s2",  // x18-x27	s2-s11	Callee-saved registers
  REG_S3,    // "s3",
  REG_S4,    // "s4",
  REG_S5,    // "s5",
  REG_S6,    // "s6",
  REG_S7,    // "s7",
  REG_S8,    // "s8",
  REG_S9,    // "s9",
  REG_S10,   // "s10",
  REG_S11,   // "s11",
  REG_T3,    // "t3",  // x28-x31	t3-t6	Temporary registers
  REG_T4,    // "t4",
  REG_T5,    // "t5",
  REG_T6,    // "t6",
  REG_FT0,   // "f0",
  REG_FT1,   // "f1",
  REG_FT2,   // "f2",
  REG_FT3,   // "f3",
  REG_FT4,   // "f4",
  REG_FT5,   // "f5",
  REG_FT6,   // "f6",
  REG_FT7,   // "f7",
  REG_FS0,   // "f8",
  REG_FS1,   // "f9",
  REG_FA0,   // "f10",
  REG_FA1,   // "f11",
  REG_FA2,   // "f12",
  REG_FA3,   // "f13",
  REG_FA4,   // "f14",
  REG_FA5,   // "f15",
  REG_FA6,   // "f16",
  REG_FA7,   // "f17",
  REG_FS2,   // "f18",
  REG_FS3,   // "f19",
  REG_FS4,   // "f20",
  REG_FS5,   // "f21",
  REG_FS6,   // "f22",
  REG_FS7,   // "f23",
  REG_FS8,   // "f24",
  REG_FS9,   // "f25",
  REG_FS10,  // "f26",
  REG_FS11,  // "f27",
  REG_FT8,   // "f28",
  REG_FT9,   // "f29",
  REG_FT10,  // "f30",
  REG_FT11,  // "f31",
};*/

int ndsv5_disassemble_c_load(unsigned int opcode, unsigned int *p_insn,
	unsigned int *p_rd, unsigned int *p_rs1, int *p_imm) {
	unsigned int i, chk_opcode = (opcode & MASK_C_LOAD_STORE);
	int get_imm = 0;

	for (i = MATCH_ID_C_LOAD; i < MATCH_ID_LOAD_NUMS; i++) {
		if (chk_opcode == g_insn_load_match[i])
			break;
	}
	if (i >= MATCH_ID_LOAD_NUMS) {
		return ERROR_FAIL;
	} else if ((i == MATCH_ID_C_LQ) && (ndsv5_cur_target_xlen != 128)) {
		/* C.FLD is an RV32DC/RV64DC-only
		   C.LQ is an RV128C-only */
		i = MATCH_ID_C_FLD;
	} else if ((i == MATCH_ID_C_LQSP) && (ndsv5_cur_target_xlen != 128)) {
		/* C.FLDSP is an RV32DC/RV64DC-only
		   C.LQSP is an RV128C-only */
		i = MATCH_ID_C_FLDSP;
	} else if ((i == MATCH_ID_C_LD) && (ndsv5_cur_target_xlen < 64)) {
		/* C.LD is an RV64C/RV128C-only
		   C.FLW is an RV32FC-only */
		i = MATCH_ID_C_FLW;
	} else if ((i == MATCH_ID_C_LDSP) && (ndsv5_cur_target_xlen < 64)) {
		/* C.LDSP is an RV64C/RV128C-only
		   C.FLWSP is an RV32FC-only */
		i = MATCH_ID_C_FLWSP;
	}

	/* Stack-Pointer-Based Loads and Stores */
	if (i >= MATCH_ID_C_LOADSP) {
		*p_rd = ((opcode >> 7) & 0x1f);
		*p_rs1 = 2; /* sp; */
		if ((i == MATCH_ID_C_LWSP) || (i == MATCH_ID_C_FLWSP)) {
			get_imm = (int)(((opcode >> 4) & 0x07) << 2);
			get_imm |= (int)(((opcode >> 12) & 0x01) << 5);
			get_imm |= (int)(((opcode >> 2) & 0x03) << 6);
		} else if ((i == MATCH_ID_C_LDSP) || (i == MATCH_ID_C_FLDSP)) {
			get_imm = (int)(((opcode >> 5) & 0x03) << 3);
			get_imm |= (int)(((opcode >> 12) & 0x01) << 5);
			get_imm |= (int)(((opcode >> 2) & 0x07) << 6);
		} else if (i == MATCH_ID_C_LQSP) {
			get_imm = (int)(((opcode >> 6) & 0x01) << 4);
			get_imm |= (int)(((opcode >> 12) & 0x01) << 5);
			get_imm |= (int)(((opcode >> 2) & 0x0f) << 6);
		}
	} else {
		*p_rd = ((opcode >> 2) & 0x07);
		*p_rs1 = ((opcode >> 7) & 0x07);
		if ((i == MATCH_ID_C_LW) || (i == MATCH_ID_C_FLW)) {
			get_imm = (int)(((opcode >> 10) & 0x07) << 3);
			get_imm |= (int)(((opcode >> 6) & 0x01) << 2);
			get_imm |= (int)(((opcode >> 5) & 0x01) << 6);
		} else if ((i == MATCH_ID_C_LD) || (i == MATCH_ID_C_FLD)) {
			get_imm = (int)(((opcode >> 10) & 0x07) << 3);
			get_imm |= (int)(((opcode >> 5) & 0x03) << 6);
		} else if (i == MATCH_ID_C_LQ) {
			get_imm = (int)(((opcode >> 11) & 0x03) << 4);
			get_imm |= (int)(((opcode >> 5) & 0x03) << 6);
			get_imm |= (int)(((opcode >> 10) & 0x01) << 8);
		}
	}
	*p_insn = i;
	*p_imm = get_imm;
	return ERROR_OK;
}

int ndsv5_disassemble_load(unsigned int opcode, unsigned int *p_insn,
	unsigned int *p_rd, unsigned int *p_rs1, int *p_imm) {
	unsigned int i, chk_opcode = (opcode & MASK_LW);
	int get_imm = 0;

	for (i = MATCH_ID_LOAD; i < MATCH_ID_C_LOAD; i++) {
		if (chk_opcode == g_insn_load_match[i])
			break;
	}
	if (i == MATCH_ID_C_LOAD)
		return ERROR_FAIL;

	/* Atomic Memory Operations */
	else if (i >= MATCH_ID_LR) {
		/* LR loads a word from the address in rs1
		 *      31-27    26 25  24-20 19-15  14-12   11-7  6-0
		 *      funct5   aq rl  0      rs1   funct3   rd   opcode
		 *        5      1  1   5       5      3      5     7        */
		*p_insn = i;
		*p_rd = ((opcode >> 7) & 0x1f);
		*p_rs1 = ((opcode >> 15) & 0x1f);
		return ERROR_OK;
	}
	/* GP-Based Loads */
	else if (i >= MATCH_ID_LOADGP) {
		*p_insn = i;
		*p_rd = ((opcode >> 7) & 0x1f);
		*p_rs1 = 3; /* gp; */
		if ((i == MATCH_ID_LBGP) || (i == MATCH_ID_LBUGP)) {
			/*  31      30 21    20     19 17     16 15     14   13 12  11 7    6 0
			    imm17 imm[10:1] imm11 imm[14:12] imm[16:15] imm0   LBGP   Rd   Custom-0 */
			get_imm = (int)(((opcode >> 14) & 0x01) << 0);
			get_imm |= (int)(((opcode >> 21) & 0x3ff) << 1);
			get_imm |= (int)(((opcode >> 20) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 17);
			if (get_imm & 0x20000)
				get_imm |= 0xFFFC0000;
		} else if ((i == MATCH_ID_LHGP) || (i == MATCH_ID_LHUGP)) {
			/*  31     30 21     20     19 17      16 15   14 12  11 7   6 0
			    imm17 imm[10:1] imm11 imm[14:12] imm[16:15] LHGP    Rd   Custom-1 */
			get_imm = (int)(((opcode >> 21) & 0x3ff) << 1);
			get_imm |= (int)(((opcode >> 20) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 17);
			if (get_imm & 0x20000)
				get_imm |= 0xFFFC0000;
		} else if ((i == MATCH_ID_LWGP) || (i == MATCH_ID_LWUGP)) {
			/*  31     30 22    21     20   19 17     16 15      14 12   11 7    6 0
			    imm18 imm[10:2] imm17 imm11 imm[14:12] imm[16:15] LWGP     Rd    Custom-1 */
			get_imm = (int)(((opcode >> 22) & 0x1ff) << 2);
			get_imm |= (int)(((opcode >> 20) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 21) & 0x01) << 17);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 18);
			if (get_imm & 0x40000)
				get_imm |= 0xFFF80000;
		} else if (i == MATCH_ID_LDGP) {
			/*  31     30 23     22 21      20    19 17      16 15     14 12    11 7   6 0
			    imm19 imm[10:3] imm[18:17] imm11 imm[14:12] imm[16:15]   LDGP     Rd   Custom-1 */
			get_imm = (int)(((opcode >> 23) & 0xff) << 3);
			get_imm |= (int)(((opcode >> 20) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 21) & 0x03) << 17);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 18);
			if (get_imm & 0x80000)
				get_imm |= 0xFFF00000;
		}
		*p_imm = get_imm;
		return ERROR_OK;
	}
	*p_insn = i;
	*p_rd = ((opcode >> 7) & 0x1f);
	*p_rs1 = ((opcode >> 15) & 0x1f);
	get_imm = (int)(opcode >> 20);
	if (get_imm & 0x800)
		get_imm |= 0xFFFFF000;
	*p_imm = get_imm;
	return ERROR_OK;
}

int ndsv5_disassemble_c_store(unsigned int opcode, unsigned int *p_insn,
	unsigned int *p_rs1, unsigned int *p_rs2, int *p_imm) {
	unsigned int i, chk_opcode = (opcode & MASK_C_LOAD_STORE);
	int get_imm = 0;

	for (i = MATCH_ID_C_STORE; i < MATCH_ID_STORE_NUMS; i++) {
		if (chk_opcode == g_insn_store_match[i])
			break;
	}
	if (i >= MATCH_ID_STORE_NUMS) {
		return ERROR_FAIL;
	} else if ((i == MATCH_ID_C_SQ) && (ndsv5_cur_target_xlen != 128)) {
		/* C.FSD is an RV32DC/RV64DC-only instruction
		   C.SQ is an RV128C-only */
		i = MATCH_ID_C_FSD;
	} else if ((i == MATCH_ID_C_SQSP) && (ndsv5_cur_target_xlen != 128)) {
		/* C.FSDSP is an RV32DC/RV64DC-only instruction
		   C.SQSP is an RV128C-only */
		i = MATCH_ID_C_FSDSP;
	} else if ((i == MATCH_ID_C_SD) && (ndsv5_cur_target_xlen < 64)) {
		/* C.FSW is an RV32FC-only
		   C.SD is an RV64C/RV128C-only */
		i = MATCH_ID_C_FSW;
	} else if ((i == MATCH_ID_C_SDSP) && (ndsv5_cur_target_xlen < 64)) {
		/* C.SDSP is an RV64C/RV128C
		   C.FSWSP is an RV32FC-only */
		i = MATCH_ID_C_FSWSP;
	}

	/* Stack-Pointer-Based Loads and Stores */
	if (i >= MATCH_ID_C_STORESP) {
		*p_rs2 = ((opcode >> 2) & 0x1f);
		*p_rs1 = 2; /* sp */
		if ((i == MATCH_ID_C_SWSP) || (i == MATCH_ID_C_FSWSP)) {
			get_imm = (int)(((opcode >> 9) & 0x0f) << 2);
			get_imm |= (int)(((opcode >> 7) & 0x03) << 6);
		} else if ((i == MATCH_ID_C_SDSP) || (i == MATCH_ID_C_FSDSP)) {
			get_imm = (int)(((opcode >> 10) & 0x07) << 3);
			get_imm |= (int)(((opcode >> 7) & 0x07) << 6);
		} else if (i == MATCH_ID_C_SQSP) {
			get_imm = (int)(((opcode >> 11) & 0x03) << 4);
			get_imm |= (int)(((opcode >> 7) & 0x0f) << 6);
		}

	} else {
		*p_rs2 = ((opcode >> 2) & 0x07);
		*p_rs1 = ((opcode >> 7) & 0x07);
		if ((i == MATCH_ID_C_SW) || (i == MATCH_ID_C_FSW)) {
			get_imm = (int)(((opcode >> 10) & 0x07) << 3);
			get_imm |= (int)(((opcode >> 6) & 0x01) << 2);
			get_imm |= (int)(((opcode >> 5) & 0x01) << 6);
		} else if ((i == MATCH_ID_C_SD) || (i == MATCH_ID_C_FSD)) {
			get_imm = (int)(((opcode >> 10) & 0x07) << 3);
			get_imm |= (int)(((opcode >> 5) & 0x03) << 6);
		} else if (i == MATCH_ID_C_SQ) {
			get_imm = (int)(((opcode >> 11) & 0x03) << 4);
			get_imm |= (int)(((opcode >> 5) & 0x03) << 6);
			get_imm |= (int)(((opcode >> 10) & 0x01) << 8);
		}
	}
	*p_insn = i;
	*p_imm = get_imm;
	return ERROR_OK;
}

int ndsv5_disassemble_store(unsigned int opcode, unsigned int *p_insn,
	unsigned int *p_rs1, unsigned int *p_rs2, int *p_imm) {

	unsigned int i, chk_opcode = (opcode & MASK_SW);
	int get_imm = 0;

	for (i = MATCH_ID_STORE; i < MATCH_ID_C_STORE; i++) {
		if (chk_opcode == g_insn_store_match[i])
			break;
	}
	if (i == MATCH_ID_C_STORE)
		return ERROR_FAIL;

	/* Atomic Memory Operations */
	else if (i >= MATCH_ID_SC) {
		/*  sc.d.rl    rd   rs2, (rs1)     SC writes a word in rs2 to the address in rs1,
		    31-27    26 25  24-20 19-15  14-12   11-7  6-0
		    funct5   aq rl  rs2    rs1   funct3   rd   opcode
		      5      1  1   5       5      3      5     7        */
		*p_insn = i;
		*p_rs1 = ((opcode >> 15) & 0x1f);
		*p_rs2 = ((opcode >> 20) & 0x1f);
		return ERROR_OK;
	}
	/* GP-Based Stores */
	else if (i >= MATCH_ID_STOREGP) {
		*p_insn = i;
		*p_rs1 = 3; /* gp */
		*p_rs2 = ((opcode >> 20) & 0x1f);
		if (i == MATCH_ID_SBGP) {
			/*  31     30 25   24 20   19 17     16 15      14    13 12   11 8       7     6 0
			    imm17 imm[10:5]   Rs2  imm[14:12] imm[16:15] imm0   SBGP   imm[4:1] imm11   Custom-0 */
			get_imm = (int)(((opcode >> 14) & 0x01) << 0);
			get_imm |= (int)(((opcode >> 8) & 0x0f) << 1);
			get_imm |= (int)(((opcode >> 25) & 0x3f) << 5);
			get_imm |= (int)(((opcode >> 7) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 17);
			if (get_imm & 0x20000)
				get_imm |= 0xFFFC0000;
		} else if (i == MATCH_ID_SHGP) {
			/*  31     30 25   24 20   19 17     16 15       14 12    11 8       7      6 0
			    imm17 imm[10:5]   Rs2  imm[14:12] imm[16:15]   SHGP   imm[4:1]  imm11   Custom-1 */
			get_imm = (int)(((opcode >> 8) & 0x0f) << 1);
			get_imm |= (int)(((opcode >> 25) & 0x3f) << 5);
			get_imm |= (int)(((opcode >> 7) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 17);
			if (get_imm & 0x20000)
				get_imm |= 0xFFFC0000;
		} else if (i == MATCH_ID_SWGP) {
			/*  31     30 25   24 20  19 17      16 15       14 12    11 9      8     7     6 0
			    imm18 imm[10:5]   Rs2  imm[14:12] imm[16:15]   SWGP   imm[4:2] imm17 imm11 Custom-1 */
			get_imm = (int)(((opcode >> 9) & 0x07) << 2);
			get_imm |= (int)(((opcode >> 25) & 0x3f) << 5);
			get_imm |= (int)(((opcode >> 7) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 8) & 0x01) << 17);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 18);
			if (get_imm & 0x40000)
				get_imm |= 0xFFF80000;
		} else if (i == MATCH_ID_SDGP) {
			/*  31     30 25   24 20  19 17      16 15       14 12    11 10    9 8        7      6 0
			    imm19 imm[10:5]   Rs2  imm[14:12] imm[16:15]   SDGP   imm[4:3] imm[18:17] imm11 Custom-1 */
			get_imm = (int)(((opcode >> 10) & 0x03) << 3);
			get_imm |= (int)(((opcode >> 25) & 0x3f) << 5);
			get_imm |= (int)(((opcode >> 7) & 0x01) << 11);
			get_imm |= (int)(((opcode >> 17) & 0x07) << 12);
			get_imm |= (int)(((opcode >> 15) & 0x03) << 15);
			get_imm |= (int)(((opcode >> 8) & 0x03) << 17);
			get_imm |= (int)(((opcode >> 31) & 0x01) << 19);
			if (get_imm & 0x80000)
				get_imm |= 0xFFF00000;
		}
		*p_imm = get_imm;
		return ERROR_OK;
	}
	*p_insn = i;
	*p_rs1 = ((opcode >> 15) & 0x1f);
	*p_rs2 = ((opcode >> 20) & 0x1f);
	get_imm = (int)((opcode >> 25) << 5);
	get_imm |= (int)((opcode >> 7) & 0x1f);
	if (get_imm & 0x800)
		get_imm |= 0xFFFFF000;
	*p_imm = get_imm;
	return ERROR_OK;
}

int ndsv5_get_watched_address(struct target *target)
{
	NDS_INFO("%s", __func__);
	static char pkt_decoded_data[256];
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint64_t watched_addr = 0;
	uint32_t watched_length = 0;
	struct watchpoint *wp;
	uint32_t insn_idx, insn_rd, insn_rs1, insn_rs2;
	int ret_value = 0, insn_imm;
	char *p_insn, *short_insn_reg1, *short_insn_reg2;
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	uint32_t cur_instr = 0;
	struct reg *reg_rs1;
	uint64_t reg_rs1_value = 0;
	uint64_t watchpoint_start, watchpoint_end;
	ndsv5_cur_target_xlen = (uint32_t)riscv_xlen(target);

	nds32->watched_address = 0;
	nds32->watched_length = 0;

	if (target_read_memory(target, (target_addr_t)reg_pc_value, 4, 1, (uint8_t *)&cur_instr) != ERROR_OK) {
		LOG_ERROR("can't read memory: 0x%" TARGET_PRIxADDR, reg_pc_value);
		return ERROR_FAIL;
	}

	if ((cur_instr & 0x03) != 0x03)
		goto nds_disassemble_16_insn;

	ret_value = ndsv5_disassemble_load(cur_instr, &insn_idx, &insn_rd, &insn_rs1, &insn_imm);
	if (ret_value != ERROR_OK) {
		ret_value = ndsv5_disassemble_store(cur_instr, &insn_idx, &insn_rs1, &insn_rs2, &insn_imm);
		if (ret_value == ERROR_OK) {
			p_insn = (char *)nds_insn_store[insn_idx].name;
			watched_length = nds_insn_store[insn_idx].length;
			sprintf(pkt_decoded_data, "0x%" TARGET_PRIxADDR ": %08x   %s %s,%d(%s)\n",
				reg_pc_value, cur_instr, p_insn, gpr_and_fpu_name[insn_rs2], insn_imm, gpr_and_fpu_name[insn_rs1]);
		} else {
			LOG_DEBUG("can't disassemble 32-insn: 0x%x", cur_instr);
			return ERROR_FAIL;
		}
	} else {
		p_insn = (char *)nds_insn_load[insn_idx].name;
		watched_length = nds_insn_load[insn_idx].length;
		sprintf(pkt_decoded_data, "0x%" TARGET_PRIxADDR ": %08x   %s %s,%d(%s)\n",
			reg_pc_value, cur_instr, p_insn, gpr_and_fpu_name[insn_rd], insn_imm, gpr_and_fpu_name[insn_rs1]);
	}
	NDS_INFO("%s", pkt_decoded_data);
	NDS_INFO("%s", gpr_and_fpu_name[insn_rs1]);
	reg_rs1 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[insn_rs1], 1);
	if (reg_rs1 == NULL)
		NDS_INFO("reg_rs1 == NULL");

	reg_rs1->type->get(reg_rs1);
	reg_rs1_value = buf_get_u64(reg_rs1->value, 0, reg_rs1->size);
	watched_addr = reg_rs1_value + insn_imm;
	NDS_INFO("reg_rs1_value=0x%" TARGET_PRIxADDR ", insn_imm=%d, watched_addr=0x%" TARGET_PRIxADDR,
			reg_rs1_value, insn_imm, watched_addr);
goto nds_get_watched_address;

nds_disassemble_16_insn:
	/* Compressed Instruction Formats */
	NDS_INFO("disassemble: Compressed Instruction Formats");
	cur_instr &= 0x0000FFFF;
	NDS_INFO("CUR_INSTR: 0x%x", cur_instr);
	ret_value = ndsv5_disassemble_c_load(cur_instr, &insn_idx, &insn_rd, &insn_rs1, &insn_imm);
	NDS_INFO("insn_rs1: %d", insn_rs1);
	NDS_INFO("insn_rd: %d", insn_rd);
	if (ret_value != ERROR_OK) {
		ret_value = ndsv5_disassemble_c_store(cur_instr, &insn_idx, &insn_rs1, &insn_rs2, &insn_imm);
		NDS_INFO("insn_rs1: %d", insn_rs1);
		NDS_INFO("insn_rs2: %d", insn_rs2);
		if (ret_value == ERROR_OK) {
			/* short_insn_reg1 = (insn_idx >= MATCH_ID_C_STORESP ?
			 * gpr_and_fpu_name[insn_rs1] : gp_rvc_reg_name[insn_rs1]);
			 * short_insn_reg2 = (insn_idx >= MATCH_ID_C_STORESP ?
			 * gpr_and_fpu_name[insn_rs2] : gp_rvc_reg_name[insn_rs2]);
			 */
			if (insn_idx < MATCH_ID_C_STORESP) {
				/* range: s0-a5 or x8-x15 */
				if (insn_rs1 > 7 || insn_rs2 > 7) {
					NDS_INFO("insn_rs1 and insn_rs2 cannot > 7, range : s0-a5 or x8-x15");
					NDS_INFO("insn_rs1: %d, insn_rs2: %d", insn_rs1, insn_rs2);
					return ERROR_FAIL;
				}
			}
			short_insn_reg1 = (insn_idx >= MATCH_ID_C_STORESP ?
					gpr_and_fpu_name[insn_rs1] : gpr_and_fpu_name[insn_rs1 + 8]);
			short_insn_reg2 = (insn_idx >= MATCH_ID_C_STORESP ?
					gpr_and_fpu_name[insn_rs2] : gpr_and_fpu_name[insn_rs2 + 8]);
			p_insn = (char *)nds_insn_store[insn_idx].name;
			watched_length = nds_insn_store[insn_idx].length;
			sprintf(pkt_decoded_data, "0x%" TARGET_PRIxADDR ": %08x   %s %s,%d(%s)\n",
				reg_pc_value, cur_instr, p_insn, short_insn_reg2, insn_imm, short_insn_reg1);
		}
	} else {
		/* short_insn_reg1 = (insn_idx >= MATCH_ID_C_LOADSP ?
		 * gpr_and_fpu_name[insn_rs1] : gp_rvc_reg_name[insn_rs1]);
		 * short_insn_reg2 = (insn_idx >= MATCH_ID_C_LOADSP ?
		 * gpr_and_fpu_name[insn_rd] : gp_rvc_reg_name[insn_rd]);
		 */
		if (insn_idx < MATCH_ID_C_LOADSP) {
			/* range: s0-a5 or x8-x15 */
			if (insn_rs1 > 7 || insn_rd > 7) {
				NDS_INFO("insn_rs1 and insn_rd cannot > 7, range : s0-a5 or x8-x15");
				NDS_INFO("insn_rs1: %d, insn_rd: %d", insn_rs1, insn_rd);
				return ERROR_FAIL;
			}
		}
		short_insn_reg1 = (insn_idx >= MATCH_ID_C_LOADSP ?
				gpr_and_fpu_name[insn_rs1] : gpr_and_fpu_name[insn_rs1 + 8]);
		short_insn_reg2 = (insn_idx >= MATCH_ID_C_LOADSP ?
				gpr_and_fpu_name[insn_rd] : gpr_and_fpu_name[insn_rd + 8]);
		p_insn = (char *)nds_insn_load[insn_idx].name;
		watched_length = nds_insn_load[insn_idx].length;
		sprintf(pkt_decoded_data, "0x%" TARGET_PRIxADDR ": %08x   %s %s,%d(%s)\n",
			reg_pc_value, cur_instr, p_insn, short_insn_reg2, insn_imm, short_insn_reg1);
	}

	if (ret_value != ERROR_OK) {
		NDS_INFO("can't disassemble compressed-insn: 0x%x", cur_instr);
		return ERROR_FAIL;
	} else {
		NDS_INFO("%s", pkt_decoded_data);
		reg_rs1 = register_get_by_name(target->reg_cache, short_insn_reg1, 1);
		reg_rs1->type->get(reg_rs1);
		reg_rs1_value = buf_get_u64(reg_rs1->value, 0, reg_rs1->size);
		watched_addr = reg_rs1_value + insn_imm;
		NDS_INFO("reg_rs1_value=0x%" TARGET_PRIxADDR ", insn_imm=%d, watched_addr=0x%" TARGET_PRIxADDR,
				reg_rs1_value, insn_imm, watched_addr);
	}
nds_get_watched_address:
	for (wp = target->watchpoints; wp; wp = wp->next) {
		watchpoint_start = wp->address;
		watchpoint_end = watchpoint_start + wp->length;
		if (((watchpoint_start >= (watched_addr + watched_length)) || (watchpoint_end <= watched_addr)) == false) {
			nds32->watched_address = watched_addr;
			nds32->watched_length = watched_length;
			NDS_INFO("hit watchpoints=0x%" TARGET_PRIxADDR ", watched_length=0x%x", watched_addr, watched_length);
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

/* count watchpoint number*/
int ndsv5_watchpoint_count(struct target *target)
{
	NDS_INFO("%s", __func__);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint64_t watched_addr = nds32->watched_address;
	uint32_t watched_length = nds32->watched_length;
	struct watchpoint *wp;
	int watch_count = 0;

	LOG_DEBUG("watched_addr = 0x%lx, watched_length = %d", watched_addr, watched_length);
	for (wp = target->watchpoints; wp; wp = wp->next) {
		LOG_DEBUG("wp->addr: 0x%lx, wp->length: %d", wp->address, wp->length);
		if (((wp->address >= (watched_addr + watched_length)) ||
		    ((wp->address + wp->length) <= watched_addr)) == false)
			watch_count++;

		if (watch_count > 1) {
			/* hit multi_watch */
			NDS_INFO("hit multiple watchpoints");
			nds32->watched_address = 0;
			break;
		}
	}
	LOG_DEBUG("watch_count = %d", watch_count);
	return ERROR_OK;
}

static struct watchpoint scan_all_watchpoint;
/**
 * find out which watchpoint hits
 * get exception address and compare the address to watchpoints
 */
int ndsv5_hit_watchpoint(struct target *target,
		struct watchpoint **hit_watchpoint)
{
	NDS_INFO("%s", __func__);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct watchpoint *wp;
	uint64_t watchpoint_start, watchpoint_end;

	ndsv5_watchpoint_count(target);

	/* currently HW can NOT get any info. for watched_address, SW disassemble */
	uint64_t watched_addr = nds32->watched_address;
	uint32_t watched_length = nds32->watched_length;

	LOG_DEBUG("watch_addr = 0x%lx, watched_length = %d", watched_addr, watched_length);
	if (watched_addr == 0xFFFFFFFF)
		return ERROR_FAIL;

	if (watched_addr == 0) {
		scan_all_watchpoint.address = 0;
		scan_all_watchpoint.rw = WPT_WRITE;
		scan_all_watchpoint.next = 0;
		scan_all_watchpoint.unique_id = 0x5CA8;

		*hit_watchpoint = &scan_all_watchpoint;
		return ERROR_OK;
	}

	for (wp = target->watchpoints; wp; wp = wp->next) {
		watchpoint_start = wp->address;
		watchpoint_end = watchpoint_start + wp->length;
		LOG_DEBUG("start: 0x%lx, end: 0x%lx", watchpoint_start, watchpoint_end);
		if (((watchpoint_start >= (watched_addr + watched_length)) || (watchpoint_end <= watched_addr)) == false) {
			*hit_watchpoint = wp;
			NDS_INFO("hit watchpoints=0x%" TARGET_PRIxADDR, watched_addr);
			LOG_DEBUG("hit watchpoints=0x%" TARGET_PRIxADDR, watched_addr);
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int ndsv5_get_ebreak_length(struct target *target, uint64_t reg_pc_value)
{
	uint32_t cur_instr = 0;

	if (target_read_memory(target, (target_addr_t)reg_pc_value, 4, 1, (uint8_t *)&cur_instr) != ERROR_OK) {
		LOG_ERROR("can't read memory: 0x%" TARGET_PRIxADDR, reg_pc_value);
		return 0;
	}
	NDS_INFO("%s, cur_instr=0x%x", __func__, cur_instr);
	/* MATCH_C_EBREAK 0x9002
	   MATCH_EBREAK 0x100073 */
	if ((cur_instr & MASK_EBREAK) == MATCH_EBREAK)
		return 4;
	/*
	else if ((cur_instr & MASK_C_EBREAK) == MATCH_C_EBREAK)
		return 2;
	*/
	return 2;
}

#define NDS32_STRUCT_STAT_SIZE 104	/* 60 */
#define NDS32_STRUCT_TIMEVAL_SIZE 8
static uint8_t stat_buffer[NDS32_STRUCT_STAT_SIZE];
static uint8_t timeval_buffer[NDS32_STRUCT_TIMEVAL_SIZE];

#define NDS32_STRUCT_TIMEVAL_SIZE_64 16
static uint8_t timeval_buffer_64[NDS32_STRUCT_TIMEVAL_SIZE_64];
int ndsv5_gdb_fileio_write_memory(struct target *target, target_addr_t address,
		uint32_t *psize, uint8_t **pbuffer)
{
	uint8_t *buffer = (uint8_t *)*pbuffer;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	if ((NDS_EBREAK_STAT  == nds32->active_syscall_id) ||
	    (NDS_EBREAK_FSTAT == nds32->active_syscall_id)) {
		/* If doing GDB file-I/O, target should convert 'struct stat'
		 * from gdb-format to target-format */
		/* st_dev 2 */
		stat_buffer[0] = buffer[3];
		stat_buffer[1] = buffer[2];
		/* st_ino 2 */
		stat_buffer[2] = buffer[7];
		stat_buffer[3] = buffer[6];
		/* st_mode 4 */
		stat_buffer[4] = buffer[11];
		stat_buffer[5] = buffer[10];
		stat_buffer[6] = buffer[9];
		stat_buffer[7] = buffer[8];
		/* st_nlink 2 */
		stat_buffer[8] = buffer[15];
		stat_buffer[9] = buffer[16];
		/* st_uid 2 */
		stat_buffer[10] = buffer[19];
		stat_buffer[11] = buffer[18];
		/* st_gid 2 */
		stat_buffer[12] = buffer[23];
		stat_buffer[13] = buffer[22];
		/* st_rdev 2 */
		stat_buffer[14] = buffer[27];
		stat_buffer[15] = buffer[26];

		if (riscv_xlen(target) == 64) {
			/* st_size 8 */
			stat_buffer[16] = buffer[35];
			stat_buffer[17] = buffer[34];
			stat_buffer[18] = buffer[33];
			stat_buffer[19] = buffer[32];
			stat_buffer[20] = 0;
			stat_buffer[21] = 0;
			stat_buffer[22] = 0;
			stat_buffer[23] = 0;
			/* st_atime 8 */
			stat_buffer[24] = buffer[55];
			stat_buffer[25] = buffer[54];
			stat_buffer[26] = buffer[53];
			stat_buffer[27] = buffer[52];
			stat_buffer[28] = 0;
			stat_buffer[29] = 0;
			stat_buffer[30] = 0;
			stat_buffer[31] = 0;
			/* st_spare1 8 */
			stat_buffer[32] = 0;
			stat_buffer[33] = 0;
			stat_buffer[34] = 0;
			stat_buffer[35] = 0;
			stat_buffer[36] = 0;
			stat_buffer[37] = 0;
			stat_buffer[38] = 0;
			stat_buffer[39] = 0;
			/* st_mtime 8 */
			stat_buffer[40] = buffer[59];
			stat_buffer[41] = buffer[58];
			stat_buffer[42] = buffer[57];
			stat_buffer[43] = buffer[56];
			stat_buffer[44] = 0;
			stat_buffer[45] = 0;
			stat_buffer[46] = 0;
			stat_buffer[47] = 0;
			/* st_spare2 8 */
			stat_buffer[48] = 0;
			stat_buffer[49] = 0;
			stat_buffer[50] = 0;
			stat_buffer[51] = 0;
			stat_buffer[52] = 0;
			stat_buffer[53] = 0;
			stat_buffer[54] = 0;
			stat_buffer[55] = 0;
			/* st_ctime 8 */
			stat_buffer[56] = buffer[63];
			stat_buffer[57] = buffer[62];
			stat_buffer[58] = buffer[61];
			stat_buffer[59] = buffer[60];
			stat_buffer[60] = 0;
			stat_buffer[61] = 0;
			stat_buffer[62] = 0;
			stat_buffer[63] = 0;
			/* st_spare3 8 */
			stat_buffer[64] = 0;
			stat_buffer[65] = 0;
			stat_buffer[66] = 0;
			stat_buffer[67] = 0;
			stat_buffer[68] = 0;
			stat_buffer[69] = 0;
			stat_buffer[70] = 0;
			stat_buffer[71] = 0;
			/* st_blksize 8 */
			stat_buffer[72] = buffer[43];
			stat_buffer[73] = buffer[42];
			stat_buffer[74] = buffer[41];
			stat_buffer[75] = buffer[40];
			stat_buffer[76] = 0;
			stat_buffer[77] = 0;
			stat_buffer[78] = 0;
			stat_buffer[79] = 0;
			/* st_blocks 8 */
			stat_buffer[80] = buffer[51];
			stat_buffer[81] = buffer[50];
			stat_buffer[82] = buffer[49];
			stat_buffer[83] = buffer[48];
			stat_buffer[84] = 0;
			stat_buffer[85] = 0;
			stat_buffer[86] = 0;
			stat_buffer[87] = 0;
			/* st_spare4[0] 8 */
			stat_buffer[88] = 0;
			stat_buffer[89] = 0;
			stat_buffer[90] = 0;
			stat_buffer[91] = 0;
			stat_buffer[92] = 0;
			stat_buffer[93] = 0;
			stat_buffer[94] = 0;
			stat_buffer[95] = 0;
			/* st_spare4[1] 8 */
			stat_buffer[96] = 0;
			stat_buffer[97] = 0;
			stat_buffer[98] = 0;
			stat_buffer[99] = 0;
			stat_buffer[100] = 0;
			stat_buffer[101] = 0;
			stat_buffer[102] = 0;
			stat_buffer[103] = 0;
			*psize = 104;
		} else {
			/* st_size 4 */
			stat_buffer[16] = buffer[35];
			stat_buffer[17] = buffer[34];
			stat_buffer[18] = buffer[33];
			stat_buffer[19] = buffer[32];
			/* st_atime 4 */
			stat_buffer[20] = buffer[55];
			stat_buffer[21] = buffer[54];
			stat_buffer[22] = buffer[53];
			stat_buffer[23] = buffer[52];
			/* st_spare1 4 */
			stat_buffer[24] = 0;
			stat_buffer[25] = 0;
			stat_buffer[26] = 0;
			stat_buffer[27] = 0;
			/* st_mtime 4 */
			stat_buffer[28] = buffer[59];
			stat_buffer[29] = buffer[58];
			stat_buffer[30] = buffer[57];
			stat_buffer[31] = buffer[56];
			/* st_spare2 4 */
			stat_buffer[32] = 0;
			stat_buffer[33] = 0;
			stat_buffer[34] = 0;
			stat_buffer[35] = 0;
			/* st_ctime 4 */
			stat_buffer[36] = buffer[63];
			stat_buffer[37] = buffer[62];
			stat_buffer[38] = buffer[61];
			stat_buffer[39] = buffer[60];
			/* st_spare3 4 */
			stat_buffer[40] = 0;
			stat_buffer[41] = 0;
			stat_buffer[42] = 0;
			stat_buffer[43] = 0;
			/* st_blksize 4 */
			stat_buffer[44] = buffer[43];
			stat_buffer[45] = buffer[42];
			stat_buffer[46] = buffer[41];
			stat_buffer[47] = buffer[40];
			/* st_blocks 4 */
			stat_buffer[48] = buffer[51];
			stat_buffer[49] = buffer[50];
			stat_buffer[50] = buffer[49];
			stat_buffer[51] = buffer[48];
			/* st_spare4 8 */
			stat_buffer[52] = 0;
			stat_buffer[53] = 0;
			stat_buffer[54] = 0;
			stat_buffer[55] = 0;
			stat_buffer[56] = 0;
			stat_buffer[57] = 0;
			stat_buffer[58] = 0;
			stat_buffer[59] = 0;
			*psize = 60;
		}
		*pbuffer = (uint8_t *) &stat_buffer[0];
	} else if (NDS_EBREAK_GETTIMEOFDAY == nds32->active_syscall_id) {
		/* If doing GDB file-I/O, target should convert 'struct timeval'
		 * from gdb-format to target-format */
		if (riscv_xlen(target) == 64) {
			timeval_buffer_64[0] = buffer[3];
			timeval_buffer_64[1] = buffer[2];
			timeval_buffer_64[2] = buffer[1];
			timeval_buffer_64[3] = buffer[0];
			timeval_buffer_64[4] = 0;
			timeval_buffer_64[5] = 0;
			timeval_buffer_64[6] = 0;
			timeval_buffer_64[7] = 0;

			timeval_buffer_64[8]  = buffer[11];
			timeval_buffer_64[9]  = buffer[10];
			timeval_buffer_64[10] = buffer[9];
			timeval_buffer_64[11] = buffer[8];
			timeval_buffer_64[12] = buffer[7];
			timeval_buffer_64[13] = buffer[6];
			timeval_buffer_64[14] = buffer[5];
			timeval_buffer_64[15] = buffer[4];

			*psize = NDS32_STRUCT_TIMEVAL_SIZE_64;
			*pbuffer = (uint8_t *) &timeval_buffer_64[0];
		} else {
			timeval_buffer[0] = buffer[3];
			timeval_buffer[1] = buffer[2];
			timeval_buffer[2] = buffer[1];
			timeval_buffer[3] = buffer[0];
			timeval_buffer[4] = buffer[11];
			timeval_buffer[5] = buffer[10];
			timeval_buffer[6] = buffer[9];
			timeval_buffer[7] = buffer[8];

			*psize = NDS32_STRUCT_TIMEVAL_SIZE;
			*pbuffer = (uint8_t *) &timeval_buffer[0];
		}
	}

	return ERROR_OK;
}

#define DIM(x)		(sizeof(x)/sizeof(*x))
void ndsv5_decode_progbuf(char *text, uint32_t cur_instr)
{
	static const struct {
		uint32_t match;
		uint32_t mask;
		const char *name;
	} description[] = {
		{ MATCH_FENCE, MASK_FENCE, "fence" },
		{ MATCH_FENCE_I, MASK_FENCE_I, "fencei" },
		{ MATCH_EBREAK, MASK_EBREAK, "ebreak" },
		{ MATCH_ADDI, MASK_ADDI, "addi" },
		{ MATCH_AUIPC, MASK_AUIPC, "auipc" },
		{ MATCH_CSRRW, MASK_CSRRW, "csrrw" },
		{ MATCH_CSRRS, MASK_CSRRS, "csrrs" },
	};

	for (uint32_t i = 0; i < DIM(description); i++) {
		if ((cur_instr & description[i].mask) == description[i].match) {
				strcpy(text, description[i].name);
				return;
		}
	}
	uint32_t insn_idx, insn_rd, insn_rs1, insn_rs2;
	int ret_value = 0, insn_imm;
	char *p_insn;

	ret_value = ndsv5_disassemble_load(cur_instr, &insn_idx, &insn_rd, &insn_rs1, &insn_imm);
	if (ret_value != ERROR_OK) {
		ret_value = ndsv5_disassemble_store(cur_instr, &insn_idx, &insn_rs1, &insn_rs2, &insn_imm);
		if (ret_value == ERROR_OK) {
			p_insn = (char *)nds_insn_store[insn_idx].name;
			/*
			watched_length = nds_insn_store[insn_idx].length;
			*/
			sprintf(text, " %s %s,%d(%s)",
				p_insn, gpr_and_fpu_name[insn_rs2], insn_imm, gpr_and_fpu_name[insn_rs1]);
		} else {
			static const struct {
				uint32_t match;
				uint32_t mask;
				const char *name;
			} vector_description[] = {
				{ MATCH_VMV_S_X, MASK_VMV_S_X, "vmv.s.x" },
				{ MATCH_VMV_X_S, MASK_VMV_X_S, "vmv.x.s" },
				{ MATCH_VSLIDE1DOWN_VX, MASK_VSLIDE1DOWN_VX, "vslide1down.vx" },
				{ MATCH_VSETVLI, MASK_VSETVLI, "vsetvli" },
				{ MATCH_VSETVL, MASK_VSETVL, "vsetvl" }
			};
			for (uint32_t i = 0; i < DIM(vector_description); i++) {
				if ((cur_instr & vector_description[i].mask) == vector_description[i].match) {
					switch (i) {
						case 0:
							/* vmv.s.x vd, rs1 */
							sprintf(text, " %s v%d, %s",
								vector_description[i].name,
								bits(cur_instr, 11, 7),
								gpr_and_fpu_name[bits(cur_instr, 19, 15)]);
							break;
						case 1:
							/* vmv.x.s rd, vs2 */
							sprintf(text, " %s %s, v%d",
								vector_description[i].name,
								gpr_and_fpu_name[bits(cur_instr, 11, 7)],
								bits(cur_instr, 24, 20));
							break;
						case 2:
							/* vslide1down.vx vd, vs2, rs1, vm */
							sprintf(text, " %s v%d, v%d, %s, %d",
								vector_description[i].name,
								bits(cur_instr, 11, 7),
								bits(cur_instr, 24, 20),
								gpr_and_fpu_name[bits(cur_instr, 19, 15)],
								bit(cur_instr, 25));
							break;
						case 3:
							/* vsetvli rd, rs1, vtypei */
							sprintf(text, "%s %s, %s, 0x%x",
								vector_description[i].name,
								gpr_and_fpu_name[bits(cur_instr, 11, 7)],
								gpr_and_fpu_name[bits(cur_instr, 19, 15)],
								bits(cur_instr, 30, 20));
							break;
						case 4:/* vsetvl rd, rs1, rs2 */
							sprintf(text, "%s %s, %s, %s",
								vector_description[i].name,
								gpr_and_fpu_name[bits(cur_instr, 11, 7)],
								gpr_and_fpu_name[bits(cur_instr, 19, 15)],
								gpr_and_fpu_name[bits(cur_instr, 24, 20)]);
							break;
					}
					return;
				}
			}
			LOG_ERROR("can't disassemble 32-insn: 0x%x", cur_instr);
			return;
		}
	} else {
		p_insn = (char *)nds_insn_load[insn_idx].name;
		/*
		watched_length = nds_insn_load[insn_idx].length;
		*/
		sprintf(text, " %s %s,%d(%s)",
			p_insn, gpr_and_fpu_name[insn_rd], insn_imm, gpr_and_fpu_name[insn_rs1]);
	}
	/*
	NDS_INFO("%s", text);
	*/
}

__COMMAND_HANDLER(handle_ndsv5_tracer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	LOG_DEBUG("NDS tracer on [%s] hart %d", target->tap->dotted_name, target->coreid);

	if (nds32 == NULL) {
		LOG_ERROR("gpnds32_v5 is NULL");
		return ERROR_FAIL;
	}

	if (strcmp(CMD_ARGV[0], "on") == 0) {
		LOG_DEBUG("trace on");
		nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_OFF;
		nds_tracer_stop_on_wrap = 1;
		ndsv5_tracer_all_cores_setting();
	} else if (strcmp(CMD_ARGV[0], "off") == 0) {
		LOG_DEBUG("trace off");
		ndsv5_tracer_disable(target);
	} else if (strcmp(CMD_ARGV[0], "to") == 0) {
		LOG_DEBUG("trace to");
		nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_OFF;
		nds_tracer_stop_on_wrap = 0;
		ndsv5_tracer_all_cores_setting();
	} else if (strcmp(CMD_ARGV[0], "from") == 0) {
		LOG_DEBUG("trace from");
		/* nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_ON; */
		nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_OFF;
		nds_tracer_stop_on_wrap = 1;
		ndsv5_tracer_all_cores_setting();
	} else if (strcmp(CMD_ARGV[0], "dump-trace-file") == 0) {
		LOG_DEBUG("trace dump-trace-file %s", CMD_ARGV[1]);
		ndsv5_tracer_dumpfile(target, (char *)CMD_ARGV[1]);
	} else if (strcmp(CMD_ARGV[0], "decode-trace-file") == 0) {
		LOG_DEBUG("trace decode-trace-file %s", CMD_ARGV[1]);
		ndsv5_tracer_decode_pktfile((char *)CMD_ARGV[1]);
	} else if (strcmp(CMD_ARGV[0], "sync-period") == 0) {
		unsigned int sync_period = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], sync_period);
		LOG_DEBUG("sync-period 0x%x", sync_period);
		nds_trTeSyncMax = sync_period;
	} else if ((strcmp(CMD_ARGV[0], "src-field") == 0) && (CMD_ARGC > 1)) {
		unsigned int IfSrcBit = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], IfSrcBit);
		LOG_DEBUG("src-field 0x%x", IfSrcBit);
		if (IfSrcBit)
			nds_teInhibitSrc = 0;
		else
			nds_teInhibitSrc = 1;
	} else if ((strcmp(CMD_ARGV[0], "trace-hart") == 0) && (CMD_ARGC > 1)) {
		unsigned long active_src = 0;
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], active_src);
		LOG_DEBUG("trace-hart 0x%lx", active_src);
		nds_tracer_active_id = active_src;
	} else if ((strcmp(CMD_ARGV[0], "inst-mode") == 0) && (CMD_ARGC > 1)) {
		unsigned int InstMode = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], InstMode);
		LOG_DEBUG("inst-mode 0x%x", InstMode);
		nds_trTeInstMode = InstMode;
	} else if ((strcmp(CMD_ARGV[0], "match-inst") == 0) && (CMD_ARGC > 1)) {
		unsigned int matchinst = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], matchinst);
		LOG_DEBUG("match-inst 0x%x", matchinst);
		nds_trTeFilteriMatchInst = matchinst;
	} else if (strcmp(CMD_ARGV[0], "timestamp") == 0) {
		unsigned int timestamp_on = 0;
		if (CMD_ARGC > 1) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], timestamp_on);
			LOG_DEBUG("timestamp_on = 0x%x", timestamp_on);
		}

		unsigned int trTsControl = 0;
		if (CMD_ARGC > 2) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], trTsControl);
			LOG_DEBUG("trTsControl = 0x%x", trTsControl);
		}
		LOG_DEBUG("timestamp 0x%x", trTsControl);
		nds_trTsControl = trTsControl;
		nds_timestamp_on = timestamp_on;
	} else {
		command_print(CMD, "NDS tracer command ERROR");
	}
	return ERROR_OK;
}

