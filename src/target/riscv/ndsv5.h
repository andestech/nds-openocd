/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifndef __NDSV5_H_
#define __NDSV5_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "target/target_type.h"
#include "target/ndsv5_ace.h"
#include "opcodes.h"
#include "gdb_regs.h"
#include "encoding.h"
#include "riscv.h"


/********************************************************************/
/* NDSV5 compiler flags */
/********************************************************************/
#define _NDS_DMI_CHECK_TIMEOUT_ 1
#define _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF 1
#define _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF 1
#define _NDS_JTAG_SCANS_OPTIMIZE_ 1
#define _NDS_MEM_Q_ACCESS_ 1
#define _NDS_DISABLE_ABORT_ 1
#define _NDS_IDE_MESSAGE_ 1
#define _NDS_USE_SCRIPT_ 1
#define _NDS_BATCH_RUN_RETRY_ 1
#define _NDS_SUPPORT_WITHOUT_ANNOUNCING_ 1


/********************************************************************/


/*
// gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
// its source tree. We must interpret the numbers the same here.
enum {
	REG_XPR0 = 0,				// GDB_REGNO_ZERO
	REG_XPR31 = 31,				// GDB_REGNO_XPR31
	REG_PC = 32,				// GDB_REGNO_PC
	REG_FPR0 = 33,				// GDB_REGNO_FPR0
	REG_FPR31 = 64,				// GDB_REGNO_FPR31
	REG_CSR0 = 65,				// GDB_REGNO_CSR0
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,	// GDB_REGNO_MSTATUS
	REG_CSR4095 = 4160,			// GDB_REGNO_CSR4095
	REG_PRIV = 4161,			// GDB_REGNO_PRIV
	REG_V0,					// GDB_REGNO_V0
	REG_V31 = REG_V0 + 31,			// GDB_REGNO_V31
	REG_ALL_COUNT				// GDB_REGNO_COUNT
};
*/

enum nds_memory_access {
	NDS_MEMORY_ACC_BUS = 0,
	NDS_MEMORY_ACC_CPU,
};

extern char **gpr_and_fpu_name;




/********************************************************************/
/* Extern function/var. from riscv.c                                */
/********************************************************************/
extern struct reg_arch_type riscv_reg_arch_type;
extern int old_or_new_riscv_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints);
extern int riscv_select_current_hart(struct target *target);
extern int old_or_new_riscv_poll(struct target *target);
extern int riscv_examine(struct target *target);
/********************************************************************/




/********************************************************************/
/* NDSV5 flags */
/********************************************************************/
uint32_t ndsv5_use_mprv_mode;
uint32_t nds_skip_dmi;
uint32_t ndsv5_system_bus_access;
uint32_t ndsv5_without_announce;
uint32_t ndsv5_dmi_abstractcs;
uint32_t ndsv5_byte_access_from_burn;

#if _NDS_MEM_Q_ACCESS_
uint32_t nds_dmi_quick_access;
uint32_t nds_dmi_abstractcs;
uint32_t nds_dmi_quick_access_ena;
#endif /* _NDS_MEM_Q_ACCESS_ */
/********************************************************************/




/********************************************************************/
/* NDSV5 global Var. */
/********************************************************************/
int ndsv5_triggered_hart;
extern uint64_t LM_BASE;
extern uint32_t ndsv5_dis_cache_busmode;
extern uint32_t ndsv5_dmi_busy_retry_times;
extern unsigned* global_acr_reg_count_v5;
extern unsigned* global_acr_type_count_v5;
extern ACR_INFO_T_V5 *acr_info_list_v5;
extern struct reg_arch_type nds_ace_reg_access_type;
extern int nds_targetburn_corenum;

uint64_t ndsv5_ilm_bpa, ndsv5_ilm_lmsz;
uint64_t ndsv5_dlm_bpa, ndsv5_dlm_lmsz;
uint32_t ndsv5_ilm_ena, ndsv5_dlm_ena;
uint32_t ndsv5_local_memory_slave_port;
uint32_t ndsv5_check_idlm_capability_before;
uint64_t ndsv5_backup_mstatus;
/********************************************************************/



#define REG_DDCAUSE_MASK    0xFF
#define REG_DDCAUSE_EBREAK  0  /* Software Breakpoint (EBREAK)       */
#define REG_DDCAUSE_IAM     1  /* Instruction Access Misaligned(IAM) */
#define REG_DDCAUSE_IAF     2  /* Instruction Access Fault (IAF)     */
#define REG_DDCAUSE_II      3  /* Illegal Instruction (II)           */
#define REG_DDCAUSE_NMI     4  /* Non-Maskable Interrupt (NMI)       */
#define REG_DDCAUSE_LAM     5  /* Load Access Misaligned (LAM)       */
#define REG_DDCAUSE_LAF     6  /* Load Access Fault (LAF)            */
#define REG_DDCAUSE_SAM     7  /* Store Access Misaligned (SAM)      */
#define REG_DDCAUSE_SAF     8  /* Store Access Fault (SAF)           */
#define REG_DDCAUSE_UEC     9  /* U-mode Environment Call (UEC)      */
#define REG_DDCAUSE_SEC     10 /* S-mode Environment Call (SEC)      */
#define REG_DDCAUSE_HEC     11 /* H-mode Environment Call (HEC)      */
#define REG_DDCAUSE_MEC     12 /* M-mode Environment Call (MEC)      */

#define REG_DDCAUSE_II_II           0  /* Illegal instruction    */
#define REG_DDCAUSE_II_PRIV_INST    1  /* Privileged instruction */
#define REG_DDCAUSE_II_NONEXIT_CSR  2  /* Non-existent CSR       */
#define REG_DDCAUSE_II_RO_CSR       3  /* Read-only CSR update   */
#define REG_DDCAUSE_II_PRIV_CSR     4  /* Privileged CSR access  */

/* for profiling */
#define NDS32_MAX_PROFILE_SAMPLES    (512u)
#define NDS32_MAX_PROFILE_ITERATIONS ((1u << 16) - 1)

#define CHECK_RETVAL(action)			\
	do {					\
		int __retval = (action);	\
		if (__retval != ERROR_OK) {	\
			LOG_DEBUG("error while calling \"%s\"",	\
				# action);     \
			return __retval;	\
		}				\
	} while (0)


enum target_property {
	PROPERTY_RUN_MODE,
	PROPERTY_PROF_ADDR_MIN,
	PROPERTY_PROF_ADDR_MAX,
	PROPERTY_PROF_WITH_RANGE,
	PROPERTY_CAN_PROFILE,
	PROPERTY_MAX,
};

enum target_bench {
	BENCH_ITEM_PROFILE_RATE,
	BENCH_ITEM_MAX,
};

enum target_run_mode {
	RUN_MODE_DEBUG = 0x00,
	RUN_MODE_PROFILE = 0x01,
	RUN_MODE_PWR_MONITOR = 0x02,
	RUN_MODE_MAX,
};

#define NDS_EBREAK_NUMS 11
enum nds32_syscall_id {
	NDS_EBREAK_UNDEFINED    = 0,
	NDS_EBREAK_OPEN         = 0x400,
	NDS_EBREAK_LSEEK        = 0x3E,
	NDS_EBREAK_READ         = 0x3F,
	NDS_EBREAK_WRITE        = 0x40,
	NDS_EBREAK_FSTAT        = 0x50,
	NDS_EBREAK_STAT         = 0x40E,
	NDS_EBREAK_CLOSE        = 0x39,
	/* NDS_EBREAK_LINK      = 0x401, not supported by VH */
	NDS_EBREAK_UNLINK       = 0x402,
	NDS_EBREAK_GETTIMEOFDAY = 0xA9,
	NDS_EBREAK_EXIT         = 0x5D,
	NDS_EBREAK_RENAME       = 0x40A, /* if compiled with -DHAVE_RENAME */

	/*
	NDS_EBREAK_OPENNAT      = 0x38,  ,defined but rarely used
	NDS_EBREAK_LSTAT        = 0x40F, ,defined but rarely used
	NDS_EBREAK_FSTATAT      = 0x4F,  ,defined but rarely used
	NDS_EBREAK_ACCESS       = 0x409, ,defined but rarely used
	NDS_EBREAK_FACCESSAT    = 0x30,  ,defined but rarely used
	NDS_EBREAK_SKIP_BREAK   = 0x7E00,
	NDS_EBREAK_ISATTY       = 0x7F2B,
	NDS_EBREAK_SYSTEM       = 0x7F2C,
	NDS_EBREAK_ERRNO        = 0x7F2D,
	*/
};

struct nds32_v5_cache {
	bool enable;

	uint64_t set;
	uint64_t way;
	uint64_t line_size;

	uint64_t log2_set;
	uint64_t log2_line_size;
};

struct nds32_v5_memory {
	/** ICache */
	struct nds32_v5_cache icache;

	/** DCache */
	struct nds32_v5_cache dcache;

	/** Memory access method */
	enum nds_memory_access  access_channel;
};

enum cache_t {
	ICACHE,
	DCACHE
};

struct nds32_v5 {
	int common_magic;

	/* next nds32_v5 in list */
	struct nds32_v5 *next;

	/** Backpointer to the target. */
	struct target *target;
	/* void *arch_info; */

	/** Memory information */
	struct nds32_v5_memory memory;

	/** Flag to indicate if auto convert software breakpoints to
	 *  hardware breakpoints or not in ROM */
	bool auto_convert_hw_bp;

	/* Flag to indicate the target is attached by debugger or not */
	bool attached;
	bool is_program_exit;

	/* Flag to indicate the target is attached by target_burn or not */
	bool target_burn_attached;

	/** Flag reporting whether continue/step hits syscall or not */
	bool hit_syscall;

	/** Value to be returned by virtual hosting SYS_ERRNO request. */
	int virtual_hosting_errno;

	/** Flag reporting whether syscall is aborted */
	bool virtual_hosting_ctrl_c;

	/** Record syscall ID for other operations to do special processing for target */
	uint32_t active_syscall_id;

	/** gdb run mode */
	enum target_run_mode gdb_run_mode;
	bool gdb_run_mode_acting;
	bool gdb_run_mode_halt;

	/** profiling data */
	uint32_t prof_num_request;                        /* number samples expected */
	uint32_t prof_num_samples;                        /* number samples in prof_samples[] */
	uint64_t prof_samples[NDS32_MAX_PROFILE_SAMPLES]; /* sample data */
	uint32_t prof_total_samples;                      /* accumulate in buckets */
	uint32_t prof_num_buckets;
	uint32_t prof_sample_threshold;
	uint32_t *prof_buckets;

	/** profiling parameters */
	bool is_in_profiling;
	bool prof_with_range;
	uint64_t prof_addr_min;
	uint64_t prof_addr_max;
	uint32_t gmon_64_bit;
	uint32_t profiling_support;

	/** ACE conf file name if given */
	const char *aceconf;

	uint64_t watched_address;
	uint32_t watched_length;

	uint32_t ddcause_maintype;

	/** Period to wait after SRST. */
	uint32_t boot_time;

	/** Period to wait after RESTART. */
	uint32_t reset_time;

	/** reset-halt as target examine */
	bool reset_halt_as_examine;

	/** original global variable */
	uint32_t nds_va_to_pa_off;

	uint32_t nds_const_addr_mode;

	/** register initial */
	bool execute_register_init;

	uint32_t nds_vector_length;
	uint32_t nds_vector_SEW;
	uint32_t nds_vector_vl;
};

extern FILE *nds_script_custom_reset;
extern FILE *nds_script_custom_reset_halt;
extern FILE *nds_script_custom_initial;

extern struct target_type *get_target_type(struct target *target);
/* when polling target halted, do NOT announce gdb */
extern int ndsv5_poll_wo_announce(struct target *target);

extern struct nds32_v5 *target_to_nds32_v5(struct target *target);
extern int ndsv5_target_create(struct target *target, Jim_Interp *interp);
extern int ndsv5_handle_poll(struct target *target);
extern int ndsv5_handle_halt(struct target *target);
extern int ndsv5_resume_check(struct target *target);
extern int ndsv5_handle_resume(struct target *target);
extern int ndsv5_step_check(struct target *target);
extern int ndsv5_handle_step(struct target *target);
extern int ndsv5_handle_examine(struct target *target);
extern int ndsv5_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

extern int ndsv5_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info);
extern int ndsv5_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c);
extern int ndsv5_watchpoint_count(struct target *target);
extern int ndsv5_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint);
extern int ndsv5_virtual_hosting_check(struct target *target);
extern int ndsv5_hit_watchpoint_check(struct target *target);
extern int ndsv5_get_watched_address(struct target *target);
extern int ndsv5_get_ebreak_length(struct target *target, uint64_t reg_pc_value);
extern char *ndsv5_get_CSR_name(struct target *target, uint32_t csr_id);
extern int ndsv5_reset_target(struct target *target, enum target_reset_mode reset_mode);

extern int ndsv5_script_do_custom_reset(struct target *target, FILE *script_fd);
extern int ndsv5_script_dmi_read(uint16_t address, uint64_t *dmi_read_data);
extern int ndsv5_script_dmi_write(uint16_t address, uint64_t value);
extern int ndsv5_script_reg_read(uint64_t *value, uint32_t number);
extern int ndsv5_script_reg_write(unsigned number, uint64_t value);

extern int ndsv5_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
extern int ndsv5_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
extern int ndsv5_readwrite_byte(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer, bool if_write);
extern int ndsv5_write_buffer(struct target *target, target_addr_t address, uint32_t writesize, const uint8_t *buffer);

extern int ndsv5_init_cache(struct target *target);
extern int ndsv5_dump_cache(struct target *target, unsigned int cache_type, const char* filename);
extern int ndsv5_dump_cache_va(struct target *target, unsigned int cache_type, uint64_t va);
extern int ndsv5_enableornot_cache(struct target *target, unsigned int cache_type, const char* enableornot);
extern int ndsv5_dcache_wb(struct target *target);
extern int ndsv5_dcache_invalidate(struct target *target);
extern int ndsv5_dcache_wb_invalidate(struct target *target);
extern int ndsv5_icache_invalidate(struct target *target);
//extern int ndsv5_redefine_CSR_name(struct target *target);
//extern int ndsv5_redefine_GPR_FPU_name(struct target *target);

#define NDSV5_COMMON_MAGIC (int)0xADE55555

int ndsv5_step(struct target *target, int current, target_addr_t address, int handle_breakpoints);
int strict_step(struct target *target, bool announce);
int ndsv5_handle_triggered(struct target *target);
int ndsv5_poll(struct target *target);
int ndsv5_halt(struct target *target);
int ndsv5_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution);
int ndsv5_examine(struct target *target);
int modify_trigger_address_mbit_match(struct target *target, struct trigger *trigger);
int ndsv5_writebuffer(struct target *target, target_addr_t address,
		uint32_t writesize, const uint8_t *buffer);
int ndsv5_get_physical_address(struct target *target, target_addr_t addr, target_addr_t *physical);
int ndsv5_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical);
void bus_mode_on(struct target *target, uint64_t *reg_value_backup);
void bus_mode_off(struct target *target, uint64_t reg_value);
struct reg *ndsv5_get_reg_by_CSR(struct target *target, uint32_t csr_id);
uint64_t ndsv5_get_register_value(struct reg *reg);
void ndsv5_set_register_value(struct reg *reg, uint64_t reg_value);
const char *ndsv5_get_gdb_arch(struct target *target);

int ndsv5_openocd_poll_one_hart(struct target *target, int hartid);
int ndsv5_openocd_halt_one_hart(struct target *target, int hartid);
int ndsv5_openocd_resume_one_hart(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		int hartid
);
int ndsv5_lm_slvp_support(struct target *target, target_addr_t address, uint32_t csr_id_lmb);
char *ndsv5_base64_decode(const char *data, int input_length, int *output_length);





/********************************************************************/
/* NDSv5 program functions                                          */
/********************************************************************/
int riscv_program_lui(struct riscv_program *p, enum gdb_regno d, int32_t u);
int riscv_program_slli(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int32_t u);
int riscv_program_or(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s1, enum gdb_regno s2);
int riscv_program_bfoz64(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, uint8_t msb, uint8_t lsb);
int riscv_program_li(struct riscv_program *p, enum gdb_regno d, riscv_reg_t c);
int riscv_program_li64(struct riscv_program *p, enum gdb_regno d, enum gdb_regno t, riscv_reg_t c);
int riscv_program_vsetvl(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno rs2);
int riscv_program_vsetvli(struct riscv_program *p, enum gdb_regno rd, uint32_t SEW);
int riscv_program_vmv_x_s(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2);
int riscv_program_vslide1down_vx(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2, enum gdb_regno rs1);
/********************************************************************/

#endif /* __NDSV5_H_ */
