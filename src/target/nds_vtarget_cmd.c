#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "register.h"
#include "nds_vtarget.h"
#include "breakpoints.h"
#include "nds_vtarget_common.h"
#include "jtag/interface.h"
#include "jtag/interfaces.h"
#include "nds32_log.h"

/** prototype **/
/** other functions **/
static int vtarget_log_callback(void *priv);

static void vtarget_reset_buffer_access_size(void);
static int vtarget_set_buffer_access_size(uint64_t start_addr, uint64_t end_addr,
uint32_t access_size);
extern int vtarget_get_buffer_access_size(uint64_t start_addr, uint32_t bufsize,
		uint32_t *pcurr_size, uint32_t *paccess_size);

/** handle functions **/


/* global variable */
uint32_t vtarget_bak_debug_buf_size;
uint32_t vtarget_dmi_busy_retry_times = 100;
uint32_t vtarget_reg_symbolic_name;
uint32_t vtarget_jtag_scans_optimize;
uint32_t vtarget_dmi_busy_delay_count;
uint32_t vtarget_xlen = 32;
struct nds_vtarget *gpnds_vtarget;

static const char *const NDS_MEMORY_ACCESS_NAME[] = {
	"BUS",
	"CPU",
};

static unsigned int vtarget_MaxLogFileSize	= 0xA00000; /* default: 10MB */

extern char *log_output_path;
/* global command context from openocd.c */
extern struct command_context *global_cmd_ctx;
uint32_t vtarget_sys_bus_supported;

/* from vtarget.c */
extern uint32_t vtarget_force_aligned_access;
extern char *gpBitFieldFileNameVtarget;
extern uint32_t vtarget_system_bus_access;

/* from log.c */
extern char *p_nds_bak_debug_buffer_cur;
extern char *p_nds_bak_debug_buffer_start;
extern char *p_nds_bak_debug_buffer_end;
extern uint32_t nds_bak_debug_file_nums;


/*
core.c v5_cmd.c       : scan_retry_times
ftdi.c                : jtag_max_scans
*/
/* from v5_cmd.c */
extern uint32_t nds_jtag_max_scans;
extern uint32_t nds_ftdi_devices;
extern uint32_t nds_scan_retry_times;

static FILE *pLogFile;
static unsigned int LogFileIdx;
static const char *Log_File_Name[2] = {
	"iceman_debug0.log",
	"iceman_debug1.log",
};

struct nds_csr_reg {
	uint32_t csr_id;
	const char *name;
};

static struct nds_csr_reg nds_all_csr[] = {
	/*
	{ 0, "csr0"},
	{ 1, "csr1"},
	*/
	{ CSR_MVENDORID, "mvendorid"},
	{ CSR_MARCHID, "marchid"},
	{ CSR_MIMPID, "mimpid"},
	{ CSR_MHARTID, "mhartid"},
	{ CSR_MSTATUS, "mstatus"},
	{ CSR_MISA, "misa"},
	{ CSR_MEDELEG, "medeleg"},
	{ CSR_MIDELEG, "mideleg"},
	{ CSR_MIE, "mie"},
	{ CSR_MTVEC, "mtvec"},
	{ CSR_MSCRATCH, "mscratch"},
	{ CSR_MEPC, "mepc"},
	{ CSR_MCAUSE, "mcause"},
	{ CSR_MTVAL, "mtval"},
	{ CSR_MIP, "mip"},

	{ CSR_PMPCFG0, "pmpcfg0"},
	{ CSR_PMPCFG1, "pmpcfg1"},
	{ CSR_PMPCFG2, "pmpcfg2"},
	{ CSR_PMPCFG3, "pmpcfg3"},
	{ CSR_PMPCFG4, "pmpcfg4"},
	{ CSR_PMPCFG5, "pmpcfg5"},
	{ CSR_PMPCFG6, "pmpcfg6"},
	{ CSR_PMPCFG7, "pmpcfg7"},
	{ CSR_PMPCFG8, "pmpcfg8"},
	{ CSR_PMPCFG9, "pmpcfg9"},
	{ CSR_PMPCFG10, "pmpcfg10"},
	{ CSR_PMPCFG11, "pmpcfg11"},
	{ CSR_PMPCFG12, "pmpcfg12"},
	{ CSR_PMPCFG13, "pmpcfg13"},
	{ CSR_PMPCFG14, "pmpcfg14"},
	{ CSR_PMPCFG15, "pmpcfg15"},
	{ CSR_PMPADDR0, "pmpaddr0"},
	{ CSR_PMPADDR1, "pmpaddr1"},
	{ CSR_PMPADDR2, "pmpaddr2"},
	{ CSR_PMPADDR3, "pmpaddr3"},
	{ CSR_PMPADDR4, "pmpaddr4"},
	{ CSR_PMPADDR5, "pmpaddr5"},
	{ CSR_PMPADDR6, "pmpaddr6"},
	{ CSR_PMPADDR7, "pmpaddr7"},
	{ CSR_PMPADDR8, "pmpaddr8"},
	{ CSR_PMPADDR9, "pmpaddr9"},
	{ CSR_PMPADDR10, "pmpaddr10"},
	{ CSR_PMPADDR11, "pmpaddr11"},
	{ CSR_PMPADDR12, "pmpaddr12"},
	{ CSR_PMPADDR13, "pmpaddr13"},
	{ CSR_PMPADDR14, "pmpaddr14"},
	{ CSR_PMPADDR15, "pmpaddr15"},
	{ CSR_PMPADDR16, "pmpaddr16"},
	{ CSR_PMPADDR17, "pmpaddr17"},
	{ CSR_PMPADDR18, "pmpaddr18"},
	{ CSR_PMPADDR19, "pmpaddr19"},
	{ CSR_PMPADDR20, "pmpaddr20"},
	{ CSR_PMPADDR21, "pmpaddr21"},
	{ CSR_PMPADDR22, "pmpaddr22"},
	{ CSR_PMPADDR23, "pmpaddr23"},
	{ CSR_PMPADDR24, "pmpaddr24"},
	{ CSR_PMPADDR25, "pmpaddr25"},
	{ CSR_PMPADDR26, "pmpaddr26"},
	{ CSR_PMPADDR27, "pmpaddr27"},
	{ CSR_PMPADDR28, "pmpaddr28"},
	{ CSR_PMPADDR29, "pmpaddr29"},
	{ CSR_PMPADDR30, "pmpaddr30"},
	{ CSR_PMPADDR31, "pmpaddr31"},
	{ CSR_PMPADDR32, "pmpaddr32"},
	{ CSR_PMPADDR33, "pmpaddr33"},
	{ CSR_PMPADDR34, "pmpaddr34"},
	{ CSR_PMPADDR35, "pmpaddr35"},
	{ CSR_PMPADDR36, "pmpaddr36"},
	{ CSR_PMPADDR37, "pmpaddr37"},
	{ CSR_PMPADDR38, "pmpaddr38"},
	{ CSR_PMPADDR39, "pmpaddr39"},
	{ CSR_PMPADDR40, "pmpaddr40"},
	{ CSR_PMPADDR41, "pmpaddr41"},
	{ CSR_PMPADDR42, "pmpaddr42"},
	{ CSR_PMPADDR43, "pmpaddr43"},
	{ CSR_PMPADDR44, "pmpaddr44"},
	{ CSR_PMPADDR45, "pmpaddr45"},
	{ CSR_PMPADDR46, "pmpaddr46"},
	{ CSR_PMPADDR47, "pmpaddr47"},
	{ CSR_PMPADDR48, "pmpaddr48"},
	{ CSR_PMPADDR49, "pmpaddr49"},
	{ CSR_PMPADDR50, "pmpaddr50"},
	{ CSR_PMPADDR51, "pmpaddr51"},
	{ CSR_PMPADDR52, "pmpaddr52"},
	{ CSR_PMPADDR53, "pmpaddr53"},
	{ CSR_PMPADDR54, "pmpaddr54"},
	{ CSR_PMPADDR55, "pmpaddr55"},
	{ CSR_PMPADDR56, "pmpaddr56"},
	{ CSR_PMPADDR57, "pmpaddr57"},
	{ CSR_PMPADDR58, "pmpaddr58"},
	{ CSR_PMPADDR59, "pmpaddr59"},
	{ CSR_PMPADDR60, "pmpaddr60"},
	{ CSR_PMPADDR61, "pmpaddr61"},
	{ CSR_PMPADDR62, "pmpaddr62"},
	{ CSR_PMPADDR63, "pmpaddr63"},

	{ CSR_MCYCLE, "mcycle"},
	{ CSR_MINSTRET, "minstret"},
	{ CSR_MHPMCOUNTER3, "mhpmcounter3"},
	{ CSR_MHPMCOUNTER4, "mhpmcounter4"},
	{ CSR_MHPMCOUNTER5, "mhpmcounter5"},
	{ CSR_MHPMCOUNTER6, "mhpmcounter6"},
	{ CSR_MCYCLEH, "mcycleh"},
	{ CSR_MINSTRETH, "minstreth"},
	{ CSR_MHPMCOUNTER3H, "mhpmcounter3h"},
	{ CSR_MHPMCOUNTER4H, "mhpmcounter4h"},
	{ CSR_MHPMCOUNTER5H, "mhpmcounter5h"},
	{ CSR_MHPMCOUNTER6H, "mhpmcounter6h"},

	{ CSR_MHPMEVENT3, "mhpmevent3"},
	{ CSR_MHPMEVENT4, "mhpmevent4"},
	{ CSR_MHPMEVENT5, "mhpmevent5"},
	{ CSR_MHPMEVENT6, "mhpmevent6"},
	{ CSR_MCOUNTEREN, "mcounteren"},
	{ CSR_TSELECT, "tselect"},
	{ CSR_TDATA1, "tdata1"},
	{ CSR_TDATA2, "tdata2"},
	{ CSR_TDATA3, "tdata3"},
	{ CSR_DCSR, "dcsr"},
	{ CSR_DPC, "dpc"},
	{ CSR_DSCRATCH0, "dscratch0"},
	{ CSR_DSCRATCH1, "dscratch1"},
	{ CSR_MICM_CFG, "micm_cfg"},
	{ CSR_MDCM_CFG, "mdcm_cfg"},
	{ CSR_MMSC_CFG, "mmsc_cfg"},
	{ CSR_MMSC_CFG2, "mmsc_cfg2"},
	{ CSR_MILMB, "milmb"},
	{ CSR_MDLMB, "mdlmb"},
	{ CSR_MECC_CODE, "mecc_code"},
	{ CSR_MNVEC, "mnvec"},
	{ CSR_MCACHE_CTL, "mcache_ctl"},
	{ CSR_MCCTLBEGINADDR, "mcctlbeginaddr"},
	{ CSR_MCCTLCOMMAND, "mcctlcommand"},
	{ CSR_MCCTLDATA, "mcctldata"},
	{ CSR_MPPIB, "mppib"},
	{ CSR_MFIOB, "mfiob"},
	{ CSR_MHSP_CTL, "mhsp_ctl"},
	{ CSR_MSP_BOUND, "msp_bound"},
	{ CSR_MSP_BASE, "msp_base"},
	{ CSR_MXSTATUS, "mxstatus"},
	{ CSR_MDCAUSE, "mdcause"},
	{ CSR_MPFT_CTL, "mpft_ctl"},
	{ CSR_MMISC_CTL, "mmisc_ctl"},
	{ CSR_MCOUNTERWEN, "mcounterwen"},
	{ CSR_MCOUNTERINTEN, "mcounterinten"},
	{ CSR_MCOUNTERMASK_M, "mcountermask_m"},
	{ CSR_MCOUNTERMASK_S, "mcountermask_s"},
	{ CSR_MCOUNTERMASK_U, "mcountermask_u"},
	{ CSR_MCOUNTEROVF, "mcounterovf"},

	/*
	{ CSR_MRANDSEQ, "mrandseq"},
	{ CSR_MRANDSEQH, "mrandseqh"},
	{ CSR_MRANDSTATE, "mrandstate"},
	{ CSR_MRANDSTATEH, "mrandstateh"},
	*/
	{ CSR_DEXC2DBG, "dexc2dbg"},
	{ CSR_DDCAUSE, "ddcause"},
	{ CSR_SSTATUS, "sstatus"},
	{ CSR_SIE, "sie"},
	{ CSR_STVEC, "stvec"},
	{ CSR_SCOUNTEREN, "scounteren"},
	{ CSR_SSCRATCH, "sscratch"},
	{ CSR_SEPC, "sepc"},
	{ CSR_SCAUSE, "scause"},
	{ CSR_STVAL, "stval"},
	{ CSR_SIP, "sip"},
	{ CSR_SATP, "satp"},

	{ CSR_SDCAUSE, "sdcause"},
	{ CSR_SCOUNTERINTEN, "scounterinten"},
	{ CSR_SCOUNTERMASK_M, "scountermask_m"},
	{ CSR_SCOUNTERMASK_S, "scountermask_s"},
	{ CSR_SCOUNTERMASK_U, "scountermask_u"},
	{ CSR_SCOUNTEROVF, "scounterovf"},
	{ CSR_SCCTLDATA, "scctldata"},
	{ CSR_SCCTLDATA, "smisc_ctl"},
	{ CSR_CYCLE, "cycle"},
	{ CSR_INSTRET, "instret"},
	{ CSR_HPMCOUNTER3, "hpmcounter3"},
	{ CSR_HPMCOUNTER4, "hpmcounter4"},
	{ CSR_HPMCOUNTER5, "hpmcounter5"},
	{ CSR_HPMCOUNTER6, "hpmcounter6"},
	{ CSR_CYCLEH, "cycleh"},
	{ CSR_INSTRETH, "instreth"},
	{ CSR_HPMCOUNTER3H, "hpmcounter3h"},
	{ CSR_HPMCOUNTER4H, "hpmcounter4h"},
	{ CSR_HPMCOUNTER5H, "hpmcounter5h"},
	{ CSR_HPMCOUNTER6H, "hpmcounter6h"},
	{ CSR_FFLAGS, "fflags"},
	{ CSR_FRM, "frm"},
	{ CSR_FCSR, "fcsr"},
	{ CSR_UITB, "uitb"},
	{ CSR_UCCTLBEGINADDR, "ucctlbeginaddr"},
	{ CSR_UCCTLCOMMAND, "ucctlcommand"},

	/* HW not implement, do not exist csr registers */
	/*
	{ CSR_SEDELEG, "sedeleg"},
	{ CSR_SIDELEG, "sideleg"},
	*/
	{ CSR_MSLIDELEG, "mslideleg"},
	{ CSR_SLIE, "slie"},
	{ CSR_SLIP, "slip"},

	{ CSR_SHPMEVENT3, "shpmevent3"},
	{ CSR_SHPMEVENT4, "shpmevent4"},
	{ CSR_SHPMEVENT5, "shpmevent5"},
	{ CSR_SHPMEVENT6, "shpmevent6"},

	{ CSR_MSAVESTATUS, "msavestatus"},
	{ CSR_MSAVEEPC1, "msaveepc1"},
	{ CSR_MSAVECAUSE1, "msavecause1"},
	{ CSR_MSAVEEPC2, "msaveepc2"},
	{ CSR_MSAVECAUSE2, "msavecause2"},
	{ CSR_MSAVEDCAUSE1, "msavedcause1"},
	{ CSR_MSAVEDCAUSE2, "msavedcause2"},

	{ CSR_MCOUNTINHIBIT, "mcountinhibit"},
	{ CSR_SCOUNTINHIBIT, "scountinhibit"},

	{ CSR_USTATUS, "ustatus"},
	{ CSR_UIE, "uie"},
	{ CSR_UTVEC, "utvec"},
	{ CSR_USCRATCH, "uscratch"},
	{ CSR_UEPC, "uepc"},
	{ CSR_UCAUSE, "ucause"},
	{ CSR_UTVAL, "utval"},
	{ CSR_UIP, "uip"},
	{ CSR_UDCAUSE, "udcause"},
	{ CSR_WFE, "wfe"},
	{ CSR_SLEEPVALUE, "sleepvalue"},
	{ CSR_TXEVT, "txevt"},

	{ CSR_MTVT, "mtvt"},
	{ CSR_MNXTI, "mnxti"},
	{ CSR_MINTSTATUS, "mintstatus"},
	{ CSR_MSCRATCHCSW, "mscratchcsw"},
	{ CSR_MSCRATCHCSWL, "mscratchcswl"},

	{ CSR_UCODE, "ucode"},
	{ CSR_MIRQ_ENTRY, "mirq_entry"},

	{ CSR_VSTART, "vstart"},
	{ CSR_VXSAT, "vxsat"},
	{ CSR_VXRM, "vxrm"},
	{ CSR_VL, "vl"},
	{ CSR_VTYPE, "vtype"},

	{ CSR_PMACFG0, "pmacfg0"},
	{ CSR_PMACFG1, "pmacfg1"},
	{ CSR_PMACFG2, "pmacfg2"},
	{ CSR_PMACFG3, "pmacfg3"},
	{ CSR_PMAADDR0, "pmaaddr0"},
	{ CSR_PMAADDR1, "pmaaddr1"},
	{ CSR_PMAADDR2, "pmaaddr2"},
	{ CSR_PMAADDR3, "pmaaddr3"},
	{ CSR_PMAADDR4, "pmaaddr4"},
	{ CSR_PMAADDR5, "pmaaddr5"},
	{ CSR_PMAADDR6, "pmaaddr6"},
	{ CSR_PMAADDR7, "pmaaddr7"},
	{ CSR_PMAADDR8, "pmaaddr8"},
	{ CSR_PMAADDR9, "pmaaddr9"},
	{ CSR_PMAADDR10, "pmaaddr10"},
	{ CSR_PMAADDR11, "pmaaddr11"},
	{ CSR_PMAADDR12, "pmaaddr12"},
	{ CSR_PMAADDR13, "pmaaddr13"},
	{ CSR_PMAADDR14, "pmaaddr14"},
	{ CSR_PMAADDR15, "pmaaddr15"},
	{ 0, NULL},
};

int vtarget_redefine_CSR_name(struct target *target)
{
	uint32_t i, csr_id;
	struct nds_csr_reg *p_nds_csr = (struct nds_csr_reg *)&nds_all_csr[0];

	NDS_INFO("%s", __func__);
	/* disable all CSR register */
	for (i = REG_CSR0; i <= REG_CSR4095; i++)
		target->reg_cache->reg_list[i].exist = false;

	while (1) {
		if ((p_nds_csr->csr_id > 4095) ||
			(p_nds_csr->name == NULL)) {
			break;
		}
		NDS_INFO("%d, %s", p_nds_csr->csr_id, p_nds_csr->name);
		csr_id = GDB_REGNO_CSR0 + p_nds_csr->csr_id;
		target->reg_cache->reg_list[csr_id].exist = true;
		if (vtarget_reg_symbolic_name == 1)
			target->reg_cache->reg_list[csr_id].name = p_nds_csr->name;
		p_nds_csr++;
	}
	return ERROR_OK;
}

char *vtarget_get_CSR_name(struct target *target, uint32_t csr_id)
{
	if (csr_id > 4095) {
		NDS_INFO("get wrong csr_id: %d", csr_id);
		return NULL;
	}
	char *nds32_csr_name = (char *)target->reg_cache->reg_list[csr_id + REG_CSR0].name;
	if (nds32_csr_name == NULL) {
		NDS_INFO("get null csr_name: %d", csr_id);
		return NULL;
	}
	NDS_INFO("%s", nds32_csr_name);
	return (char *)nds32_csr_name;
}

/*
for disassembler
x0	zero	Zero
x1	ra	Return address
x2	sp	Stack pointer
x3	gp	Global pointer
x4	tp	Thread pointer
x5-x7	t0-t2	Temporary registers
x8-x9	s0-s1	Callee-saved registers
x10-x17	a0-a7	Argument registers
x18-x27	s2-s11	Callee-saved registers
x28-x31	t3-t6	Temporary registers
f0-f7	ft0-ft7	Temporary registers
f8-f9	fs0-fs1	Callee-saved registers
f10-f17	fa0-fa7	Argument registers
f18-f27	fs2-fs11	Callee-saved registers
f28-f31	ft8-ft11	Temporary registers
*/
/* abi name */
static char *reg_abi_name[] = {
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

/* architecture name */
static char *reg_arch_name[] = {
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
char **vtarget_gpr_and_fpu_name = reg_arch_name;

int vtarget_redefine_GPR_FPU_name(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	if (vtarget_reg_symbolic_name == 1) {
		LOG_DEBUG("change to abi name");
		vtarget_gpr_and_fpu_name = reg_abi_name;
		uint32_t i = 0;
		while (1) {
			if (i >= 65)
				break;
			NDS_INFO("%d, %s", i, vtarget_gpr_and_fpu_name[i]);
			target->reg_cache->reg_list[i].name = vtarget_gpr_and_fpu_name[i];
			i++;
		}
	}
	return ERROR_OK;
}

static int vtarget_log_callback(void *priv)
{
	char log_buffer[2048];
	unsigned int FileSize = 0;

	char *c;
	if (log_output_path) {
		c = strstr(log_output_path, "iceman_debug0.log");
		if (c)
			*c = '\0';
	}

	if (pLogFile) {
		/* fseek(pLogFile, 0, SEEK_END); */
		FileSize = ftell(pLogFile);
	} else {
		FILE *pStartLogFile = get_log_output();
		if (pStartLogFile)
			fclose(pStartLogFile);
	}

	if ((FileSize >= vtarget_MaxLogFileSize) ||
	    (pLogFile == NULL)) {
		if (pLogFile)
			fclose(pLogFile);
		LogFileIdx ^= 0x01;

		memset(log_buffer, 0, sizeof(log_buffer));
		if (log_output_path)
			strncpy(log_buffer, log_output_path, strlen(log_output_path));
		strncat(log_buffer, Log_File_Name[LogFileIdx], strlen(Log_File_Name[LogFileIdx]));
		pLogFile = fopen(log_buffer, "w");
		set_log_output(NULL, pLogFile);
	}
	/* LOG_DEBUG("LogFileSize = 0x%x\n", FileSize); */
	return ERROR_OK;
}

/** Convert target handle to generic Andes target state handle. */
struct nds_vtarget *target_to_nds_vtarget(struct target *target)
{
	struct nds_vtarget *pnds_vtarget = NULL;

	for (pnds_vtarget = gpnds_vtarget; pnds_vtarget; pnds_vtarget = pnds_vtarget->next) {
		if (pnds_vtarget->target == target)
			break;
	}
	return pnds_vtarget;
}

static int vtarget_init_arch_info(struct target *target, struct nds_vtarget *new_vtarget)
{
	struct nds_vtarget *pnds_vtarget;

	new_vtarget->target = target;
	new_vtarget->next = NULL;

	if (gpnds_vtarget) {
		for (pnds_vtarget = gpnds_vtarget; pnds_vtarget; pnds_vtarget = pnds_vtarget->next) {
			if (pnds_vtarget->next == NULL) {
				pnds_vtarget->next = new_vtarget;
				break;
			}
		}
	} else {
		gpnds_vtarget = new_vtarget;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_vtarget_NOT_support_command)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds_write_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t addr;
	uint32_t count;
	uint8_t *data;
	uint32_t i;
	int result;

	if (CMD_ARGC < 3) {
		command_print(CMD, "usage: %s <address> <count> <data>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);

	data = malloc(count);

	for (i = 0; i < count; i++)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2 + i], data[i]);

	result = target_write_buffer(target, addr, count, data);

	free(data);

	return result;
}

COMMAND_HANDLER(handle_nds_read_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t addr;
	uint32_t count;
	uint8_t *data;
	uint32_t i;
	int result;

	if (CMD_ARGC < 2) {
		command_print(CMD, "usage: %s <address> <count>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);

	data = malloc(count);

	result = target_read_buffer(target, addr, count, data);

	for (i = 0; i < count; i++) {
		if ((i & 0xF) == 0)
			command_print_sameline(NULL, "0x%08x: ", (int)(addr + i));

		command_print_sameline(CMD, "%02x ", data[i]);

		if (((i & 0xF) == 0xF) || (i == count - 1))
			command_print_sameline(NULL, "\n");
	}

	free(data);

	return result;
}

COMMAND_HANDLER(handle_vtarget_query_target_command)
{
	command_print(CMD, "OCD");
	return ERROR_OK;
}

COMMAND_HANDLER(handle_vtarget_query_endian_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t big_endian = 0;

	if (big_endian)
		command_print(CMD, "%s: BE", target_name(target));
	else
		command_print(CMD, "%s: LE", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_vtarget_query_cpuid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct target *tmptarget;
	uint32_t number_of_core = 0;

	for (tmptarget = all_targets; tmptarget; tmptarget = tmptarget->next)
		number_of_core++;

	if (number_of_core == 0)
		command_print(CMD, "");
	else if (number_of_core == 1)
		command_print(CMD, "");
	else {
		/*
		command_print(CMD, "%s%d_target%d", target->tap->tapname,
			target->tap->abs_chain_position, target->target_number);
		*/
		command_print(CMD, "%s", target_name(target));
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_vtarget_query_capability_command)
{
	/* AndeSight query disbus value to decide bus mode icon exist or not
	 * (default value 0 mean bus mode icon exist) */
	uint32_t if_tracer = 0, if_profiling = 1, disable_busmode = 0;
	uint32_t hit_exception = 1, if_targetburn = 1;
	uint32_t if_pwr_sample = 0;
	uint32_t q_access_mode = 0;
	/* according to eticket 16199: system bus access incomplete support for hardware,
	 * so report 0 to AndeSight query(means:bus mode auto refresh not support)*/
	/* before hardware incomplete support system bus access, nds_sys_bus_supported is 0 */
	uint32_t system_bus_access = vtarget_sys_bus_supported;

	command_print(CMD,
		"tracer:%d;profiling:%d;disbus:%d;exception:%d;targetburn:%d;pwr:%d;q_access_mode:%d;sysbusaccess:%d",
		if_tracer,
		if_profiling,
		disable_busmode,
		hit_exception,
		if_targetburn,
		if_pwr_sample,
		q_access_mode,
		system_bus_access);
	return ERROR_OK;
}

__COMMAND_HANDLER(handle_vtarget_memory_access_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	LOG_DEBUG("NDS mem_access on [%s] hart %d", target->tap->dotted_name, target->coreid);

	if (nds32 == NULL) {
		LOG_ERROR("gpnds_vtarget is NULL");
		return ERROR_FAIL;
	}
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "bus") == 0) {
			memory->access_channel = NDS_MEMORY_ACC_BUS;
		} else if (strcmp(CMD_ARGV[0], "cpu") == 0) {
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		} else {
			/* default access channel is NDS_MEMORY_ACC_CPU */
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		}
		NDS_INFO("memory access channel is change to %s",
				NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	} else {
		command_print(CMD, "%s: memory access channel: %s",
				target_name(target), NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_vtarget_memAccSize_command)
{
	uint32_t access_size;
	uint64_t lowAddr, highAddr;
	int result;

	if (CMD_ARGC < 3) {
		command_print(CMD, "usage: %s <lowAddr> <highAddr> <access_size>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], lowAddr);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], highAddr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], access_size);
	highAddr = (uint64_t)(highAddr - 1);
	if ((highAddr < lowAddr) ||
			((access_size != 8) && (access_size != 16) &&
			 (access_size != 32) && (access_size != 64))) {
		command_print(CMD, "Invalid parameter");
		return ERROR_FAIL;
	}
	if (access_size == 64)
		access_size = 32;
	access_size = (access_size >> 3);
	result = vtarget_set_buffer_access_size(lowAddr, highAddr, access_size);
	return result;
}

COMMAND_HANDLER(handle_vtarget_reset_memAccSize_command)
{
	vtarget_reset_buffer_access_size();
	return ERROR_OK;
}

#define SIZE_PROPERTY (sizeof(property_names)/sizeof(char *))

COMMAND_HANDLER(handle_vtarget_configure_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);

	if (nds32 == NULL) {
		LOG_ERROR("gpnds_vtarget is NULL");
		return ERROR_FAIL;
	}

	/* check if any property is given */
	if (0 >= CMD_ARGC) {
		command_print(CMD, "configure: no property given!");
		return ERROR_FAIL;
	}

	LOG_DEBUG("NDS configure on [%s] hart %d", target->tap->dotted_name, target->coreid);

	/* check if property is supported */
	if (strcmp(CMD_ARGV[0], "tdesc_bit") == 0) {
		gpBitFieldFileNameVtarget = strdup(CMD_ARGV[1]);
		command_print(CMD, "configure: %s %s", CMD_ARGV[0], gpBitFieldFileNameVtarget);
	} else if (strcmp(CMD_ARGV[0], "aligned_access") == 0) {
		vtarget_force_aligned_access = 1;
		command_print(CMD, "configure: %s", CMD_ARGV[0]);
	} else if (strcmp(CMD_ARGV[0], "reg_symbolic_name") == 0) {
		vtarget_reg_symbolic_name = 1;
		command_print(CMD, "configure: %s", CMD_ARGV[0]);
	} else if (strcmp(CMD_ARGV[0], "jtag_scans_optimize") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vtarget_jtag_scans_optimize);
		else
			vtarget_jtag_scans_optimize = 1;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], vtarget_jtag_scans_optimize);
	} else if (strcmp(CMD_ARGV[0], "sys_bus_access") == 0) {
		/* according to eticket 16199, default no support system bus access,
		 * so vtarget_system_bus_access default value is 0 */
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vtarget_system_bus_access);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], vtarget_system_bus_access);
	} else if (strcmp(CMD_ARGV[0], "dmi_busy_retry_times") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vtarget_dmi_busy_retry_times);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], vtarget_dmi_busy_retry_times);
	} else if (strcmp(CMD_ARGV[0], "bak_debug_buf") == 0) {
		/* for log files */
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vtarget_bak_debug_buf_size);
		if (vtarget_bak_debug_buf_size < 0x80000)
			vtarget_bak_debug_buf_size = 0x80000;
		else if (vtarget_bak_debug_buf_size >= 0x800000)
			vtarget_bak_debug_buf_size = 0x800000;
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
		p_nds_bak_debug_buffer_start = (char *)malloc(vtarget_bak_debug_buf_size);
		p_nds_bak_debug_buffer_end = p_nds_bak_debug_buffer_start + vtarget_bak_debug_buf_size;
		p_nds_bak_debug_buffer_cur = p_nds_bak_debug_buffer_start;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], vtarget_bak_debug_buf_size);
		NDS_INFO("p_nds_bak_debug_buffer_start = 0x%lx, p_nds_bak_debug_buffer_end = 0x%lx",
			(long unsigned int)p_nds_bak_debug_buffer_start, (long unsigned int)p_nds_bak_debug_buffer_end);
	} else if (strcmp(CMD_ARGV[0], "scan_retry_times") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_scan_retry_times);
		if (nds_scan_retry_times > 5)
			nds_scan_retry_times = 5;
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_scan_retry_times);
	} else if (strcmp(CMD_ARGV[0], "jtag_max_scans") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds_jtag_max_scans);
		if (nds_jtag_max_scans < 32)
			nds_jtag_max_scans = 32;

		/* For Andes FTDI Device(0x1cfc, 0x0001) */
		if (nds_ftdi_devices == 1)
			nds_jtag_max_scans = 32;

		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds_jtag_max_scans);
	} else if (strcmp(CMD_ARGV[0], "target_arch_xlen") == 0) {
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vtarget_xlen);
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], vtarget_xlen);
	} else {
		command_print(CMD, "configure: property '%s' unknown!", CMD_ARGV[0]);
		NDS32_LOG("<-- configure: property '%s' unknown! -->", CMD_ARGV[0]);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int vtarget_jtag_cur_tap_name(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc-1, argv + 1);
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

static const struct command_registration vtarget_query_command_handlers[] = {
	{
		.name = "target",
		.handler = handle_vtarget_query_target_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "reply 'OCD' for gdb to identify server-side is OpenOCD",
	},
	{
		.name = "endian",
		.handler = handle_vtarget_query_endian_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query target endian vtarget",
	},
	{
		.name = "cpuid",
		.handler = handle_vtarget_query_cpuid_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query CPU ID vtarget",
	},
	{
		.name = "capability",
		.handler = handle_vtarget_query_capability_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query target capability",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration vtarget_exec_command_handlers[] = {
	{
		.name = "dssim",
		.handler = handle_vtarget_NOT_support_command,
		.mode = COMMAND_EXEC,
		.usage = "['on'|'off']",
		.help = "display/change $INT_MASK.DSSIM status",
	},
	{
		.name = "mem_access",
		.handler = handle_vtarget_memory_access_command,
		.mode = COMMAND_EXEC,
		.usage = "['bus'|'cpu']",
		.help = "display/change memory access channel",
	},
	{
		.name = "memAccSize",
		.handler = handle_vtarget_memAccSize_command,
		.mode = COMMAND_ANY,
		.usage = "<lowaddr> <highaddr> <access_size>",
		.help = "Write memory access attribute.",
	},
	{
		.name = "reset_memAccSize",
		.handler = handle_vtarget_reset_memAccSize_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "reset memory access attribute.",
	},
	{
		.name = "configure",
		.handler = handle_vtarget_configure_command,
		.mode = COMMAND_ANY,
		.help = "Configure/Query property",
		.usage = "property [value]",
	},
	{
		.name = "query",
		.mode = COMMAND_EXEC,
		.help = "Andes query command group",
		.usage = "",
		.chain = vtarget_query_command_handlers,
	},
	{
		.name = "jtag_tap_name",
		.jim_handler = vtarget_jtag_cur_tap_name,
		.mode = COMMAND_ANY,
		.help = " ",
		.usage = " ",
	},
	{
		.name = "write_buffer",
		.handler = handle_nds_write_buffer_command,
		.mode = COMMAND_ANY,
		.usage = "<address> <count> <data>",
		.help = "Write data to target buffer.",
	},
	{
		.name = "read_buffer",
		.handler = handle_nds_read_buffer_command,
		.mode = COMMAND_ANY,
		.usage = "<address> <count>",
		.help = "Read data from target buffer.",
	},

	COMMAND_REGISTRATION_DONE
};

const struct command_registration vtarget_command_handlers[] = {
	{
		.name = "nds",
		.mode = COMMAND_ANY,
		.help = "vtarget command group",
		.usage = "",
		.chain = vtarget_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int vtarget_target_create(struct target *target, Jim_Interp *interp)
{
	LOG_DEBUG("%s", __func__);
	struct nds_vtarget *pnds_vtarget;

	pnds_vtarget = calloc(1, sizeof(struct nds_vtarget));
	vtarget_init_arch_info(target, pnds_vtarget);

	static unsigned int vtarget_do_once;
	if (vtarget_do_once)
		return ERROR_OK;

	target_register_timer_callback(vtarget_log_callback, 5000, 1, target);
	vtarget_do_once = 1;

	return ERROR_OK;
}

static const char * const TARGET_EVENT_Name[] = {
	"TARGET_EVENT_GDB_HALT",
	"HALTED",
	"RESUMED",
	"RESUME_START",
	"RESUME_END",
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

/** Other functions **/
extern int vtarget_init_reg(struct target *target);
extern int vtarget_reexamine(struct target *target);

static int vtarget_gdb_attach(struct target *target)
{
	/* NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid); */
	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);

	if (!target_was_examined(target)) {
		NDS_INFO("target was NOT examined");
		if (vtarget_reexamine(target) != ERROR_OK) {
			NDS_INFO("[%s] hart %d reexamine failed!!", target->tap->dotted_name, target->coreid);
			return ERROR_FAIL;
		}
	}
	if (nds32->attached == false) {
		vtarget_init_reg(target);

		nds32->attached = true;
		nds32->hit_syscall = false;
		/* reset to debug mode in case abnormal operations */
		nds32->gdb_run_mode = RUN_MODE_DEBUG;
		nds32->is_program_exit = false;
	}

	return ERROR_OK;
}

static int vtarget_gdb_detach(struct target *target)
{
	NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);

	if (nds32->attached) {
		nds32->gdb_run_mode = RUN_MODE_DEBUG;
		nds32->attached = false;	/* Set attached to false before resume */
	}

	return ERROR_OK;
}

/** handle functions **/
static int vtarget_callback_event_handler(struct target *target,
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
			retval = vtarget_gdb_attach(target);
			break;
		case TARGET_EVENT_GDB_DETACH:
			retval = vtarget_gdb_detach(target);
			break;
		default:
			break;
	}

	return retval;
}

int vtarget_handle_examine(struct target *target)
{
	NDS_INFO("%s", __func__);
	/* register event callback */
	target_register_event_callback(vtarget_callback_event_handler, &target->target_number);
	return ERROR_OK;
}

#define MEM_ACCESS_ATTR_MAX		128
static void vtarget_reset_buffer_access_size(void)
{
	uint32_t i;
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr *)&all_mem_access_attr[0];
	for (i = 0; i <= MEM_ACCESS_ATTR_MAX; i++) {
		pmem_attr->lowAddr = 0;
		pmem_attr->highAddr = 0;
		pmem_attr->access_size = 0;
		pmem_attr++;
	}
	curr_mem_access_attr_index = 0;
}

static int vtarget_set_buffer_access_size(uint64_t start_addr, uint64_t end_addr,
		uint32_t access_size)
{
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr *)&all_mem_access_attr[0];
	if (curr_mem_access_attr_index >= MEM_ACCESS_ATTR_MAX) {
		LOG_ERROR("vtarget_set_buffer_access_size error!");
		return ERROR_FAIL;
	}

	curr_mem_access_attr_index++;  /* start from (idx = 1) */
	pmem_attr += curr_mem_access_attr_index;
	pmem_attr->lowAddr = start_addr;
	pmem_attr->highAddr = end_addr;
	pmem_attr->access_size = access_size;
	LOG_DEBUG("vtarget_set_buffer_access_size %d: 0x%016" PRIx64 ", 0x%016" PRIx64 ", %d\n",
		curr_mem_access_attr_index, pmem_attr->lowAddr, pmem_attr->highAddr, pmem_attr->access_size);
	return ERROR_OK;
}

