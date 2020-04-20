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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_V5_H
#define OPENOCD_TARGET_NDS32_V5_H

// gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
// its source tree. We must interpret the numbers the same here.
enum {
	REG_XPR0 = 0,
	REG_XPR31 = 31,
	REG_PC = 32,
	REG_FPR0 = 33,
	REG_FPR31 = 64,
	REG_CSR0 = 65,
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,
	REG_CSR4095 = 4160,
	REG_PRIV = 4161,
	REG_V0,
	REG_V31 = REG_V0 + 31,
	REG_ALL_COUNT
};

enum nds_memory_access {
	NDS_MEMORY_ACC_BUS = 0,
	NDS_MEMORY_ACC_CPU,
};

extern char **gpr_and_fpu_name;

/*#if 1
#define REG_ZERO  "x0"  // x0	zero	Zero
#define REG_RA    "x1"  // x1	ra	Return address
#define REG_SP    "x2"  // x2	sp	Stack pointer
#define REG_GP    "x3"  // x3	gp	Global pointer
#define REG_TP    "x4"  // x4	tp	Thread pointer
#define REG_T0    "x5"  // x5-x7	t0-t2	Temporary registers
#define REG_T1    "x6"
#define REG_T2    "x7"
#define REG_S0    "x8"  // x8-x9	s0-s1	Callee-saved registers
#define REG_S1    "x9"
#define REG_A0    "x10" // x10-x17	a0-a7	Argument registers
#define REG_A1    "x11"
#define REG_A2    "x12"
#define REG_A3    "x13"
#define REG_A4    "x14"
#define REG_A5    "x15"
#define REG_A6    "x16"
#define REG_A7    "x17"
#define REG_S2    "x18" // x18-x27	s2-s11	Callee-saved registers
#define REG_S3    "x19"
#define REG_S4    "x20"
#define REG_S5    "x21"
#define REG_S6    "x22"
#define REG_S7    "x23"
#define REG_S8    "x24"
#define REG_S9    "x25"
#define REG_S10   "x26"
#define REG_S11   "x27"
#define REG_T3    "x28" // x28-x31	t3-t6	Temporary registers
#define REG_T4    "x29"
#define REG_T5    "x30"
#define REG_T6    "x31"
#define REG_FT0   "f0"  // f0-f7	ft0-ft7	Temporary registers
#define REG_FT1   "f1"
#define REG_FT2   "f2"
#define REG_FT3   "f3"
#define REG_FT4   "f4"
#define REG_FT5   "f5"
#define REG_FT6   "f6"
#define REG_FT7   "f7"
#define REG_FS0   "f8"  // f8-f9	fs0-fs1	Callee-saved registers
#define REG_FS1   "f9"
#define REG_FA0   "f10" // f10-f17	fa0-fa7	Argument registers
#define REG_FA1   "f11"
#define REG_FA2   "f12"
#define REG_FA3   "f13"
#define REG_FA4   "f14"
#define REG_FA5   "f15"
#define REG_FA6   "f16"
#define REG_FA7   "f17"
#define REG_FS2   "f18" // f18-f27	fs2-fs11	Callee-saved registers
#define REG_FS3   "f19"
#define REG_FS4   "f20"
#define REG_FS5   "f21"
#define REG_FS6   "f22"
#define REG_FS7   "f23"
#define REG_FS8   "f24"
#define REG_FS9   "f25"
#define REG_FS10  "f26"
#define REG_FS11  "f27"
#define REG_FT8   "f28" // f28-f31	ft8-ft11	Temporary registers
#define REG_FT9   "f29"
#define REG_FT10  "f30"
#define REG_FT11  "f31"
#else  //for gdb default name
#define REG_ZERO  "zero" //"x0"  // x0	zero	Zero
#define REG_RA    "ra" //"x1"  // x1	ra	Return address
#define REG_SP    "sp" //"x2"  // x2	sp	Stack pointer
#define REG_GP    "gp" //"x3"  // x3	gp	Global pointer
#define REG_TP    "tp" //"x4"  // x4	tp	Thread pointer
#define REG_T0    "t0" //"x5"  // x5-x7	t0-t2	Temporary registers
#define REG_T1    "t1" //"x6"
#define REG_T2    "t2" //"x7"
#define REG_S0    "s0" //"x8"  // x8-x9	s0-s1	Callee-saved registers
#define REG_S1    "s1" //"x9"
#define REG_A0    "a0" //"x10" // x10-x17	a0-a7	Argument registers
#define REG_A1    "a1" //"x11"
#define REG_A2    "a2" //"x12"
#define REG_A3    "a3" //"x13"
#define REG_A4    "a4" //"x14"
#define REG_A5    "a5" //"x15"
#define REG_A6    "a6" //"x16"
#define REG_A7    "a7" //"x17"
#define REG_S2    "s2" //"x18" // x18-x27	s2-s11	Callee-saved registers
#define REG_S3    "s3" //"x19"
#define REG_S4    "s4" //"x20"
#define REG_S5    "s5" //"x21"
#define REG_S6    "s6" //"x22"
#define REG_S7    "s7" //"x23"
#define REG_S8    "s8" //"x24"
#define REG_S9    "s9" //"x25"
#define REG_S10   "s10" //"x26"
#define REG_S11   "s11" //"x27"
#define REG_T3    "t3" //"x28" // x28-x31	t3-t6	Temporary registers
#define REG_T4    "t4" //"x29"
#define REG_T5    "t5" //"x30"
#define REG_T6    "t6" //"x31"
#define REG_FT0   "ft0" //"f0"  // f0-f7	ft0-ft7	Temporary registers
#define REG_FT1   "ft1" //"f1"
#define REG_FT2   "ft2" //"f2"
#define REG_FT3   "ft3" //"f3"
#define REG_FT4   "ft4" //"f4"
#define REG_FT5   "ft5" //"f5"
#define REG_FT6   "ft6" //"f6"
#define REG_FT7   "ft7" //"f7"
#define REG_FS0   "fs0" //"f8"  // f8-f9	fs0-fs1	Callee-saved registers
#define REG_FS1   "fs1" //"f9"
#define REG_FA0   "fa0" //"f10" // f10-f17	fa0-fa7	Argument registers
#define REG_FA1   "fa1" //"f11"
#define REG_FA2   "fa2" //"f12"
#define REG_FA3   "fa3" //"f13"
#define REG_FA4   "fa4" //"f14"
#define REG_FA5   "fa5" //"f15"
#define REG_FA6   "fa6" //"f16"
#define REG_FA7   "fa7" //"f17"
#define REG_FS2   "fs2" //"f18" // f18-f27	fs2-fs11	Callee-saved registers
#define REG_FS3   "fs3" //"f19"
#define REG_FS4   "fs4" //"f20"
#define REG_FS5   "fs5" //"f21"
#define REG_FS6   "fs6" //"f22"
#define REG_FS7   "fs7" //"f23"
#define REG_FS8   "fs8" //"f24"
#define REG_FS9   "fs9" //"f25"
#define REG_FS10  "fs10" //"f26"
#define REG_FS11  "fs11" //"f27"
#define REG_FT8   "ft8" //"f28" // f28-f31	ft8-ft11	Temporary registers
#define REG_FT9   "ft9" //"f29"
#define REG_FT10  "ft10" //"f30"
#define REG_FT11  "ft11" //"f31"
#endif
*/
/* AndeStar V5 CSRs */
//#define CSR_MVENDORID    0xF11
//#define CSR_MARCHID      0xF12
//#define CSR_MIMPID       0xF13
//#define CSR_MHARTID      0xF14
//#define CSR_MSTATUS      0x300
//#define CSR_MISA         0x301
//#define CSR_MEDELEG      0x302
//#define CSR_MIDELEG      0x303
//#define CSR_MIE          0x304
//#define CSR_MTVEC        0x305
//#define CSR_MCOUNTEREN     0x306
//#define CSR_MSCRATCH     0x340
//#define CSR_MEPC         0x341
//#define CSR_MCAUSE       0x342
//#define CSR_MTVAL          0x343
//#define CSR_MIP          0x344
////#define CSR_PMPCFG0        0x3A0
////#define CSR_PMPCFG1        0x3A1
////#define CSR_PMPCFG2        0x3A2
////#define CSR_PMPCFG3        0x3A3
////#define CSR_PMPADDR0       0x3B0
////#define CSR_PMPADDR1       0x3B1
////#define CSR_PMPADDR2       0x3B2
////#define CSR_PMPADDR3       0x3B3
////#define CSR_PMPADDR4       0x3B4
////#define CSR_PMPADDR5       0x3B5
////#define CSR_PMPADDR6       0x3B6
////#define CSR_PMPADDR7       0x3B7
////#define CSR_PMPADDR8       0x3B8
////#define CSR_PMPADDR9       0x3B9
//#define CSR_PMPADDRA       0x3BA
//#define CSR_PMPADDRB       0x3BB
//#define CSR_PMPADDRC       0x3BC
//#define CSR_PMPADDRD       0x3BD
//#define CSR_PMPADDRE       0x3BE
//#define CSR_PMPADDRF       0x3BF
//#define CSR_MCYCLE       0xb00
//#define CSR_MINSTRET     0xb02
//#define CSR_MHPMCOUNTER3 0xb03
//#define CSR_MCYCLEH      0xb80
//#define CSR_MINSTRETH     0xb82
//#define CSR_MHPMCOUNTER3H 0xb83
//#define CSR_MHPMEVENT3    0x323
//#define CSR_TSELECT      0x7a0
//#define CSR_TDATA1       0x7a1
//#define CSR_TDATA2       0x7a2
//#define CSR_TDATA3       0x7a3
//#define CSR_DCSR         0x7b0
//#define CSR_DPC          0x7b1
//#define CSR_DSCRATCH     0x7b2
#define CSR_MICM_CFG       0xFC0
#define CSR_MDCM_CFG       0xFC1
#define CSR_MMSC_CFG       0xFC2
#define CSR_MMSC_CFG2      0xFC3

#define CSR_MCRASH_STATESAVE  0xFC8
#define CSR_MSTATUS_CRASHSAVE 0xFC9

#define CSR_MILMB          0x7C0
#define CSR_MDLMB          0x7C1
#define CSR_MECC_CODE      0x7C2
#define CSR_MNVEC          0x7C3
#define CSR_MCACHE_CTL     0x7CA
#define CSR_MCCTLBEGINADDR 0x7CB
#define CSR_MCCTLCOMMAND   0x7CC
#define CSR_MCCTLDATA      0x7CD

#define CSR_MPPIB          0x7F0
#define CSR_MFIOB          0x7F1

#define CSR_MHSP_CTL       0x7C6
#define CSR_MSP_BOUND      0x7C7
#define CSR_MSP_BASE       0x7C8

#define CSR_MXSTATUS       0x7C4
#define CSR_MDCAUSE        0x7C9
#define CSR_MSLIDELEG      0x7D5

#define CSR_MPFT_CTL       0x7C5
#define CSR_MMISC_CTL      0x7D0
#define CSR_MCLK_CTL       0x7DF

#define CSR_MCOUNTERWEN    0x7CE
#define CSR_MCOUNTERINTEN  0x7CF
#define CSR_MCOUNTERMASK_M 0x7D1
#define CSR_MCOUNTERMASK_S 0x7D2
#define CSR_MCOUNTERMASK_U 0x7D3
#define CSR_MCOUNTEROVF    0x7D4

#define CSR_MSAVESTATUS    0x7D6
#define CSR_MSAVEEPC1      0x7D7
#define CSR_MSAVECAUSE1    0x7D8
#define CSR_MSAVEEPC2      0x7D9
#define CSR_MSAVECAUSE2    0x7DA
#define CSR_MSAVEDCAUSE1   0x7DB
#define CSR_MSAVEDCAUSE2   0x7DC

#define CSR_DEXC2DBG       0x7E0
#define CSR_DDCAUSE        0x7E1

//bug17785 comment9 :remove other csrs
//#define CSR_PUSHMXSTATUS   0x7EB
#define CSR_MIRQ_ENTRY     0x7EC
//#define CSR_MINTSEL_JAL    0x7ED
//#define CSR_PUSHMCAUSE     0x7EE
//#define CSR_PUSHMEPC       0x7EF

#define CSR_SLIE           0x9C4
#define CSR_SLIP           0x9C5
#define CSR_SDCAUSE        0x9C9

#define CSR_SCOUNTERINTEN  0x9CF
#define CSR_SCOUNTERMASK_M 0x9D1
#define CSR_SCOUNTERMASK_S 0x9D2
#define CSR_SCOUNTERMASK_U 0x9D3
#define CSR_SCOUNTEROVF    0x9D4

#define CSR_SHPMEVENT3     0x9E3
#define CSR_SHPMEVENT4     0x9E4
#define CSR_SHPMEVENT5     0x9E5
#define CSR_SHPMEVENT6     0x9E6

#define CSR_SCCTLDATA      0x9CD
#define CSR_SMISC_CTL      0x9D0

#define CSR_UITB           0x800
#define CSR_UCODE          0x801
#define CSR_UDCAUSE        0x809
#define CSR_UCCTLBEGINADDR 0x80B
#define CSR_UCCTLCOMMAND   0x80C
#define CSR_WFE            0x810
#define CSR_SLEEPVALUE     0x811
#define CSR_TXEVT          0x812

#define CSR_MCOUNTINHIBIT  0x320
#define CSR_SCOUNTINHIBIT  0x9E0

#define CSR_MTVT           0x307

#define CSR_MNXTI          0x345
#define CSR_MINTSTATUS     0x346
#define CSR_MSCRATCHCSW    0x348
#define CSR_MSCRATCHCSWL   0x349

//#define CSR_MRANDSEQ       0x7FC
//#define CSR_MRANDSEQH      0x7FD
//#define CSR_MRANDSTATE     0x7FE
//#define CSR_MRANDSTATEH    0x7FF
//#define CSR_CYCLE        0xc00
//#define CSR_TIME         0xc01
//#define CSR_INSTRET      0xc02
//#define CSR_HPMCOUNTER3  0xc03
//#define CSR_CYCLEH       0xc80
//#define CSR_TIMEH        0xc81
//#define CSR_INSTRETH     0xc82
//#define CSR_HPMCOUNTER3H 0xc83
#define CSR_USTATUS        0x000
#define CSR_UIE            0x004
#define CSR_UTVEC          0x005
#define CSR_USCRATCH       0x040
#define CSR_UEPC           0x041
#define CSR_UCAUSE         0x042
#define CSR_UTVAL          0x043
#define CSR_UIP            0x044

#define CSR_VSTART         0x008  // vstart Vector start position
#define CSR_VXSAT          0x009  // vxsat Fixed-Point Saturate Flag
#define CSR_VXRM           0x00A  // vxrm  Fixed-Point Rounding Mode
#define CSR_VL             0xC20  // vl    Vector length
#define CSR_VTYPE          0xC21  // Vector data type register

#if 0
#define REG_MICM_CFG      "csr4032"
#define REG_MDCM_CFG      "csr4033"
#define REG_MMSC_CFG      "csr4034"
#define REG_MDETAILCAUSE  "csr4035"
#define REG_MILMB         "csr1984"
#define REG_MDLMB         "csr1985"
#define REG_MECC_CODE     "csr1986"
#define REG_MNVEC         "csr1987"
#define REG_MRANDSEQ      "csr2044"
#define REG_MRANDSEQH     "csr2045"
#define REG_MRANDSTATE    "csr2046"
#define REG_MRANDSTATEH   "csr2047"
#define REG_DEXC2DBG      "csr2016"
#define REG_DDCAUSE       "csr2017"

#define REG_MICM_CFG      "micm_cfg"
#define REG_MDCM_CFG      "mdcm_cfg"
#define REG_MMSC_CFG      "mmsc_cfg"
//#define REG_MDETAILCAUSE  "csr4035"
#define REG_MILMB         "milmb"
#define REG_MDLMB         "mdlmb"
#define REG_MECC_CODE     "mecc_code"
#define REG_MNVEC         "mnvec"
//#define REG_MRANDSEQ      "csr2044"
//#define REG_MRANDSEQH     "csr2045"
//#define REG_MRANDSTATE    "csr2046"
//#define REG_MRANDSTATEH   "csr2047"
#define REG_DEXC2DBG      "dexc2dbg"
#define REG_DDCAUSE       "ddcause"
#endif

#define REG_DDCAUSE_MASK    0xFF
#define REG_DDCAUSE_EBREAK  0  //Software Breakpoint (EBREAK)
#define REG_DDCAUSE_IAM     1  //Instruction Access Misaligned (IAM)
#define REG_DDCAUSE_IAF     2  //Instruction Access Fault (IAF)
#define REG_DDCAUSE_II      3  //Illegal Instruction (II)
#define REG_DDCAUSE_NMI     4  //Non-Maskable Interrupt (NMI)
#define REG_DDCAUSE_LAM     5  //Load Access Misaligned (LAM)
#define REG_DDCAUSE_LAF     6  //Load Access Fault (LAF)
#define REG_DDCAUSE_SAM     7  //Store Access Misaligned (SAM)
#define REG_DDCAUSE_SAF     8  //Store Access Fault (SAF)
#define REG_DDCAUSE_UEC     9  //U-mode Environment Call (UEC)
#define REG_DDCAUSE_SEC     10 //S-mode Environment Call (SEC)
#define REG_DDCAUSE_HEC     11 //H-mode Environment Call (HEC)
#define REG_DDCAUSE_MEC     12 //M-mode Environment Call (MEC)

#define REG_DDCAUSE_II_II           0  // Illegal instruction
#define REG_DDCAUSE_II_PRIV_INST    1  // Privileged instruction
#define REG_DDCAUSE_II_NONEXIT_CSR  2  // Non-existent CSR
#define REG_DDCAUSE_II_RO_CSR       3  // Read-only CSR update
#define REG_DDCAUSE_II_PRIV_CSR     4  // Privileged CSR access

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
	NDS_EBREAK_UNDEFINED = 0,
	//NDS_EBREAK_SKIP_BREAK = 0x7E00,
	/* break SWID */
	NDS_EBREAK_EXIT   = 0x005D005D,  //93,
	NDS_EBREAK_OPEN   = 0x04000400,  //1024,
	NDS_EBREAK_CLOSE  = 0x00390039,  //57,
	NDS_EBREAK_READ   = 0x003F003F,  //63,
	NDS_EBREAK_WRITE  = 0x00400040,  //64,
	NDS_EBREAK_LSEEK  = 0x003E003E,  //62,
	NDS_EBREAK_UNLINK = 0x04020402,  //1026,
	NDS_EBREAK_RENAME = 0x040A040A,  //1034,
	NDS_EBREAK_FSTAT  = 0x00500050,  //80,
	NDS_EBREAK_STAT   = 0x040E040E,  //1038,
	NDS_EBREAK_GETTIMEOFDAY = 0x00A900A9, //169,
	//NDS_EBREAK_ISATTY = 0x7F2B,
	//NDS_EBREAK_SYSTEM = 0x7F2C,
	//NDS_EBREAK_ERRNO = 0x7F2D,
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

enum cache_t{ICACHE, DCACHE};

struct nds32_v5 {
	int common_magic;

	/* next nds32_v5 in list */
	struct nds32_v5 *next;

	/** Backpointer to the target. */
	struct target *target;
	//void *arch_info;

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
	uint32_t prof_num_request;  //number samples expected
	uint32_t prof_num_samples;  //number samples in prof_samples[]
	uint64_t prof_samples[NDS32_MAX_PROFILE_SAMPLES]; //sample data
	uint32_t prof_total_samples;  //accumulate in buckets
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
extern const struct command_registration ndsv5_command_handlers[];
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

#define NDSV5_COMMON_MAGIC (int)0xADE55555

#endif /* OPENOCD_TARGET_ndsv5_H */
