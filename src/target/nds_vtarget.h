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

#ifndef OPENOCD_TARGET_NDS_VTARGET_H
#define OPENOCD_TARGET_NDS_VTARGET_H

/* AndeStar V5 CSRs */
#define CSR_MICM_CFG       0xFC0
#define CSR_MDCM_CFG       0xFC1
#define CSR_MMSC_CFG       0xFC2
#define CSR_MMSC_CFG2      0xFC3

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

/* bug17785 comment9 :remove other csrs */
#define CSR_MIRQ_ENTRY     0x7EC
/*
#define CSR_PUSHMXSTATUS   0x7EB
#define CSR_MINTSEL_JAL    0x7ED
#define CSR_PUSHMCAUSE     0x7EE
#define CSR_PUSHMEPC       0x7EF
*/

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

enum target_run_mode {
	RUN_MODE_DEBUG,
	RUN_MODE_PROFILE,
	RUN_MODE_MAX,
};

/* gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
 * its source tree. We must interpret the numbers the same here. */

enum nds_memory_access {
	NDS_MEMORY_ACC_BUS = 0,
	NDS_MEMORY_ACC_CPU,
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

struct nds_vtarget {
	/* next nds_vtarget in list */
	struct nds_vtarget *next;

	/** Backpointer to the target. */
	struct target *target;
	/* void *arch_info; */

	/** Memory information */
	struct nds32_v5_memory memory;

	/* Flag to indicate the target is attached by debugger or not */
	bool attached;
	bool is_program_exit;

	/** Flag reporting whether continue/step hits syscall or not */
	bool hit_syscall;

	/** gdb run mode */
	enum target_run_mode gdb_run_mode;
	bool gdb_run_mode_acting;
	bool gdb_run_mode_halt;
	uint32_t gmon_64_bit;

	/** register initial */
	bool execute_register_init;

	uint32_t nds_vector_length;
};

struct nds32_mem_access_attr {
	/** the start address of memory block */
	uint64_t lowAddr;

	/** the end address of memory block */
	uint64_t highAddr;

	/** accesses memory size (1/2/4) */
	unsigned int access_size;
};

/** extern varlable */
extern char **vtarget_gpr_and_fpu_name;

/* from nds32.c */
extern struct nds32_mem_access_attr all_mem_access_attr[];
extern uint32_t curr_mem_access_attr_index;

/** extern functions */
extern struct nds_vtarget *target_to_nds_vtarget(struct target *target);
extern int vtarget_target_create(struct target *target, Jim_Interp *interp);
extern const struct command_registration ndsvtarget_command_handlers[];

extern int vtarget_handle_examine(struct target *target);
extern char *vtarget_get_CSR_name(struct target *target, uint32_t csr_id);
extern const struct command_registration vtarget_command_handlers[];

#endif /* OPENOCD_TARGET_NDS_VTARGET_H */
