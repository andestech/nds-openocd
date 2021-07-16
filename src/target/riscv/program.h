#ifndef TARGET__RISCV__PROGRAM_H
#define TARGET__RISCV__PROGRAM_H

#include "riscv.h"

#define RISCV_MAX_DEBUG_BUFFER_SIZE 32
#define RISCV_REGISTER_COUNT 32
#define RISCV_DSCRATCH_COUNT 2

/* The various RISC-V debug specifications all revolve around setting up
 * program buffers and executing them on the target.  This structure contains a
 * single program, which can then be executed on targets.  */
struct riscv_program {
	struct target *target;

	uint32_t debug_buffer[RISCV_MAX_DEBUG_BUFFER_SIZE];

	/* Number of 32-bit instructions in the program. */
	size_t instruction_count;

	/* Side effects of executing this program.  These must be accounted for
	 * in order to maintain correct executing of the target system.  */
	bool writes_xreg[RISCV_REGISTER_COUNT];

	/* XLEN on the target. */
	int target_xlen;
};

/* Initializes a program with the header. */
int riscv_program_init(struct riscv_program *p, struct target *t);

/* Write the program to the program buffer. */
int riscv_program_write(struct riscv_program *program);

/* Executes a program, returning 0 if the program successfully executed.  Note
 * that this may cause registers to be saved or restored, which could result to
 * calls to things like riscv_save_register which itself could require a
 * program to execute.  That's OK, just make sure this eventually terminates.
 * */
int riscv_program_exec(struct riscv_program *p, struct target *t);
int riscv_program_load(struct riscv_program *p, struct target *t);

/* Clears a program, removing all the state associated with it. */
int riscv_program_clear(struct riscv_program *p, struct target *t);

/* A lower level interface, you shouldn't use this unless you have a reason. */
int riscv_program_insert(struct riscv_program *p, riscv_insn_t i);

/* There is hardware support for saving at least one register.  This register
 * doesn't need to be saved/restored the usual way, which is useful during
 * early initialization when we can't save/restore arbitrary registerrs to host
 * memory. */
int riscv_program_save_to_dscratch(struct riscv_program *p, enum gdb_regno to_save);

/* Helpers to assembly various instructions.  Return 0 on success.  These might
 * assembly into a multi-instruction sequence that overwrites some other
 * register, but those will be properly saved and restored. */
int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);

int riscv_program_swr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_shr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_sbr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr);
int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr);

int riscv_program_fence_i(struct riscv_program *p);
int riscv_program_fence(struct riscv_program *p);
int riscv_program_ebreak(struct riscv_program *p);

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t i);

#if _NDS_RW_MEMORY_64BITS_
int riscv_program_sdr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset);
int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset);
#endif

#if _NDS_V5_ONLY_
int riscv_program_lui(struct riscv_program *p, enum gdb_regno d, int32_t u);
int riscv_program_slli(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int32_t u);
int riscv_program_or(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s1, enum gdb_regno s2);
int riscv_program_li(struct riscv_program *p, enum gdb_regno d, riscv_reg_t c);
int riscv_program_li64(struct riscv_program *p, enum gdb_regno d, enum gdb_regno t, riscv_reg_t c);
int riscv_program_vsetvli(struct riscv_program *p, enum gdb_regno rd, uint32_t SEW);
int riscv_program_vsetvl(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno rs2);
int riscv_program_vmv_x_s(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2);
int riscv_program_vslide1down_vx(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2, enum gdb_regno rs1);
#endif

#endif
