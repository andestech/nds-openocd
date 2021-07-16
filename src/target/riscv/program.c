#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/register.h"
#include "riscv.h"
#include "program.h"
#include "helper/log.h"

#include "asm.h"
#include "encoding.h"

/* Program interface. */
int riscv_program_init(struct riscv_program *p, struct target *target)
{
	memset(p, 0, sizeof(*p));
	p->target = target;
	p->instruction_count = 0;
	p->target_xlen = riscv_xlen(target);
	for (size_t i = 0; i < RISCV_REGISTER_COUNT; ++i)
		p->writes_xreg[i] = 0;

	for (size_t i = 0; i < RISCV_MAX_DEBUG_BUFFER_SIZE; ++i)
		p->debug_buffer[i] = -1;

	return ERROR_OK;
}

#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
extern struct riscv_batch *write_debug_buffer_batch;
extern int riscv_batch_run(struct riscv_batch *batch);
extern void riscv_batch_free(struct riscv_batch *batch);
#endif

int riscv_program_write(struct riscv_program *program)
{
	for (unsigned i = 0; i < program->instruction_count; ++i) {
		LOG_DEBUG("%p: debug_buffer[%02x] = DASM(0x%08x)", program, i, program->debug_buffer[i]);
		if (riscv_write_debug_buffer(program->target, i,
					program->debug_buffer[i]) != ERROR_OK)
			return ERROR_FAIL;
	}
	#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
	if (write_debug_buffer_batch) {
		if (riscv_batch_run(write_debug_buffer_batch) != ERROR_OK) {
			LOG_DEBUG("riscv_batch_run ERROR");
		}
		riscv_batch_free(write_debug_buffer_batch);
		write_debug_buffer_batch = NULL;
	}
	#endif
	return ERROR_OK;
}

/** Add ebreak and execute the program. */
int riscv_program_exec(struct riscv_program *p, struct target *t)
{
	keep_alive();

	riscv_reg_t saved_registers[GDB_REGNO_XPR31 + 1];
	for (size_t i = GDB_REGNO_ZERO + 1; i <= GDB_REGNO_XPR31; ++i) {
		if (p->writes_xreg[i]) {
			LOG_DEBUG("Saving register %d as used by program", (int)i);
			int result = riscv_get_register(t, &saved_registers[i], i);
			if (result != ERROR_OK)
				return result;
		}
	}

	if (riscv_program_ebreak(p) != ERROR_OK) {
		LOG_ERROR("Unable to write ebreak");
		for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
			LOG_ERROR("ram[%02x]: DASM(0x%08lx) [0x%08lx]", (int)i, (long)p->debug_buffer[i], (long)p->debug_buffer[i]);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Unable to write ebreak");
#endif
		return ERROR_FAIL;
	}

	if (riscv_program_write(p) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_execute_debug_buffer(t) != ERROR_OK) {
		LOG_ERROR("Unable to execute program %p", p);
		return ERROR_FAIL;
	}

	for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
		if (i >= riscv_debug_buffer_size(p->target))
			p->debug_buffer[i] = riscv_read_debug_buffer(t, i);

	for (size_t i = GDB_REGNO_ZERO; i <= GDB_REGNO_XPR31; ++i)
		if (p->writes_xreg[i])
			riscv_set_register(t, i, saved_registers[i]);

	return ERROR_OK;
}

#if _NDS_RW_MEMORY_64BITS_
int riscv_program_sdr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	LOG_DEBUG("%s, d=0x%x, b=0x%x, offset=0x%x", __func__, d, b, offset);
	return riscv_program_insert(p, sd(d, b, offset));
}
#endif

int riscv_program_swr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sw(d, b, offset));
}

int riscv_program_shr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sh(d, b, offset));
}

int riscv_program_sbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sb(d, b, offset));
}

#if _NDS_RW_MEMORY_64BITS_
int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	LOG_DEBUG("%s, d=0x%x, b=0x%x, offset=0x%x", __func__, d, b, offset);
	return riscv_program_insert(p, ld(d, b, offset));
}
#endif

int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lw(d, b, offset));
}

int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lh(d, b, offset));
}

int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lb(d, b, offset));
}

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0 && csr <= GDB_REGNO_CSR4095);
	return riscv_program_insert(p, csrrs(d, GDB_REGNO_ZERO, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrw(GDB_REGNO_ZERO, s, csr - GDB_REGNO_CSR0));
}

int riscv_program_fence_i(struct riscv_program *p)
{
	return riscv_program_insert(p, fence_i());
}

int riscv_program_fence(struct riscv_program *p)
{
	return riscv_program_insert(p, fence());
}

int riscv_program_ebreak(struct riscv_program *p)
{
	struct target *target = p->target;
	RISCV_INFO(r);
	if (p->instruction_count == riscv_debug_buffer_size(p->target) &&
			r->impebreak) {
		return ERROR_OK;
	}
	return riscv_program_insert(p, ebreak());
}

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t u)
{
	return riscv_program_insert(p, addi(d, s, u));
}

int riscv_program_insert(struct riscv_program *p, riscv_insn_t i)
{
	if (p->instruction_count >= riscv_debug_buffer_size(p->target)) {
		LOG_ERROR("Unable to insert instruction:");
		LOG_ERROR("  instruction_count=%d", (int)p->instruction_count);
		LOG_ERROR("  buffer size      =%d", (int)riscv_debug_buffer_size(p->target));
		return ERROR_FAIL;
	}

	p->debug_buffer[p->instruction_count] = i;
	p->instruction_count++;
	return ERROR_OK;
}

#if _NDS_V5_ONLY_	//For ACE
int riscv_program_lui(struct riscv_program *p, enum gdb_regno d, int32_t u)
{
	return riscv_program_insert(p, lui(d, u));
}

int riscv_program_slli(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int32_t u)
{
 return riscv_program_insert(p, slli(d, s, u));
}

int riscv_program_or(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s1, enum gdb_regno s2)
{
 return riscv_program_insert(p, or_r(d, s1, s2));
}

int riscv_program_bfoz64(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, uint8_t msb, uint8_t lsb)
{
	return riscv_program_insert(p, bfoz64(d, s, msb, lsb));
}

int riscv_program_li(struct riscv_program *p, enum gdb_regno d, riscv_reg_t c)
{
	riscv_reg_t sign_ext = (c & 0x800) ? (-1 - 0xFFF) : 0;
	if (riscv_program_lui(p, d, (c - sign_ext) >> 12) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_addi(p, d, d, c & 0xFFF) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int riscv_program_li64(struct riscv_program *p, enum gdb_regno d, enum gdb_regno t, riscv_reg_t c)
{
	riscv_reg_t sign_ext = (c & 0x800) ? (-1 - 0xFFF) : 0;

	if (riscv_program_lui(p, t, (c - sign_ext) >> 12) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_addi(p, t, t, c & 0xFFF) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_bfoz64(p, t, t, 31, 0) != ERROR_OK)	/* zero extension */
		return ERROR_FAIL;

	// store high part value to destination register (i.e., d)
	c >>= 32;	
	sign_ext = (c & 0x800) ? (-1 - 0xFFF) : 0;
	if (riscv_program_lui(p, d, (c - sign_ext) >> 12) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_addi(p, d, d, c & 0xFFF) != ERROR_OK)
		return ERROR_FAIL;
	// shift high value to high part
	if (riscv_program_slli(p, d, d, 32) != ERROR_OK)
		return ERROR_FAIL;
	// OR high and low values to form 64-bits one
	if (riscv_program_or(p, d, d, t) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

int riscv_program_vsetvl(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno rs2)
{
	// vsetvl  rd, rs1, rs2    # rd = new vl, rs1 = AVL, rs2 = new vtype value
	//                         # if rs1 = x0, then use maximum vector length
	//  31 30    25 24  20 19 15 14 12 11   7 6     0
	//  1 | 000000 | rs2  | rs1 | 111 |  rd  |1010111|         vsetvl
	//  1     6       5      5     3     5       7
	riscv_insn_t opcode = 0;

	opcode = 0x80007057;
	opcode |= ((rd << 7) | (rs2 << 20));
	return riscv_program_insert(p, opcode);
}

int riscv_program_vsetvli(struct riscv_program *p, enum gdb_regno rd, uint32_t SEW)
{
  //  31 30         20 19  15 14   12 11    7 6        0
  //  0 | zimm[10:0] |  rs1  |  111  |  rd   | 1010111 |    =>  vsetvli
	// 00c07557          	vsetvli	a0,zero,e64,m1,d1
	// 00807557          	vsetvli	a0,zero,e32,m1,d1
	// 00407557          	vsetvli	a0,zero,e16,m1,d1
	// 00007557          	vsetvli	a0,zero,e8,m1,d1
	uint32_t vtypei_SEW = 0, vtypei = 0;
	riscv_insn_t opcode = 0;

	if (SEW == 8)
		vtypei_SEW = 0;
	else if (SEW == 16)
		vtypei_SEW = 1;
	else if (SEW == 32)
		vtypei_SEW = 2;
	else if (SEW == 64)
		vtypei_SEW = 3;
	else {
		return ERROR_FAIL;
	}

	vtypei = ((vtypei_SEW << 2) << 20);
	LOG_DEBUG("vtypei: 0x%x", vtypei);
	opcode = 0x7057;
	opcode |= ((rd << 7) | vtypei);
	return riscv_program_insert(p, opcode);
}

int riscv_program_vmv_x_s(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2)
{
	// 31     26  25   24      20 19      15 14   12 11      7 6     0
  // funct6   | vm  |   vs2    |    rs1   | 0 1 0 |  vd/rd  |1010111| OP-V (OPMVV)
	// 010000
	riscv_insn_t opcode;
	enum gdb_regno rs1 = 0;
	opcode = 0x42002057;
	opcode |= ((rd << 7) | (rs1 << 15) | ((vs2 - GDB_REGNO_V0) << 20));
	return riscv_program_insert(p, opcode);
}

int riscv_program_vslide1down_vx(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2, enum gdb_regno rs1)
{
	// 31     26  25  24      20 19      15 14   12 11      7 6     0
  // funct6   | vm  |   vs2   |   rs1    | 1 1 0 |  vd/rd  |1010111| OP-V (OPMVX)
	// 001111     1      0 0000    0111 0    110        0       57      3e076057: vslide1down.vx	v0,v0,a4
	riscv_insn_t opcode;

	opcode = 0x3e006057;
	opcode |= (((rd - GDB_REGNO_V0) << 7) | (rs1 << 15) | ((vs2 - GDB_REGNO_V0) << 20));
	return riscv_program_insert(p, opcode);
}

#endif

