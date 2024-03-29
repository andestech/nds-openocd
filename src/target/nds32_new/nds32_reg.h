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
#ifndef __NDS32_REG_H__
#define __NDS32_REG_H__

#include "nds32.h"

#define SRIDX(a, b, c)			((a << 7) | (b << 3) | c)
#define NDS32_REGISTER_DISABLE		(0x0)

enum nds32_reg_number_s {
	R0 = 0, /* general registers */
	R1,
	R2,
	R3,
	R4,
	R5,
	R6,
	R7,
	R8,
	R9,
	R10,
	R11,
	R12,
	R13,
	R14,
	R15,
	R16,
	R17,
	R18,
	R19,
	R20,
	R21,
	R22,
	R23,
	R24,
	R25,
	R26,
	R27,
	R28,
	R29,
	R30,
	R31,
	PC,
	D0LO,
	D0HI,
	D1LO,
	D1HI,
	DSP_LB,
	DSP_LE,
	DSP_LC,
	ITB,
	IFC_LP,
	CR0, /* system registers */
	CR1,
	CR2,
	CR3,
	CR4,
	CR5,
	CR6,
	CR7,
	IR0,
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	IR8,
	IR9,
	IR10,
	IR11,
	IR12,
	IR13,
	IR14,
	IR15,
	IR16,
	IR17,
	IR18,
	IR19,
	IR20,
	IR21,
	IR22,
	IR23,
	IR24,
	IR25,
	IR26,
	IR27,
	IR28,
	IR29,
	IR30,
	IR31,
	IR32,
	IR33,
	IR34,
	IR35,
	IR36,
	MR0,
	MR1,
	MR2,
	MR3,
	MR4,
	MR5,
	MR6,
	MR7,
	MR8,
	MR9,
	MR10,
	MR11,
	DR0,
	DR1,
	DR2,
	DR3,
	DR4,
	DR5,
	DR6,
	DR7,
	DR8,
	DR9,
	DR10,
	DR11,
	DR12,
	DR13,
	DR14,
	DR15,
	DR16,
	DR17,
	DR18,
	DR19,
	DR20,
	DR21,
	DR22,
	DR23,
	DR24,
	DR25,
	DR26,
	DR27,
	DR28,
	DR29,
	DR30,
	DR31,
	DR32,
	DR33,
	DR34,
	DR35,
	DR36,
	DR37,
	DR38,
	DR39,
	DR40,
	DR41,
	DR42,
	DR43,
	DR44,
	DR45,
	DR46,
	DR47,
	DR48,
	PFR0,
	PFR1,
	PFR2,
	PFR3,
	PFR4,
	DMAR0,
	DMAR1,
	DMAR2,
	DMAR3,
	DMAR4,
	DMAR5,
	DMAR6,
	DMAR7,
	DMAR8,
	DMAR9,
	DMAR10,
	DMAR11,
	DMAR12,
	RACR,
	FUCPR,
	IDR0,
	IDR1,
	IDR2,
	SECUR0,
	SECUR1,
	SECUR2,
	SECUR3,
	HSPR0,
	HSPR1,
	HSPR2,
	HSPR3,
	HSPR4,
	D0L24, /* audio registers */
	D1L24,
	I0,
	I1,
	I2,
	I3,
	I4,
	I5,
	I6,
	I7,
	M1,
	M2,
	M3,
	M5,
	M6,
	M7,
	MOD,
	LBE,
	LE,
	LC,
	ADM_VBASE,
	SHFT_CTL0,
	SHFT_CTL1,
	CB_CTL,
	CBB0,
	CBB1,
	CBB2,
	CBB3,
	CBE0,
	CBE1,
	CBE2,
	CBE3,
	FPCSR, /* fpu */
	FPCFG,
	FS0,
	FS1,
	FS2,
	FS3,
	FS4,
	FS5,
	FS6,
	FS7,
	FS8,
	FS9,
	FS10,
	FS11,
	FS12,
	FS13,
	FS14,
	FS15,
	FS16,
	FS17,
	FS18,
	FS19,
	FS20,
	FS21,
	FS22,
	FS23,
	FS24,
	FS25,
	FS26,
	FS27,
	FS28,
	FS29,
	FS30,
	FS31,
	FD0,
	FD1,
	FD2,
	FD3,
	FD4,
	FD5,
	FD6,
	FD7,
	FD8,
	FD9,
	FD10,
	FD11,
	FD12,
	FD13,
	FD14,
	FD15,
	FD16,
	FD17,
	FD18,
	FD19,
	FD20,
	FD21,
	FD22,
	FD23,
	FD24,
	FD25,
	FD26,
	FD27,
	FD28,
	FD29,
	FD30,
	FD31,

	TOTAL_REG_NUM,
};

enum nds32_reg_type_s {
	NDS32_REG_TYPE_GPR = 0,
	NDS32_REG_TYPE_SPR,
	NDS32_REG_TYPE_CR,
	NDS32_REG_TYPE_IR,
	NDS32_REG_TYPE_MR,
	NDS32_REG_TYPE_DR,
	NDS32_REG_TYPE_PFR,
	NDS32_REG_TYPE_DMAR,
	NDS32_REG_TYPE_RACR,
	NDS32_REG_TYPE_IDR,
	NDS32_REG_TYPE_AUMR,
	NDS32_REG_TYPE_SECURE,
	NDS32_REG_TYPE_FPU,
	NDS32_REG_TYPE_ACE,     /* Currently, user defined register is not supported yet. */
	NDS32_REG_TYPE_COP0,	/* Enforce the continuity - program logic depends on it. */
	NDS32_REG_TYPE_COP1 = NDS32_REG_TYPE_COP0 + 1,
	NDS32_REG_TYPE_COP2 = NDS32_REG_TYPE_COP0 + 2,
	NDS32_REG_TYPE_COP3 = NDS32_REG_TYPE_COP0 + 3,
	NDS32_REG_TYPE_HSPR,  /* HW Stack Protection Control Register */
};

struct nds32_reg_s {
	const char *simple_mnemonic;
	const char *symbolic_mnemonic;
	uint32_t sr_index;
	enum nds32_reg_type_s type;
	uint32_t size;
};

struct nds32_reg_exception_s {
	uint32_t reg_num;
	uint32_t ex_value_bit_pos;
	uint32_t ex_value_mask;
	uint32_t ex_value;
};

struct nds32_reg_access_op_s {
	uint32_t *read_insn;
	unsigned *size_of_read_insn;
	uint32_t *write_insn;
	unsigned *size_of_write_insn;
};

extern struct nds32_reg_s *nds32_regs;

void nds32_reg_init(struct nds32 *nds32_p);
uint32_t nds32_reg_sr_index(uint32_t number);
enum nds32_reg_type_s nds32_reg_type(uint32_t number);
uint32_t nds32_reg_size(uint32_t number);
const char *nds32_reg_simple_name(uint32_t number);
const char *nds32_reg_symbolic_name(uint32_t number);
bool nds32_reg_exception(uint32_t number, uint32_t value);
int nds32_get_reg_number(const char *name);

#endif
