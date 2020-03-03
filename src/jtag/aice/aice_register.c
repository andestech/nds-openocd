/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
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

#include <target/target.h>
#include <target/nds32_log.h>
#include "aice_port.h"
#include "aice_apis.h"
#include "aice_jdp.h"
#include "aice_usb_pack_format.h"
#include "aice_usb_command.h"

#define DTR_INDEX (0)
#define DTR_BUF (R0)

extern uint32_t nds32_cop_reg_base_id[4];
static uint32_t (*nds32_get_cop_reg_read_insn)(uint32_t num);
static uint32_t (*nds32_get_cop_reg_write_insn)(uint32_t num);
static uint32_t* (*nds32_get_ace_reg_read_insn)(uint32_t num);
static uint32_t* (*nds32_get_ace_reg_write_insn)(uint32_t tnum);

extern uint32_t aice_usb_pack_command;
extern unsigned nds32_reg_get_size_of_read_insn(unsigned number);
extern unsigned nds32_reg_get_size_of_write_insn(unsigned number);
extern uint32_t* nds32_reg_get_read_insn(uint32_t number);
extern uint32_t* nds32_reg_get_write_insn(uint32_t number);
int aice_reg_set_ace_access_op(struct target *target)
{
	nds32_get_ace_reg_read_insn = nds32_reg_get_read_insn;
	nds32_get_ace_reg_write_insn = nds32_reg_get_write_insn;
	return ERROR_OK;
}

int aice_reg_set_cop_access_op(uint32_t (*get_read_insn)(uint32_t), uint32_t (*get_write_insn)(uint32_t))
{
	nds32_get_cop_reg_read_insn = get_read_insn;
	nds32_get_cop_reg_write_insn = get_write_insn;
	return ERROR_OK;
}

int aice_get_read_reg_inst(uint32_t num, uint32_t *instructions)
{
	if (NDS32_REG_TYPE_GPR == nds32_reg_type(num)) { /* general registers */
		instructions[0] = MTSR_DTR(num);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_SPR == nds32_reg_type(num)) { /* user special registers */
		instructions[0] = MFUSR_G0(DTR_BUF, nds32_reg_sr_index(num));
		instructions[1] = MTSR_DTR(DTR_BUF);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_AUMR == nds32_reg_type(num)) { /* audio registers */
		if ((CB_CTL <= num) && (num <= CBE3)) {
			instructions[0] = AMFAR2(DTR_BUF, nds32_reg_sr_index(num));
			instructions[1] = MTSR_DTR(DTR_BUF);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			instructions[0] = AMFAR(DTR_BUF, nds32_reg_sr_index(num));
			instructions[1] = MTSR_DTR(DTR_BUF);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		}
	} else if (NDS32_REG_TYPE_FPU == nds32_reg_type(num)) { /* fpu registers */
		if (FPCSR == num) {
			instructions[0] = FMFCSR(DTR_BUF);
			instructions[1] = MTSR_DTR(DTR_BUF);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else if (FPCFG == num) {
			instructions[0] = FMFCFG(DTR_BUF);
			instructions[1] = MTSR_DTR(DTR_BUF);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			if (FS0 <= num && num <= FS31) { /* single precision */
				instructions[0] = FMFSR(DTR_BUF, nds32_reg_sr_index(num));
				instructions[1] = MTSR_DTR(DTR_BUF);
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			} else if (FD0 <= num && num <= FD31) { /* double precision */
				instructions[0] = FMFDR(DTR_BUF, nds32_reg_sr_index(num));
				instructions[1] = MTSR_DTR(DTR_BUF);
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			}
		}
	} else if (NDS32_REG_TYPE_COP0 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP1 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP2 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP3 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_ACE == nds32_reg_type(num)) {
		uint32_t cop_id = nds32_reg_type(num) - NDS32_REG_TYPE_COP0;
		uint32_t imm12 = (num - nds32_cop_reg_base_id[cop_id]);

		instructions[0] = MFCPW(0, imm12, cop_id);//nds32_get_cop_reg_read_insn(num);
		//LOG_DEBUG("cop_id=%d, imm12=%d, [0x%x]", cop_id, imm12, instructions[0]);
		instructions[1] = MTSR_DTR(0);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else { /* system registers */
		instructions[0] = MFSR(DTR_BUF, nds32_reg_sr_index(num));
		instructions[1] = MTSR_DTR(DTR_BUF);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	}
	return ERROR_OK;
}

int aice_get_write_reg_inst(uint32_t num, uint32_t *instructions)
{
	if (NDS32_REG_TYPE_GPR == nds32_reg_type(num)) { /* general registers */
		instructions[0] = MFSR_DTR(num);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_SPR == nds32_reg_type(num)) { /* user special registers */
		instructions[0] = MFSR_DTR(DTR_BUF);
		instructions[1] = MTUSR_G0(DTR_BUF, nds32_reg_sr_index(num));
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_AUMR == nds32_reg_type(num)) { /* audio registers */
		if ((CB_CTL <= num) && (num <= CBE3)) {
			instructions[0] = MFSR_DTR(DTR_BUF);
			instructions[1] = AMTAR2(DTR_BUF, nds32_reg_sr_index(num));
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			instructions[0] = MFSR_DTR(DTR_BUF);
			instructions[1] = AMTAR(DTR_BUF, nds32_reg_sr_index(num));
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		}
	} else if (NDS32_REG_TYPE_FPU == nds32_reg_type(num)) { /* fpu registers */
		if (FPCSR == num) {
			instructions[0] = MFSR_DTR(DTR_BUF);
			instructions[1] = FMTCSR(DTR_BUF);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else if (FPCFG == num) {
			/* FPCFG is readonly, no instruction to write FPCFG !! */
			return ERROR_FAIL;
		} else {
			if (FS0 <= num && num <= FS31) { /* single precision */
				instructions[0] = MFSR_DTR(DTR_BUF);
				instructions[1] = FMTSR(DTR_BUF, nds32_reg_sr_index(num));
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			} else if (FD0 <= num && num <= FD31) { /* double precision */
				instructions[0] = MFSR_DTR(DTR_BUF);
				instructions[1] = FMTDR(DTR_BUF, nds32_reg_sr_index(num));
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			}
		}
	} else if (NDS32_REG_TYPE_COP0 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP1 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP2 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_COP3 == nds32_reg_type(num) ||
			NDS32_REG_TYPE_ACE == nds32_reg_type(num)) {
		uint32_t cop_id = nds32_reg_type(num) - NDS32_REG_TYPE_COP0;
		uint32_t imm12 = (num - nds32_cop_reg_base_id[cop_id]);

		instructions[0] = MFSR_DTR(0);
		instructions[1] = MTCPW(0, imm12, cop_id);//nds32_get_cop_reg_write_insn(num);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else {
		instructions[0] = MFSR_DTR(DTR_BUF);
		instructions[1] = MTSR(DTR_BUF, nds32_reg_sr_index(num));
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	}
	return ERROR_OK;
}

int aice_read_reg(struct target *target, uint32_t num, uint32_t *val)
{
	LOG_DEBUG("aice_read_reg, reg_no: 0x%08x", num);
	uint32_t instructions[4]; /** execute instructions in DIM */
	aice_get_read_reg_inst(num, &instructions[0]);

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		uint32_t append_mode = 0;
		uint32_t value_dtr = 0, value_edmsw = 0, value_dbger = 0;
		if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK)
			append_mode = 1;
		else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		}
		/** fill DIM */
		aice_write_dim(target, &instructions[0], 4);
		/** clear DBGER.DPED */
		aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_DPED);
		/** execute DIM */
		aice_do_execute(target);
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);
		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		aice_read_edm(target, JDP_R_DTR, 0, &value_dtr, 1);

		uint32_t d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);  // WRITE_DIM
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);          // WRITE_MISC
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);          // EXECUTE
		uint32_t read0_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_MISC
		uint32_t read1_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_EDMSR
		uint32_t read2_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_DTR

		if (append_mode == 1) {
			aice_packet_append_size += d2h_size;
			return ERROR_OK;
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[256];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], d2h_size);

		unsigned char *pread0_byte = (unsigned char *)&usbin_data[read0_byte_offset];
		unsigned char *pread1_byte = (unsigned char *)&usbin_data[read1_byte_offset];
		unsigned char *pread2_byte = (unsigned char *)&usbin_data[read2_byte_offset];
		value_dbger = (unsigned char)pread0_byte[3];
		value_dbger |= ((unsigned char)pread0_byte[2] << 8);
		value_dbger |= ((unsigned char)pread0_byte[1] << 16);
		value_dbger |= ((unsigned char)pread0_byte[0] << 24);
		if ((value_dbger & NDS_DBGER_DPED) == 0) {
			return ERROR_FAIL;
		}
		value_edmsw = (unsigned char)pread1_byte[3];
		value_edmsw |= ((unsigned char)pread1_byte[2] << 8);
		value_edmsw |= ((unsigned char)pread1_byte[1] << 16);
		value_edmsw |= ((unsigned char)pread1_byte[0] << 24);
		if ((value_edmsw & NDS_EDMSW_WDV) == 0) {
			return ERROR_FAIL;
		}
		value_dtr = (unsigned char)pread2_byte[3];
		value_dtr |= ((unsigned char)pread2_byte[2] << 8);
		value_dtr |= ((unsigned char)pread2_byte[1] << 16);
		value_dtr |= ((unsigned char)pread2_byte[0] << 24);
		*val = value_dtr;
		LOG_DEBUG("DTR:0x%x, EDMSW: 0x%x, DBGER: 0x%x", value_dtr, value_edmsw, value_dbger);
		return ERROR_OK;
	} else {
		aice_execute_dim(target, instructions, 4);
		if (aice_read_dtr(target, val) != ERROR_OK) {
			NDS32_LOG(NDS32_ERRMSG_TARGET_READ_DTR);
			return ERROR_FAIL;
		}
		return ERROR_OK;
	}
}

int aice_write_reg(struct target *target, uint32_t num, uint32_t val)
{
	LOG_DEBUG("aice_write_reg, reg_no: 0x%08x, value: 0x%08x", num, val);
	uint32_t instructions[4]; /** execute instructions in DIM */
	if (aice_get_write_reg_inst(num, &instructions[0]) != ERROR_OK) {
		/* if readonly, do nothing */
		return ERROR_OK;
	}

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		uint32_t append_mode = 0;
		uint32_t EDMData = val, value_edmsw = 0, value_dbger = 0;

		if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK)
			append_mode = 1;
		else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		}
		aice_write_edm(target, JDP_W_DTR, 0, (uint32_t*)&EDMData, 1);
		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		/** fill DIM */
		aice_write_dim(target, &instructions[0], 4);
		/** clear DBGER.DPED */
		aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_DPED);
		/** execute DIM */
		aice_do_execute(target);
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);

		uint32_t d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);  // WRITE_DTR
		uint32_t read1_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_EDMSR
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);          // WRITE_DIM
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);          // WRITE_MISC
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);          // EXECUTE
		uint32_t read2_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_MISC

		if (append_mode == 1) {
			aice_packet_append_size += d2h_size;
			return ERROR_OK;
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[256];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], d2h_size);

		unsigned char *pread1_byte = (unsigned char *)&usbin_data[read1_byte_offset];
		unsigned char *pread2_byte = (unsigned char *)&usbin_data[read2_byte_offset];
		value_edmsw = (unsigned char)pread1_byte[3];
		value_edmsw |= ((unsigned char)pread1_byte[2] << 8);
		value_edmsw |= ((unsigned char)pread1_byte[1] << 16);
		value_edmsw |= ((unsigned char)pread1_byte[0] << 24);
		if ((value_edmsw & NDS_EDMSW_RDV) == 0) {
			return ERROR_FAIL;
		}
		value_dbger = (unsigned char)pread2_byte[3];
		value_dbger |= ((unsigned char)pread2_byte[2] << 8);
		value_dbger |= ((unsigned char)pread2_byte[1] << 16);
		value_dbger |= ((unsigned char)pread2_byte[0] << 24);
		if ((value_dbger & NDS_DBGER_DPED) == 0) {
			return ERROR_FAIL;
		}
		LOG_DEBUG("EDMSW: 0x%x, DBGER: 0x%x", value_edmsw, value_dbger);
		return ERROR_OK;
	} else {
		if (aice_write_dtr(target, val) != ERROR_OK)
			return ERROR_FAIL;
		int result = aice_execute_dim(target, instructions, 4);
		return result;
	}
}

int aice_backup_tmp_registers(struct target *target)
{
	uint32_t coreid = target_to_coreid(target);
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t value_r0 = 0, value_r1 = 0, dtr_backup = 0;
	uint32_t value_edmsw = 0;

	enum target_state backup_state = nds32->target->state;
	nds32->target->state = TARGET_HALTED;

	LOG_DEBUG("backup_tmp_registers -");

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		uint32_t read1_byte_offset = 0;
		uint32_t read2_byte_offset = 0;
		uint32_t read3_byte_offset = 0;

		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;

		/* backup target DTR first(if the target DTR is valid) */
		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, (uint32_t*)&value_edmsw, 1);
		aice_packet_append_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);  // READ_EDMSR
		read1_byte_offset = (aice_packet_append_size + 4);
		aice_read_edm(target, JDP_R_DTR, 0, &dtr_backup, 1);
		aice_packet_append_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);  // READ_DTR

		nds32->core_cache->reg_list[R0].valid = false;
		nds32->core_cache->reg_list[R1].valid = false;

		read2_byte_offset = (aice_packet_append_size + 32);
		nds32_get_mapped_reg(nds32, R0, &value_r0);

		read3_byte_offset = (aice_packet_append_size + 32);
		nds32_get_mapped_reg(nds32, R1, &value_r1);

		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[256];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], aice_packet_append_size);
		unsigned char *pread_byte = (unsigned char *)&usbin_data[4];
		value_edmsw = (unsigned char)pread_byte[3];
		value_edmsw |= ((unsigned char)pread_byte[2] << 8);
		value_edmsw |= ((unsigned char)pread_byte[1] << 16);
		value_edmsw |= ((unsigned char)pread_byte[0] << 24);

		pread_byte = (unsigned char *)&usbin_data[read1_byte_offset];
		dtr_backup = (unsigned char)pread_byte[3];
		dtr_backup |= ((unsigned char)pread_byte[2] << 8);
		dtr_backup |= ((unsigned char)pread_byte[1] << 16);
		dtr_backup |= ((unsigned char)pread_byte[0] << 24);

		pread_byte = (unsigned char *)&usbin_data[read2_byte_offset];
		value_r0 = (unsigned char)pread_byte[3];
		value_r0 |= ((unsigned char)pread_byte[2] << 8);
		value_r0 |= ((unsigned char)pread_byte[1] << 16);
		value_r0 |= ((unsigned char)pread_byte[0] << 24);

		pread_byte = (unsigned char *)&usbin_data[read3_byte_offset];
		value_r1 = (unsigned char)pread_byte[3];
		value_r1 |= ((unsigned char)pread_byte[2] << 8);
		value_r1 |= ((unsigned char)pread_byte[1] << 16);
		value_r1 |= ((unsigned char)pread_byte[0] << 24);
		//LOG_DEBUG("value_edmsw: 0x%08x, dtr_backup: 0x%08x",
		//	value_edmsw, dtr_backup);
		struct reg *r;
		r = nds32->core_cache->reg_list + R0;
		buf_set_u32(r->value, 0, 32, value_r0);
		r = nds32->core_cache->reg_list + R1;
		buf_set_u32(r->value, 0, 32, value_r1);
	} else {
		/* backup target DTR first(if the target DTR is valid) */
		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, (uint32_t*)&value_edmsw, 1);
		aice_read_edm(target, JDP_R_DTR, 0, &dtr_backup, 1);

		nds32->core_cache->reg_list[R0].valid = false;
		nds32->core_cache->reg_list[R1].valid = false;
		nds32_get_mapped_reg(nds32, R0, &value_r0);
		nds32_get_mapped_reg(nds32, R1, &value_r1);
	}

	core_info[coreid].edmsw_backup = value_edmsw;
	if (value_edmsw & NDS_EDMSW_WDV) { /* EDMSW.WDV == 1 */
		core_info[coreid].target_dtr_valid = true;
		core_info[coreid].target_dtr_backup = dtr_backup;
		LOG_DEBUG("Backup target DTR: 0x%08x", core_info[coreid].target_dtr_backup);
	} else {
		core_info[coreid].target_dtr_valid = false;
	}

	/* backup host DTR(if the host DTR is valid) */
	if (value_edmsw & NDS_EDMSW_RDV) { /* EDMSW.RDV == 2*/
		/* read out host DTR and write into target DTR, then use aice_read_edmsr to
		 * read out */
		uint32_t instructions[4] = {
			MFSR_DTR(R0), /* R0 has already been backup */
			DSB,
			MTSR_DTR(R0),
			BEQ_MINUS_12
		};
		aice_execute_dim(target, instructions, 4);

		aice_read_dtr(target, &core_info[coreid].host_dtr_backup);
		core_info[coreid].host_dtr_valid = true;

		LOG_DEBUG("Backup host DTR: 0x%08x", core_info[coreid].host_dtr_backup);
	} else {
		core_info[coreid].host_dtr_valid = false;
	}

	LOG_DEBUG("r0: 0x%08x, r1: 0x%08x",
			value_r0, value_r1);
	nds32->target->state = backup_state;
	return ERROR_OK;
}

int aice_restore_tmp_registers(struct target *target)
{
	uint32_t coreid = target_to_coreid(target);
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t value_r0 = 0, value_r1 = 0;
	enum target_state backup_state = nds32->target->state;
	nds32->target->state = TARGET_HALTED;

	nds32_get_mapped_reg(nds32, R0, &value_r0);
	nds32_get_mapped_reg(nds32, R1, &value_r1);
	LOG_DEBUG("restore_tmp_registers - r0: 0x%08x, r1: 0x%08x",
			value_r0, value_r1);

	uint32_t append_mode = 0;
	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK)
			append_mode = 1;
		else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
			aice_packet_append_size = 0;
		}
	}

	if (core_info[coreid].target_dtr_valid) {
		uint32_t instructions[4] = {
			SETHI(R0, core_info[coreid].target_dtr_backup >> 12),
			ORI(R0, R0, core_info[coreid].target_dtr_backup & 0x00000FFF),
			NOP,
			BEQ_MINUS_12
		};
		aice_execute_dim(target, instructions, 4);

		instructions[0] = MTSR_DTR(R0);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
		aice_execute_dim(target, instructions, 4);

		LOG_DEBUG("Restore target DTR: 0x%08x", core_info[coreid].target_dtr_backup);
	}

	nds32_set_mapped_reg(nds32, R0, value_r0);
	nds32_set_mapped_reg(nds32, R1, value_r1);
	nds32->target->state = backup_state;

	if (core_info[coreid].host_dtr_valid) {
		if (aice_write_dtr(target, core_info[coreid].host_dtr_backup) != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("Restore host DTR: 0x%08x", core_info[coreid].host_dtr_backup);
	}

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		if (append_mode == 1) {
			return ERROR_OK;
		} else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		}
	}

	return ERROR_OK;
}

int aice_read_acr(struct target *target, uint32_t num, char *val)
{
	LOG_DEBUG("aice_read_acr, reg_no 0x%08x", num);
	uint32_t instructions[4]; /** execute instructions in DIM */

	unsigned *read_insn_list = nds32_reg_get_read_insn(num);
	unsigned size_of_read_insn_list = nds32_reg_get_size_of_read_insn(num);
	uint32_t val_32 = 0;

	LOG_DEBUG("aice_read_acr, size of read_insn_list: %d", size_of_read_insn_list);
	assert (size_of_read_insn_list % 4 == 0);

	for (unsigned i = 0; i < size_of_read_insn_list; i+=4) {
		instructions[0] = read_insn_list[i];
		instructions[1] = read_insn_list[i+1];
		instructions[2] = read_insn_list[i+2];
		instructions[3] = read_insn_list[i+3];

		aice_execute_dim(target, instructions, 4);

		// To read ACM's data, we must set the memory address to GPR by using sethi/ori firstly.
		// Then, ACM read utility instruction read the data according to the address stored in GPR.
		// For this instruction sequence, the DTR would not be updated. So, we do not check
		// whether DTR is updated or not.
		// instructions[0]: sethi
		// instructions[1]: ori
		// instructions[2]: nop
		if ((instructions[0] >> 24) == 0x46 &&
				(instructions[1] >> 24) == 0x58 &&
				instructions[2] == 0x40000009) {
			continue;
		}

		if (aice_read_dtr(target, &val_32) != ERROR_OK) {
			NDS32_LOG(NDS32_ERRMSG_TARGET_READ_DTR);
			return ERROR_FAIL;
		}

		LOG_DEBUG("val_32 %x", val_32);
		memcpy(val, &val_32, sizeof(val_32));
		val += sizeof(val_32);
	}

	return ERROR_OK;
}

int aice_write_acr(struct target *target, uint32_t num, char *val)
{
	LOG_DEBUG("aice_write_acr, reg_no: 0x%08x", num);
	uint32_t instructions[4]; /** execute instructions in DIM */

	unsigned *write_insn_list = nds32_reg_get_write_insn(num);
	unsigned size_of_write_insn_list = nds32_reg_get_size_of_write_insn(num);
	uint32_t val_32 = 0;

	LOG_DEBUG("aice_write_acr, size of write_insn_list: %d", size_of_write_insn_list);
	assert (size_of_write_insn_list % 4 == 0);

	for (unsigned i = 0; i < size_of_write_insn_list; i+=4) {
		instructions[0] = write_insn_list[i];
		instructions[1] = write_insn_list[i+1];
		instructions[2] = write_insn_list[i+2];
		instructions[3] = write_insn_list[i+3];

		// Refer the the comment in aice_read_acr().
		if ((instructions[0] >> 24) == 0x46 &&
				(instructions[1] >> 24) == 0x58 &&
				instructions[2] == 0x40000009) {
			aice_execute_dim(target, instructions, 4);
			continue;
		}


		memcpy(&val_32, val, sizeof(val_32));
		LOG_DEBUG("val_32 %x", val_32);
		val += sizeof(val_32);

		if (aice_write_dtr(target, val_32) != ERROR_OK)
			return ERROR_FAIL;

		aice_execute_dim(target, instructions, 4);
	}
	return ERROR_OK;
}



int aice_read_register(struct target *target, uint32_t num, uint32_t *val)
{
	uint32_t coreid = target_to_coreid(target);

	LOG_DEBUG("aice_read_register");

if (num == DR41) {
		/* As target is halted, OpenOCD will backup DR41/DR42/DR43.
		 * As user wants to read these registers, OpenOCD should return
		 * the backup values, instead of reading the real values.
		 * As user wants to write these registers, OpenOCD should write
		 * to the backup values, instead of writing to real registers. */
		*val = core_info[coreid].edmsw_backup;
#if BACKUP_EDM_CTRL
	} else if (num == DR42) {
		*val = core_info[coreid].edm_ctl_backup;
#endif
	} else if ((core_info[coreid].target_dtr_valid == true) && (num == DR43)) {
		*val = core_info[coreid].target_dtr_backup;
	} else {
		if (ERROR_OK != aice_read_reg(target, num, val))
			*val = 0xBBADBEEF;
        }

	return ERROR_OK;
}

int aice_write_register(struct target *target, uint32_t num, uint32_t val)
{
	uint32_t coreid = target_to_coreid(target);
	LOG_DEBUG("aice_write_register");

	if ((core_info[coreid].target_dtr_valid == true) && (num == DR43))
		core_info[coreid].target_dtr_backup = val;
#if BACKUP_EDM_CTRL
	else if (num == DR42)
		/* As target is halted, OpenOCD will backup DR41/DR42/DR43.
		 * As user wants to read these registers, OpenOCD should return
		 * the backup values, instead of reading the real values.
		 * As user wants to write these registers, OpenOCD should write
		 * to the backup values, instead of writing to real registers. */
		core_info[coreid].edm_ctl_backup = val;
#endif
	else
		return aice_write_reg(target, num, val);

	return ERROR_OK;
}

int aice_read_reg_64(struct target *target, uint32_t num, uint64_t *val)
{
	LOG_DEBUG("aice_read_reg_64, %s", nds32_reg_simple_name(num));

	uint32_t value;
	uint32_t high_value;

	if (ERROR_OK != aice_read_reg(target, num, &value))
		value = 0xBBADBEEF;

	aice_read_reg(target, R1, &high_value);

	LOG_DEBUG("low: 0x%08x, high: 0x%08x\n", value, high_value);
	struct nds32 *nds32 = target_to_nds32(target);
	if (nds32->data_endian != TARGET_BIG_ENDIAN)
		*val = (((uint64_t)high_value) << 32) | value;
	else
		*val = (((uint64_t)value) << 32) | high_value;

	return ERROR_OK;
}

int aice_write_reg_64(struct target *target, uint32_t num, uint64_t val)
{
	uint32_t value;
	uint32_t high_value;
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->data_endian != TARGET_BIG_ENDIAN) {
		value = val & 0xFFFFFFFF;
		high_value = (val >> 32) & 0xFFFFFFFF;
	} else {
		high_value = val & 0xFFFFFFFF;
		value = (val >> 32) & 0xFFFFFFFF;
	}

	LOG_DEBUG("aice_write_reg_64, %s, low: 0x%08x, high: 0x%08x\n",
			nds32_reg_simple_name(num), value, high_value);

	aice_write_reg(target, R1, high_value);
	return aice_write_reg(target, num, value);
}

int aice_bitset_sysreg(struct target *target, uint32_t num, uint32_t bit_set)
{
	uint32_t instructions[4];

	instructions[0] = SETHI(R0, bit_set >> 12);
	instructions[1] = ORI(R0, R0, bit_set & 0x00000FFF);
	instructions[2] = MFSR(R1, nds32_reg_sr_index(num));
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	/* system registers */
	instructions[0] = OR(R0, R1, R0);
	instructions[1] = MTSR(R0, nds32_reg_sr_index(num));
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);
	return ERROR_OK;
}

int aice_bitclr_sysreg(struct target *target, uint32_t num, uint32_t bit_clr)
{
	uint32_t instructions[4];

	instructions[0] = SETHI(R0, ~bit_clr >> 12);
	instructions[1] = ORI(R0, R0, ~bit_clr & 0x00000FFF);
	instructions[2] = MFSR(R1, nds32_reg_sr_index(num));
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	/* system registers */
	instructions[0] = AND(R0, R1, R0);
	instructions[1] = MTSR(R0, nds32_reg_sr_index(num));
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	return ERROR_OK;
}
