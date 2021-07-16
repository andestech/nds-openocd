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

#include <helper/time_support.h>
#include <helper/binarybuffer.h>
#include "breakpoints.h"
#include "nds32_insn.h"
#include "nds32_reg.h"
#include "nds32_edm.h"
#include "nds32_cmd.h"
#include "nds32_v2.h"
#include "nds32_aice.h"
#include "target_type.h"
#include "nds32_disassembler.h"
#include "nds32_v3_common.h"
#include "nds32_log.h"

extern struct nds32_v3_common_callback *v3_common_callback;
extern int v3_add_all_sw_breakpoint(struct target *target);
extern int v3_remove_all_sw_breakpoint(struct target *target);

int nds32_v2_check_interrupt_stack(struct nds32 *nds32);
int nds32_v2_restore_interrupt_stack(struct nds32 *nds32);
static int nds32_v2_activate_hardware_breakpoint(struct target *target);
static int nds32_v2_activate_hardware_watchpoint(struct target *target);
static int nds32_v2_deactivate_hardware_breakpoint(struct target *target);
static int nds32_v2_deactivate_hardware_watchpoint(struct target *target);

struct nds32_v3_common_callback nds32_v2_common_callback = {
	.check_interrupt_stack = nds32_v2_check_interrupt_stack,
	.restore_interrupt_stack = nds32_v2_restore_interrupt_stack,
	.activate_hardware_breakpoint = nds32_v2_activate_hardware_breakpoint,
	.activate_hardware_watchpoint = nds32_v2_activate_hardware_watchpoint,
	.deactivate_hardware_breakpoint = nds32_v2_deactivate_hardware_breakpoint,
	.deactivate_hardware_watchpoint = nds32_v2_deactivate_hardware_watchpoint,
};

int nds32_v2_register_mapping(struct nds32 *nds32, int reg_no)
{
	uint32_t max_level = nds32->max_interrupt_level;
	uint32_t cur_level = nds32->current_interrupt_level;

	if ((1 <= cur_level) && (cur_level < max_level)) {
		if (IR0 == reg_no) {
			LOG_DEBUG("Map PSW to IPSW");
			return IR1;
		} else if (PC == reg_no) {
			LOG_DEBUG("Map PC to IPC");
			return IR9;
		}
	} else if ((2 <= cur_level) && (cur_level < max_level)) {
		if (R26 == reg_no) {
			LOG_DEBUG("Mapping P0 to P_P0");
			return IR12;
		} else if (R27 == reg_no) {
			LOG_DEBUG("Mapping P1 to P_P1");
			return IR13;
		} else if (IR1 == reg_no) {
			LOG_DEBUG("Mapping IPSW to P_IPSW");
			return IR2;
		} else if (IR4 == reg_no) {
			LOG_DEBUG("Mapping EVA to P_EVA");
			return IR5;
		} else if (IR6 == reg_no) {
			LOG_DEBUG("Mapping ITYPE to P_ITYPE");
			return IR7;
		} else if (IR9 == reg_no) {
			LOG_DEBUG("Mapping IPC to P_IPC");
			return IR10;
		}
	} else if (cur_level == max_level) {
		if (PC == reg_no) {
			LOG_DEBUG("Mapping PC to O_IPC");
			return IR11;
		}
	}

	return reg_no;
}

int nds32_v2_get_debug_reason(struct nds32 *nds32, uint32_t *reason)
{
	uint32_t val_itype;

	aice_read_register(nds32->target, IR6, &val_itype);

	*reason = val_itype & 0x0F;

	return ERROR_OK;
}

static int nds32_v2_activate_hardware_breakpoint(struct target *target)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);
	struct breakpoint *bp;
	int32_t hbr_index = 0;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT) {
			/* already set at nds32_v2_add_breakpoint() */
			continue;
		} else if (bp->type == BKPT_HARD) {
			/* set address */
			aice_write_debug_reg(target, NDS_EDM_SR_BPA0 + hbr_index, bp->address);
			/* set mask */
			aice_write_debug_reg(target, NDS_EDM_SR_BPAM0 + hbr_index, 0);
			/* set value */
			aice_write_debug_reg(target, NDS_EDM_SR_BPV0 + hbr_index, 0);

			if (nds32_v2->nds32.memory.address_translation)
				/* enable breakpoint (virtual address) */
				aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + hbr_index, 0x2);
			else
				/* enable breakpoint (physical address) */
				aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + hbr_index, 0xA);

			LOG_DEBUG("Add hardware BP %" PRId32 " at %08" TARGET_PRIxADDR, hbr_index,
					bp->address);

			hbr_index++;
		} else {
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int nds32_v2_deactivate_hardware_breakpoint(struct target *target)
{
	struct breakpoint *bp;
	int32_t hbr_index = 0;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT)
			continue;
		else if (bp->type == BKPT_HARD)
			/* disable breakpoint */
			aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + hbr_index, 0x0);
		else
			return ERROR_FAIL;

		LOG_DEBUG("Remove hardware BP %" PRId32 " at %08" TARGET_PRIxADDR, hbr_index,
				bp->address);

		hbr_index++;
	}

	return ERROR_OK;
}

static int nds32_v2_activate_hardware_watchpoint(struct target *target)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);
	struct watchpoint *wp;
	int32_t wp_num = nds32_v2->next_hbr_index;
	uint32_t wp_config = 0;

	for (wp = target->watchpoints; wp; wp = wp->next) {

		wp_num--;
		wp->mask = wp->length - 1;
		if ((wp->address % wp->length) != 0)
			wp->mask = (wp->mask << 1) + 1;

		if (wp->rw == WPT_READ)
			wp_config = 0x3;
		else if (wp->rw == WPT_WRITE)
			wp_config = 0x5;
		else if (wp->rw == WPT_ACCESS)
			wp_config = 0x7;

		/* set/unset physical address bit of BPCn according to PSW.DT */
		if (nds32_v2->nds32.memory.address_translation == false)
			wp_config |= 0x8;

		/* set address */
		aice_write_debug_reg(target, NDS_EDM_SR_BPA0 + wp_num,
				wp->address - (wp->address % wp->length));
		/* set mask */
		aice_write_debug_reg(target, NDS_EDM_SR_BPAM0 + wp_num, wp->mask);
		/* enable watchpoint */
		aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + wp_num, wp_config);
		/* set value */
		aice_write_debug_reg(target, NDS_EDM_SR_BPV0 + wp_num, 0);

		LOG_DEBUG("Add hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR " mask %08" PRIx32, wp_num,
				wp->address, wp->mask);

	}

	return ERROR_OK;
}

static int nds32_v2_deactivate_hardware_watchpoint(struct target *target)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);
	int32_t wp_num = nds32_v2->next_hbr_index;
	struct watchpoint *wp;

	for (wp = target->watchpoints; wp; wp = wp->next) {
		wp_num--;
		/* disable watchpoint */
		aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + wp_num, 0x0);

		LOG_DEBUG("Remove hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR " mask %08" PRIx32,
				wp_num, wp->address, wp->mask);
	}

	return ERROR_OK;
}

int nds32_v2_check_interrupt_stack(struct nds32 *nds32)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(nds32->target);
	uint32_t val_ir0;
	uint32_t val_ir1;
	uint32_t val_ir2;
	uint32_t modified_psw;

	/* Save interrupt level */
	aice_read_register(nds32->target, IR0, &val_ir0); /* get $IR0 directly */

	/* backup $IR0 */
	nds32_v2->backup_ir0 = val_ir0;

	nds32->current_interrupt_level = (val_ir0 >> 1) & 0x3;

	if (nds32_reach_max_interrupt_level(nds32)) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_INTLVL3,
				nds32->current_interrupt_level);

		/* decrease interrupt level */
		modified_psw = val_ir0 - 0x2;

		/* disable GIE, IT, DT, HSS */
		modified_psw &= (~0x8C1);

		aice_write_register(nds32->target, IR0, modified_psw);

		return ERROR_OK;
	}

	/* There is a case that single step also trigger another interrupt,
	   then HSS bit in psw(ir0) will push to ipsw(ir1).
	   Then hit debug interrupt HSS bit in ipsw(ir1) will push to (p_ipsw)ir2
	   Therefore, HSS bit in p_ipsw(ir2) also need clear.

	   Only update $ir2 as current interrupt level is 2, because $ir2 will be random
	   value if the target never reaches interrupt level 2. */
	if ((nds32->max_interrupt_level == 3) && (nds32->current_interrupt_level == 2)) {
		aice_read_register(nds32->target, IR2, &val_ir2); /* get $IR2 directly */
		val_ir2 &= ~(0x01 << 11);
		aice_write_register(nds32->target, IR2, val_ir2);
	}

	/* get origianl DT bit and set to current state let debugger has same memory view
	   PSW.IT MUST be turned off.  Otherwise, DIM could not operate normally. */
	aice_read_register(nds32->target, IR1, &val_ir1);
	modified_psw = val_ir0 | (val_ir1 & 0x80);
	aice_write_register(nds32->target, IR0, modified_psw);

	return ERROR_OK;
}

int nds32_v2_restore_interrupt_stack(struct nds32 *nds32)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(nds32->target);

	/* restore origin $IR0 */
	aice_write_register(nds32->target, IR0, nds32_v2->backup_ir0);

	return ERROR_OK;
}

/**
 * Save processor state.  This is called after a HALT instruction
 * succeeds, and on other occasions the processor enters debug mode
 * (breakpoint, watchpoint, etc).
 */
int nds32_v2_debug_entry(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v2_debug_entry");
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(nds32->target);
	/* Save interrupt level */
	uint32_t value_ir0 = 0;
	aice_read_register(nds32->target, IR0, &value_ir0); /* get $IR0 directly */
	/* backup $IR0 */
	nds32_v2->backup_ir0 = value_ir0;
	nds32->current_interrupt_level = (value_ir0 >> 1) & 0x3;
	/* get max interrupt level */
	uint32_t value_cr4 = 0;
	aice_read_register(nds32->target, CR4, &value_cr4);
	uint32_t interruption_level = (value_cr4 >> 12) & 0x1;
	if (interruption_level)
		nds32->max_interrupt_level = 2;
	else
		nds32->max_interrupt_level = 3;

	LOG_DEBUG("nds32_v2->backup_ir0 0x%x" ,nds32_v2->backup_ir0);
	LOG_DEBUG("nds32->current_interrupt_level 0x%x" ,nds32->current_interrupt_level);
	LOG_DEBUG("nds32->max_interrupt_level 0x%x" ,nds32->max_interrupt_level);

	enum target_state backup_state = nds32->target->state;
	nds32->target->state = TARGET_HALTED;

	if (nds32->init_arch_info_after_halted == false) {
		/* init architecture info according to config registers */
		CHECK_RETVAL(nds32_config(nds32));

		nds32->init_arch_info_after_halted = true;
	}

	/* REVISIT entire cache should already be invalid !!! */
	register_cache_invalidate(nds32->core_cache);

	/* Save registers. */
	nds32_full_context(nds32);

	/* deactivate all hardware breakpoints */
	CHECK_RETVAL(v3_common_callback->deactivate_hardware_breakpoint(nds32->target));

	if (enable_watchpoint)
		CHECK_RETVAL(v3_common_callback->deactivate_hardware_watchpoint(nds32->target));

	/* Andes-defined virtual call */
	uint32_t etype;
	nds32->get_debug_reason(nds32, &etype);
	nds32->hit_syscall = false;
	nds32->active_syscall_id = 0;
	uint32_t nds32_skip_break = 0;
	// sw breakpoint use break16 only
	// so only break32 need to check active_syscall_id for virtual hosting
	if (NDS32_DEBUG_BREAK == etype) {
		uint32_t opcode = 0;
		uint32_t value_pc = 0;
		nds32_get_mapped_reg(nds32, PC, &value_pc);

		if (ERROR_OK != nds32_read_opcode(nds32, value_pc, &opcode))
			return ERROR_FAIL;

		//if ((opcode & 0xFE000000) == 0xEA000000) {
		//	nds32->active_syscall_id = BREAK16_SWID9(opcode);
		//}
		//else if ((opcode & 0xFFF0001F) == 0x6400000A) {
			nds32->active_syscall_id = BREAK_SWID15(opcode);
		//}
	}

	/* must doing before nds32->hit_syscall being set=1 (ref in nds32_write_buffer()) */
	v3_remove_all_sw_breakpoint(nds32->target);

	if ((NDS32_SYSCALL_FOPEN <= nds32->active_syscall_id) &&
		(nds32->active_syscall_id <= NDS32_VIO_LIB_API_MAX_ID) ) {
		nds32->hit_syscall = true;
	} else if (nds32->active_syscall_id == NDS32_SKIP_BREAK) {
		nds32_skip_break = 1;
		/* skip break */
		uint32_t value_ir11 = 0;
		int reg_num = nds32_v2_register_mapping(nds32, PC);
		aice_read_register(nds32->target, reg_num, &value_ir11);
		value_ir11 += 4;
		aice_write_register(nds32->target, reg_num, value_ir11);
	}

	if ((ERROR_OK != nds32_examine_debug_reason(nds32)) ||
		(nds32_skip_break == 1)) {
		nds32->target->state = backup_state;

		/* re-activate all hardware breakpoints & watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(nds32->target));

		if (enable_watchpoint) {
			/* activate all watchpoints */
			CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(nds32->target));
		}
		v3_add_all_sw_breakpoint(nds32->target);
		return ERROR_FAIL;
	}

	/* check interrupt level before .full_context(), because
	 * get_mapped_reg() in nds32_full_context() needs current_interrupt_level
	 * information */
	v3_common_callback->check_interrupt_stack(nds32);

	/* Save registers. */
	//nds32_full_context(nds32);

	return ERROR_OK;
}

/* target request support */
static int nds32_v2_target_request_data(struct target *target,
		uint32_t size, uint8_t *buffer)
{
	/* AndesCore could use DTR register to communicate with OpenOCD
	 * to output messages
	 * Target data will be put in buffer
	 * The format of DTR is as follow
	 * DTR[31:16] => length, DTR[15:8] => size, DTR[7:0] => target_req_cmd
	 * target_req_cmd has three possible values:
	 *   TARGET_REQ_TRACEMSG
	 *   TARGET_REQ_DEBUGMSG
	 *   TARGET_REQ_DEBUGCHAR
	 * if size == 0, target will call target_asciimsg(),
	 * else call target_hexmsg()
	 */
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_OK;
}

/**
 * Restore processor state.
 */
int nds32_v2_leave_debug_state(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v2_leave_debug_state");

	struct target *target = nds32->target;

	v3_add_all_sw_breakpoint(target);

	/* activate all hardware breakpoints */
	CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(nds32->target));

	if (enable_watchpoint) {
		/* activate all watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(nds32->target));
	}

	/* restore interrupt stack */
	v3_common_callback->restore_interrupt_stack(nds32);

	/* restore PSW, PC, and R0 ... after flushing any modified
	 * registers.
	 */
	CHECK_RETVAL(nds32_restore_context(target));

	if (nds32->hit_syscall) {
		uint32_t etype;
		uint32_t value_ir11;
		nds32->get_debug_reason(nds32, &etype);

		/* skip break */
		int reg_num = nds32_v2_register_mapping(nds32, PC);
		aice_read_register(target, reg_num, &value_ir11);

		if (NDS32_DEBUG_BREAK == etype)
			value_ir11 += 4;
		else if (NDS32_DEBUG_BREAK_16 == etype)
			value_ir11 += 2;

		aice_write_register(target, reg_num, value_ir11);

		nds32->hit_syscall = false;
	}

	//register_cache_invalidate(nds32->core_cache);

	return ERROR_OK;
}

static int nds32_v2_deassert_reset(struct target *target)
{
	int retval;

	CHECK_RETVAL(nds32_poll(target));

	if (target->state != TARGET_HALTED) {
		/* reset only */
		LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(target));
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int nds32_v2_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

static int nds32_v2_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);
	struct nds32 *nds32 = &(nds32_v2->nds32);
	int result;

	if (breakpoint->type == BKPT_HARD) {
		/* check hardware resource */
		if (nds32_v2->n_hbr <= nds32_v2->next_hbr_index) {
			NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT, nds32_v2->n_hbr);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* update next place to put hardware breakpoint */
		nds32_v2->next_hbr_index++;

		/* hardware breakpoint insertion occurs before 'continue' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
		result = nds32_add_software_breakpoint(target, breakpoint);
		if (ERROR_OK != result) {
			/* auto convert to hardware breakpoint if failed */
			if (nds32->auto_convert_hw_bp) {
				/* convert to hardware breakpoint */
				breakpoint->type = BKPT_HARD;

				return nds32_v2_add_breakpoint(target, breakpoint);
			}
		}

		return result;
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v2_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);

	if (breakpoint->type == BKPT_HARD) {
		if (nds32_v2->next_hbr_index <= 0)
			return ERROR_FAIL;

		/* update next place to put hardware breakpoint */
		nds32_v2->next_hbr_index--;

		/* hardware breakpoint removal occurs after 'halted' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
		return nds32_remove_software_breakpoint(target, breakpoint);
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v2_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);

	/* check hardware resource */
	if (nds32_v2->n_hbr <= nds32_v2->next_hbr_index) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT, nds32_v2->n_hbr);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* update next place to put hardware watchpoint */
	nds32_v2->next_hbr_index++;

	return ERROR_OK;
}

static int nds32_v2_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);

	if (nds32_v2->next_hbr_index <= 0)
		return ERROR_FAIL;

	/* update next place to put hardware breakpoint */
	nds32_v2->next_hbr_index--;

	return ERROR_OK;
}

int nds32_v2_get_exception_address(struct nds32 *nds32,
		uint32_t *address, uint32_t reason)
{
	struct target *target = nds32->target;
	struct watchpoint *wp;
	struct nds32_instruction instruction;
	bool hit = false;
	uint32_t val_pc;

	aice_read_register(target, IR4, address); /* read $EVA directly */
	LOG_DEBUG("nds32_v2_get_exception_address");
	
	nds32_get_mapped_reg(nds32, PC, &val_pc);
	if ((NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE == reason) ||
			(NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE == reason)) {
		val_pc -= 4;
	}
	nds32_evaluate_opcode(nds32, val_pc, &instruction);
	/* Bug 9501 - breakpoint/read_watch.exp fail */
	/* for get-address-error-cases like "lwi450 R0,[R0]" - start--- */
	unsigned int data_length, val_eva;
	val_eva = *address;
	data_length = instruction.access_end - instruction.access_start;
	instruction.access_start = val_eva;
	instruction.access_end   = instruction.access_start + data_length;
	/* for get-address-error-cases like "lwi450 R0,[R0]" - end--- */

	LOG_DEBUG("PC: 0x%08x, access start: 0x%08x, end: 0x%08x", val_pc,
			instruction.access_start, instruction.access_end);

	/* check if multiple hits in the access range */
	uint32_t in_range_watch_count = 0;
	for (wp = target->watchpoints; wp; wp = wp->next) {
		if ((instruction.access_start <= wp->address) &&
				(wp->address < instruction.access_end))
			in_range_watch_count++;
	}
	if (in_range_watch_count > 1) {
		/* Hit LSMW instruction. */
		*address = 0;
		return ERROR_OK;
	}

	/* dispel false match */
	for (wp = target->watchpoints; wp; wp = wp->next) {
		if (((*address ^ wp->address) & (~wp->mask)) == 0) {
			uint32_t watch_start;
			uint32_t watch_end;

			watch_start = wp->address;
			watch_end = wp->address + wp->length;

			if ((watch_end <= instruction.access_start) ||
					(instruction.access_end <= watch_start))
				continue;

			hit = true;
			break;
		}
	}

	if (hit)
		return ERROR_OK;

	*address = 0xFFFFFFFF;
	return ERROR_FAIL;
}

/**
 * find out which watchpoint hits
 * get exception address and compare the address to watchpoints
 */
static int nds32_v2_hit_watchpoint(struct target *target,
		struct watchpoint **hit_watchpoint)
{
	uint32_t exception_address;
	struct watchpoint *wp;
	static struct watchpoint scan_all_watchpoint;
	struct nds32 *nds32 = target_to_nds32(target);

	scan_all_watchpoint.address = 0;
	scan_all_watchpoint.rw = WPT_WRITE;
	scan_all_watchpoint.next = 0;
	scan_all_watchpoint.unique_id = 0x5CA8;

	exception_address = nds32->watched_address;

	if (exception_address == 0) {
		/* send watch:0 to tell GDB to do software scan for hitting multiple watchpoints */
		*hit_watchpoint = &scan_all_watchpoint;
		return ERROR_OK;
	}

	for (wp = target->watchpoints; wp; wp = wp->next) {
		if (((exception_address ^ wp->address) & (~wp->mask)) == 0) {
			/* TODO: dispel false match */
			*hit_watchpoint = wp;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int nds32_v2_run_algorithm(struct target *target,
		int num_mem_params,
		struct mem_param *mem_params,
		int num_reg_params,
		struct reg_param *reg_params,
		target_addr_t entry_point,
		target_addr_t exit_point,
		int timeout_ms,
		void *arch_info)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

static int nds32_v2_target_create(struct target *target, Jim_Interp *interp)
{
	struct nds32_v2_common *nds32_v2;

	nds32_v2 = calloc(1, sizeof(*nds32_v2));
	if (!nds32_v2)
		return ERROR_FAIL;

	nds32_v3_common_register_callback(&nds32_v2_common_callback);

	nds32_v2->nds32.register_map = nds32_v2_register_mapping;
	nds32_v2->nds32.get_debug_reason = nds32_v2_get_debug_reason;
	nds32_v2->nds32.enter_debug_state = nds32_v2_debug_entry;
	nds32_v2->nds32.leave_debug_state = nds32_v2_leave_debug_state;
	nds32_v2->nds32.get_watched_address = nds32_v2_get_exception_address;

	nds32_init_arch_info(target, &(nds32_v2->nds32));

	return ERROR_OK;
}

static int nds32_v2_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* Initialize anything we can set up without talking to the target */

	struct nds32 *nds32 = target_to_nds32(target);

	nds32_init(nds32);

	return ERROR_OK;
}

/* talk to the target and set things up */
static int nds32_v2_examine(struct target *target)
{
	struct nds32_v2_common *nds32_v2 = target_to_nds32_v2(target);
	struct nds32 *nds32 = &(nds32_v2->nds32);

	if (target->tap->hasidcode == false) {
		LOG_ERROR("no IDCODE present on device");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!target_was_examined(target)) {
		CHECK_RETVAL(nds32_edm_config(nds32));

		if (nds32->reset_halt_as_examine)
			CHECK_RETVAL(nds32_reset_halt(nds32));
		LOG_DEBUG("nds32_v2_examine: nds32->edm.version = %x", nds32->edm.version);
	}

	uint32_t edm_cfg;
	aice_read_debug_reg(target, NDS_EDM_SR_EDM_CFG, &edm_cfg);

	/* get the number of hardware breakpoints */
	nds32_v2->n_hbr = (edm_cfg & 0x7) + 1;

	nds32_v2->next_hbr_index = 0;

	LOG_INFO("%s: total hardware breakpoint %d", target_name(target),
			nds32_v2->n_hbr);

	nds32->target->state = TARGET_RUNNING;
	nds32->target->debug_reason = DBG_REASON_NOTHALTED;

	aice_reg_set_ace_access_op(target);

	target_set_examined(target);

	return ERROR_OK;
}

static int nds32_v2_translate_address(struct target *target, target_addr_t *address)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	target_addr_t physical_address;

	/* Following conditions need to do address translation
	 * 1. BUS mode
	 * 2. CPU mode under maximum interrupt level */
	if ((NDS_MEMORY_ACC_BUS == memory->access_channel) ||
			((NDS_MEMORY_ACC_CPU == memory->access_channel) &&
			 nds32_reach_max_interrupt_level(nds32))) {
		if (ERROR_OK == target->type->virt2phys(target, *address, &physical_address))
			*address = physical_address;
		else
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_v2_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	nds32_v2_translate_address(target, &address);

	return nds32_read_buffer(target, address, size, buffer);
}

static int nds32_v2_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer)
{
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	nds32_v2_translate_address(target, &address);

	return nds32_write_buffer(target, address, size, buffer);
}

static int nds32_v2_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	nds32_v2_translate_address(target, &address);

	return nds32_read_memory(target, address, size, count, buffer);
}

static int nds32_v2_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	nds32_v2_translate_address(target, &address);

	return nds32_write_memory(target, address, size, count, buffer);
}

/** Holds methods for V2 targets. */
struct target_type nds32_v2_target = {
	.name = "nds32_v2",

	.poll = nds32_poll,
	.arch_state = nds32_arch_state,

	.target_request_data = nds32_v2_target_request_data,

	.halt = nds32_halt,
	.resume = nds32_resume,
	.step = nds32_step,

	.assert_reset = nds32_assert_reset,
	.deassert_reset = nds32_v2_deassert_reset,

	/* register access */
	.get_gdb_reg_list = nds32_get_gdb_reg_list,

	/* memory access */
	.read_buffer = nds32_v2_read_buffer,
	.write_buffer = nds32_v2_write_buffer,
	.read_memory = nds32_v2_read_memory,
	.write_memory = nds32_v2_write_memory,

	.checksum_memory = nds32_v2_checksum_memory,

	/* breakpoint/watchpoint */
	.add_breakpoint = nds32_v2_add_breakpoint,
	.remove_breakpoint = nds32_v2_remove_breakpoint,
	.add_watchpoint = nds32_v2_add_watchpoint,
	.remove_watchpoint = nds32_v2_remove_watchpoint,
	.hit_watchpoint = nds32_v2_hit_watchpoint,

	/* MMU */
	.mmu = nds32_mmu,
	.virt2phys = nds32_virtual_to_physical,
	.read_phys_memory = nds32_read_phys_memory,
	.write_phys_memory = nds32_write_phys_memory,

	.run_algorithm = nds32_v2_run_algorithm,

	.commands = nds32_command_handlers,
	.target_create = nds32_v2_target_create,
	.init_target = nds32_v2_init_target,
	.examine = nds32_v2_examine,
};
