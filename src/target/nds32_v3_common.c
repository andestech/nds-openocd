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

#include "breakpoints.h"
#include "nds32_reg.h"
#include "nds32_disassembler.h"
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_v3_common.h"
#include "algorithm.h"
#include "helper/time_support.h"

struct nds32_v3_common_callback *v3_common_callback;
extern int v3_add_all_sw_breakpoint(struct target *target);
extern int v3_remove_all_sw_breakpoint(struct target *target);
extern uint32_t nds32_if_smw_i, nds32_if_smw_d;

int nds32_v3_register_mapping(struct nds32 *nds32, int reg_no)
{
	if (reg_no == PC)
		return IR11;

	return reg_no;
}

int nds32_v3_get_debug_reason(struct nds32 *nds32, uint32_t *reason)
{
	uint32_t edmsw;
	aice_read_debug_reg(nds32->target, NDS_EDM_SR_EDMSW, &edmsw);
	/*
	enum target_state backup_state = nds32->target->state;
	nds32->target->state = TARGET_HALTED;
	nds32_get_mapped_reg(nds32, DR41, &edmsw);
	nds32->target->state = backup_state;
	*/
	LOG_DEBUG("edmsw(DR41) = 0x%x", edmsw);
	*reason = (edmsw >> 12) & 0x0F;

	return ERROR_OK;
}

/**
 * Save processor state.  This is called after a HALT instruction
 * succeeds, and on other occasions the processor enters debug mode
 * (breakpoint, watchpoint, etc).
 */
static int nds32_v3_debug_entry(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v3_debug_entry");

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
		aice_read_register(nds32->target, IR11, &value_ir11);
		value_ir11 += 4;
		aice_write_register(nds32->target, IR11, value_ir11);
	}

	if ((ERROR_OK != nds32_examine_debug_reason(nds32)) ||
		(nds32_skip_break == 1)) {
		nds32->target->state = backup_state;

		/* re-activate all hardware breakpoints & watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(nds32->target));

		if (enable_watchpoint)
			CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(nds32->target));

		v3_add_all_sw_breakpoint(nds32->target);
		return ERROR_FAIL;
	}

	/* Save registers. */
	//nds32_full_context(nds32);

	/* check interrupt level */
	v3_common_callback->check_interrupt_stack(nds32);

	return ERROR_OK;
}

/**
 * Restore processor state.
 */
static int nds32_v3_leave_debug_state(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v3_leave_debug_state");

	struct target *target = nds32->target;

	v3_add_all_sw_breakpoint(target);

	/* activate all hardware breakpoints */
	CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(target));

	if (enable_watchpoint) {
		/* activate all watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(target));
	}

	/* restore interrupt stack */
	v3_common_callback->restore_interrupt_stack(nds32);

	/* REVISIT once we start caring about MMU and cache state,
	 * address it here ...
	 */

	/* restore PSW, PC, and R0 ... after flushing any modified
	 * registers.
	 */
	CHECK_RETVAL(nds32_restore_context(target));

	if (nds32->hit_syscall) {
		uint32_t etype;
		uint32_t value_ir11;
		nds32->get_debug_reason(nds32, &etype);

		/* skip break */
		aice_read_register(target, IR11, &value_ir11);

		if (NDS32_DEBUG_BREAK == etype)
			value_ir11 += 4;
		else if (NDS32_DEBUG_BREAK_16 == etype)
			value_ir11 += 2;

		aice_write_register(target, IR11, value_ir11);

		nds32->hit_syscall = false;
	}
	return ERROR_OK;
}

static int nds32_v3_get_exception_address(struct nds32 *nds32,
		uint32_t *address, uint32_t reason)
{
	LOG_DEBUG("nds32_v3_get_exception_address");
	struct target *target = nds32->target;
	uint32_t edmsw;
	uint32_t edm_cfg;
	uint32_t match_bits;
	uint32_t match_count;
	int32_t i;
	static int32_t number_of_hard_break;
	uint32_t bp_control;
	struct watchpoint *wp;
	struct nds32_instruction instruction;
	bool hit = false;

	if (number_of_hard_break == 0) {
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CFG, &edm_cfg);
		number_of_hard_break = (edm_cfg & 0x7) + 1;
	}

	aice_read_debug_reg(target, NDS_EDM_SR_EDMSW, &edmsw);
	/* clear matching bits (write-one-clear) */
	aice_write_debug_reg(target, NDS_EDM_SR_EDMSW, edmsw);
	match_bits = (edmsw >> 4) & 0xFF;
	match_count = 0;
	for (i = 0 ; i < number_of_hard_break ; i++) {
		if (match_bits & (1 << i)) {
			aice_read_debug_reg(target, NDS_EDM_SR_BPA0 + i, address);
			match_count++;

			/* If target hits multiple read/access watchpoint,
			 * select the first one. */
			aice_read_debug_reg(target, NDS_EDM_SR_BPC0 + i, &bp_control);
			if (0x3 == (bp_control & 0x3)) {
				match_count = 1;
				break;
			}
		}
	}

	if (match_count > 1) { /* multiple hits */
		*address = 0;
		return ERROR_OK;
	} else if (match_count == 1) {
		uint32_t val_pc;

		nds32_get_mapped_reg(nds32, PC, &val_pc);

		if ((NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE == reason) ||
				(NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE == reason)) {
			if (edmsw & 0x4) /* check EDMSW.IS_16BIT */
				val_pc -= 2;
			else
				val_pc -= 4;
		}

		nds32_evaluate_opcode(nds32, val_pc, &instruction);
		/* Bug 9501 - breakpoint/read_watch.exp fail */
		/* for get-address-error-cases like "lwi450 R0,[R0]" - start--- */
		unsigned int data_length, val_eva;
		val_eva = *address;
		data_length = instruction.access_end - instruction.access_start;
		/* bug-12990 sw-workaround for N13, check if smw.bi and bug-18276, check if smw.ai*/
		if (nds32_if_smw_i == 1) {
			LOG_DEBUG("nds32_if_smw_i, cpu_id_family:0x%08x", nds32->cpu_version.cpu_id_family);
			if (nds32->cpu_version.cpu_id_family == 0x0d) {
				val_eva -= 4;
				LOG_DEBUG("instruction.access_start: 0x%08x, instruction.access_end: 0x%08x",
					instruction.access_start, instruction.access_end);
			}
		} else if (nds32_if_smw_d == 1) {
			LOG_DEBUG("nds32_if_smw_d, cpu_id_family:0x%08x", nds32->cpu_version.cpu_id_family);
			if (nds32->cpu_version.cpu_id_family != 0x0d) {
				val_eva -= 4;
				LOG_DEBUG("instruction.access_start: 0x%08x, instruction.access_end: 0x%08x",
					instruction.access_start, instruction.access_end);
			}
		}
		instruction.access_start = val_eva;
		instruction.access_end   = instruction.access_start + data_length;
		/* for get-address-error-cases like "lwi450 R0,[R0]" - end--- */

		/* if va_to_pa_off, PA watchpoint match, but report VA in BPA
		   so just compare 12 bits(page) */
		if (nds32->memory.va_to_pa_off) {
			uint32_t compare_addr = (instruction.access_start & 0xFFF);
			uint32_t compare_end = (instruction.access_end & 0xFFF);

			for (wp = target->watchpoints; wp; wp = wp->next) {
				if ((compare_addr <= (wp->address & 0xFFF)) &&
					((wp->address & 0xFFF) < compare_end)) {
						LOG_DEBUG("hit compare_addr: 0x%08x, wp->address: 0x%" TARGET_PRIxADDR, 
							compare_addr, wp->address);
					break;
				}
			}
			*address = wp->address;
			return ERROR_OK;
		}

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

	} else if (match_count == 0) {
		/* global stop is precise exception */
		if ((NDS32_DEBUG_LOAD_STORE_GLOBAL_STOP == reason) && nds32->global_stop) {
			/* parse instruction to get correct access address */
			uint32_t val_pc;

			nds32_get_mapped_reg(nds32, PC, &val_pc);
			nds32_evaluate_opcode(nds32, val_pc, &instruction);

			*address = instruction.access_start;
		}
	}

	/* dispel false match */
	for (wp = target->watchpoints; wp; wp = wp->next) {
		if (((*address ^ wp->address) & (~wp->mask)) == 0) {
			uint32_t watch_start;
			uint32_t watch_end;

			watch_start = wp->address;
			watch_end = wp->address + (wp->length & BP_WP_LENGTH_MASK);
			if ((watch_end <= instruction.access_start) ||
					(instruction.access_end <= watch_start))
				continue;

			hit = true;
			break;
		}
	}

	if (hit) {
		if (wp->length & BP_WP_USER_WATCH) {
				LOG_DEBUG("wp->length & BP_WP_USER_WATCH");
				nds32->hit_user_def_wp = true;
		} else
			nds32->hit_user_def_wp = false;
		return ERROR_OK;
	}
	*address = 0xFFFFFFFF;
	return ERROR_FAIL;
}

void nds32_v3_common_register_callback(struct nds32_v3_common_callback *callback)
{
	v3_common_callback = callback;
}

/** target_type functions: */
/* target request support */
int nds32_v3_target_request_data(struct target *target,
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

int nds32_v3_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/**
 * find out which watchpoint hits
 * get exception address and compare the address to watchpoints
 */
int nds32_v3_hit_watchpoint(struct target *target,
		struct watchpoint **hit_watchpoint)
{
	static struct watchpoint scan_all_watchpoint;

	uint32_t exception_address;
	struct watchpoint *wp;
	struct nds32 *nds32 = target_to_nds32(target);

	exception_address = nds32->watched_address;

	if (exception_address == 0xFFFFFFFF)
		return ERROR_FAIL;

	if (exception_address == 0) {
		scan_all_watchpoint.address = 0;
		scan_all_watchpoint.rw = WPT_WRITE;
		scan_all_watchpoint.next = 0;
		scan_all_watchpoint.unique_id = 0x5CA8;

		*hit_watchpoint = &scan_all_watchpoint;
		return ERROR_OK;
	}

	for (wp = target->watchpoints; wp; wp = wp->next) {
		if (((exception_address ^ wp->address) & (~wp->mask)) == 0) {
			*hit_watchpoint = wp;

			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int nds32_v3_target_create_common(struct target *target, struct nds32 *nds32)
{
	nds32->register_map = nds32_v3_register_mapping;
	nds32->get_debug_reason = nds32_v3_get_debug_reason;
	nds32->enter_debug_state = nds32_v3_debug_entry;
	nds32->leave_debug_state = nds32_v3_leave_debug_state;
	nds32->get_watched_address = nds32_v3_get_exception_address;

	/* Init target->arch_info in nds32_init_arch_info().
	 * After this, user could use target_to_nds32() to get nds32 object */
	nds32_init_arch_info(target, nds32);

	return ERROR_OK;
}

#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
/* Algorithm must end with a software breakpoint instruction. */
int nds32_v3_run_algorithm(struct target *target,
		int num_mem_params,
		struct mem_param *mem_params,
		int num_reg_params,
		struct reg_param *reg_params,
		target_addr_t entry_point,
		target_addr_t exit_point,
		int timeout_ms,
		void *arch_info)
{
	if (num_mem_params > 0) {
		LOG_ERROR("Memory parameters are not supported for nds32_v3 algorithms.");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// Save registers
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t saved_pc = buf_get_u32(reg_pc->value, 0, reg_pc->size);

	uint32_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > 31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK) {
			return ERROR_FAIL;
		}
		saved_regs[r->number] = buf_get_u32(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK)
			return ERROR_FAIL;
	}

	// Disable Interrupts before attempting to run the algorithm.
	uint32_t current_mstatus;
	uint8_t mstatus_bytes[8];

	LOG_DEBUG("Disabling Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"ir0", 1);
	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u32(reg_mstatus->value, 0, reg_mstatus->size);
	uint32_t ie_mask = 0x1;
	buf_set_u32(mstatus_bytes, 0, 32, set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	// Run algorithm
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (nds32_resume(target, 0, entry_point, 0, 1) != ERROR_OK)
		return ERROR_FAIL;

	int32_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int32_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %d ms.", timeout_ms);
			LOG_ERROR("  now   = 0x%08x", (uint32_t) now);
			LOG_ERROR("  start = 0x%08x", (uint32_t) start);
			nds32_halt(target);
			nds32_poll(target);
			return ERROR_TARGET_TIMEOUT;
		}

		int result = nds32_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t final_pc = buf_get_u32(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx32 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	// Restore Interrupts
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u32(mstatus_bytes, 0, 32, current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/// Restore registers
	uint8_t buf[8];
	buf_set_u32(buf, 0, 32, saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u32(buf, 0, 32, saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

int nds32_v3_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;

	if ((nds32->hit_syscall) ||
		 (nds32->memory.va_to_pa_off)) {
		LOG_DEBUG("nds32_v3_read_buffer: do nds32_cache_sync");
		/* write_back & invalidate dcache & invalidate icache */
		if (nds32->memory.dcache.enable == true)
			nds32_cache_sync(target, address, size);
		/* Use bus mode to access memory during virtual hosting */
		nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	}

	int result;
	if (ERROR_OK == target->type->virt2phys(target, address, &physical_address)) {
		address = physical_address;
		result = nds32_read_buffer(target, address, size, buffer);
	}
	else
		result = ERROR_FAIL;

	/* Restore access_channel after virtual hosting */
	nds32->memory.access_channel = orig_channel;

	return result;
}

int nds32_v3_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer)
{
	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;

	if ((nds32->hit_syscall) ||
		 (nds32->memory.va_to_pa_off)) {
		LOG_DEBUG("nds32_v3_write_buffer: do nds32_cache_sync");
		/* write_back & invalidate dcache & invalidate icache */
		if (nds32->memory.dcache.enable == true)
			nds32_cache_sync(target, address, size);
		/* Use bus mode to access memory during virtual hosting */
		nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	}

	int result;
	if (ERROR_OK == target->type->virt2phys(target, address, &physical_address)) {
		address = physical_address;
		result = nds32_write_buffer(target, address, size, buffer);
	}
	else
		result = ERROR_FAIL;

	/* Restore access_channel after virtual hosting */
	nds32->memory.access_channel = orig_channel;

	return result;
}

int nds32_v3_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	target_addr_t  physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (ERROR_OK == target->type->virt2phys(target, address, &physical_address))
		address = physical_address;
	else
		return ERROR_FAIL;

	int result;
	result = nds32_read_memory(target, address, size, count, buffer);
	return result;
}

int nds32_v3_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (ERROR_OK == target->type->virt2phys(target, address, &physical_address))
		address = physical_address;
	else
		return ERROR_FAIL;

	return nds32_write_memory(target, address, size, count, buffer);
}

int nds32_v3_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* Initialize anything we can set up without talking to the target */
	struct nds32 *nds32 = target_to_nds32(target);

	nds32_init(nds32);

	target->fileio_info = malloc(sizeof(struct gdb_fileio_info));
	target->fileio_info->identifier = NULL;

	return ERROR_OK;
}
