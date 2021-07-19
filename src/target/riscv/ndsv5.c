/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#include "binarybuffer.h"
#include "target/target.h"
#include "target/register.h"
#include "jtag/interfaces.h"
#include "riscv.h"
#include "encoding.h"
#include "program.h"
#include "opcodes.h"
#include "ndsv5.h"
#include "ndsv5-013.h"
#include "log.h"
#include "target/nds32_log.h"

/********************************************************************/
/* Copy from riscv.c                                                */
/********************************************************************/
#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
/********************************************************************/



/********************************************************************/
/* Extern function/var. from riscv.c                                */
/********************************************************************/
extern uint32_t dtmcontrol_scan(struct target *target, uint32_t out);	/* declear here because riscv-013.c has same function*/
/********************************************************************/




/********************************************************************/
/* ndsv5.c global Var. */
/********************************************************************/
uint64_t LM_BASE = 0xa0000000;
uint32_t ndsv5_dis_cache_busmode = 1;
uint32_t ndsv5_dmi_busy_retry_times = 100;
uint64_t MSTATUS_VS = 0x01800000;
/********************************************************************/




int ndsv5_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints)
{
	if (ndsv5_step_check(target) != ERROR_OK) {
		LOG_ERROR("ndsv5_step_check failed");
		return ERROR_OK;
	}

	if (old_or_new_riscv_step(target, current, address, handle_breakpoints) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_step failed");
		return ERROR_FAIL;
	}

#if 0 /* _NDS_SUPPORT_WITHOUT_ANNOUNCING_ */
	if (target->state == TARGET_HALTED) {
		if (ndsv5_without_announce) {
			ndsv5_without_announce = 0;
			LOG_DEBUG("ndsv5_without_announce");
		} else {
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}
#endif
	return ERROR_OK;
}

int strict_step(struct target *target, bool announce)
{
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("strict_step (before): 0x%lx", (long unsigned int)reg_pc_value);

	struct breakpoint *breakpoint = target->breakpoints;
	while (breakpoint) {
		riscv_remove_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	struct watchpoint *watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_remove_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	if (announce == false)
		ndsv5_without_announce = 1;
	else
		ndsv5_without_announce = 0;

	int result = riscv_openocd_step(target, 1, 0, 0);
	if (result != ERROR_OK) {
		LOG_ERROR("riscv_openocd_step failed");
		return ERROR_FAIL;
	}

	breakpoint = target->breakpoints;
	while (breakpoint) {
		riscv_add_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_add_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	reg_pc->type->get(reg_pc);
	reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("strict_step (after): 0x%lx", (long unsigned int)reg_pc_value);
	return ERROR_OK;
}

int ndsv5_handle_triggered(struct target *target)
{
	int single_step_cmd = 0;

	uint64_t dcsr;
	riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
	int cause = get_field(dcsr, DCSR_CAUSE);

	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("halt at 0x%lx", (long unsigned int)reg_pc_value);

	switch (cause) {
		case DCSR_CAUSE_SWBP:
			ndsv5_virtual_hosting_check(target);
			break;
		case DCSR_CAUSE_HWBP:
			/* step and watchpoint, record */
			if (target->debug_reason == DBG_REASON_SINGLESTEP)
				single_step_cmd = 1;

			target->debug_reason = DBG_REASON_WPTANDBKPT;
			/* If we halted because of a data trigger, gdb doesn't know to do
			 * the disable-breakpoints-step-enable-breakpoints dance. */
			if (ndsv5_hit_watchpoint_check(target) == ERROR_OK) {
				if (ndsv5_get_watched_address(target) == ERROR_OK) {
					LOG_DEBUG("match watchpoint");
					target->debug_reason = DBG_REASON_WATCHPOINT;
				} else {
					/* false match watchpoint, resume target */
					LOG_DEBUG("false match watchpoint");
					/* current pc, addr = 0, do not handle breakpoints, not debugging */
					strict_step(target, false);

					/* step and watchpoint false match , return ok */
					if (single_step_cmd == 1) {
						target->debug_reason = DBG_REASON_SINGLESTEP;
						LOG_DEBUG("single step and false match watchpoint");
						return ERROR_OK;
					}

					riscv_resume(target, 1, 0, 0, 0);
					return ERROR_FAIL;
				}
			}
		break;
	}
	return ERROR_OK;
}

int ndsv5_poll(struct target *target)
{
	if (nds_skip_dmi == 1)
		return ERROR_OK;

	ndsv5_triggered_hart = -1;
	if (old_or_new_riscv_poll(target) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_poll failed");
		return ERROR_FAIL;
	}
	if (ndsv5_triggered_hart != -1) {
		if (ndsv5_handle_triggered(target) != ERROR_OK) {
			/* resume target */
		} else {
#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
			if (ndsv5_without_announce) {
				ndsv5_without_announce = 0;
				LOG_DEBUG("ndsv5_without_announce");
			} else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
#endif
		}
	}
	return ndsv5_handle_poll(target);
}

int ndsv5_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	if (riscv_halt(target) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_halt failed");
		return ERROR_FAIL;
	}
#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
	if (ndsv5_without_announce) {
		ndsv5_without_announce = 0;
		LOG_DEBUG("ndsv5_without_announce");
	} else {
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}
#endif
	/* to update if target->state == TARGET_HALTED */
	ndsv5_poll(target);

	return ndsv5_handle_halt(target);
}

int ndsv5_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	if (ndsv5_resume_check(target) != ERROR_OK) {
		LOG_ERROR("ndsv5_resume_check failed");
		return ERROR_OK;
	}

	if (riscv_resume(target, current, address, handle_breakpoints,
				debug_execution) != ERROR_OK) {
		LOG_ERROR("riscv_resume failed");
		return ERROR_FAIL;
	}

	return ndsv5_handle_resume(target);
}

extern struct jtag_interface *jtag_interface;
int ndsv5_examine(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	if (nds_script_custom_initial != NULL) {
		LOG_DEBUG("doing custom_initial_script...");
		if (ndsv5_script_do_custom_reset(target, nds_script_custom_initial) != ERROR_OK)
			return ERROR_FAIL;
		nds_script_custom_initial = NULL;
		LOG_DEBUG("custom_initial_script finish");
	}

	if (target_was_examined(target))
		return ERROR_OK;

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	uint32_t retry_cnt = 0;
	uint32_t dtmcontrol = 0;
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
		dtmcontrol = dtmcontrol_scan(target, 0);
		LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
		if ((dtmcontrol == 0x0) || (dtmcontrol == 0xFFFFFFFF)) {
			/* do jtag_interface->init() again when JTAG examine chain failed (SW workaround) */
			jtag_interface->quit();
			alive_sleep(1000);
			jtag_interface->init();
			jtag_init_inner(NULL);
		} else
			break;
	}

	riscv_examine(target);
	ndsv5_handle_examine(target);
	return ERROR_OK;
}

int modify_trigger_address_mbit_match(struct target *target, struct trigger *trigger)
{
	uint64_t new_address = trigger->address;
	uint32_t i, new_length = trigger->length;
	uint64_t mbit_mask, mbit_value;

	while (1) { /* (((new_address & mbit_mask) + new_length) < (trigger->address + trigger->length)) */
		for (i = 0; i < 32; i++) {
			if ((uint32_t)(0x01 << i) >= new_length)
				break;
		}
		new_length = (0x01 << i);
		if (new_address % new_length) {
			new_length <<= 1;
			i++;
		}
#if 0
		for (i = 0; i < 32; i++) {
			if ((uint32_t)(0x01 << i) == new_length)
				break;
		}
#endif
		if (i == 0) {
			LOG_DEBUG("ERROR length, new_address:0x%" PRIx64 ", new_length:0x%x", new_address, new_length);
			return ERROR_OK;
		}
		mbit_mask = ~((0x01 << i) - 1);
		mbit_value = (0x01 << (i - 1)) - 1;
		new_address &= mbit_mask;
		new_address |= mbit_value;

		LOG_DEBUG("new_address:0x%" PRIx64 ", new_length:0x%x", new_address, new_length);
		if (((new_address & mbit_mask) + new_length) < (trigger->address + trigger->length))
			new_address = trigger->address;
		else
			break;
	}
	LOG_DEBUG("real new_address:0x%" PRIx64 ", new_length:0x%x", new_address, new_length);

	/* redefine: trigger->address */
	trigger->address = new_address;
	return ERROR_OK;
}

int ndsv5_writebuffer(struct target *target, target_addr_t address,
		uint32_t writesize, const uint8_t *buffer)
{
	/*
	target_addr_t physical_address;
	*/
	riscv_select_current_hart(target);

#if 0
	/* write_memory will doing va2pa!!! */
	physical_address = address;
	if (ndsv5_virtual_to_physical(target, address, &physical_address) != ERROR_OK)
		return ERROR_FAIL;
	return ndsv5_write_buffer(target, physical_address, writesize, buffer);
#endif
	return ndsv5_write_buffer(target, address, writesize, buffer);
}

#if 0
/* Page table PPN shift amount */
#define PTE_PPN_SHIFT       10
#define RISCV_PGSHIFT       12
#define RISCV_PGSIZE (1 << RISCV_PGSHIFT)
#endif

/* Leaf page shift amount */
#define PGSHIFT             12

extern uint64_t ndsv5_reg_misa_value;
int ndsv5_get_physical_address(struct target *target, target_addr_t addr, target_addr_t *physical)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
	nds32->nds_va_to_pa_off = 1;
	uint64_t physical_addr = addr;
	uint64_t value_misa = 0;

	if (ndsv5_reg_misa_value == 0) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			/* use quick mode to read CSR while target_not_halted */
			if (riscv_debug_buffer_size(target) < 6)
				goto ndsv5_get_physical_address_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MISA + GDB_REGNO_CSR0, &value_misa);
		} else {
			struct reg *reg_misa = ndsv5_get_reg_by_CSR(target, CSR_MISA);
			if (reg_misa != NULL)
				value_misa = ndsv5_get_register_value(reg_misa);
		}
		if ((value_misa & 0x40000) == 0)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist = false;
		else
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist = true;
	}
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist == false)
		goto ndsv5_get_physical_address_OK;

	uint64_t base, satp;
	uint32_t levels, ptidxbits, ptesize, vm;

	if ((nds_dmi_quick_access) && (!riscv_is_halted(target)))
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_CSR0 + CSR_SATP, &satp);
	else
		riscv_get_register(target, &satp, GDB_REGNO_CSR0 + CSR_SATP);

	if (riscv_xlen(target) == 64) {
		base = ((satp & SATP64_PPN) << PGSHIFT);
		vm = ((satp & SATP64_MODE) >> 60);
	} else {
		base = ((satp & SATP32_PPN) << PGSHIFT);
		vm = ((satp & SATP32_MODE) >> 31);
	}
	switch (vm) {
		case SATP_MODE_SV32:
			levels = 2; ptidxbits = 10; ptesize = 4; break;
		case SATP_MODE_SV39:
			levels = 3; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_SV48:
			levels = 4; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_SV57:
			levels = 5; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_OFF:
			goto ndsv5_get_physical_address_OK;
		default:
			goto ndsv5_get_physical_address_ERR;
	}

#if 0
	uint32_t va_bits = PGSHIFT + levels * ptidxbits;
	uint64_t mask = (1L << (TARGET_LONG_BITS - (va_bits - 1))) - 1;
	uint64_t masked_msbs = (addr >> (va_bits - 1)) & mask;
	if (masked_msbs != 0 && masked_msbs != mask) {
		LOG_ERROR("ERROR");
		return ERROR_FAIL;
	}
#endif

	uint32_t ptshift = (levels - 1) * ptidxbits;
	LOG_DEBUG("ptshift: 0x%x, levels: 0x%x, ptesize: 0x%x", (int)ptshift, (int)levels, (int)ptesize);
	uint32_t i;
	for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
		uint64_t idx = (addr >> (PGSHIFT + ptshift)) &
			((1 << ptidxbits) - 1);
		idx &= ((1 << ptidxbits) - 1);
		/* check that physical address of PTE is legal */
		uint64_t pte_addr = base + idx * ptesize;
		uint64_t pte = 0; /* ldl_phys(cs->as, pte_addr); */
		target_read_memory(target, pte_addr, ptesize, 1, (uint8_t *)&pte);

		uint64_t ppn = pte >> PTE_PPN_SHIFT;
		LOG_DEBUG("i: 0x%x, pte: 0x%" PRIx64 " ppn: 0x%" PRIx64, i, pte, ppn);
		LOG_DEBUG("pte_addr: 0x%" PRIx64 " base: 0x%" PRIx64, pte_addr, base);
		if (!(pte & PTE_V)) {
			/* Invalid PTE */
			LOG_ERROR("Invalid PTE");
			goto ndsv5_get_physical_address_ERR;
		} else if (!(pte & (PTE_R | PTE_W | PTE_X))) {
			/* Inner PTE, continue walking */
			base = ppn << PGSHIFT;
		} else if ((pte & (PTE_R | PTE_W | PTE_X)) == PTE_W) {
			/* Reserved leaf PTE flags: PTE_W */
			LOG_ERROR("Reserved leaf PTE flags: PTE_W");
			goto ndsv5_get_physical_address_ERR;
		} else if ((pte & (PTE_R | PTE_W | PTE_X)) == (PTE_W | PTE_X)) {
			/* Reserved leaf PTE flags: PTE_W + PTE_X */
			LOG_ERROR("Reserved leaf PTE flags: PTE_W + PTE_X");
			goto ndsv5_get_physical_address_ERR;
#if 0
			} else if ((pte & PTE_U) && ((mode != PRV_U) &&
				   (!sum || access_type == MMU_INST_FETCH))) {
			    /* User PTE flags when not U mode and mstatus.SUM is not set,
			       or the access type is an instruction fetch */
			    goto ndsv5_get_physical_address_ERR;
			} else if (!(pte & PTE_U) && (mode != PRV_S)) {
			    /* Supervisor PTE flags when not S mode */
			    goto ndsv5_get_physical_address_ERR;
#endif
		} else if (ppn & ((1ULL << ptshift) - 1)) {
			/* Misaligned PPN */
			LOG_ERROR("Misaligned PPN");
			goto ndsv5_get_physical_address_ERR;
#if 0
		} else if (access_type == MMU_DATA_LOAD && !((pte & PTE_R) ||
			   ((pte & PTE_X) && mxr))) {
			/* Read access check failed */
			LOG_ERROR("Read access check failed");
			goto ndsv5_get_physical_address_ERR;
		} else if (access_type == MMU_DATA_STORE && !(pte & PTE_W)) {
			/* Write access check failed */
			LOG_ERROR("Write access check failed");
			goto ndsv5_get_physical_address_ERR;
		} else if (access_type == MMU_INST_FETCH && !(pte & PTE_X)) {
			/* Fetch access check failed */
			LOG_ERROR("Fetch access check failed");
			goto ndsv5_get_physical_address_ERR;
#endif
		} else {
			/* for superpage mappings, make a fake leaf PTE for the TLB's
			   benefit. */
			uint64_t vpn = addr >> PGSHIFT;
			physical_addr = (ppn | (vpn & ((1L << ptshift) - 1))) << PGSHIFT;
			physical_addr |= (addr & ((1L << PGSHIFT) - 1));
			goto ndsv5_get_physical_address_OK;
		}
	}  /* for (i = 0; i < levels; ... */

ndsv5_get_physical_address_ERR:
	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	LOG_DEBUG("ERROR: VP: 0x%" PRIx64 " PA: 0x%" PRIx64, addr, physical_addr);
	*physical = physical_addr;
	return ERROR_FAIL;

ndsv5_get_physical_address_OK:
	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	LOG_DEBUG("OK: VP: 0x%" PRIx64 " PA: 0x%" PRIx64, addr, physical_addr);
	*physical = physical_addr;
	return ERROR_OK;
}

int ndsv5_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	uint64_t mstatus = 0;
	uint64_t dcsr = 0;

	LOG_DEBUG("va: 0x%" TARGET_PRIxADDR ", va_mode: %s", address,
			(nds32->nds_va_to_pa_off == 1) ? "OFF" : "ON");
	if (nds32->nds_va_to_pa_off == 1) {  /* va: off */
		*physical = address;
		return ERROR_OK;
	}
	ndsv5_use_mprv_mode = 0;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6) {
			*physical = address;
			return ERROR_OK;
		}
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
		NDS_INFO("qmode get dcsr: 0x%" PRIx64, dcsr);
	} else
		riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);

	/* Check prv */
	if ((dcsr & 0x3) == 0x3) {		/* M-mode => VA=PA */
		*physical = address;
		return ERROR_OK;
	}

	if (memory->access_channel == NDS_MEMORY_ACC_BUS)
		return ndsv5_get_physical_address(target, address, physical);
	/* else if (memory->access_channel == NDS_MEMORY_ACC_CPU) */
	/* Enable mprven */
	dcsr |= 0x10;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, &mstatus);
		ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_DCSR, dcsr);
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
	} else {
		riscv_get_register(target, &mstatus, GDB_REGNO_MSTATUS);
		riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
		riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
	}
	if ((dcsr&0x10) == 0x0) {
		LOG_DEBUG("This core doesn't support use mprv mode!!");
		*physical = address;
		return ERROR_OK;
	}

	ndsv5_use_mprv_mode = 1;
	ndsv5_backup_mstatus = mstatus;
	mstatus |= MSTATUS_MPRV;	/* Enable mprv */
	mstatus &= ~(MSTATUS_MPP);
	mstatus |= (1) << 11;		/* Set mstatus.MPP to supervisor */
	mstatus |= MSTATUS_SUM;		/* Set SUM = 1 */
	mstatus |= MSTATUS_MXR;		/* Set MXR = 1 */

	if ((nds_dmi_quick_access) && (!riscv_is_halted(target)))
		ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, mstatus);
	else
		riscv_set_register(target, GDB_REGNO_MSTATUS, mstatus);

	*physical = address;
	return ERROR_OK;
}

void bus_mode_on(struct target *target, uint64_t *reg_value_backup)
{
	uint64_t old_mcache_ctl = 0;
	uint64_t new_mcache_ctl = 0;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6)
				return;
		ndsv5_get_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, &old_mcache_ctl);
		*reg_value_backup = old_mcache_ctl;
		new_mcache_ctl = (old_mcache_ctl & ~(0x3));
		ndsv5_set_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, new_mcache_ctl);
		return;
	}
	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG("No mcache_ctl, No support dis_cache_busmode");
		return;
	}
	old_mcache_ctl = ndsv5_get_register_value(reg_mcache_ctl);
	*reg_value_backup = old_mcache_ctl;
	new_mcache_ctl = (old_mcache_ctl & ~(0x3));
	ndsv5_set_register_value(reg_mcache_ctl, new_mcache_ctl);
	LOG_DEBUG("old_mcache_ctl: 0x%x, new_mcache_ctl: 0x%x", (int)old_mcache_ctl, (int)new_mcache_ctl);
}

void bus_mode_off(struct target *target, uint64_t reg_value)
{
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6)
				return;
		ndsv5_set_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, reg_value);
		return;
	}
	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG("No mcache_ctl, No support dis_cache_busmode");
		return;
	}
	ndsv5_set_register_value(reg_mcache_ctl, reg_value);
}

struct reg *ndsv5_get_reg_by_CSR(struct target *target, uint32_t csr_id)
{
	char *reg_name = ndsv5_get_CSR_name(target, csr_id);
	struct reg *reg;

	if (reg_name == NULL) {
		LOG_DEBUG("get reg_name ERROR");
		return NULL;
	}

	reg = register_get_by_name(target->reg_cache, reg_name, 1);
	if (reg == NULL) {
		LOG_DEBUG("get reg ERROR");
		return NULL;
	}
	return reg;
}

uint64_t ndsv5_get_register_value(struct reg *reg)
{
	uint64_t reg_value;
	reg->type->get(reg);
	reg_value = buf_get_u64(reg->value, 0, reg->size);
	return reg_value;
}

void ndsv5_set_register_value(struct reg *reg, uint64_t reg_value)
{
	uint8_t reg_bytes[8];
	buf_set_u64(reg_bytes, 0, reg->size, reg_value);
	reg->type->set(reg, reg_bytes);
}

extern uint64_t nds_support_syscall_id[];
static int riscv_step_virtual_hosting_checking(struct target *target)
{
	uint32_t cur_instr = 0;
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	if (target_read_memory(target, reg_pc_value, 4, 1, (uint8_t *)&cur_instr) != ERROR_OK) {
		LOG_ERROR("can't read memory: 0x%" TARGET_PRIxADDR, reg_pc_value);
		return ERROR_FAIL;
	}
	LOG_DEBUG("reg_pc_value = 0x%" TARGET_PRIxADDR ", cur_instr=0x%x", reg_pc_value, cur_instr);
	cur_instr &= MASK_C_EBREAK;
	if (cur_instr == MATCH_C_EBREAK) {
		/* ebreak, check a7 value */
		struct reg *reg_a7 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[17], 1);
		reg_a7->type->get(reg_a7);
		uint64_t reg_a7_value = buf_get_u64(reg_a7->value, 0, reg_a7->size);
		uint32_t i;
		for (i = 0; i < NDS_EBREAK_NUMS; i++) {
			if (reg_a7_value == nds_support_syscall_id[i])
				break;
		}
		if (i == NDS_EBREAK_NUMS) {
			/* NOT syscall_id (maybe target break insn. ) */
			return ERROR_FAIL;
		}
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);

		/* WARNING: potential issue on target64 */
		nds32->active_syscall_id = (uint32_t)reg_a7_value;
		nds32->hit_syscall = true;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

int ndsv5_step_check(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->hit_syscall == true) {
		/* skip ebreak */
		struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
		reg_pc->type->get(reg_pc);
		uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
		int ebreak_length = ndsv5_get_ebreak_length(target, reg_pc_value);
		reg_pc_value += ebreak_length;  /* "ebreak" length */
		reg_pc->type->set(reg_pc, (uint8_t *)&reg_pc_value);
		nds32->hit_syscall = false;
		LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, reg_pc_value);
	}
	if (nds32->virtual_hosting_ctrl_c == true) {
		LOG_DEBUG("virtual_hosting_ctrl_c = true");
		nds32->virtual_hosting_ctrl_c = false;
		return ERROR_FAIL;
	}

	if (riscv_step_virtual_hosting_checking(target) == ERROR_OK) {
		LOG_DEBUG("it's virtual hosting, skip stepping !!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int ndsv5_reset_target(struct target *target, enum target_reset_mode reset_mode)
{
	int reset_halt_bak = target->reset_halt;

	if (reset_mode == RESET_HALT)
		target->reset_halt = 1;
	else
		target->reset_halt = 0;

	CHECK_RETVAL(target->type->assert_reset(target));
	CHECK_RETVAL(target->type->deassert_reset(target));

	target->reset_halt = reset_halt_bak;
	return ERROR_OK;
}

int ndsv5_srst_reset_target(struct target *target)
{
	LOG_DEBUG("Initializing with hard TRST+SRST reset");
	int retval;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

#if 0
	jtag_add_reset(1, 0);   /* TAP_RESET */
	if (jtag_reset_config & RESET_HAS_SRST) {
		jtag_add_reset(1, 1);
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) == 0)
			jtag_add_reset(0, 1);
	}
	jtag_add_reset(0, 0);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("Reset execute queue failed!!");
		return retval;
	}
#endif

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (jtag_reset_config & RESET_HAS_SRST)
		jtag_add_reset(1, 1);

	alive_sleep(1000);

	/* deassert_reset */
	jtag_add_reset(0, 0);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("Reset execute queue failed!!");
		return retval;
	}
	alive_sleep(nds32->reset_time);

	return ERROR_OK;
}

int ndsv5_resume_check(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->hit_syscall == true) {
		/* skip ebreak */
		struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
		reg_pc->type->get(reg_pc);
		uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
		int ebreak_length = ndsv5_get_ebreak_length(target, reg_pc_value);
		reg_pc_value += ebreak_length;
		reg_pc->type->set(reg_pc, (uint8_t *)&reg_pc_value);
		nds32->hit_syscall = false;
		LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, reg_pc_value);
	}
	if (nds32->virtual_hosting_ctrl_c == true) {
		LOG_DEBUG("virtual_hosting_ctrl_c = true");
		nds32->virtual_hosting_ctrl_c = false;
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

extern int nds32_get_buffer_access_size(uint64_t start_addr, uint32_t bufsize,
		uint32_t *pcurr_size, uint32_t *paccess_size);
extern uint32_t nds_force_word_access;
extern uint32_t nds_force_aligned_access;
int ndsv5_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	riscv_info_t *info = (riscv_info_t *)target->arch_info;

	/* check if 011, or  word_access/aligned_access disable */
	if ((info->dtm_version == 0) ||
		((nds_force_word_access == 0) && (nds_force_aligned_access == 0))) {
		return tt->read_memory(target, address, size, count, buffer);
	}
	uint32_t i;
	if ((nds_force_aligned_access == 1) && ((address % size) != 0)) {
		/* if 013, unaligned access */
		uint64_t align_addr = 0;
		uint32_t data_val1 = 0, data_val2 = 0;

		for (i = 0; i < count; i++) {
			align_addr = (address & ~0x03);
			tt->read_memory(target, align_addr, 4, 1, (uint8_t *)&data_val1);
			tt->read_memory(target, align_addr+4, 4, 1, (uint8_t *)&data_val2);
			LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", data_val1=0x%x, data_val2=0x%x", align_addr, data_val1, data_val2);
			*buffer++ = (data_val1 >> 16) & 0xff;
			*buffer++ = (data_val1 >> 24) & 0xff;
			*buffer++ = (data_val2 & 0xff);
			*buffer++ = (data_val2 >> 8) & 0xff;
			address += 4;
		}
		return ERROR_OK;
	}

	if (nds_force_word_access == 1) {
		for (i = 0; i < count; i++) {
			tt->read_memory(target, address, size, 1, buffer);
			address += size;
			buffer += size;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, readsize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	nds32_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
	if (((total_size % access_size) != 0) ||
		((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return tt->read_memory(target, address, access_size, access_cnt, buffer);
	}

	while (total_size) {
		nds32_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
		access_cnt = readsize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = tt->read_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

int ndsv5_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	riscv_info_t *info = (riscv_info_t *)target->arch_info;

	/* check if 011, or  word_access/aligned_access disable */
	if ((info->dtm_version == 0) || (nds_force_aligned_access == 0))
		return tt->write_memory(target, address, size, count, buffer);

	uint32_t i;
	if ((nds_force_aligned_access == 1) && ((address % size) != 0)) {
		/* if 013, unaligned access */
		for (i = 0; i < count; i++) {
			tt->write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
			tt->write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, writesize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	nds32_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
	if (((total_size % access_size) != 0) ||
		((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return tt->write_memory(target, address, access_size, access_cnt, buffer);
	}

	while (total_size) {
		nds32_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
		access_cnt = writesize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = tt->write_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

int ndsv5_readwrite_byte(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer, bool if_write)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	int retval;

	if (if_write)
		retval = tt->write_memory(target, address, size, count, buffer);
	else
		retval = tt->read_memory(target, address, size, count, (uint8_t *) buffer);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

extern int ndsv5_gdb_fileio_write_memory(struct target *target, target_addr_t address,
		uint32_t *psize, uint8_t **pbuffer);
int ndsv5_write_buffer(struct target *target, target_addr_t address, uint32_t writesize, const uint8_t *buffer)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t total_size = writesize;
	uint8_t *write_buffers = (uint8_t *)buffer;

	LOG_DEBUG("writing buffer of %" PRIi32 " byte at " TARGET_ADDR_FMT, writesize, address);
	if (writesize == 0)
		return ERROR_OK;

	if (nds32->hit_syscall) {
		ndsv5_gdb_fileio_write_memory(target, address, &total_size, &write_buffers);
		LOG_DEBUG("gdb_fileio_write_memory, total_size=%d", total_size);
	}


	/* Copy from target.c */
	uint32_t count = total_size;
	uint32_t size;

	/* Align up to maximum 4 bytes. The loop condition makes sure the next pass
	 *          * will have something to do with the size we leave to it. */
	for (size = 1; size < 4 && count >= size * 2 + (address & size); size *= 2) {
		if (address & size) {
			int retval = target_write_memory(target, address, size, 1, write_buffers);
			if (retval != ERROR_OK)
				return retval;
			address += size;
			count -= size;
			write_buffers += size;
		}
	}

	/* Write the data with as large access size as possible. */
	for (; size > 0; size /= 2) {
		uint32_t aligned = count - count % size;
		if (aligned > 0) {
			int retval = target_write_memory(target, address, size, aligned / size, write_buffers);
			if (retval != ERROR_OK)
				return retval;
			address += aligned;
			count -= aligned;
			write_buffers += aligned;
		}
	}

	return ERROR_OK;
}

struct cache_element {
	uint64_t pa;
	uint64_t cacheline[32];
	uint8_t dirty;
	uint8_t valid;
	uint8_t lock;
	uint8_t inval;
	uint8_t shared;
	uint8_t exclusive;
	uint8_t modified;
};
/* command value */
#define L1D_WBINVAL_ALL 6
#define L1D_WB_ALL 7
#define L1D_IX_RTAG 19
#define L1D_IX_RDATA 20
#define L1D_INVAL_ALL 23
#define L1I_IX_INVAL 24
#define L1I_IX_RTAG 27
#define L1I_IX_RDATA 28
/* This is the number of cache entries. User should change it if necessary. */
#define CACHE_SET_NUM 0x1000
#define CACHE_WAY_NUM 0x8
struct cache_element ce[CACHE_SET_NUM][CACHE_WAY_NUM];
#define NDS_PAGE_SHIFT 12		/* for PAGE_SIZE 4KB */
#define NDS_PAGE_SIZE  (1UL << NDS_PAGE_SHIFT)

/* palen : 34bit  TAG:PA[palen-1:10 */
#define CCTL_mskTAG_32 0xffffff

/* palen : 64bit  TAG:PA[palen-1:10] */
#define CCTL_mskTAG_64 0x3fffffffffffff

int ndsv5_init_cache(struct target *target)
{
	LOG_DEBUG("ndsv5_init_cache");

	riscv_select_current_hart(target);

	uint8_t seth = 0;
	uint64_t value_cr1 = 0;
	uint64_t value_cr2 = 0;
	struct reg *reg_micm_cfg, *reg_mdcm_cfg;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *icache = &nds32->memory.icache;
	struct nds32_v5_cache *dcache = &nds32->memory.dcache;

	reg_micm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
	reg_mdcm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
	if ((reg_micm_cfg == NULL) || (reg_mdcm_cfg == NULL))
		return ERROR_FAIL;

	value_cr1 = ndsv5_get_register_value(reg_micm_cfg);
	value_cr2 = ndsv5_get_register_value(reg_mdcm_cfg);

	seth = (uint8_t)((value_cr1 >> 24) & 0x1);
	icache->set = value_cr1 & 0x7;
	if (seth) {
		icache->log2_set = 5 - icache->set;
		icache->set = 32 >> icache->set;
	} else {
		icache->log2_set = icache->set + 6;
		icache->set = 64 << icache->set;
	}
	icache->way = ((value_cr1 >> 3) & 0x7) + 1;
	icache->line_size = (value_cr1 >> 6) & 0x7;
	if (icache->line_size != 0) {
		icache->log2_line_size = icache->line_size + 2;
		icache->line_size = 8 << (icache->line_size - 1);
	} else {
		icache->log2_line_size = 0;
	}

	LOG_DEBUG("\ticache set: %lld, way: %lld, line size: %lld, "
			"log2(set): %lld, log2(line_size): %lld",
			icache->set, icache->way, icache->line_size,
			icache->log2_set, icache->log2_line_size);

	seth = (uint8_t)((value_cr2 >> 24) & 0x1);
	dcache->set = value_cr2 & 0x7;
	if (seth) {
		dcache->log2_set = 5 - dcache->set;
		dcache->set = 32 >> dcache->set;
	} else {
		dcache->log2_set = dcache->set + 6;
		dcache->set = 64 << dcache->set;
	}
	dcache->way = ((value_cr2 >> 3) & 0x7) + 1;
	dcache->line_size = (value_cr2 >> 6) & 0x7;
	if (dcache->line_size != 0) {
		dcache->log2_line_size = dcache->line_size + 2;
		dcache->line_size = 8 << (dcache->line_size - 1);
	} else {
		dcache->log2_line_size = 0;
	}

	LOG_DEBUG("\tdcache set: %lld, way: %lld, line size: %lld, "
			"log2(set): %lld, log2(line_size): %lld",
			dcache->set, dcache->way, dcache->line_size,
			dcache->log2_set, dcache->log2_line_size);

	return ERROR_OK;
}

int ndsv5_dump_cache(struct target *target, unsigned int cache_type, const char* filename)
{
	LOG_DEBUG("Dump Cache");

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand, *reg_mcctldata, *reg_mmsc_cfg, *reg_marchid;
	FILE *pFile;
	uint64_t idx, way, ra, tag, i, maskTAG, read_tag_cmd, read_data_cmd, reg_marchid_value;
	uint64_t sets, ways, line_size, set_bits, line_bits, way_offset;
	uint64_t cache_index, word_num, word_size;
	struct nds32_v5_cache *cache;
	uint64_t total_cache;
	uint64_t now_cache = 0;
	int xlen, shift_bit, mesi;
	bool idx_auto = false, new_tagformat = false;

	pFile = fopen(filename, "w");
	if (NULL == pFile) {
		LOG_ERROR("Error!! Can't open file to write");
		return ERROR_FAIL;
	}

	if (ndsv5_init_cache(target) != ERROR_OK)
		return ERROR_FAIL;

	if (cache_type == ICACHE) {
		cache = &nds32->memory.icache;
		read_tag_cmd = L1I_IX_RTAG;
		read_data_cmd = L1I_IX_RDATA;
	} else if (cache_type == DCACHE) {
		cache = &nds32->memory.dcache;
		read_tag_cmd = L1D_IX_RTAG;
		read_data_cmd = L1D_IX_RDATA;
	} else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	xlen = riscv_xlen(target);
	if (xlen == 32)
		maskTAG = CCTL_mskTAG_32;
	else if (xlen == 64)
		maskTAG = CCTL_mskTAG_64;
	else {
		LOG_ERROR("%s not supported xlen:%d", __func__, xlen);
		return ERROR_FAIL;
	}

	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	reg_mcctldata = ndsv5_get_reg_by_CSR(target, CSR_MCCTLDATA);
	reg_mmsc_cfg = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
	if ((reg_mcctlbeginaddr == NULL) ||
	    (reg_mcctlcommand == NULL) ||
	    (reg_mcctldata == NULL) ||
	    (reg_mmsc_cfg == NULL))
		return ERROR_FAIL;

	/* if vcctl=1, index will auto count */
	if (ndsv5_get_register_value(reg_mmsc_cfg) & 0x40000) {
		idx_auto = true;
		LOG_DEBUG("VCCTL=1, AUTO COUNT INDEX");
	}

	ways = cache->way;
	sets = cache->set;
	set_bits = cache->log2_set;
	line_bits = cache->log2_line_size;
	line_size = cache->line_size;
	way_offset = set_bits + line_bits;
	LOG_DEBUG("Way:%lld, Set:%lld, Line Size:%lld", ways, sets, line_size);

	/* check cpu id == 0x45 and dcache, tag no used bit = shift_bit, shift bit = log2 line_size + log2 set;
	 * others, shift bit = 10 */
	reg_marchid = ndsv5_get_reg_by_CSR(target, CSR_MARCHID);
	reg_marchid_value = ndsv5_get_register_value(reg_marchid);
	if (((reg_marchid_value & 0xff) == 0x45) && (cache_type == DCACHE)) {
		new_tagformat = true;
		shift_bit = line_bits + set_bits;
	} else
		shift_bit = 10;

	/* Index Example Format for CCTL Index Type Operation for 64bit icache
	 * same with 32bit i/dcache(from Roger email) */
	if ((xlen == 64) && (cache_type == ICACHE))
		word_size = 4;
	else
		word_size = xlen / 8;
	word_num = line_size / word_size;
	total_cache = ways * sets * word_num;
	LOG_DEBUG("Total cache:%lld", total_cache);

	/* READ TAG/DATA COMMAND :auto count idx or no auto count idx */
	if (idx_auto) {
		/* READ TAG COMMAND */
		ndsv5_set_register_value(reg_mcctlbeginaddr, 0);
		if (new_tagformat) {
			for (idx = 0; idx < sets; idx++) {
				for (way = 0; way < ways; way++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
					tag = ndsv5_get_register_value(reg_mcctldata);
					LOG_DEBUG("idx_auto tag: %llx", tag);

					mesi = tag & 0x7;
					ce[idx][way].inval = (mesi == 0);
					ce[idx][way].shared = (mesi == 1);
					ce[idx][way].exclusive = (mesi == 3);
					ce[idx][way].modified = (mesi == 7);

					ce[idx][way].lock = (uint8_t)(tag & 0x8) >> 3;
					ce[idx][way].pa = (tag >> 4) << shift_bit ;
				}
			}
		} else {
			for (idx = 0; idx < sets; idx++) {
				for (way = 0; way < ways; way++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
					tag = ndsv5_get_register_value(reg_mcctldata);
					LOG_DEBUG("idx_auto tag: %llx", tag);

					ce[idx][way].valid = (uint8_t)((tag & (1ULL << (xlen - 1))) >> (xlen - 1));
					ce[idx][way].lock = (uint8_t)((tag & (1ULL << (xlen - 2))) >> (xlen - 2));
					ce[idx][way].dirty = (uint8_t)((tag & (1ULL << (xlen - 3))) >> (xlen - 3));
					ce[idx][way].pa = (tag & maskTAG) << shift_bit;
				}
			}
		}

		/* READ DATA COMMAND */
		ndsv5_set_register_value(reg_mcctlbeginaddr, 0);
		for (idx = 0; idx < sets; idx++) {
			for (way = 0; way < ways; way++) {
				for (i = 0; i < word_num; i++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
					ce[idx][way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
				}
				now_cache += word_num;
				NDS32_LOG_R("Dump Progressing...%lld%%", ((now_cache*100)/total_cache));
			}
		}
	} else {
		for (idx = 0; idx < sets; idx++) {
			for (way = 0; way < ways; way++) {
				/* READ TAG COMMAND */
				ra = (way << way_offset) | (idx << line_bits);
				ndsv5_set_register_value(reg_mcctlbeginaddr, ra);
				ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
				tag = ndsv5_get_register_value(reg_mcctldata);
				LOG_DEBUG("tag: %llx", tag);

				if (new_tagformat) {
					mesi = tag & 0x7;
					ce[idx][way].inval = (mesi == 0);
					ce[idx][way].shared = (mesi == 1);
					ce[idx][way].exclusive = (mesi == 3);
					ce[idx][way].modified = (mesi == 7);

					ce[idx][way].lock = (uint8_t)(tag & 0x8) >> 3;
					ce[idx][way].pa = (tag >> 4) << shift_bit ;
				} else {
					ce[idx][way].valid = (uint8_t)((tag & (1ULL << (xlen - 1))) >> (xlen - 1));
					ce[idx][way].lock = (uint8_t)((tag & (1ULL << (xlen - 2))) >> (xlen - 2));
					ce[idx][way].dirty = (uint8_t)((tag & (1ULL << (xlen - 3))) >> (xlen - 3));
					ce[idx][way].pa = (tag & maskTAG) << shift_bit;
				}

				/* READ DATA COMMAND */
				for (i = 0; i < word_num; i++) {
					cache_index = (ra | (i * word_size));
					ndsv5_set_register_value(reg_mcctlbeginaddr, cache_index);
					ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
					ce[idx][way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
				}
				now_cache += word_num;
				NDS32_LOG_R("Dump Progressing...%lld%%", ((now_cache*100)/total_cache));
			}
		}
	}

	/* Print 32bit, 64bit icache, 64bit dcache */
	char *fmt_str = " %8llx";
	char *fmt_str1 = "%08llx ";
	char *fmt_str2 = "%8s %4s %4s %1s %1s %1s";
	char *fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x ";
	if (xlen == 64) {
		if (cache_type == DCACHE) {
			fmt_str = " %16llx";
			fmt_str1 = "%016llx ";
		}
		fmt_str2 = "%16s %4s %4s %1s %1s %1s";
		fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x ";
	}
	if (new_tagformat) {
		fmt_str2 = "%8s %4s %4s %1s %1s %1s %1s %1s";
		fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x %01x %01x ";
		if (xlen == 64) {
			fmt_str2 = "%16s %4s %4s %1s %1s %1s %1s %1s";
			fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x %01x %01x ";
		}
	}

	fprintf(pFile, "dump %s\n", cache_type ? "DCACHE" : "ICACHE");
	if (new_tagformat)
		fprintf(pFile, fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		fprintf(pFile, fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		fprintf(pFile, fmt_str, (i * word_size));
	fprintf(pFile, "\n");
	for (idx = 0; idx < sets; idx++) {
		for (way = 0; way < ways; way++) {
			if (new_tagformat) {
				fprintf(pFile, fmt_str3,
					ce[idx][way].pa | (idx * line_size),
					idx,
					way,
					ce[idx][way].inval,
					ce[idx][way].shared,
					ce[idx][way].exclusive,
					ce[idx][way].modified,
					ce[idx][way].lock);
			} else {
				fprintf(pFile, fmt_str3,
					ce[idx][way].pa | (idx * line_size),
					idx,
					way,
					ce[idx][way].valid,
					ce[idx][way].dirty,
					ce[idx][way].lock);
			}

			for (i = 0; i < word_num; i++)
				fprintf(pFile, fmt_str1, ce[idx][way].cacheline[i]);
			fprintf(pFile, "\n");
		}
	}

	NDS32_LOG("\nDump Finish!!");
	LOG_DEBUG("\nDump Finish!!");
	fclose(pFile);
	return ERROR_OK;
}

int ndsv5_dump_cache_va(struct target *target, unsigned int cache_type, uint64_t va)
{
	LOG_DEBUG("Dump Cache");

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand, *reg_mcctldata, *reg_marchid;
	uint64_t idx, way, ra, tag, i, maskTAG, read_tag_cmd, read_data_cmd, reg_marchid_value;
	uint64_t sets, ways, line_size, line_bits, set_bits, way_offset;
	uint64_t cache_index, word_num, word_size;
	struct nds32_v5_cache *cache;
	struct cache_element ces[8];    /* maximum mdicm_cfg.DWAY is 8 */
	int xlen, shift_bit, mesi;
	bool new_tagformat = false;

	if (ndsv5_init_cache(target) != ERROR_OK)
		return ERROR_FAIL;

	if (cache_type == ICACHE) {
		cache = &nds32->memory.icache;
		read_tag_cmd = L1I_IX_RTAG;
		read_data_cmd = L1I_IX_RDATA;
	} else if (cache_type == DCACHE) {
		cache = &nds32->memory.dcache;
		read_tag_cmd = L1D_IX_RTAG;
		read_data_cmd = L1D_IX_RDATA;
	} else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	xlen = riscv_xlen(target);
	if (xlen == 32)
		maskTAG = CCTL_mskTAG_32;
	else if (xlen == 64)
		maskTAG = CCTL_mskTAG_64;
	else {
		LOG_ERROR("%s not supported xlen:%d", __func__, xlen);
		return ERROR_FAIL;
	}

	ways = cache->way;
	sets = cache->set;
	line_size = cache->line_size;
	line_bits = cache->log2_line_size;
	set_bits = cache->log2_set;
	way_offset = set_bits + line_bits;

	uint64_t pa = va;
	ndsv5_get_physical_address(target, va, &pa);
	LOG_DEBUG("physical address:0x%llx", pa);

	/* if dcache use pa to index; if icache use va to index */
	if (cache_type == DCACHE)
		idx = (pa & (((1ULL << set_bits) - 1) << line_bits));
	else
		idx = (va & (((1ULL << set_bits) - 1) << line_bits));

	LOG_DEBUG("Way:%lld, Set:%lld, Line Size:%lld", ways, sets, line_size);

	/* check cpu id == 0x45 and dcache, tag no used bit = shift_bit, shift bit = log2 line_size + log2 set;
	 * others, shift bit = 10 */
	reg_marchid = ndsv5_get_reg_by_CSR(target, CSR_MARCHID);
	reg_marchid_value = ndsv5_get_register_value(reg_marchid);
	if (((reg_marchid_value & 0xff) == 0x45) && (cache_type == DCACHE)) {
		new_tagformat = true;
		shift_bit = line_bits + set_bits;
	} else
		shift_bit = 10;

	/* Index Example Format for CCTL Index Type Operation for 64bit icache
	 * same with 32bit i/dcache(from Roger email) */
	if ((xlen == 64) && (cache_type == ICACHE))
		word_size = 4;
	else
		word_size = xlen / 8;
	word_num = line_size / word_size;

	/* check which way is the dump data for user */
	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	reg_mcctldata = ndsv5_get_reg_by_CSR(target, CSR_MCCTLDATA);
	if ((reg_mcctlbeginaddr == NULL) || (reg_mcctlcommand == NULL) || (reg_mcctldata == NULL))
		return ERROR_FAIL;

	for (way = 0; way < ways; way++) {
		/* Read Tag first */
		ra = (way << way_offset) | idx;

		/* Read TAG command */
		ndsv5_set_register_value(reg_mcctlbeginaddr, ra);
		ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
		tag = ndsv5_get_register_value(reg_mcctldata);
		LOG_DEBUG("tag: %llx", tag);
		if (new_tagformat) {
			mesi = tag & 0x7;
			ces[way].inval = (mesi == 0);
			ces[way].shared = (mesi == 1);
			ces[way].exclusive = (mesi == 3);
			ces[way].modified = (mesi == 7);

			ces[way].lock = (uint8_t)(tag & 0x8) >> 3;
			ces[way].pa = (tag >> 4) << shift_bit ;
		} else {
			ces[way].valid = (uint8_t)((tag & (1ULL << (xlen - 1))) >> (xlen - 1));
			ces[way].lock = (uint8_t)((tag & (1ULL << (xlen - 2))) >> (xlen - 2));
			ces[way].dirty = (uint8_t)((tag & (1ULL << (xlen - 3))) >> (xlen - 3));
			ces[way].pa = (tag & maskTAG) << shift_bit;
		}
		for (i = 0; i < word_num; i++) {
			cache_index = (ra | (i * word_size));
			/* Read DATA command */
			ndsv5_set_register_value(reg_mcctlbeginaddr, cache_index);
			ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
			ces[way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
		}
	}

	/* Print 32bit, 64bit icache, 64bit dcache */
	char *fmt_str = " %8llx";
	char *fmt_str1 = "%08llx ";
	char *fmt_str2 = "%8s %4s %4s %1s %1s %1s";
	char *fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x ";
	if (xlen == 64) {
		if (cache_type == DCACHE) {
			fmt_str = " %16llx";
			fmt_str1 = "%016llx ";
		}
		fmt_str2 = "%16s %4s %4s %1s %1s %1s";
		fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x ";
	}
	if (new_tagformat) {
		fmt_str2 = "%8s %4s %4s %1s %1s %1s %1s %1s";
		fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x %01x %01x ";
		if (xlen == 64) {
			fmt_str2 = "%16s %4s %4s %1s %1s %1s %1s %1s";
			fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x %01x %01x ";
		}
	}

	NDS32_LOG_LF("dump %s\n", cache_type ? "DCACHE" : "ICACHE");
	if (new_tagformat)
		NDS32_LOG_LF(fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		NDS32_LOG_LF(fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		NDS32_LOG_LF(fmt_str, (i * word_size));
	NDS32_LOG_LF("\n");
	for (way = 0; way < ways; way++) {
		if (new_tagformat) {
			NDS32_LOG_LF(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].inval,
				ces[way].shared,
				ces[way].exclusive,
				ces[way].modified,
				ces[way].lock);
		} else {
			NDS32_LOG_LF(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock);
		}

		for (i = 0; i < word_num; i++)
			NDS32_LOG_LF(fmt_str1, ces[way].cacheline[i]);
		NDS32_LOG_LF("\n");
	}

	LOG_INFO("dump %s\n", cache_type ? "DCACHE" : "ICACHE");
	if (new_tagformat)
		LOG_INFO(fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		LOG_INFO(fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		LOG_INFO(fmt_str, (i * word_size));
	LOG_INFO("\n");
	for (way = 0; way < ways; way++) {
		if (new_tagformat) {
			LOG_INFO(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].inval,
				ces[way].shared,
				ces[way].exclusive,
				ces[way].modified,
				ces[way].lock);
		} else {
			LOG_INFO(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock);
		}

		for (i = 0; i < word_num; i++)
			LOG_INFO(fmt_str1, ces[way].cacheline[i]);
		LOG_INFO("\n");
	}

	return ERROR_OK;
}

int ndsv5_enableornot_cache(struct target *target, unsigned int cache_type, const char* enableornot)
{
	LOG_DEBUG("Enable or Disable Cache");

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *cache;

	struct reg *reg_mcache_ctl;
	uint64_t enable_bit = 0x1, new_value;

	reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG("Cannot read mcache_ctl");
		return ERROR_FAIL;
	}

	if (cache_type == DCACHE)
		enable_bit = enable_bit << 1;

	/* enable/disable icache/dcache */
	if (strcmp(enableornot, "enable") == 0)
		new_value = ndsv5_get_register_value(reg_mcache_ctl) | enable_bit;
	else
		new_value = ndsv5_get_register_value(reg_mcache_ctl) & (~enable_bit);

	ndsv5_set_register_value(reg_mcache_ctl, new_value);

	/* check enable/disable icache/dcache PASS or FAIL */
	new_value = ndsv5_get_register_value(reg_mcache_ctl);
	LOG_DEBUG("reg_mcache_ctl : %llx", new_value);

	/* for set icache/dcache->enable=true/false */
	cache = cache_type ? &nds32->memory.dcache : &nds32->memory.icache;

	if ((new_value & enable_bit) != 0) {
		if (strcmp(enableornot, "disable") == 0) {
			LOG_ERROR("Unable disable %s", cache_type ? "DCACHE" : "ICACHE");
			return ERROR_FAIL;
		}
		cache->enable = true;
	} else {
		if (strcmp(enableornot, "enable") == 0) {
			LOG_ERROR("Unable enable %s", cache_type ? "DCACHE" : "ICACHE");
			return ERROR_FAIL;
		}
		cache->enable = false;
	}

	return ERROR_OK;
}

int ndsv5_dcache_wb(struct target *target)
{
	LOG_DEBUG("ndsv5_dcache_writeback");

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR("Cannot read mcache_ctl");
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support writeback");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;

	ndsv5_set_register_value(reg_mcctlcommand, L1D_WB_ALL);

	return ERROR_OK;
}

int ndsv5_dcache_invalidate(struct target *target)
{
	LOG_DEBUG("ndsv5_dcache_invalidate");

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR("Cannot read mcache_ctl");
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support invalidate");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;

	ndsv5_set_register_value(reg_mcctlcommand, L1D_INVAL_ALL);

	return ERROR_OK;
}

int ndsv5_dcache_wb_invalidate(struct target *target)
{
	LOG_DEBUG("ndsv5_dcache_wb_invalidate");

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR("Cannot read mcache_ctl");
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support wb_invalidate");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;

	ndsv5_set_register_value(reg_mcctlcommand, L1D_WBINVAL_ALL);
	return ERROR_OK;
}

int ndsv5_icache_invalidate(struct target *target)
{
	LOG_DEBUG("ndsv5_icache_invalidate");

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR("Cannot read mcache_ctl");
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x1) == 0) {
		LOG_DEBUG("Instruction cache disabled, NOT support invalidate");
		return ERROR_OK;
	}

	uint64_t set, way, way_offset, index;
	struct nds32_v5_cache *cache = &nds32->memory.icache;
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand;

	way_offset = cache->log2_set + cache->log2_line_size;
	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if ((reg_mcctlbeginaddr == NULL) || (reg_mcctlcommand == NULL))
		return ERROR_FAIL;

	for (set = 0; set < cache->set; set++) {
		for (way = 0; way < cache->way; way++) {
			index = ((way << way_offset) | (set << cache->log2_line_size));
			ndsv5_set_register_value(reg_mcctlbeginaddr, index);
			ndsv5_set_register_value(reg_mcctlcommand, L1I_IX_INVAL);
		}
	}

	return ERROR_OK;
}

const char *ndsv5_get_gdb_arch(struct target *target)
{
	if (riscv_xlen(target) == 32)
		return "riscv:rv32";
	else
		return "riscv:rv64";
}


/********************************************************************/
/* NDSv5 program functions                                          */
/********************************************************************/
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

	/* store high part value to destination register (i.e., d) */
	c >>= 32;
	sign_ext = (c & 0x800) ? (-1 - 0xFFF) : 0;
	if (riscv_program_lui(p, d, (c - sign_ext) >> 12) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_addi(p, d, d, c & 0xFFF) != ERROR_OK)
		return ERROR_FAIL;

	/* shift high value to high part */
	if (riscv_program_slli(p, d, d, 32) != ERROR_OK)
		return ERROR_FAIL;

	/* OR high and low values to form 64-bits one */
	if (riscv_program_or(p, d, d, t) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

int riscv_program_vsetvl(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno rs2)
{
	/*
	vsetvl  rd, rs1, rs2    # rd = new vl, rs1 = AVL, rs2 = new vtype value
	# if rs1 = x0, then use maximum vector length
	31 30    25 24  20 19 15 14 12 11   7 6     0
	1 | 000000 | rs2  | rs1 | 111 |  rd  |1010111|         vsetvl
	1     6       5      5     3     5       7
	*/
	riscv_insn_t opcode = 0;

	opcode = 0x80007057;
	opcode |= ((rd << 7) | (rs2 << 20));
	return riscv_program_insert(p, opcode);
}

int riscv_program_vsetvli(struct riscv_program *p, enum gdb_regno rd, uint32_t SEW)
{
	/*
	31 30         20 19  15 14   12 11    7 6        0
	0 | zimm[10:0] |  rs1  |  111  |  rd   | 1010111 |    =>  vsetvli
	00c07557		vsetvli	a0,zero,e64,m1,d1
	00807557		vsetvli	a0,zero,e32,m1,d1
	00407557		vsetvli	a0,zero,e16,m1,d1
	00007557		vsetvli	a0,zero,e8,m1,d1
	*/
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
	else
		return ERROR_FAIL;

	vtypei = ((vtypei_SEW << 2) << 20);
	LOG_DEBUG("vtypei: 0x%x", vtypei);
	opcode = 0x7057;
	opcode |= ((rd << 7) | vtypei);
	return riscv_program_insert(p, opcode);
}

int riscv_program_vmv_x_s(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2)
{
	/*
	31     26  25   24      20 19      15 14   12 11      7 6     0
	funct6   | vm  |   vs2    |    rs1   | 0 1 0 |  vd/rd  |1010111| OP-V (OPMVV)
	010000
	*/
	riscv_insn_t opcode;
	enum gdb_regno rs1 = 0;
	opcode = 0x42002057;
	opcode |= ((rd << 7) | (rs1 << 15) | ((vs2 - GDB_REGNO_V0) << 20));
	return riscv_program_insert(p, opcode);
}

int riscv_program_vslide1down_vx(struct riscv_program *p, enum gdb_regno rd, enum gdb_regno vs2, enum gdb_regno rs1)
{
	/*
	31     26  25  24      20 19      15 14   12 11      7 6     0
	funct6   | vm  |   vs2   |   rs1    | 1 1 0 |  vd/rd  |1010111| OP-V (OPMVX)
	001111     1      0 0000    0111 0    110        0       57      3e076057: vslide1down.vx	v0,v0,a4
	*/
	riscv_insn_t opcode;

	opcode = 0x3e006057;
	opcode |= (((rd - GDB_REGNO_V0) << 7) | (rs1 << 15) | ((vs2 - GDB_REGNO_V0) << 20));
	return riscv_program_insert(p, opcode);
}

int ndsv5_openocd_poll_one_hart(struct target *target, int hartid)
{
	LOG_DEBUG("polling hart : %d", hartid);
	int triggered_hart = hartid;
	if (riscv_poll_hart(target, triggered_hart) == 0)
		return ERROR_OK;

	LOG_DEBUG("  hart %d halted", triggered_hart);

	target->state = TARGET_HALTED;
	switch (riscv_halt_reason(target, triggered_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case RISCV_HALT_TRIGGER:
		target->debug_reason = DBG_REASON_WATCHPOINT;
		break;
	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;
	case RISCV_HALT_UNKNOWN:
		target->debug_reason = DBG_REASON_UNDEFINED;
		break;
	case RISCV_HALT_ERROR:
		return ERROR_FAIL;
	}

	target->state = TARGET_HALTED;
	ndsv5_triggered_hart = triggered_hart;

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return ERROR_OK;
}

int ndsv5_openocd_halt_one_hart(struct target *target, int hartid)
{
	LOG_DEBUG("halting hart : %d", hartid);

	///WARNING: hartid not use~~
	RISCV_INFO(r);
	int out = r-> halt_go(target);
	if (out != ERROR_OK) {
		LOG_ERROR("Unable to halt hart : %d", hartid);
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return out;
}

int ndsv5_openocd_resume_one_hart(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		int hartid
) {
	LOG_DEBUG("resuming hart : %d", hartid);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	RISCV_INFO(r);

	/// WARNING: hartid no use~~
	int out = r->resume_go(target);
	riscv_invalidate_register_cache(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to resume hart : %d", hartid);
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return out;
}

static int ndsv5_idlm_status_update(struct target *target)
{
	uint64_t value_micm_cfg=0, value_mdcm_cfg=0, reg_mmsc_cfg_value=0;
	uint64_t value_milmb=0, value_mdlmb=0;

	if (ndsv5_check_idlm_capability_before == 0) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			// use quick mode to read CSR while target_not_halted
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MICM_CFG + GDB_REGNO_CSR0, &value_micm_cfg);
			ndsv5_get_csr_reg_quick_access(target, CSR_MDCM_CFG + GDB_REGNO_CSR0, &value_mdcm_cfg);
			ndsv5_get_csr_reg_quick_access(target, CSR_MMSC_CFG + GDB_REGNO_CSR0, &reg_mmsc_cfg_value);
		} else {
			struct reg *reg_micm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
			if (reg_micm_cfg != NULL)
				value_micm_cfg = ndsv5_get_register_value(reg_micm_cfg);
			struct reg *reg_mdcm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
			if (reg_mdcm_cfg != NULL)
				value_mdcm_cfg = ndsv5_get_register_value(reg_mdcm_cfg);
			struct reg *reg_mmsc_cfg = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
			if (reg_mmsc_cfg != NULL)
				reg_mmsc_cfg_value = ndsv5_get_register_value(reg_mmsc_cfg);
		}
		if ((value_micm_cfg & 0x7000) == 0)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MILMB].exist = false;
		else
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MILMB].exist = true;
		if ((value_mdcm_cfg & 0x7000) == 0)
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MDLMB].exist = false;
		else
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MDLMB].exist = true;

		NDS_INFO("value_micm_cfg: 0x%" PRIx64 " value_mdcm_cfg: 0x%" PRIx64, value_micm_cfg, value_mdcm_cfg);
		NDS_INFO("reg_mmsc_cfg_value: 0x%" PRIx64, reg_mmsc_cfg_value);
		if ((reg_mmsc_cfg_value & 0x4000) == 0) {
			NDS_INFO("local memory slave port is not supported");
			ndsv5_local_memory_slave_port = 0;
		} else {
			ndsv5_local_memory_slave_port = 1;
		}
		ndsv5_check_idlm_capability_before = 1;
	}
	// checking idlm enable while target_is_halted
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MILMB].exist == true) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MILMB + GDB_REGNO_CSR0, &value_milmb);
		} else {
			struct reg *reg_milmb = ndsv5_get_reg_by_CSR(target, CSR_MILMB);
			if (reg_milmb != NULL)
				value_milmb = ndsv5_get_register_value(reg_milmb);
		}
		if (value_milmb & 0x1) {
			ndsv5_ilm_bpa = value_milmb & ~0x3ff;
			ndsv5_ilm_ena = 1;
		} else {
			ndsv5_ilm_ena = 0;
		}
	}
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_MDLMB].exist == true) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MDLMB + GDB_REGNO_CSR0, &value_mdlmb);
		} else {
			struct reg *reg_mdlmb = ndsv5_get_reg_by_CSR(target, CSR_MDLMB);
			if (reg_mdlmb != NULL)
				value_mdlmb = ndsv5_get_register_value(reg_mdlmb);
		}
		if (value_mdlmb & 0x1) {
			ndsv5_dlm_bpa = value_mdlmb & ~0x3ff;
			ndsv5_dlm_ena = 1;
		} else {
			ndsv5_dlm_ena = 0;
		}
	}
	return ERROR_OK;
}

int ndsv5_lm_slvp_support(struct target *target, target_addr_t address, uint32_t csr_id_lmb)
{
	uint64_t checking_bpa=0, checking_lmsz=0;

	LOG_DEBUG("ndsv5_check_idlm_capability_before: %d", ndsv5_check_idlm_capability_before);
	if (ndsv5_check_idlm_capability_before) {
		if (ndsv5_local_memory_slave_port == 0) {
			//LOG_ERROR("<-- local memory slave port is not supported -->");
			return ERROR_FAIL;
		}
		if ((csr_id_lmb == CSR_MILMB) && (ndsv5_ilm_lmsz == 0))
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		if ((csr_id_lmb == CSR_MDLMB) && (ndsv5_dlm_lmsz == 0))
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	ndsv5_idlm_status_update(target);
	if (csr_id_lmb == CSR_MILMB) {
		if (ndsv5_ilm_ena == 0)
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		checking_bpa = ndsv5_ilm_bpa;
		checking_lmsz = ndsv5_ilm_lmsz;
	} else { // csr_id_lmb == CSR_MDLMB
		if (ndsv5_dlm_ena == 0)
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		checking_bpa = ndsv5_dlm_bpa;
		checking_lmsz = ndsv5_dlm_lmsz;
	}
	if ((address >= checking_bpa) && (address < (checking_bpa + checking_lmsz))) {
		return ERROR_OK;
	}
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

static char encoding_table[] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
	'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3',
	'4', '5', '6', '7', '8', '9', '+', '/'
};

char *ndsv5_base64_decode(const char *data, int input_length, int *output_length)
{
	char *decoding_table = calloc(256, 1);
	for (int i = 0; i < 64; i++)
		decoding_table[(unsigned char) encoding_table[i]] = i;

	if (input_length % 4 != 0)
		return NULL;

	*output_length = input_length / 4 * 3;
	if (data[input_length - 1] == '=')
		(*output_length)--;
	if (data[input_length - 2] == '=')
		(*output_length)--;

	char *decoded_data = malloc(*output_length);
	if (decoded_data == NULL)
		return NULL;

	for (int i = 0, j = 0; i < input_length;) {

		uint32_t sextet_a = data[i] == '=' ? 0 & i++ : decoding_table[(unsigned char) data[i++]];
		uint32_t sextet_b = data[i] == '=' ? 0 & i++ : decoding_table[(unsigned char) data[i++]];
		uint32_t sextet_c = data[i] == '=' ? 0 & i++ : decoding_table[(unsigned char) data[i++]];
		uint32_t sextet_d = data[i] == '=' ? 0 & i++ : decoding_table[(unsigned char) data[i++]];

		uint32_t triple = (sextet_a << 3 * 6) + (sextet_b << 2 * 6) + (sextet_c << 1 * 6) + (sextet_d << 0 * 6);

		if (j < *output_length)
			decoded_data[j++] = (triple >> 2 * 8) & 0xFF;
		if (j < *output_length)
			decoded_data[j++] = (triple >> 1 * 8) & 0xFF;
		if (j < *output_length)
			decoded_data[j++] = (triple >> 0 * 8) & 0xFF;
	}
	free(decoding_table);
	return decoded_data;
}

