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
#include "nds32_cmd.h"
#include "nds32_aice.h"
#include "nds32_v3.h"
#include "nds32_v3_common.h"
#include "nds32_v3.h"
#include "nds32_log.h"

extern uint32_t nds32_dis_global_stop_warning;
extern uint32_t aice_usb_pack_command;
extern unsigned int aice_do_diagnosis;
extern uint32_t nds_ftdi_devices;
extern uint32_t scan_field_max;
extern uint32_t force_edm_v3;
extern int aice_execute_custom_script(struct target *target, const char *script);
extern char *custom_initial_script;
extern char *custom_restart_script;

extern int nds32_v2_register_mapping(struct nds32 *nds32, int reg_no);
extern int nds32_v2_get_debug_reason(struct nds32 *nds32, uint32_t *reason);
extern int nds32_v2_debug_entry(struct nds32 *nds32, bool enable_watchpoint);
extern int nds32_v2_leave_debug_state(struct nds32 *nds32, bool enable_watchpoint);
extern int nds32_v2_get_exception_address(struct nds32 *nds32,
		uint32_t *address, uint32_t reason);

extern int nds32_v2_check_interrupt_stack(struct nds32 *nds32);
extern int nds32_v2_restore_interrupt_stack(struct nds32 *nds32);
extern int nds_ftdi_get_info(void);
//#define BPWP_ID_MAX  32
//#define BPWP_ID_ON   0x01
//#define BPWP_ID_WP   0x80
//#define BPWP_ID_BP   0x40

//uint32_t nds32_wp_hb_id[BPWP_ID_MAX];

static int nds32_v3_reset_hb_id(struct target *target)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	int32_t i;
	struct nds32 *nds32 = target_to_nds32(target);

	for (i = 0; i<BPWP_ID_MAX; i++) {
		nds32->nds32_wp_hb_id[i] = 0x00;
	}

	for (i = 0; i<nds32_v3->n_hbr; i++) {
		nds32->nds32_wp_hb_id[i] |= BPWP_ID_BP;
	}

	for (i = 0; i<nds32_v3->n_hwp; i++) {
		nds32->nds32_wp_hb_id[i] |= BPWP_ID_WP;
	}
	for (i = 0; i<BPWP_ID_MAX; i++)
		LOG_DEBUG("nds32_wp_hb_id[]=%x", nds32->nds32_wp_hb_id[i]);
	return ERROR_OK;
}

static int nds32_v3_get_hb_id(struct target *target, uint32_t *phb_id)
{
	struct nds32 *nds32 = target_to_nds32(target);
	int32_t i;
	for (i = (BPWP_ID_MAX - 1); i >= 0; i--) {
		if ( (nds32->nds32_wp_hb_id[i] & BPWP_ID_BP) &&
			((nds32->nds32_wp_hb_id[i] & BPWP_ID_ON) == 0) ) {
			nds32->nds32_wp_hb_id[i] |= BPWP_ID_ON;
			*phb_id = (uint32_t)i;
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

static int nds32_v3_free_hb_id(struct target *target, uint32_t hb_id)
{
	struct nds32 *nds32 = target_to_nds32(target);
	nds32->nds32_wp_hb_id[hb_id] &= ~BPWP_ID_ON;
	return ERROR_OK;
}

static int nds32_v3_get_wp_id(struct target *target, uint32_t *pwp_id)
{
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t i;
	for (i = 0; i<BPWP_ID_MAX; i++) {
		if ( (nds32->nds32_wp_hb_id[i] & BPWP_ID_WP) &&
			((nds32->nds32_wp_hb_id[i] & BPWP_ID_ON) == 0) ) {
			nds32->nds32_wp_hb_id[i] |= BPWP_ID_ON;
			*pwp_id = (uint32_t)i;
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

static int nds32_v3_free_wp_id(struct target *target, uint32_t wp_id)
{
	struct nds32 *nds32 = target_to_nds32(target);
	nds32->nds32_wp_hb_id[wp_id] &= ~BPWP_ID_ON;
	return ERROR_OK;
}

static int nds32_v3_activate_hardware_breakpoint(struct target *target)
{
	//struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	struct breakpoint *bp;
	unsigned brp_num = 0; // do NOT use simple breakpoint
	int int_mask;

	/* Trigger Event Control register 0 */
	uint32_t edm_tecr0 = 0;
	aice_read_debug_reg(target, NDS_EDM_SR_TECR0, &edm_tecr0);

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT) {
			/* already set at nds32_v3_add_breakpoint() */
			continue;
		} else if (bp->type == BKPT_HARD) {
			brp_num = bp->unique_id;
			/* non-simple breakpoint */
			if (bp->length & BP_WP_NON_SIMPLE) {
				int_mask = (bp->length & BP_WP_LENGTH_MASK);
				int_mask -= 1;
			}
			/* simple breakpoint */
			else {
				int_mask = 0;
			}
			if (aice_usb_pack_command == 1) {
				aice_set_command_mode(AICE_COMMAND_MODE_PACK);
			}
			/* set address */
			aice_write_debug_reg(target, NDS_EDM_SR_BPA0 + brp_num, bp->address);
			/* set mask */
			aice_write_debug_reg(target, NDS_EDM_SR_BPAM0 + brp_num, int_mask);
			/* set value */
			aice_write_debug_reg(target, NDS_EDM_SR_BPV0 + brp_num, 0);

			/* Trigger Event Control register 0 */
			if (bp->length & BP_WP_TRIGGER_ON) {
				edm_tecr0 |= (0x01 << brp_num);
			} else {
				edm_tecr0 &= ~(0x01 << brp_num);
			}
			aice_write_debug_reg(target, NDS_EDM_SR_TECR0, edm_tecr0);

			/* BPCn.P (Physical address bit) is used only when WP==1. */
			//if ((nds32_v3->nds32.memory.address_translation) ||
			//	(bp->length & BP_WP_FORCE_VA_ON))
				/* enable breakpoint (virtual address) */
				aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + brp_num, 0x2);
			//else
				/* enable breakpoint (physical address) */
			//	aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + brp_num, 0xA);

			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			LOG_DEBUG("Add hardware BP %d at 0x%08" TARGET_PRIxADDR ", length:%d, mask:0x%x", brp_num,
					bp->address, bp->length, int_mask);
		} else {
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int nds32_v3_deactivate_hardware_breakpoint(struct target *target)
{
	struct breakpoint *bp;
	unsigned brp_num = 0; // do NOT use simple breakpoint

	if (aice_usb_pack_command == 1) {
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
	}
	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT)
			continue;
		else if (bp->type == BKPT_HARD) {
			brp_num = bp->unique_id;
			/* disable breakpoint */
			aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + brp_num, 0x0);
		}
		else
			return ERROR_FAIL;

		LOG_DEBUG("Remove hardware BP %" PRId32 " at %08" TARGET_PRIxADDR, brp_num,
				bp->address);
	}
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
	return ERROR_OK;
}

static int nds32_v3_activate_hardware_watchpoint(struct target *target)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	struct watchpoint *wp;
	int32_t  wp_num = 0;
	uint32_t wp_config = 0, wp_config_BE = 0, wp_length;
	bool ld_stop = false, st_stop = false;

	if (nds32_v3->nds32.global_stop)
		ld_stop = st_stop = false;

	/* Trigger Event Control register 0 */
	uint32_t edm_tecr0 = 0;
	aice_read_debug_reg(target, NDS_EDM_SR_TECR0, &edm_tecr0);

	for (wp = target->watchpoints; wp; wp = wp->next) {

		wp_length = (wp->length & BP_WP_LENGTH_MASK);
		if (wp->length & BP_WP_DATA_COMPARE) {
			/* Breakpoint Data Value mode: compared with the incoming load/store data value */
			/* Data value comparison bits, BPCn[5-8] BEx */
			if (wp_length == 1)
				wp_config_BE = (0x01 << 5);
			else if (wp_length == 2)
				wp_config_BE = (0x03 << 5);
			else
				wp_config_BE = (0x0f << 5);
		}
		/* for bug-14007, if wp->length is NOT 2^n, force power of 2 */
		uint32_t i, length_power_2;
		for (i = 0; i < 24; i++) {
			// 24 => #define BP_WP_LENGTH_MASK    (~0xFF000000)
			length_power_2 = 0x01 << i;
			if (wp_length <= length_power_2) {
				LOG_DEBUG("wp_length=0x%x, length_power_2=0x%x", wp_length, length_power_2);
				wp_length = length_power_2;
				break;
			}
		}

		wp->mask = wp_length - 1;
		if ((wp->address % wp_length) != 0) {
			uint32_t mask_bits;
			uint32_t wp_addr_start = wp->address;
			uint32_t wp_addr_end = (wp_addr_start + wp_length);
			for (i = 1; i <= 24; i++) {
				mask_bits = (0x01 << i) - 1;
				if ((wp_addr_start & ~mask_bits) == (wp_addr_end & ~mask_bits))
					break;
			}
			wp->mask = (0x01 << i) - 1;
		}
		else if ((wp_length == 1) || (wp_length == 2)) {
			/* sw-workaround for bug-13852, unaligned address byte watch */
			if ((wp->address % 4) != 0)
				wp->mask = 3;
		}

		//uint32_t wp_address = wp->address - (wp->address % wp_length);
		uint32_t wp_address = (wp->address & ~wp->mask);
		LOG_DEBUG("wp->address=0x%" TARGET_PRIxADDR ", wp->length=0x%x",
			wp->address, wp->length);
		LOG_DEBUG("wp_address=0x%x, wp_length=0x%x, wp->mask=0x%x",
			wp_address, wp_length, wp->mask);

		wp_num = wp->unique_id;
		if (wp_num < nds32_v3->used_n_wp) {

			if (wp->rw == WPT_READ)
				wp_config = 0x3;
			else if (wp->rw == WPT_WRITE)
				wp_config = 0x5;
			else if (wp->rw == WPT_ACCESS)
				wp_config = 0x7;

			/* watch VA as default, bug-13943 */
			struct nds32 *nds32 = target_to_nds32(target);
			if (nds32->memory.va_to_pa_off)
				wp_config |= 0x8;
			else
				wp_config &= ~0x8;

			/* set Data value comparison bits */
			wp_config |= wp_config_BE;

			if (aice_usb_pack_command == 1) {
				aice_set_command_mode(AICE_COMMAND_MODE_PACK);
			}
			/* set address */
			aice_write_debug_reg(target, NDS_EDM_SR_BPA0 + wp_num, wp_address);
			/* set mask */
			aice_write_debug_reg(target, NDS_EDM_SR_BPAM0 + wp_num, wp->mask);
			/* enable watchpoint */
			aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + wp_num, wp_config);
			/* set value */
			aice_write_debug_reg(target, NDS_EDM_SR_BPV0 + wp_num, wp->value);

			/* Trigger Event Control register 0 */
			if (wp->length & BP_WP_TRIGGER_ON) {
				edm_tecr0 |= (0x01 << wp_num);
			} else {
				edm_tecr0 &= ~(0x01 << wp_num);
			}
			aice_write_debug_reg(target, NDS_EDM_SR_TECR0, edm_tecr0);
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

			LOG_DEBUG("Add hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR " mask %08" PRIx32,
					wp_num, wp->address, wp->mask);
		} else if (nds32_v3->nds32.global_stop) {
			if (wp->rw == WPT_READ)
				ld_stop = true;
			else if (wp->rw == WPT_WRITE)
				st_stop = true;
			else if (wp->rw == WPT_ACCESS)
				ld_stop = st_stop = true;
		}
	}

	if (nds32_v3->nds32.global_stop) {
		uint32_t edm_ctl;
		aice_read_register(target, DR42, &edm_ctl);
		if (ld_stop)
			edm_ctl |= NDS_EDMCTL_LDSTOP;
		if (st_stop)
			edm_ctl |= NDS_EDMCTL_STSTOP;
		aice_write_register(target, DR42, edm_ctl);
	}

	return ERROR_OK;
}

static int nds32_v3_deactivate_hardware_watchpoint(struct target *target)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	struct watchpoint *wp;
	int32_t wp_num = 0;
	bool clean_global_stop = false;

	if (aice_usb_pack_command == 1) {
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
	}
	for (wp = target->watchpoints; wp; wp = wp->next) {
		wp_num = wp->unique_id;
		if (wp_num < nds32_v3->used_n_wp) {
			/* disable watchpoint */
			aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + wp_num, 0x0);

			LOG_DEBUG("Remove hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR
					" mask %08" PRIx32, wp_num,
					wp->address, wp->mask);
		} else if (nds32_v3->nds32.global_stop) {
			clean_global_stop = true;
		}
	}
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

	if (clean_global_stop) {
		uint32_t edm_ctl;
		aice_read_register(target, DR42, &edm_ctl);
		edm_ctl = edm_ctl & ~(NDS_EDMCTL_LDSTOP | NDS_EDMCTL_STSTOP);
		aice_write_register(target, DR42, edm_ctl);
	}

	return ERROR_OK;
}

static int nds32_v3_check_interrupt_stack(struct nds32 *nds32)
{
	uint32_t val_ir0;
	uint32_t value;
	struct nds32_misc_config *misc_config = &(nds32->misc_config);

	/* Save interrupt level */
	nds32_get_mapped_reg(nds32, IR0, &val_ir0);
	nds32->current_interrupt_level = (val_ir0 >> 1) & 0x3;

	if (nds32_reach_max_interrupt_level(nds32))
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_INTLVL3,
				nds32->current_interrupt_level);

	/* backup $ir4 & $ir6 to avoid suppressed exception overwrite */
	if (!misc_config->mcu) {
		nds32_get_mapped_reg(nds32, IR4, &value);
	}
	else {
		LOG_DEBUG("nds32_v3m_check_interrupt_stack");
	}
	nds32_get_mapped_reg(nds32, IR6, &value);

	return ERROR_OK;
}

static int nds32_v3_restore_interrupt_stack(struct nds32 *nds32)
{
	uint32_t value_ir0 = 0;
	uint32_t value_ir4 = 0;
	uint32_t value_ir6 = 0;
	struct nds32_misc_config *misc_config = &(nds32->misc_config);

	nds32_get_mapped_reg(nds32, IR0, &value_ir0);
	nds32_get_mapped_reg(nds32, IR4, &value_ir4);
	nds32_get_mapped_reg(nds32, IR6, &value_ir6);

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;
	}
	/* get backup value from cache */
	/* then set back to make the register dirty */
	nds32_set_mapped_reg(nds32, IR0, value_ir0);

	if (!misc_config->mcu) {
		nds32_set_mapped_reg(nds32, IR4, value_ir4);
	}
	nds32_set_mapped_reg(nds32, IR6, value_ir6);
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

	return ERROR_OK;
}

static int nds32_v3_deassert_reset(struct target *target)
{
	int retval;
	bool switch_to_v3_stack = false;
	uint32_t value_edm_ctl;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_misc_config *misc_config = &(nds32->misc_config);

	if (misc_config->mcu) {
		LOG_DEBUG("nds32_v3m_deassert_reset");
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

	aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &value_edm_ctl);
	if ((force_edm_v3) && ((value_edm_ctl & NDS_EDMCTL_EDM_MODE) == 0)) { /* reset to V2 EDM mode */
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, value_edm_ctl | NDS_EDMCTL_EDM_MODE);
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &value_edm_ctl);
		if (value_edm_ctl & NDS_EDMCTL_EDM_MODE)
			switch_to_v3_stack = true;
	} else
		switch_to_v3_stack = false;

	CHECK_RETVAL(nds32_poll(target));

	if (target->state != TARGET_HALTED) {
		/* reset only */
		LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(target));
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;

	} else {
		/* reset-halt */
		//struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
		//struct nds32 *nds32 = &(nds32_v3->nds32);
		uint32_t value;
		uint32_t interrupt_level;

		if (switch_to_v3_stack == true) {
			/* PSW.INTL-- */
			nds32_get_mapped_reg(nds32, IR0, &value);
			interrupt_level = (value >> 1) & 0x3;
			interrupt_level--;
			value &= ~(0x6);
			value |= (interrupt_level << 1);
			value |= 0x400;  /* set PSW.DEX */
			nds32_set_mapped_reg(nds32, IR0, value);

			/* copy IPC to OIPC */
			if ((interrupt_level + 1) < nds32->max_interrupt_level) {
				nds32_get_mapped_reg(nds32, IR9, &value);
				nds32_set_mapped_reg(nds32, IR11, value);
			}
		}
	}

	return ERROR_OK;
}

static int nds32_v3_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	struct nds32 *nds32 = &(nds32_v3->nds32);
	int result;

	if (breakpoint->type == BKPT_HARD) {
		/* non-simple breakpoint */
		if (breakpoint->length & BP_WP_NON_SIMPLE) {
			/* check hardware resource */
			//if ((nds32_v3->next_hwp_index >= nds32_v3->n_hwp) ||
			//	(nds32_v3->next_hwp_index > nds32_v3->next_hbr_index)) {
			if (nds32_v3_get_wp_id(target, &breakpoint->unique_id) != ERROR_OK) {
				/* No hardware resource */
				NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT, nds32_v3->n_hwp);
				NDS32_LOG(NDS32_ERRMSG_TARGET_HW_WATCH,
					nds32_v3->used_n_wp);
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}

			/* update next place to put hardware watchpoint */
			nds32_v3->next_hwp_index++;
			nds32_v3->used_n_wp++;
		}
		/* simple breakpoint */
		else {
			/* check hardware resource */
			//if (nds32_v3->next_hbr_index < nds32_v3->next_hwp_index) {
			if (nds32_v3_get_hb_id(target, &breakpoint->unique_id) != ERROR_OK) {
				NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT,
						nds32_v3->n_hbr);
				NDS32_LOG(NDS32_ERRMSG_TARGET_HW_BREAK_WATCH,
						nds32_v3->n_hbr - nds32_v3->next_hbr_index - 1,
						nds32_v3->used_n_wp);
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}
			/* update next place to put hardware breakpoint */
			nds32_v3->next_hbr_index--;
		}

		/* hardware breakpoint insertion occurs before 'continue' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
#if 0
		result = nds32_add_software_breakpoint(target, breakpoint);
		if (ERROR_OK != result) {
			/* auto convert to hardware breakpoint if failed */
			if (nds32->auto_convert_hw_bp) {
				/* convert to hardware breakpoint */
				breakpoint->type = BKPT_HARD;

				return nds32_v3_add_breakpoint(target, breakpoint);
			}
		}

		return result;
#else
		/* auto convert to hardware breakpoint if failed */
		if (nds32->auto_convert_hw_bp) {
			result = nds32_add_software_breakpoint(target, breakpoint);
			if (ERROR_OK != result) {
				/* convert to hardware breakpoint */
				breakpoint->type = BKPT_HARD;
				return nds32_v3_add_breakpoint(target, breakpoint);
			}
			else {
				/* restore ori-instruction */
				return nds32_remove_software_breakpoint(target, breakpoint);
			}
		}
#endif
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v3_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);

	if (breakpoint->type == BKPT_HARD) {
		/* non-simple breakpoint */
		if (breakpoint->length & BP_WP_NON_SIMPLE) {
			/* update next place to put hardware watchpoint */
			nds32_v3->next_hwp_index--;
			nds32_v3->used_n_wp--;
			nds32_v3_free_wp_id(target, breakpoint->unique_id);
		}
		/* simple breakpoint */
		else {
			if (nds32_v3->next_hbr_index >= nds32_v3->n_hbr - 1)
				return ERROR_FAIL;
			/* update next place to put hardware breakpoint */
			nds32_v3->next_hbr_index++;
			nds32_v3_free_hb_id(target, breakpoint->unique_id);
		}
		/* hardware breakpoint removal occurs after 'halted' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
		//return nds32_remove_software_breakpoint(target, breakpoint);
		return ERROR_OK;
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v3_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);

	/* check hardware resource */
#if 0
	if (nds32_v3->next_hwp_index >= nds32_v3->n_hwp) {
		/* No hardware resource */
		if (nds32_v3->nds32.global_stop) {
			if (nds32_dis_global_stop_warning == 0) {
				NDS32_LOG(NDS32_ERRMSG_TARGET_GLOBAL_STOP);
			}
			return ERROR_OK;
		}

		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT, nds32_v3->n_hwp);
		NDS32_LOG(NDS32_ERRMSG_TARGET_HW_WATCH,
				nds32_v3->used_n_wp);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
#endif
	//if (nds32_v3->next_hwp_index > nds32_v3->next_hbr_index) {
	uint32_t wp_id;
	if (nds32_v3_get_wp_id(target, &wp_id) != ERROR_OK) {
		/* No hardware resource */
		if (nds32_v3->nds32.global_stop) {
			NDS32_LOG(NDS32_ERRMSG_TARGET_GLOBAL_STOP);
			return ERROR_OK;
		}

		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT,
				nds32_v3->n_hbr);
		NDS32_LOG(NDS32_ERRMSG_TARGET_HW_BREAK_WATCH,
				nds32_v3->n_hbr - nds32_v3->next_hbr_index - 1,
				nds32_v3->used_n_wp);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	watchpoint->unique_id = (int)wp_id;
	/* update next place to put hardware watchpoint */
	nds32_v3->next_hwp_index++;
	nds32_v3->used_n_wp++;

	return ERROR_OK;
}

static int nds32_v3_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);

	if (nds32_v3->next_hwp_index <= 0) {
		if (nds32_v3->nds32.global_stop)
			return ERROR_OK;

		return ERROR_FAIL;
	}

	/* update next place to put hardware watchpoint */
	nds32_v3->next_hwp_index--;
	nds32_v3->used_n_wp--;
	nds32_v3_free_wp_id(target, watchpoint->unique_id);
	return ERROR_OK;
}


struct nds32_v3_common_callback nds32_v3_common_callback = {
	.check_interrupt_stack = nds32_v3_check_interrupt_stack,
	.restore_interrupt_stack = nds32_v3_restore_interrupt_stack,
	.activate_hardware_breakpoint = nds32_v3_activate_hardware_breakpoint,
	.activate_hardware_watchpoint = nds32_v3_activate_hardware_watchpoint,
	.deactivate_hardware_breakpoint = nds32_v3_deactivate_hardware_breakpoint,
	.deactivate_hardware_watchpoint = nds32_v3_deactivate_hardware_watchpoint,
};
extern struct nds32_v3_common_callback nds32_v2_common_callback;

static int nds32_v3_target_create(struct target *target, Jim_Interp *interp)
{
	struct nds32_v3_common *nds32_v3;

	LOG_DEBUG("nds32_v3_target_create");
	nds32_v3 = calloc(1, sizeof(*nds32_v3));
	if (!nds32_v3)
		return ERROR_FAIL;

	nds32_v3_common_register_callback(&nds32_v3_common_callback);
	nds32_v3_target_create_common(target, &(nds32_v3->nds32));

	return ERROR_OK;
}

int nds32_redirect_edm_v2(struct target *target)
{
	uint32_t coreid = target->tap->abs_chain_position;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t edm_cfg_value=0;
	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, (uint32_t*)&edm_cfg_value);
	//LOG_DEBUG("edm_cfg_value:%x", edm_cfg_value);

	nds32->edm.version = (edm_cfg_value >> 16) & 0xFFFF;
	LOG_DEBUG("nds32->edm.version 0x%x", nds32->edm.version);
	if ((nds32->edm.version & 0x1000) == 0) {
			/* edm v2 */
			nds32->register_map = nds32_v2_register_mapping;
			nds32->get_debug_reason = nds32_v2_get_debug_reason;
			nds32->enter_debug_state = nds32_v2_debug_entry;
			nds32->leave_debug_state = nds32_v2_leave_debug_state;
			nds32->get_watched_address = nds32_v2_get_exception_address;
			//nds32_v3_common_register_callback(&nds32_v2_common_callback);
			nds32_v3_common_callback.check_interrupt_stack = nds32_v2_check_interrupt_stack;
			nds32_v3_common_callback.restore_interrupt_stack = nds32_v2_restore_interrupt_stack;
			LOG_DEBUG("redirect_edm_v2 !!");
	}
	return ERROR_OK;
}

int nds32_update_edm_config(struct nds32 *nds32)
{
	struct target *target = nds32->target;
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);

	if ((nds32->edm.version & 0x1000) == 0) {
			/* edm v2 */
			nds32->register_map = nds32_v2_register_mapping;
			nds32->get_debug_reason = nds32_v2_get_debug_reason;
			nds32->enter_debug_state = nds32_v2_debug_entry;
			nds32->leave_debug_state = nds32_v2_leave_debug_state;
			nds32->get_watched_address = nds32_v2_get_exception_address;
			//nds32_v3_common_register_callback(&nds32_v2_common_callback);
			nds32_v3_common_callback.check_interrupt_stack = nds32_v2_check_interrupt_stack;
			nds32_v3_common_callback.restore_interrupt_stack = nds32_v2_restore_interrupt_stack;
	}
	uint32_t edm_cfg;
	aice_read_debug_reg(target, NDS_EDM_SR_EDM_CFG, &edm_cfg);

	/* get the number of hardware breakpoints */
	nds32_v3->n_hbr = (edm_cfg & 0x7) + 1;
	nds32_v3->used_n_wp = 0;

	/* get the number of hardware watchpoints */
	/* If the WP field is hardwired to zero, it means this is a
	 * simple breakpoint.  Otherwise, if the WP field is writable
	 * then it means this is a regular watchpoints. */
	nds32_v3->n_hwp = 0;
	for (int32_t i = 0 ; i < nds32_v3->n_hbr ; i++) {
		/** check the hardware breakpoint is simple or not */
		uint32_t tmp_value;
		aice_write_debug_reg(target, NDS_EDM_SR_BPC0 + i, 0x1);
		aice_read_debug_reg(target, NDS_EDM_SR_BPC0 + i, &tmp_value);

		if (tmp_value)
			nds32_v3->n_hwp++;
	}
	/* hardware breakpoint is inserted from high index to low index */
	nds32_v3->next_hbr_index = nds32_v3->n_hbr - 1;
	/* hardware watchpoint is inserted from low index to high index */
	nds32_v3->next_hwp_index = 0;

	LOG_INFO("%s: total hardware breakpoint %d (simple breakpoint %d)",
			target_name(target), nds32_v3->n_hbr, nds32_v3->n_hbr - nds32_v3->n_hwp);
	LOG_INFO("%s: total hardware watchpoint %d", target_name(target), nds32_v3->n_hwp);
	nds32_v3_reset_hb_id(target);

	/* low interference profiling */
	if (edm_cfg & 0x100)
		nds32_v3->low_interference_profile = true;
	else
		nds32_v3->low_interference_profile = false;
	return ERROR_OK;
}

/* talk to the target and set things up */
static int nds32_v3_examine(struct target *target)
{
	struct nds32_v3_common *nds32_v3 = target_to_nds32_v3(target);
	struct nds32 *nds32 = &(nds32_v3->nds32);

	if ((aice_port->type == AICE_PORT_FTDI) &&
		  ((custom_initial_script != NULL) || (custom_restart_script != NULL))) {
		if (custom_initial_script != NULL) {
				LOG_DEBUG("doing custom_initial_script...");
				if (aice_execute_custom_script(target, custom_initial_script) != ERROR_OK)
					return ERROR_FAIL;
				custom_initial_script = NULL;
				LOG_DEBUG("custom_initial_script finish");
		}
	}
	else {
		if (target->tap->hasidcode == false) {
			LOG_ERROR("no IDCODE present on device");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (aice_port->type == AICE_PORT_FTDI) {
		LOG_DEBUG("AICE_PORT_FTDI, disable usb_pack");
		aice_usb_pack_command = 0;
		nds_ftdi_get_info();
		// AICE-MINI-PLUS
		if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
			scan_field_max = 32;
		}

		if (aice_do_diagnosis == 1)
			aice_diagnosis(target);
	}

	if (!target_was_examined(target)) {
		if (nds32->reset_halt_as_examine)
			CHECK_RETVAL(nds32_reset_halt(nds32));
	}
	nds32->target->state = TARGET_RUNNING;
	nds32->target->debug_reason = DBG_REASON_NOTHALTED;

	aice_reg_set_ace_access_op(target);

	target_set_examined(target);

	return ERROR_OK;
}

/** Holds methods for Andes1337 targets. */
struct target_type nds32_v3_target = {
	.name = "nds32_v3",

	.poll = nds32_poll,
	.arch_state = nds32_arch_state,

	.target_request_data = nds32_v3_target_request_data,

	.halt = nds32_halt,
	.resume = nds32_resume,
	.step = nds32_step,

	.assert_reset = nds32_assert_reset,
	.deassert_reset = nds32_v3_deassert_reset,

	/* register access */
	.get_gdb_reg_list = nds32_get_gdb_reg_list,

	/* memory access */
	.read_buffer = nds32_v3_read_buffer,
	.write_buffer = nds32_v3_write_buffer,
	.read_memory = nds32_v3_read_memory,
	.write_memory = nds32_v3_write_memory,

	.checksum_memory = nds32_v3_checksum_memory,

	/* breakpoint/watchpoint */
	.add_breakpoint = nds32_v3_add_breakpoint,
	.remove_breakpoint = nds32_v3_remove_breakpoint,
	.add_watchpoint = nds32_v3_add_watchpoint,
	.remove_watchpoint = nds32_v3_remove_watchpoint,
	.hit_watchpoint = nds32_v3_hit_watchpoint,

	/* MMU */
	.mmu = nds32_mmu,
	.virt2phys = nds32_virtual_to_physical,
	.read_phys_memory = nds32_read_phys_memory,
	.write_phys_memory = nds32_write_phys_memory,

	.run_algorithm = nds32_v3_run_algorithm,

	.commands = nds32_command_handlers,
	.target_create = nds32_v3_target_create,
	.init_target = nds32_v3_init_target,
	.examine = nds32_v3_examine,

	.get_gdb_fileio_info = nds32_get_gdb_fileio_info,
	.gdb_fileio_end = nds32_gdb_fileio_end,

	.profiling = nds32_profiling,
};

int v3_remove_all_sw_breakpoint(struct target *target)
{
	int result;
	struct breakpoint *bp;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT) {
			/* restore ori-instruction */
			//struct nds32 *nds32 = target_to_nds32(target);
			//LOG_DEBUG("bp->orig_instr = 0x%x, syscall = 0x%x", (int)*bp->orig_instr, (int)nds32->hit_syscall);
			result = nds32_remove_software_breakpoint(target, bp);
			if (ERROR_OK != result) {
				LOG_ERROR("remove_software_breakpoint 0x%" TARGET_PRIxADDR " ERROR", bp->address);
			}
		}
	}
	return ERROR_OK;
}

int v3_add_all_sw_breakpoint(struct target *target)
{
	int result;
	struct breakpoint *bp;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT) {
			/* insert sw-break */
			result = nds32_add_software_breakpoint(target, bp);
			if (ERROR_OK != result) {
				LOG_ERROR("add_software_breakpoint 0x%" TARGET_PRIxADDR " ERROR", bp->address);
			}
			//struct nds32 *nds32 = target_to_nds32(target);
			//LOG_DEBUG("bp->orig_instr = 0x%x, syscall = 0x%x", (int)*bp->orig_instr, (int)nds32->hit_syscall);
		}
	}
	return ERROR_OK;
}


