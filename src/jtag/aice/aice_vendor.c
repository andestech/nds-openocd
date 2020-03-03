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

#include "aice_port.h"
#include "aice_jdp.h"
#include "aice_apis.h"
#include "aice_usb.h"
#include "target/nds32_log.h"

#define JTAG_InC      1
#define JTAG_READ     0
#define JTAG_WRITE    1
#define JTAG_RUNTEST_CYCLES  20//100
#define JTAG_SCAN_FIELD_MAX  0x8000

static int nds_sdm_set_tap_num_restore(void);
uint32_t runtest_num_clocks = JTAG_RUNTEST_CYCLES;
uint32_t scan_field_max = JTAG_SCAN_FIELD_MAX;
uint32_t nds_ftdi_log_detail = 0;
uint32_t nds_ftdi_jtag_opt = 0;
uint32_t sdm_cfg_value = 0;
uint32_t nds32_sdm_core_power_on[32];
uint32_t nds32_sdm_direct_select = 0;
extern uint32_t aice_sdm_support, aice_sdm_support_ena, aice_current_use_sdm;
extern uint32_t aice_default_use_sdm;
extern int aice_efreq_support;
unsigned int nds_mixed_mode_checking = 0;

static int nds_ftdi_open_device(struct aice_port_param_s *param)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int nds_ftdi_close(void)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int nds_ftdi_reset(void)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int nds_ftdi_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	LOG_DEBUG("%s", __func__);
	//uint32_t tap_count = jtag_tap_count();
	//*num_of_idcode = (uint8_t)tap_count;
	//LOG_DEBUG("tap_count: %d", tap_count);
	struct target *target = NULL;
	struct jtag_tap *tap = NULL;
	uint32_t i;
	struct nds32 *nds32;

	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (is_nds32(nds32)) {
			tap = target->tap;
			break;
		}
	}
	if (tap == NULL) {
		LOG_ERROR("tap == NULL !!");
		return ERROR_FAIL;
	}

	uint32_t in_value[AICE_MAX_NUM_CORE], data_value = 0xFFFFFFFF;
	uint32_t ir_scan = 0xFFFFFFFF;
	struct scan_field field[2];
	int retval = 0;
	uint32_t coreid, core_num = 0, core_num_check = 0xFFFFFFFF;

	for (coreid = 0; coreid < AICE_MAX_NUM_CORE; coreid++) {
		in_value[coreid] = 0;
	}

	ir_scan = 0xFFFFFFFF;
	field[0].num_bits = tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = 32;
	field[1].out_value = (uint8_t *)&data_value;
	field[1].in_value = (uint8_t *)&core_num_check;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;
	jtag_add_ir_scan(tap, &field[0], TAP_IDLE);
	jtag_add_dr_scan(tap, 1, &field[1], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	LOG_DEBUG("core_num_check:0x%x", core_num_check);
	for (i = 0; i < AICE_MAX_NUM_CORE; i++) {
		if ((core_num_check & 0x01) == 0) {
			core_num ++;
		} else {
			break;
		}
		core_num_check >>= 1;
	}
	if (core_num > AICE_MAX_NUM_CORE)
		core_num = AICE_MAX_NUM_CORE;
	if (nds_mixed_mode_checking == 3) // V3 + V5 mixed_mode
		core_num = 1;

	LOG_DEBUG("core_num:0x%x", core_num);
	ir_scan = IDCODE;
	field[0].num_bits = tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = 32;
	field[1].out_value = (uint8_t *)&data_value;
	field[1].in_value = (uint8_t *)&in_value[0];
	field[1].check_value = NULL;
	field[1].check_mask = NULL;
	uint32_t *p_idcode = (uint32_t *)idcode;

	for (coreid = 0; coreid < core_num; coreid++) {
		jtag_add_ir_scan(tap, &field[0], TAP_IDLE);
		field[1].in_value = (uint8_t *)&in_value[coreid];
		jtag_add_dr_scan(tap, 1, &field[1], TAP_IDLE);
	}
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	for (coreid = 0; coreid < core_num; coreid++) {
		if (in_value[coreid] == 0)
			in_value[coreid] = 0xFFFFFFFF; // END_OF_CHAIN_FLAG
		LOG_DEBUG("idcode-scan 0x%x: 0x%x", coreid, in_value[coreid]);
		*p_idcode++ = in_value[coreid];
	}

	*num_of_idcode = core_num;
	return ERROR_OK;
}

static int nds_ftdi_set_jtag_clock(uint32_t a_clock)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int nds_execute(struct target *target)
{
	uint32_t in_value;
	static uint8_t ir_scan = EXECUTE;
	struct scan_field field[2];

	if (nds_ftdi_log_detail)
		LOG_DEBUG("%s", __func__);

	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = &ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	uint32_t addr = 0;
	uint32_t addr_bits = 1;

	field[0].num_bits = addr_bits;
	field[0].out_value = (uint8_t *)&addr;
	field[0].in_value = (uint8_t *)&in_value;
	//field[0].check_value = NULL;
	//field[0].check_mask = NULL;

	jtag_add_dr_scan(target->tap, 1, &field[0], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	if (nds_ftdi_log_detail)
		LOG_DEBUG("in_value = 0x%x", in_value);
	return ERROR_OK;
}

static int nds_access_dim(struct target *target, uint32_t *p_dim_value)
{
	uint32_t in_value1=0, in_value2=0;
	uint32_t ir_scan = ACCESS_DIM;
	struct scan_field field[2];
	uint32_t i, tmpdata, dim_value[4];
	if (nds_ftdi_log_detail)
		LOG_DEBUG("%s", __func__);

	uint32_t addr = 0;
	addr |= (JTAG_WRITE << 0);
	uint32_t addr_bits = 2;

	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	for (i=0; i<4; i++) {
		tmpdata = *p_dim_value ++;
		dim_value[i] = (tmpdata & 0xFF);
		dim_value[i] = (dim_value[i] << 8);
		dim_value[i] |= ((tmpdata & 0xFF00) >> 8);
		dim_value[i] = (dim_value[i] << 8);
		dim_value[i] |= ((tmpdata & 0xFF0000) >> 16);
		dim_value[i] = (dim_value[i] << 8);
		dim_value[i] |= ((tmpdata & 0xFF000000) >> 24);
	}

	uint32_t data_out = dim_value[0];
	uint32_t issue_mask = (JTAG_InC << (addr_bits - 1));
	addr |= issue_mask;

	field[0].num_bits = 32;
	field[0].out_value = (uint8_t *)&data_out;
	field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = addr_bits;
	field[1].out_value = (uint8_t *)&addr;
	field[1].in_value = (uint8_t *)&in_value2;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	for (i=1; i<4; i++) {
		data_out = dim_value[i];
		while(1) {
			jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
			jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK) {
				LOG_ERROR("failed jtag scan: %d", retval);
				return retval;
			}
			if (nds_ftdi_log_detail) {
				LOG_DEBUG("in_value1 = 0x%x", in_value1);
				LOG_DEBUG("in_value2 = 0x%x", in_value2);
			}
			if (in_value2 & issue_mask)
				break;
		}
	}

	addr &= ~issue_mask;
	while(1) {
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
		jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("in_value1 = 0x%x", in_value1);
			LOG_DEBUG("in_value2 = 0x%x", in_value2);
		}
		if (in_value2 & issue_mask)
			break;
	}
	return ERROR_OK;
}

static int nds_edm_mem_access_opt(struct target *target, uint8_t JDPInst,
	uint32_t start_addr, uint32_t *data_value, uint32_t count)
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("start_addr=0x%x, count=0x%x", start_addr, count);
	if (count == 0)
		return ERROR_FAIL;

	uint32_t op_write = 0;
	if (JDPInst & JDP_WRITE) {
		op_write = 1;
	}
	uint8_t *byte_data = (uint8_t *)data_value;
	uint16_t *short_data = (uint16_t *)data_value;
	uint32_t *word_data = (uint32_t *)data_value;
	uint32_t addr, addr_bits, addr_offset, data_bits, data_mask;
	uint32_t data_addr = 0;
	uint32_t ir_scan = (JDPInst & 0x0f);
	uint32_t i, scanfields = count;
	uint8_t *curr_byte = NULL;
	uint16_t *curr_short = NULL;
	uint32_t *curr_word = NULL;
	uint32_t retry_start_addr = start_addr;
	uint32_t *p_in_value1 = NULL, *p_in_value2 = NULL;
	p_in_value1 = malloc(sizeof(uint32_t) * (scanfields + 1));
	p_in_value2 = malloc(sizeof(uint32_t) * (scanfields + 1));
	uint32_t num_clocks = runtest_num_clocks;

	if (ir_scan == FAST_ACCESS_MEM) {
		addr = 0;
		addr_bits = 1;
		addr_offset = 4;
		data_bits = 32;
		data_mask = 0xFFFFFFFF;
	} else if (ir_scan == ACCESS_MEM_W) {
		addr_bits = 32;
		addr_offset = 4;
		data_bits = 32;
		data_mask = 0xFFFFFFFF;
	} else if (ir_scan == ACCESS_MEM_H) {
		addr_bits = 17;
		addr_offset = 2;
		data_bits = 32;
		data_mask = 0xFFFF;
		addr = (addr >> 16);
	} else {//if (ir_scan == ACCESS_MEM_B) {
		addr_bits = 10;
		addr_offset = 1;
		data_bits = 32;
		data_mask = 0xFF;
	}
	uint32_t issue_mask = (JTAG_InC << (addr_bits - 1));
	//uint32_t in_value1=0, in_value2=0;
	struct scan_field field[2];
	int retval = 0;

nds_edm_mem_access_opt_retry:
	curr_byte = (uint8_t *)byte_data;
	curr_short = (uint16_t *)short_data;
	curr_word = (uint32_t *)word_data;
	start_addr = retry_start_addr;

	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	field[0].num_bits = data_bits;
	field[0].out_value = (uint8_t *)&data_addr;
	//field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = addr_bits;
	field[1].out_value = (uint8_t *)&addr;
	//field[1].in_value = (uint8_t *)&in_value2;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	for (i = 0; i < scanfields; i++) {
		if (ir_scan == FAST_ACCESS_MEM) {
			if (op_write == 1)
				data_addr = *curr_word++;
		} else {
			if (ir_scan == ACCESS_MEM_W) {
				addr = (start_addr >> 2);
				if (op_write == 1)
					data_addr = *curr_word++;
			} else if (ir_scan == ACCESS_MEM_H) {
				addr = (start_addr >> 1);
				data_addr = (addr << 16);
				if (op_write == 1)
					data_addr |= *curr_short++;
				addr = (addr >> 16);
			} else {//if (ir_scan == ACCESS_MEM_B) {
				addr = (start_addr >> 0);
				data_addr = (addr << 8);
				if (op_write == 1)
					data_addr |= *curr_byte++;
				addr = (addr >> 24);
			}
			start_addr += addr_offset;
			addr |= issue_mask;
			if (op_write == 1 ) {
				addr |= (JTAG_WRITE << (addr_bits - 2));
			}
		}
		addr |= issue_mask;

		field[0].in_value = (uint8_t *)&p_in_value1[i];
		field[1].in_value = (uint8_t *)&p_in_value2[i];
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
		jtag_add_runtest(num_clocks, TAP_DRSHIFT);
	}
	// the last input
	field[0].in_value = (uint8_t *)&p_in_value1[scanfields];
	field[1].in_value = (uint8_t *)&p_in_value2[scanfields];
	addr &= ~issue_mask;
	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
	jtag_add_runtest(num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	while(1) {
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("ir_scan = 0x%x, issue_mask = 0x%x", ir_scan, issue_mask);
			LOG_DEBUG("p_in_value1[0x%x] = 0x%x", scanfields, p_in_value1[scanfields]);
			LOG_DEBUG("p_in_value2[0x%x] = 0x%x", scanfields, p_in_value2[scanfields]);
		}
		if (p_in_value2[scanfields] & issue_mask)
			break;
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
		jtag_add_runtest(num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		LOG_DEBUG("last p_in_value2[0x%x] = 0x%x", scanfields, p_in_value2[scanfields]);
	}

	for (i = 1; i < scanfields; i++) {
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("p_in_value1[%d] = 0x%x", i, p_in_value1[i]);
			LOG_DEBUG("p_in_value2[%d] = 0x%x", i, p_in_value2[i]);
		}
		if ((p_in_value2[i] & issue_mask) == 0) {
			num_clocks ++;
			runtest_num_clocks = num_clocks;
			LOG_ERROR("nds_edm_mem_access_opt_retry, runtest_num_clocks=0x%x", runtest_num_clocks);
			if (ir_scan == FAST_ACCESS_MEM) {
				if (op_write == 1)
					aice_write_misc(target, NDS_EDM_MISC_SBAR, retry_start_addr|0x01);
				else
					aice_write_misc(target, NDS_EDM_MISC_SBAR, retry_start_addr);
			}
			goto nds_edm_mem_access_opt_retry;
		}
	}

	if (op_write == 0) {
		if ((ir_scan == ACCESS_MEM_W) || (ir_scan == FAST_ACCESS_MEM)) {
			for (i = 0; i < scanfields; i++)
				*word_data++ = p_in_value1[i+1] & data_mask;
		} else if (ir_scan == ACCESS_MEM_H) {
			for (i = 0; i < scanfields; i++)
				*short_data++ = p_in_value1[i+1] & data_mask;
		} else if (ir_scan == ACCESS_MEM_B) {
			for (i = 0; i < scanfields; i++)
				*byte_data++ = p_in_value1[i+1] & data_mask;
		}
	}
	free(p_in_value1);
	free(p_in_value2);
	return ERROR_OK;
}

static int nds_edm_mem_access(struct target *target, uint8_t JDPInst,
	uint32_t start_addr, uint32_t *data_value, uint32_t count)
{
	if (nds_ftdi_jtag_opt) {
		return nds_edm_mem_access_opt(target, JDPInst, start_addr, data_value, count);
	}

	if (nds_ftdi_log_detail)
		LOG_DEBUG("start_addr=0x%x, count=0x%x", start_addr, count);
	if (count == 0)
		return ERROR_FAIL;

	uint32_t op_write = 0;
	if (JDPInst & JDP_WRITE) {
		op_write = 1;
	}

	uint8_t *byte_data = (uint8_t *)data_value;
	uint16_t *short_data = (uint16_t *)data_value;
	uint32_t *word_data = (uint32_t *)data_value;
	uint32_t addr, addr_bits, addr_offset, data_bits, data_mask;
	uint32_t data_addr = 0;
	uint32_t ir_scan = (JDPInst & 0x0f);
	uint32_t original_start_addr = start_addr;
	uint32_t original_count = count;

nds_edm_mem_access_retry:
	byte_data = (uint8_t *)data_value;
	short_data = (uint16_t *)data_value;
	word_data = (uint32_t *)data_value;
	start_addr = original_start_addr;
	data_addr = 0;
	count = original_count;
	ir_scan = (JDPInst & 0x0f);
	if (ir_scan == FAST_ACCESS_MEM) {
		addr_bits = 1;
		addr = 0;
		addr_offset = 4;
		data_bits = 32;
		data_mask = 0xFFFFFFFF;
		if (op_write == 1)
			data_addr = *word_data++;
	} else if (ir_scan == ACCESS_MEM_W) {
		addr_bits = 32;
		addr = (start_addr >> 2);
		addr_offset = 4;
		data_bits = 32;
		data_mask = 0xFFFFFFFF;
		if (op_write == 1)
			data_addr = *word_data++;
	} else if (ir_scan == ACCESS_MEM_H) {
		addr_bits = 17;
		addr = (start_addr >> 1);
		addr_offset = 2;
		data_bits = 32;
		data_mask = 0xFFFF;
		data_addr = (addr << 16);
		if (op_write == 1)
			data_addr |= *short_data++;
		addr = (addr >> 16);
	} else {//if (ir_scan == ACCESS_MEM_B) {
		addr_bits = 10;
		addr = (start_addr >> 0);
		addr_offset = 1;
		data_bits = 32;
		data_mask = 0xFF;
		data_addr = (addr << 8);
		if (op_write == 1)
			data_addr |= *byte_data++;
		addr = (addr >> 24);
	}
	uint32_t issue_mask = (JTAG_InC << (addr_bits - 1));
	addr |= issue_mask;
	if ((op_write == 1 ) && (ir_scan != FAST_ACCESS_MEM)) {
		addr |= (JTAG_WRITE << (addr_bits - 2));
	}

	uint32_t in_value1=0, in_value2=0;
	struct scan_field field[2];
	int retval = 0;

	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	field[0].num_bits = data_bits;
	field[0].out_value = (uint8_t *)&data_addr;
	field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = addr_bits;
	field[1].out_value = (uint8_t *)&addr;
	field[1].in_value = (uint8_t *)&in_value2;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	count --;

	uint32_t i, scanfields = (scan_field_max/2);
	uint32_t *p_in_value1 = NULL, *p_in_value2 = NULL;
	if (count) {
		if (scanfields > count)
			scanfields = count;
		p_in_value1 = malloc(sizeof(uint32_t) * scanfields);
		p_in_value2 = malloc(sizeof(uint32_t) * scanfields);
		if (nds_ftdi_log_detail)
			LOG_DEBUG("scanfields = 0x%x", scanfields);
	}

	uint32_t retry_start_addr = start_addr + addr_offset;
	uint8_t *curr_byte = NULL;
	uint16_t *curr_short = NULL;
	uint32_t *curr_word = NULL;

	while (count) {
		curr_byte = (uint8_t *)byte_data;
		curr_short = (uint16_t *)short_data;
		curr_word = (uint32_t *)word_data;
		start_addr = retry_start_addr;

		for (i=0; i<scanfields; i++) {
			if (ir_scan == FAST_ACCESS_MEM) {
				if (op_write == 1)
					data_addr = *curr_word++;
			} else {
				if (ir_scan == ACCESS_MEM_W) {
					addr = (start_addr >> 2);
					if (op_write == 1)
						data_addr = *curr_word++;
				} else if (ir_scan == ACCESS_MEM_H) {
					addr = (start_addr >> 1);
					data_addr = (addr << 16);
					if (op_write == 1)
						data_addr |= *curr_short++;
					addr = (addr >> 16);
				} else {//if (ir_scan == ACCESS_MEM_B) {
					addr = (start_addr >> 0);
					data_addr = (addr << 8);
					if (op_write == 1)
						data_addr |= *curr_byte++;
					addr = (addr >> 24);
				}
				start_addr += addr_offset;
				addr |= issue_mask;
				if (op_write == 1 ) {
					addr |= (JTAG_WRITE << (addr_bits - 2));
				}
			}
			field[0].in_value = (uint8_t *)&p_in_value1[i];
			field[1].in_value = (uint8_t *)&p_in_value2[i];
			jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
			jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
		}
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		for (i=0; i<scanfields; i++) {
			if (nds_ftdi_log_detail) {
				LOG_DEBUG("p_in_value1[%d] = 0x%x", i, p_in_value1[i]);
				LOG_DEBUG("p_in_value2[%d] = 0x%x", i, p_in_value2[i]);
			}
			if ((p_in_value2[i] & issue_mask) == 0) {
				runtest_num_clocks ++;
				LOG_ERROR("nds_edm_mem_access_retry, runtest_num_clocks=0x%x", runtest_num_clocks);
				if (ir_scan == FAST_ACCESS_MEM) {
					if (op_write == 1)
						aice_write_misc(target, NDS_EDM_MISC_SBAR, original_start_addr|0x01);
					else
						aice_write_misc(target, NDS_EDM_MISC_SBAR, original_start_addr);
				}
				free(p_in_value1);
				free(p_in_value2);
				goto nds_edm_mem_access_retry;
			}
		}

		if (op_write == 0) {
			if ((ir_scan == ACCESS_MEM_W) || (ir_scan == FAST_ACCESS_MEM)) {
				for (i=0; i<scanfields; i++)
					*word_data++ = p_in_value1[i] & data_mask;
			} else if (ir_scan == ACCESS_MEM_H) {
				for (i=0; i<scanfields; i++)
					*short_data++ = p_in_value1[i] & data_mask;
			} else if (ir_scan == ACCESS_MEM_B) {
				for (i=0; i<scanfields; i++)
					*byte_data++ = p_in_value1[i] & data_mask;
			}
		} else {
			word_data += scanfields;
			short_data += scanfields;
			byte_data += scanfields;
		}
		retry_start_addr += (addr_offset * scanfields);
		count -= scanfields;
		if (scanfields > count)
			scanfields = count;
		//LOG_DEBUG("count = 0x%x, scanfields = 0x%x", count, scanfields);
	}  // end of while (count)

	// restore
	field[0].in_value = (uint8_t *)&in_value1;
	field[1].in_value = (uint8_t *)&in_value2;
	free(p_in_value1);
	free(p_in_value2);

	addr &= ~issue_mask;
	while(1) {
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
		jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("in_value1 = 0x%x", in_value1);
			LOG_DEBUG("in_value2 = 0x%x", in_value2);
		}
		if (in_value2 & issue_mask)
			break;
	}
	if (op_write == 0) {
		if ((ir_scan == ACCESS_MEM_W) || (ir_scan == FAST_ACCESS_MEM)) {
			*word_data++ = in_value1 & data_mask;
		} else if (ir_scan == ACCESS_MEM_H) {
			*short_data++ = in_value1 & data_mask;
		} else if (ir_scan == ACCESS_MEM_B) {
			*byte_data++ = in_value1 & data_mask;
		}
	}
	return ERROR_OK;
}

static int nds_edm_reg_access_opt(struct target *target, uint32_t ir_scan_value, uint32_t addr, uint32_t addr_bits, uint32_t *data_value)
{
	uint32_t in_value1=0, in_value2=0;
	uint32_t in_value1_last=0, in_value2_last=0;
	uint32_t ir_scan = ir_scan_value;
	struct scan_field field[2];
	int retval = 0;
	uint32_t num_clocks = runtest_num_clocks;

nds_edm_reg_access_opt_retry:
	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	uint32_t data_out = *data_value;
	uint32_t issue_mask = (JTAG_InC << (addr_bits - 1));
	addr |= issue_mask;
	uint32_t op_write = 0;
	if (addr & (JTAG_WRITE << (addr_bits - 2))) {
		op_write = 1;
	}
	field[0].num_bits = 32;
	field[0].out_value = (uint8_t *)&data_out;
	field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = addr_bits;
	field[1].out_value = (uint8_t *)&addr;
	field[1].in_value = (uint8_t *)&in_value2;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	if (nds_ftdi_log_detail) {
		LOG_DEBUG("addr = 0x%x, addr_bits = 0x%x, issue_mask = 0x%x", addr, addr_bits, issue_mask);
	}
	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
	jtag_add_runtest(num_clocks, TAP_IDLE);
	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
	jtag_add_runtest(num_clocks, TAP_IDLE);

	// last
	addr &= ~issue_mask;
	field[0].in_value = (uint8_t *)&in_value1_last;
	field[1].in_value = (uint8_t *)&in_value2_last;

	jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
	jtag_add_runtest(num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	if (nds_ftdi_log_detail) {
		LOG_DEBUG("in_value1 = 0x%x", in_value1);
		LOG_DEBUG("in_value2 = 0x%x", in_value2);
		LOG_DEBUG("in_value1_last = 0x%x", in_value1_last);
		LOG_DEBUG("in_value2_last = 0x%x", in_value2_last);
	}

	while(1) {
		if (in_value2_last & issue_mask)
			break;
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_DRPAUSE);
		jtag_add_runtest(num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		LOG_DEBUG("in_value2_last = 0x%x", in_value2_last);
	}

	if ((in_value2 & issue_mask) == 0) {
		 num_clocks ++;
		 runtest_num_clocks = num_clocks;
		 goto nds_edm_reg_access_opt_retry;
	}

	if (op_write == 0)
		*data_value = in_value1_last;
	return ERROR_OK;
}

static int nds_edm_reg_access(struct target *target, uint32_t ir_scan_value, uint32_t addr, uint32_t addr_bits, uint32_t *data_value)
{
	uint32_t in_value1=0, in_value2=0;
	uint32_t ir_scan = ir_scan_value;
	struct scan_field field[2];
	int retval = 0;

	if (nds_ftdi_jtag_opt) {
		return nds_edm_reg_access_opt(target, ir_scan_value, addr, addr_bits, data_value);
	}

	field[0].num_bits = target->tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(target->tap, &field[0], TAP_IDLE);

	uint32_t data_out = *data_value;
	uint32_t issue_mask = (JTAG_InC << (addr_bits - 1));
	addr |= issue_mask;
	uint32_t op_write = 0;
	if (addr & (JTAG_WRITE << (addr_bits - 2))) {
		op_write = 1;
	}
	field[0].num_bits = 32;
	field[0].out_value = (uint8_t *)&data_out;
	field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	field[1].num_bits = addr_bits;
	field[1].out_value = (uint8_t *)&addr;
	field[1].in_value = (uint8_t *)&in_value2;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	if (nds_ftdi_log_detail) {
		LOG_DEBUG("addr = 0x%x, addr_bits = 0x%x, issue_mask = 0x%x", addr, addr_bits, issue_mask);
	}
	while(1) {
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
		jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("in_value1 = 0x%x", in_value1);
			LOG_DEBUG("in_value2 = 0x%x", in_value2);
		}
		if (in_value2 & issue_mask)
			break;
	}

	addr &= ~issue_mask;
	while(1) {
		jtag_add_dr_scan(target->tap, 2, &field[0], TAP_IDLE);
		jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("failed jtag scan: %d", retval);
			return retval;
		}
		if (nds_ftdi_log_detail) {
			LOG_DEBUG("in_value1 = 0x%x", in_value1);
			LOG_DEBUG("in_value2 = 0x%x", in_value2);
		}
		if (in_value2 & issue_mask)
			break;
	}
	if (op_write == 0)
		*data_value = in_value1;
	return ERROR_OK;
}

static int nds_access_misc(struct target *target, uint32_t op_write, uint32_t misc_reg, uint32_t *misc_value)
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("%s", __func__);

	uint32_t addr = (misc_reg & 0xf);
	if (op_write == 1) {
		addr |= (JTAG_WRITE << 4);
	}
	uint32_t addr_bits = 6;
	int result = nds_edm_reg_access(target, ACCESS_MISC_REG, addr, addr_bits, misc_value);
	if (op_write == 0) {
		if (*misc_value == 0xFFFFFFFF) {
			LOG_ERROR("read EDM fail !!");
			NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
			exit(-1);
			return ERROR_FAIL;
		}
	}
	return result;
}

static int nds_access_sr(struct target *target, uint32_t op_write, uint32_t sr_reg, uint32_t *sr_value)
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("%s", __func__);

	uint32_t addr = (sr_reg & 0x7f);
	if (op_write == 1 ) {
		addr |= (JTAG_WRITE << 7);
	}
	uint32_t addr_bits = 9;
	int result = nds_edm_reg_access(target, ACCESS_DBG_SR, addr, addr_bits, sr_value);
	return result;
}

static int nds_access_dtr(struct target *target, uint32_t op_write, uint32_t *dtr_value)
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("%s", __func__);

	uint32_t addr = 0;
	if (op_write == 1 ) {
		addr |= (JTAG_WRITE << 0);
	}
	uint32_t addr_bits = 2;
	int result = nds_edm_reg_access(target, ACCESS_DTR, addr, addr_bits, dtr_value);
	return result;
}

static int nds_ftdi_read_edm( uint32_t core_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("JDPInst:0x%x, ADDR:0x%x", JDPInst, address);
	int result = 0;

	struct target *target = coreid_to_target(core_id);
	if ((JDPInst & 0x0f)== ACCESS_MISC_REG)
		result = nds_access_misc(target, 0, address, EDMData);
	else if ((JDPInst & 0x0f)== ACCESS_DBG_SR)
		result = nds_access_sr(target, 0, address, EDMData);
	else if ((JDPInst & 0x0f)== ACCESS_DTR)
		result = nds_access_dtr(target, 0, EDMData);
	else if (((JDPInst & 0x0f)== ACCESS_MEM_W) ||
		((JDPInst & 0x0f)== ACCESS_MEM_H) ||
		((JDPInst & 0x0f)== ACCESS_MEM_B)) {
		result = nds_edm_mem_access(target, JDPInst, address, EDMData, num_of_words);
	} else if ((JDPInst & 0x0f)== FAST_ACCESS_MEM) {
		result = nds_edm_mem_access(target, JDPInst, address, EDMData, num_of_words);
	}
	if (result == ERROR_OK) {
		aice_print_info(AICE_READ_EDM, address, (unsigned int *)EDMData, core_id, JDPInst);
	}
	if (nds32_sdm_direct_select) {
		nds_sdm_set_tap_num_restore();
		nds32_sdm_direct_select = 0;
	}
	return result;
}

static int nds_ftdi_write_edm( uint32_t core_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	if (nds_ftdi_log_detail)
		LOG_DEBUG("JDPInst:0x%x, ADDR:0x%x, DATA:0x%x", JDPInst, address, *EDMData);
	int result = 0;

	struct target *target = coreid_to_target(core_id);
	if ((JDPInst & 0x0f)== ACCESS_MISC_REG)
		result = nds_access_misc(target, 1, address, EDMData);
	else if ((JDPInst & 0x0f)== ACCESS_DBG_SR)
		result = nds_access_sr(target, 1, address, EDMData);
	else if ((JDPInst & 0x0f)== EXECUTE)
		result = nds_execute(target);
	else if ((JDPInst & 0x0f)== ACCESS_DTR)
		result = nds_access_dtr(target, 1, EDMData);
	else if ((JDPInst & 0x0f)== ACCESS_DIM)
		result = nds_access_dim(target, EDMData);
	else if (((JDPInst & 0x0f)== ACCESS_MEM_W) ||
		((JDPInst & 0x0f)== ACCESS_MEM_H) ||
		((JDPInst & 0x0f)== ACCESS_MEM_B)) {
		result = nds_edm_mem_access(target, JDPInst, address, EDMData, num_of_words);
	} else if ((JDPInst & 0x0f)== FAST_ACCESS_MEM) {
		result = nds_edm_mem_access(target, JDPInst, address, EDMData, num_of_words);
	}
	if (result == ERROR_OK) {
		aice_print_info(AICE_WRITE_EDM, address, (unsigned int *)EDMData, core_id, JDPInst);
	}
	if (nds32_sdm_direct_select) {
		nds_sdm_set_tap_num_restore();
		nds32_sdm_direct_select = 0;
	}
	return result;
}

int nds_ftdi_get_state(void)
{
	#if 0
	uint32_t ice_state = 0xFF;

	int result = aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &ice_state);
	if (result != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_AICE_DISCONNECT);
		exit(-1);
		return ERROR_FAIL;
	}
	else if ((ice_state & 0x20) == 0) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
		return ERROR_FAIL;
	}
	#endif
	return ERROR_OK;
}

static int nds_sdm_registers(struct jtag_tap *tap, uint32_t address, uint32_t data_value)
{
	uint32_t in_value1=0;
	uint32_t ir_scan = BYPASS;
	struct scan_field field[4];
	int retval = 0;
	uint32_t checksum = 0;
	uint32_t sdm_conn_code = 0x800001;

	unsigned int ctrl_data = data_value;
	aice_print_info(AICE_WRITE_CTRL, AICE_WRITE_CTRL_SDMCONN, (unsigned int *)&ctrl_data, 0, 0);

	checksum = (address ^ (data_value & 0xff));
	checksum ^= ((data_value >> 8) & 0xff);
	checksum ^= ((data_value >> 16) & 0xff);
	checksum ^= ((data_value >> 24) & 0xff);

	LOG_DEBUG("address: 0x%x, data_value: 0x%x, checksum: 0x%x", address, data_value, checksum);
	field[0].num_bits = tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(tap, &field[0], TAP_IDLE);

	field[3].num_bits = 8;
	field[3].out_value = (uint8_t *)&checksum;
	field[3].in_value = (uint8_t *)&in_value1;
	field[3].check_value = NULL;
	field[3].check_mask = NULL;

	field[2].num_bits = 32;
	field[2].out_value = (uint8_t *)&data_value;
	field[2].in_value = (uint8_t *)&in_value1;
	field[2].check_value = NULL;
	field[2].check_mask = NULL;

	field[1].num_bits = 8;
	field[1].out_value = (uint8_t *)&address;
	field[1].in_value = (uint8_t *)&in_value1;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;

	field[0].num_bits = 24;
	field[0].out_value = (uint8_t *)&sdm_conn_code;
	field[0].in_value = (uint8_t *)&in_value1;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;

	jtag_add_dr_scan(tap, 4, &field[0], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	//drscan $NDS_TAP 24 $sdm_conn_code 8 $addr 32 $data 8 $checksum
	return ERROR_OK;
}

/*
static int nds_sdm_set_tap_num_1(void)
{
	uint32_t tap_count = jtag_tap_count_enabled();
	if (tap_count > 1) {
		LOG_DEBUG("tap_count: %d", tap_count);
		struct jtag_tap *tap = jtag_tap_next_enabled(NULL);
		while(tap != NULL) {
			tap = tap->next_tap;
			if (tap) {
				if (tap->enabled) {
					//if (nds32_sdm_core_power_on[tap->abs_chain_position] != 0x01) {
						tap->enabled = 0;
						LOG_DEBUG("set tap[%d]->enabled = %d", tap->abs_chain_position, tap->enabled);
					//}
				}
			}
		}
	}
	return ERROR_OK;
}
*/
int nds_sdm_only_active_one_tap(uint32_t coreid)
{
	/* enable tap->abs_chain_position = coreid */
	struct jtag_tap *tap = jtag_all_taps();
	while (tap) {
		if (tap->abs_chain_position == (int)coreid) {
			tap->enabled = 1;
		} else {
			tap->enabled = 0;
		}
		LOG_DEBUG("set tap[%d]->enabled = %d", tap->abs_chain_position, tap->enabled);
		tap = tap->next_tap;
	}
	return ERROR_OK;
}

static int nds_sdm_set_tap_num_restore(void)
{
	/* enable all disabled-taps before */
	struct jtag_tap *tap = jtag_all_taps();
	while (tap) {
		if (tap->enabled == 0) {
			if (nds32_sdm_core_power_on[tap->abs_chain_position] == 0x01) {
				tap->enabled = 1;
				LOG_DEBUG("set tap[%d]->enabled = %d", tap->abs_chain_position, tap->enabled);
			}
		}
		tap = tap->next_tap;
	}
	return ERROR_OK;
}

int nds_sdm_check_all_powered_down(uint32_t check_coreid)
{
	uint32_t sdm_core_nums = 0, coreid, core_stat;
	struct jtag_tap *tap = jtag_tap_next_enabled(NULL);

	LOG_DEBUG("active_tap: %d, check_coreid: %d", tap->abs_chain_position, check_coreid);
	nds_sdm_only_active_one_tap(tap->abs_chain_position);

	sdm_core_nums = (sdm_cfg_value & NDS_SDM_MISC_SDM_CFG_SDM_PROCESSOR_NUM_MASK);
	/* Clear the PROCESSOR_STATUS.EVER_DISABLED value
	   Read PROCESSOR_STATUS
	   If PROCESSOR_STATUS.EVER_DISABLED is set, the processor is powered down. */
	for (coreid = 0; coreid < sdm_core_nums; coreid++) {
		nds_sdm_registers(tap, 0, (NDS_SDM_SELECT_SDM|(0x01 << coreid)));

		uint32_t setValue = NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED;
		nds_ftdi_write_edm(tap->abs_chain_position, ACCESS_MISC_REG, NDS_SDM_MISC_PROCESSOR_STATUS, &setValue, 1);

		int retval = nds_ftdi_read_edm(tap->abs_chain_position, ACCESS_MISC_REG, NDS_SDM_MISC_PROCESSOR_STATUS, &core_stat, 1);
		if (retval == ERROR_OK) {
			if (core_stat & NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED) {
				if (nds32_sdm_core_power_on[coreid] != 0xFF) {
					NDS32_LOG("The core #%d is power-off", coreid);
					nds32_sdm_core_power_on[coreid] = 0xFF;
				}
			} else {
				if (nds32_sdm_core_power_on[coreid] != 0x01) {
					NDS32_LOG("The core #%d is running", coreid);
					nds32_sdm_core_power_on[coreid] = 0x01;
				}
			}
		}
	}
	nds_sdm_set_tap_num_restore();
	if (nds32_sdm_core_power_on[check_coreid] == 0x01) {
		return 0;
	}
	return 1;
}

int nds_sdm_idcode_scan(struct jtag_tap *tap, uint8_t *idcode_buffer, uint32_t max_count)
{
	uint32_t in_value1=0, data_value = 0xFFFFFFFF;
	uint32_t ir_scan = IDCODE;
	struct scan_field field[2];
	int retval = 0;
	uint32_t sdm_core_nums = 0, coreid, core_stat, i;
	uint32_t *p_idcode = (uint32_t *)idcode_buffer;
	uint32_t idcode_core0 = 0xFFFFFFFF;

	for (i=0; i<max_count; i++)
		*p_idcode++ = 0xFFFFFFFF; // END_OF_CHAIN_FLAG

	LOG_DEBUG("active_tap = %d", tap->abs_chain_position);
	nds_sdm_only_active_one_tap(tap->abs_chain_position);
	nds_sdm_registers(tap, 0, 0x80000000); // addr=0 => SDM SELECT.type=2 => SDM registers
	field[0].num_bits = tap->ir_length;
	field[0].out_value = (uint8_t *)&ir_scan;
	field[0].in_value = NULL;
	field[0].check_value = NULL;
	field[0].check_mask = NULL;
	jtag_add_ir_scan(tap, &field[0], TAP_IDLE);

	field[1].num_bits = 32;
	field[1].out_value = (uint8_t *)&data_value;
	field[1].in_value = (uint8_t *)&in_value1;
	field[1].check_value = NULL;
	field[1].check_mask = NULL;
	jtag_add_dr_scan(tap, 1, &field[1], TAP_IDLE);
	jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	if (in_value1 != 0x1000163d) {
		LOG_ERROR("NOT SDM(0x1000163d) exp-id: 0x%x", in_value1);
		nds_sdm_registers(tap, 0, 0x00000000); // addr=0 => SDM SELECT.type=0 => Daisy-chain
		nds_sdm_set_tap_num_restore();
		return ERROR_OK;
	}

	aice_sdm_support = 1;
	aice_sdm_support_ena = 1;
	aice_current_use_sdm = 1;

	if (sdm_cfg_value == 0) {
		nds_ftdi_read_edm(tap->abs_chain_position, ACCESS_MISC_REG, NDS_SDM_MISC_SDM_CFG, &sdm_cfg_value, 1);
		LOG_DEBUG("sdm_cfg_value: 0x%x", sdm_cfg_value);
	}
	//if (sdm_cfg_value & NDS_SDM_MISC_SDM_CFG_SDM_PARALLEL_DBG_BUS)
	//	sdm_parallel_debug_bus = 1;

	sdm_core_nums = (sdm_cfg_value & NDS_SDM_MISC_SDM_CFG_SDM_PROCESSOR_NUM_MASK);
	p_idcode = (uint32_t *)idcode_buffer;
	if (sdm_core_nums) {
		if (sdm_core_nums == 1) {
			NDS32_LOG(NDS32_MSG_TARGET_NUM_ONE);
		}
		else {
			NDS32_LOG(NDS32_MSG_TARGET_NUMS, sdm_core_nums);
		}
		for (coreid = 0; coreid < sdm_core_nums; coreid++) {
			nds_sdm_registers(tap, 0, (NDS_SDM_SELECT_SDM|(0x01 << coreid)));
			nds_ftdi_read_edm(tap->abs_chain_position, ACCESS_MISC_REG, NDS_SDM_MISC_PROCESSOR_STATUS, &core_stat, 1);
			if (core_stat & NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED) {
				NDS32_LOG("The core #%d is power-off", coreid);
				nds32_sdm_core_power_on[coreid] = 0xFF;
			} else {
				NDS32_LOG("The core #%d is running", coreid);
				nds32_sdm_core_power_on[coreid] = 0x01;
			}
			nds_sdm_registers(tap, 0, (NDS_SDM_SELECT_DIRECT|(0x01 << coreid)));
			jtag_add_ir_scan(tap, &field[0], TAP_IDLE);
			jtag_add_dr_scan(tap, 1, &field[1], TAP_IDLE);
			jtag_add_runtest(runtest_num_clocks, TAP_IDLE);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK) {
				LOG_ERROR("failed jtag scan: %d", retval);
				return retval;
			}
			if (in_value1 == 0)
				in_value1 = idcode_core0; // if core1 is power-off, set idcode the same as core0
			else
				idcode_core0 = in_value1;

			LOG_DEBUG("idcode-scan 0x%x: 0x%x", coreid, in_value1);
			*p_idcode++ = in_value1;
		}
		*p_idcode++ = 0xFFFFFFFF; // END_OF_CHAIN_FLAG
	}
	//nds_sdm_registers(tap, 0, 0x00000000); // addr=0 => SDM SELECT.type=0 => Daisy-chain

	/* enable all disabled-taps before */
	//nds_sdm_set_tap_num_restore();
	tap = jtag_all_taps();
	while (tap) {
		if (tap->enabled == 0) {
				tap->enabled = 1;
				LOG_DEBUG("set tap[%d]->enabled = 1", tap->abs_chain_position);
		}
		tap = tap->next_tap;
	}

	return ERROR_OK;
}

extern struct command_context *global_cmd_ctx;
extern uint32_t nds_ftdi_devices;
static int nds_ftdi_write_ctrl(uint32_t address, uint32_t WriteData)
{
	struct command_context *cmd_ctx = global_cmd_ctx;
	//LOG_DEBUG("ADDR:0x%x, DATA:0x%x", address, WriteData);
	//unsigned int ctrl_data = WriteData;
	//aice_print_info(AICE_WRITE_CTRL, address, (unsigned int *)&ctrl_data, 0, 0);
	uint32_t core_id = 0;
	struct target *target = coreid_to_target(core_id);

	if (address == AICE_WRITE_CTRL_TCK_CONTROL) {
		LOG_DEBUG("AICE_WRITE_CTRL_TCK_CONTROL, NOT support");
	} else if (address == AICE_WRITE_CTRL_JTAG_PIN_CONTROL) {
		if (WriteData == AICE_JTAG_PIN_CONTROL_SRST) {
			LOG_DEBUG("AICE_JTAG_PIN_CONTROL_SRST");
			// AICE-MINI-PLUS
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST 1");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 0");
			}
			//aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 4);
			alive_sleep(300);
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST z");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 1");
			}
			//aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 5);
			alive_sleep(500);
		} else if (WriteData == AICE_JTAG_PIN_CONTROL_TRST) {
			LOG_DEBUG("AICE_JTAG_PIN_CONTROL_TRST");
			/*
			command_run_line(cmd_ctx, "ftdi_write_pin TMS 1");
			command_run_line(cmd_ctx, "ftdi_write_pin TDI 1");
			command_run_line(cmd_ctx, "ftdi_write_pin nTRST 0");
			alive_sleep(300);
			command_run_line(cmd_ctx, "ftdi_write_pin nTRST 1");
			*/
			jtag_add_tlr();
			jtag_execute_queue();
			LOG_DEBUG("use jtag_add_tlr");
		} else if (WriteData == AICE_JTAG_PIN_CONTROL_STOP) {
			LOG_DEBUG("AICE_JTAG_PIN_CONTROL_STOP");
			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0);
		} else if (WriteData == AICE_JTAG_PIN_CONTROL_RESTART) {
			LOG_DEBUG("AICE_JTAG_PIN_CONTROL_RESTART");
			// AICE-MINI-PLUS
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST 1");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 0");
			}
			alive_sleep(300);
			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0);
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST z");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 1");
			}
			alive_sleep(500);
		}
	} else if (address == AICE_WRITE_CTRL_CUSTOM_DELAY) {
		uint32_t delay_count = (WriteData >> 16) & 0xFFFF;
		if (WriteData & AICE_CUSTOM_DELAY_SET_SRST) {
			LOG_DEBUG("AICE_CUSTOM_DELAY_SET_SRST");
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST 1");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 0");
			}
		} else if (WriteData & AICE_CUSTOM_DELAY_CLEAN_SRST) {
			LOG_DEBUG("AICE_CUSTOM_DELAY_CLEAN_SRST");
			if (nds_ftdi_devices == 1) { // ftdi_vid[i] == 0x1cfc
				command_run_line(cmd_ctx, "ftdi_set_signal nSRST z");
			} else {
				command_run_line(cmd_ctx, "ftdi_write_pin nSRST 1");
			}
		} else if (WriteData & AICE_CUSTOM_DELAY_SET_DBGI) {
			LOG_DEBUG("AICE_CUSTOM_DELAY_SET_DBGI");
			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0);
		} else if (WriteData & AICE_CUSTOM_DELAY_SET_TRST) {
			LOG_DEBUG("AICE_CUSTOM_DELAY_SET_TRST");
			jtag_add_tlr();
			jtag_execute_queue();
			LOG_DEBUG("use jtag_add_tlr");
			/*
			command_run_line(cmd_ctx, "ftdi_write_pin TMS 1");
			command_run_line(cmd_ctx, "ftdi_write_pin TDI 1");
			command_run_line(cmd_ctx, "ftdi_write_pin nTRST 0");
			*/
		} else if (WriteData & AICE_CUSTOM_DELAY_CLEAN_TRST) {
			LOG_DEBUG("AICE_CUSTOM_DELAY_CLEAN_TRST");
			//command_run_line(cmd_ctx, "ftdi_write_pin nTRST 1");
		}
		alive_sleep(delay_count);
	} else if (address == AICE_WRITE_CTRL_SDMCONN) {
		struct jtag_tap *tap = jtag_tap_next_enabled(NULL);
		nds_sdm_only_active_one_tap(tap->abs_chain_position);

		nds_sdm_registers(tap, 0, WriteData); // addr=0 => SDM SELECT
		if ((WriteData & NDS_SDM_SELECT_MASK) != NDS_SDM_SELECT_DIRECT) {
			nds_sdm_set_tap_num_restore();
		}
	}

	return ERROR_OK;
}

static int nds_ftdi_read_ctrl(uint32_t address, uint32_t *pReadData)
{
	uint32_t ice_config = 0;

	//LOG_DEBUG("ADDR:0x%x", address);
	if (address == AICE_READ_CTRL_ICE_CONFIG) {
		ice_config |= (0x01 << 30);    // WRITE_PINS_SUPPORT
		//ice_config |= (0x01 << 31);  // BATCH_SUPPORT
		//ice_config |= (0x01 << 4);   // EFREQ
		//ice_config |= (0x01 << 5);   // XRW
		// if use option --use_sdm
		if (aice_default_use_sdm == 1)
			ice_config |= (0x01 << 6);     // SDM
		*pReadData = ice_config;
	} else if (address == AICE_READ_CTRL_GET_ICE_STATE) {
		*pReadData = 0x29;
	}

	aice_print_info(AICE_READ_CTRL, address, (unsigned int *)pReadData, 0, 0);
	return ERROR_OK;
}

static int nds_ftdi_set_command_mode(enum aice_command_mode command_mode)
{
	LOG_DEBUG("command_mode = 0x%x", command_mode);
	return ERROR_OK;
}

struct aice_nds32_api_s aice_nds32_ftdi = {
	/** */
	.write_ctrl = nds_ftdi_write_ctrl,
	/** */
	.read_ctrl = nds_ftdi_read_ctrl,
	/** AICE read_dtr */
	.read_dtr_to_buffer = NULL,
	/** AICE write_dtr */
	.write_dtr_from_buffer = NULL,
	/** AICE batch_buffer_write */
	.batch_buffer_write = NULL,
	/** AICE batch_buffer_read */
	.batch_buffer_read = NULL,
	/** */
	.execute_custom_script = aice_usb_execute_custom_script,
	/** */
	.set_command_mode = nds_ftdi_set_command_mode,
	/** AICE pack_buffer_read */
	.pack_buffer_read = NULL,
	/** */
	.monitor_command = NULL,
	/** */
	.xwrite = NULL,
	/** */
	.xread = NULL,
};

/** */
struct aice_port_api_s nds_ftdi_api = {
	/** */
	.open = nds_ftdi_open_device,
	/** */
	.close = nds_ftdi_close,
	/** */
	.reset = nds_ftdi_reset,
	/** */
	.idcode = nds_ftdi_idcode,
	/** */
	.set_jtag_clock = nds_ftdi_set_jtag_clock,
	/** */
	.assert_srst = aice_usb_assert_srst,
	/** */
	.state = nds_ftdi_get_state,
	/** */
	.read_edm = nds_ftdi_read_edm,
	/** */
	.write_edm = nds_ftdi_write_edm,
	/** */
	.profiling = aice_usb_profile_entry,
	/** */
	.diagnosis = aice_usb_do_diagnosis,
	/** */
	.pnds32 = &aice_nds32_ftdi,
};


//extern int aice_set_write_pins_support(uint32_t if_support);
int nds_ftdi_get_info(void)
{
	if (aice_read_ctrl(AICE_READ_CTRL_ICE_CONFIG, &aice_ice_config) != ERROR_OK)
		return ERROR_FAIL;
	LOG_INFO("hardware features: 0x%08x", aice_ice_config);
	// check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module
	if (aice_ice_config & (0x01 << 6))
		aice_sdm_support = 1;

	if ((aice_ice_config & (0x01 << 30)) == 0) {
		aice_set_write_pins_support(0);
	}
	else {
		aice_set_write_pins_support(1);
	}

	if ((aice_ice_config & (0x01 << 4)) == 0 )
		aice_efreq_support = 0;
	else
		aice_efreq_support = 1;

	return ERROR_OK;
}

struct aice_port_api_s aice_vendor_api = {
	/** */
	.open = NULL,
	/** */
	.close = NULL,
	/** */
	.reset = NULL,
	/** */
	.idcode = NULL,
	/** */
	.set_jtag_clock = NULL,
	/** */
	.assert_srst = NULL,
	/** */
	.state = NULL,
	/** */
	.read_edm = NULL,
	/** */
	.write_edm = NULL,
	/** */
	.profiling = NULL,
	/** */
	.diagnosis = NULL,
	/** */
	.pnds32 = NULL,
};

