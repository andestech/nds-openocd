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
#include "aice_port.h"
#include "aice_apis.h"
#include "aice_jdp.h"
#include "aice_usb_pack_format.h"
#include "target/nds32_log.h"
#include <helper/time_support.h>

#define EDMv311_STANDBY_MODE   1

#define DIMBR_DEFAULT (0xFFFF0000u)
uint32_t DIMBR_D4 = DIMBR_DEFAULT;
struct aice_nds32_info core_info[AICE_MAX_NUM_CORE];
uint32_t aice_ice_config = 0;
uint32_t aice_hardware_version = 0;
uint32_t aice_firmware_version = 0;
uint32_t aice_fpga_version = 0;
uint32_t aice_batch_data_buf1_size = 0;
uint32_t force_edm_v3 = 1;
uint32_t aice_packet_append_size = 0;
uint32_t aice_skip_tckscan_before_restart = 1;

static uint32_t aice_dis_DEH_SEL = 0;
extern uint32_t aice_no_crst_detect;
extern int vid_pid_array_top;
extern uint32_t nds32_reset_halt_as_init;
extern unsigned int aice_set_clk_first;
extern char *nds32_edm_passcode_init;
extern uint32_t aice_usb_pack_command;
struct nds32_edm_operation nds32_edm_ops[64];
uint32_t nds32_edm_ops_num = 0;
uint32_t force_to_v3 = 0;
extern int nds32_redirect_edm_v2(struct target *target);

char *custom_srst_script;
char *custom_trst_script;
char *custom_restart_script;
char *custom_initial_script = NULL;
struct target g_default_target;
struct jtag_tap g_default_tap;

/* SDM(System_Debug_Module) apis */
uint32_t aice_default_use_sdm = 0, aice_current_use_sdm = 0;
uint32_t aice_sdm_support = 0, aice_sdm_support_ena = 0;
uint32_t aice_sdm_cfg_value = 0;
uint32_t aice_sdm_parallel_debug_bus = 0;
int aice_sdm_direct_select(uint32_t coreid);
int aice_sdm_daisy_chain(void);
int aice_sdm_read_misc(uint32_t coreid, uint32_t address, uint32_t *pReadData);
int aice_sdm_write_misc(uint32_t coreid, uint32_t address, uint32_t val);
int aice_sdm_scan_chain(uint32_t *idcode, uint8_t *num_of_idcode);
int aice_sdm_check_powered_down(uint32_t coreid);
int nds_freerun_all_targets(void);
extern int nds_sdm_check_all_powered_down(uint32_t coreid);
extern int nds_sdm_only_active_one_tap(uint32_t coreid);
extern uint32_t nds32_sdm_direct_select;

int aice_xread(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr)
{
	return aice_port->api->pnds32->xread(lo_addr, hi_addr, pGetData, num_of_words, attr);
}

int aice_xwrite(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr)
{
 	return aice_port->api->pnds32->xwrite(lo_addr, hi_addr, pSetData, num_of_words, attr);
}

int aice_write_ctrl(uint32_t address, uint32_t WriteData)
{
	return aice_port->api->pnds32->write_ctrl(address, WriteData);
}

int aice_read_ctrl(uint32_t address, uint32_t *pReadData)
{
	return aice_port->api->pnds32->read_ctrl(address, pReadData);
}

int aice_read_edmsr(uint32_t coreid, uint32_t address, uint32_t *pReadData)
{
	aice_sdm_direct_select(coreid);
	return aice_port->api->read_edm(coreid, JDP_R_DBG_SR, address, (uint32_t*)pReadData, 1);
}

int aice_write_edmsr(uint32_t coreid, uint32_t address, uint32_t val)
{
	aice_sdm_direct_select(coreid);
	uint32_t WriteData = val;
	return aice_port->api->write_edm(coreid, JDP_W_DBG_SR, address, (uint32_t*)&WriteData, 1);
}

int aice_read_edmmisc(uint32_t coreid, uint32_t address, uint32_t *pReadData)
{
	aice_sdm_direct_select(coreid);
	return aice_port->api->read_edm(coreid, JDP_R_MISC_REG, address, (uint32_t*)pReadData, 1);
}

/* helper functions */
static struct aice_port_s *target_to_aice(struct target *target)
{
	assert(target != NULL);
	return target->tap->priv;
}

struct target* coreid_to_target(uint32_t coreid)
{
	struct target *target;
	uint32_t compare_coreid;

	for (target = all_targets; target; target = target->next) {
		//if ((uint32_t)target->tap->abs_chain_position == coreid)
		//	break;
		struct aice_port_s *aice = target_to_aice(target);
		if (aice != NULL) {
			compare_coreid = aice->coreid;
		} else {
			compare_coreid = target->tap->abs_chain_position;
		}

		if (coreid == compare_coreid)
			break;
	}

	return target;
}

uint32_t target_to_coreid(struct target* target)
{
	struct aice_port_s *aice = target_to_aice(target);
	if (aice != NULL)
		return aice->coreid;
	//LOG_DEBUG("target->tap->abs_chain_position: %d", target->tap->abs_chain_position);
	return target->tap->abs_chain_position;
}

int aice_write_edm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	uint32_t coreid = target_to_coreid(target);
	aice_sdm_direct_select(coreid);
	return aice_port->api->write_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_read_edm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	uint32_t coreid = target_to_coreid(target);
	aice_sdm_direct_select(coreid);
	return aice_port->api->read_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_write_sdm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	uint32_t coreid = target_to_coreid(target);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	return aice_port->api->write_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_read_sdm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	uint32_t coreid = target_to_coreid(target);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	return aice_port->api->read_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_write_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	aice_sdm_direct_select(coreid);
	return aice_port->api->write_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_read_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words)
{
	aice_sdm_direct_select(coreid);
	return aice_port->api->read_edm(coreid, JDPInst, address, pEDMData, num_of_words);
}

int aice_write_dtr(struct target *target, uint32_t val)
{
	uint32_t WriteData = val, value_edmsw = 0;
	uint32_t append_mode = 0;

	if (aice_usb_pack_command != 0) {
		if ((aice_usb_pack_command == 2) &&
			(aice_get_command_mode() == AICE_COMMAND_MODE_PACK)) {
			/* without DBGER.DPED checking */
			append_mode = 1;
		} else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		}
		int result = aice_write_edm(target, JDP_W_DTR, 0, (uint32_t*)&WriteData, 1);
		if (result != ERROR_OK)
			return ERROR_FAIL;
		result = aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		uint32_t d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);  // WRITE_DTR
		uint32_t read1_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_EDMSR

		if (append_mode == 1) {
			aice_packet_append_size += d2h_size;
			return ERROR_OK;
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[256];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], d2h_size);

		unsigned char *pread1_byte = (unsigned char *)&usbin_data[read1_byte_offset];
		value_edmsw = (unsigned char)pread1_byte[3];
		value_edmsw |= ((unsigned char)pread1_byte[2] << 8);
		value_edmsw |= ((unsigned char)pread1_byte[1] << 16);
		value_edmsw |= ((unsigned char)pread1_byte[0] << 24);
		LOG_DEBUG("EDMSW: 0x%x", value_edmsw);
		if ((value_edmsw & NDS_EDMSW_RDV) == 0) {
			return ERROR_FAIL;
		}
		return result;
	} else {
		int result = aice_write_edm(target, JDP_W_DTR, 0, (uint32_t*)&WriteData, 1);
		if (result != ERROR_OK)
			return ERROR_FAIL;
		result = aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		if ((value_edmsw & NDS_EDMSW_RDV) == 0) {
			return ERROR_FAIL;
		}
		return result;
	}
}

int aice_read_dtr(struct target *target, uint32_t *pReadData)
{
	uint32_t append_mode = 0;

	if (aice_usb_pack_command != 0) {
		uint32_t value_dtr = 0, value_edmsw = 0;
		if ((aice_usb_pack_command == 2) &&
			(aice_get_command_mode() == AICE_COMMAND_MODE_PACK)) {
			/* without DBGER.DPED checking */
			append_mode = 1;
		} else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		}

		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		aice_read_edm(target, JDP_R_DTR, 0, &value_dtr, 1);
		uint32_t read1_byte_offset = 4;
		uint32_t d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);  // READ_EDMSR
		uint32_t read2_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);          // READ_DTR

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
		if ((value_edmsw & NDS_EDMSW_WDV) == 0) {
			return ERROR_FAIL;
		}
		value_dtr = (unsigned char)pread2_byte[3];
		value_dtr |= ((unsigned char)pread2_byte[2] << 8);
		value_dtr |= ((unsigned char)pread2_byte[1] << 16);
		value_dtr |= ((unsigned char)pread2_byte[0] << 24);
		*pReadData = value_dtr;
		LOG_DEBUG("DTR:0x%x, EDMSW: 0x%x", value_dtr, value_edmsw);
		return ERROR_OK;
	} else {
		uint32_t value_edmsw = 0;

		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, &value_edmsw, 1);
		if ((value_edmsw & NDS_EDMSW_WDV) == 0) {
			return ERROR_FAIL;
		}
		int result = aice_read_edm(target, JDP_R_DTR, 0, (uint32_t*)pReadData, 1);
		return result;
	}
}

int aice_write_misc(struct target *target, uint32_t address, uint32_t val)
{
	uint32_t WriteData = val;
	return aice_write_edm(target, JDP_W_MISC_REG, address, (uint32_t*)&WriteData, 1);
}

int aice_read_misc(struct target *target, uint32_t address, uint32_t *pReadData)
{
	return aice_read_edm(target, JDP_R_MISC_REG, address, (uint32_t*)pReadData, 1);
}

int aice_write_dim(struct target *target, uint32_t *pWriteData, uint32_t num_of_words)
{
	return aice_write_edm(target, JDP_W_DIM, 0, (uint32_t*)pWriteData, num_of_words);
}

int aice_do_execute(struct target *target)
{
	uint32_t WriteData = 0;
	return aice_write_edm(target, JDP_W_EXECUTE, 0, (uint32_t*)&WriteData, 1);
}

/* SDM(System_Debug_Module) apis */
int aice_sdm_direct_select(uint32_t coreid)
{
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return ERROR_FAIL;

	if (aice_current_use_sdm == 0)
		return ERROR_FAIL;

	if (coreid > 15)
		return ERROR_FAIL;

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, (NDS_SDM_SELECT_DIRECT|(0x01 << coreid)));
	if (aice_port->type == AICE_PORT_FTDI) {
		nds_sdm_only_active_one_tap(coreid);
		nds32_sdm_direct_select = 1;
		struct jtag_tap *tap = jtag_tap_next_enabled(NULL);
		LOG_DEBUG("active_tap = %d", tap->abs_chain_position);
	}
	return ERROR_OK;
}

int aice_sdm_daisy_chain(void)
{
	aice_current_use_sdm = 0;
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return ERROR_FAIL;
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	return ERROR_OK;
}

int aice_sdm_read_misc(uint32_t coreid, uint32_t address, uint32_t *pReadData)
{
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return ERROR_FAIL;

	return aice_port->api->read_edm(coreid, JDP_R_MISC_REG, address, pReadData, 1);
}

int aice_sdm_write_misc(uint32_t coreid, uint32_t address, uint32_t val)
{
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return ERROR_FAIL;

	uint32_t WriteData = val;
	return aice_port->api->write_edm(coreid, JDP_W_MISC_REG, address, &WriteData, 1);
}

int aice_sdm_scan_chain(uint32_t *p_idcode, uint8_t *num_of_idcode)
{
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return ERROR_FAIL;

	uint32_t idcode[16], coreid, nds_idcode = 0;
	unsigned char length = 0;

	idcode[0] = 0x0;
	// switch to SDM registers
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	int retval = aice_port->api->idcode(&idcode[0], &length);

	if ((retval != ERROR_OK) || (length == 0xFF)) {
		LOG_ERROR("No target connected");
	} else if (idcode[0] == NDS_SDM_TAPID) {
		aice_sdm_read_misc(0, NDS_SDM_MISC_SDM_CFG, &aice_sdm_cfg_value);
		if (aice_sdm_cfg_value & NDS_SDM_MISC_SDM_CFG_SDM_PARALLEL_DBG_BUS)
			aice_sdm_parallel_debug_bus = 1;

		uint32_t sdm_core_nums = (aice_sdm_cfg_value & NDS_SDM_MISC_SDM_CFG_SDM_PROCESSOR_NUM_MASK);
		if (sdm_core_nums) {
			for (coreid = 0; coreid < sdm_core_nums; coreid++) {
				//aice_sdm_direct_select(coreid);
				aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, (NDS_SDM_SELECT_DIRECT|(0x01 << coreid)));
				retval = aice_port->api->idcode(&idcode[0], &length);
				if (retval == ERROR_OK) {
					if (idcode[0] != 0) {
						nds_idcode = idcode[0];
						//p_idcode[coreid] = nds_idcode;
						LOG_DEBUG("nds_idcode = 0x%x", nds_idcode);
					}
				}
			}

			for (coreid = 0; coreid < sdm_core_nums; coreid++) {
				p_idcode[coreid] = nds_idcode;
				LOG_DEBUG("*p_idcode[%d] = 0x%x", coreid, p_idcode[coreid]);
			}
			*num_of_idcode = sdm_core_nums;
		}
		LOG_DEBUG("aice_sdm_cfg_value=0x%x", aice_sdm_cfg_value);
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
		return ERROR_OK;
	} else {
		LOG_DEBUG("NOT SDM");
	}
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	return ERROR_FAIL;
}

int aice_sdm_check_powered_down(uint32_t coreid)
{
	/* check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module */
	if ((aice_sdm_support == 0) || (aice_sdm_support_ena == 0))
		return 0;

	if (aice_port->type == AICE_PORT_FTDI) {
		return nds_sdm_check_all_powered_down(coreid);
	}
	/* Clear the PROCESSOR_STATUS.EVER_DISABLED value
	   Read PROCESSOR_STATUS
	   If PROCESSOR_STATUS.EVER_DISABLED is set, the processor is powered down. */
	//aice_sdm_direct_select(coreid);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM | (0x01 << coreid));
	//aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, (NDS_SDM_SELECT_DIRECT|(0x01 << coreid)));

	uint32_t core_stat = 0;
	aice_sdm_write_misc(coreid, NDS_SDM_MISC_PROCESSOR_STATUS, NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED);
	int retval = aice_sdm_read_misc(coreid, NDS_SDM_MISC_PROCESSOR_STATUS, &core_stat);
	if (retval == ERROR_OK) {
		if (core_stat & NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED)
			return 1;
	}
	return 0;
}


static int check_suppressed_exception(struct target *target, uint32_t dbger_value)
{
	/* the default value of handling_suppressed_exception is false */
	static bool handling_suppressed_exception;

	if (handling_suppressed_exception)
		return ERROR_OK;

	if ((dbger_value & NDS_DBGER_ALL_SUPRS_EX) == NDS_DBGER_ALL_SUPRS_EX) {
		if (target == NULL)
			return ERROR_OK;
		struct nds32 *nds32 = target_to_nds32(target);
		struct nds32_misc_config *misc_config = &(nds32->misc_config);
		uint32_t ir4_value = 0;
		uint32_t ir6_value = 0;

		NDS32_LOG(NDS32_ERRMSG_TARGET_EXCEPTION);
		handling_suppressed_exception = true;

		if (!misc_config->mcu)
			aice_read_reg(target, IR4, &ir4_value);
		/* Clear IR6.SUPRS_EXC, IR6.IMP_EXC */
		aice_read_reg(target, IR6, &ir6_value);
		/*
		 * For MCU version(MSC_CFG.MCU == 1) like V3m
		 *  | SWID[30:16] | Reserved[15:10] | SUPRS_EXC[9]  | IMP_EXC[8]
		 *  |VECTOR[7:5] | INST[4] | Exc Type[3:0] |
		 *
		 * For non-MCU version(MSC_CFG.MCU == 0) like V3
		 *  | SWID[30:16] | Reserved[15:14] | SUPRS_EXC[13] | IMP_EXC[12]
		 *  | VECTOR[11:5] | INST[4] | Exc Type[3:0] |
		 */
		LOG_INFO("EVA: 0x%08x", ir4_value);
		LOG_INFO("ITYPE: 0x%08x", ir6_value);
		LOG_INFO("mcu: %x", misc_config->mcu);

		if (misc_config->mcu)
			ir6_value = ir6_value & (~0x300); /* for MCU */
		else
			ir6_value = ir6_value & (~0x3000); /* for non-MCU */
		aice_write_reg(target, IR6, ir6_value);

		handling_suppressed_exception = false;
	}

	return ERROR_OK;
}

static int check_privilege(struct target *target, uint32_t dbger_value)
{
	if ((dbger_value & NDS_DBGER_ILL_SEC_ACC) == NDS_DBGER_ILL_SEC_ACC) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_ILL_ACCESS);

		/* Clear DBGER.ILL_SEC_ACC */
		if (aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_ILL_SEC_ACC) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int check_processor_status(uint32_t target_id)
{
	uint32_t dbger_value=0;
	aice_read_edmmisc(target_id, NDS_EDM_MISC_DBGER, &dbger_value);

	if ((dbger_value & (NDS_DBGER_STANDBY|NDS_DBGER_GLOBAL_STALL|NDS_DBGER_MEM_PEND)) == 0) {
		return ERROR_OK;
	}

	uint32_t value_probe=0;
	aice_read_edmmisc(target_id, NDS_EDM_MISC_EDM_PROBE, &value_probe);
	value_probe &= ~(0x01);

	//aice_read_edmmisc(target_id, NDS_EDM_MISC_DBGER, &dbger_value);
	if (dbger_value & NDS_DBGER_STANDBY) {
		NDS32_LOG("<-- TARGET ERROR! The processor is in the standby mode, PC=0x%x -->", value_probe);
		while(1) {
			alive_sleep(10);
			aice_read_edmmisc(target_id, NDS_EDM_MISC_DBGER, &dbger_value);
			if ((dbger_value & NDS_DBGER_STANDBY) == 0)
				break;
		}
	}
	else if (dbger_value & NDS_DBGER_GLOBAL_STALL) {
		LOG_DEBUG("TARGET ERROR! The processor is stalled by the global_stall signal, PC=0x%x", value_probe);
		//return ERROR_FAIL;
	}
	else if (dbger_value & NDS_DBGER_MEM_PEND) {
		LOG_DEBUG("TARGET ERROR! The processor has pending memory accesses, PC=0x%x", value_probe);
		//return ERROR_FAIL;
	}
	return ERROR_OK;
}

int aice_check_dbger(struct target *target, uint32_t expect_status)
{
	long long then = 0;
	uint32_t i = 0;
	uint32_t value_dbger = 0;

	if (target == NULL)
		return ERROR_FAIL;
	while (1) {
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);

		if ((value_dbger & expect_status) == expect_status) {
			if (ERROR_OK != check_suppressed_exception(target, value_dbger))
				return ERROR_FAIL;
			if (ERROR_OK != check_privilege(target, value_dbger))
				return ERROR_FAIL;
			return ERROR_OK;
		}
		/* if target code freerun to trigger reset&hold */
		if ((value_dbger & (NDS_DBGER_CRST | NDS_DBGER_DEX)) ==
			(NDS_DBGER_CRST | NDS_DBGER_DEX)) {
			LOG_ERROR("target code freerun to trigger reset&hold");
			return ERROR_OK;
		}

		if ((i % 30) == 0)
			keep_alive();

		if (i == 0)
			then = timeval_ms();
		else {
			if ((timeval_ms() - then) > aice_count_to_check_dbger) {
				LOG_ERROR("Timeout (%dms) waiting for $DBGER status "
						"being 0x%08x", aice_count_to_check_dbger, expect_status);
				if ((expect_status & NDS_DBGER_CRST) && !(value_dbger & NDS_DBGER_CRST))
					LOG_ERROR("nSRST did not happen or TRST was	triggered unexpectedly");
				if ((expect_status & NDS_DBGER_DEX) && !(value_dbger & NDS_DBGER_DEX))
					LOG_ERROR("debug exception is not taken");
				return ERROR_FAIL;
			}
		}
		alive_sleep(1);
		i++;
	}

	return ERROR_FAIL;
}

#define CHK_EDMCTL_TIMEOUT   5000
static uint32_t aice_count_to_check_edmctl = CHK_EDMCTL_TIMEOUT;

int aice_check_edmctl(struct target *target, uint32_t expect_status)
{
	long long then = 0;
	uint32_t i = 0;
	uint32_t edm_ctl_value = 0;

	while (1) {
		if (aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDM_CTL, (uint32_t*)&edm_ctl_value, 1) != ERROR_OK)
			return ERROR_FAIL;
		if ((edm_ctl_value & expect_status) == expect_status)
			return ERROR_OK;

		if ((i % 30) == 0)
			keep_alive();

		if (i == 0)
			then = timeval_ms();
		else {
			if ((timeval_ms() - then) > aice_count_to_check_edmctl) {
				LOG_DEBUG("Timeout (%dms) waiting for $EDMCTL status "
						"being 0x%08x", aice_count_to_check_edmctl, expect_status);
				if ((expect_status & NDS_EDMCTL_DEH_SEL) && !(edm_ctl_value & NDS_EDMCTL_DEH_SEL))
					LOG_DEBUG("debug memory is not activated");
				return ERROR_FAIL;
			}
		}
		alive_sleep(1);
		i++;
	}

	return ERROR_FAIL;
}

int aice_execute_dim(struct target *target, uint32_t *insts, uint32_t n_inst)
{
	LOG_DEBUG("%s", __func__);
	uint32_t append_mode = 0;

	if (aice_usb_pack_command != 0) {
		if ((aice_usb_pack_command == 2) &&
				(aice_get_command_mode() == AICE_COMMAND_MODE_PACK)) {
			/* without DBGER.DPED checking */
			append_mode = 1;
		} else {
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		}
	}
	/** fill DIM */
	if (aice_write_dim(target, insts, n_inst) != ERROR_OK)
		return ERROR_FAIL;

	/** clear DBGER.DPED */
	if (aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_DPED) != ERROR_OK)
		return ERROR_FAIL;

	/** execute DIM */
	if (aice_do_execute(target) != ERROR_OK)
		return ERROR_FAIL;

	if (aice_usb_pack_command != 0) {
		uint32_t value_dbger = 0;
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);

		uint32_t d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB);  // WRITE_DIM
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB); // WRITE_MISC
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMB); // EXECUTE
		uint32_t read1_byte_offset = d2h_size + 4;
		d2h_size += aice_get_usb_cmd_size(AICE_CMDTYPE_DTHMA);  // READ_MISC

		if (append_mode == 1) {
			aice_packet_append_size += d2h_size;
			return ERROR_OK;
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[256];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], d2h_size);

		unsigned char *pread1_byte = (unsigned char *)&usbin_data[read1_byte_offset];
		value_dbger = (unsigned char)pread1_byte[3];
		value_dbger |= ((unsigned char)pread1_byte[2] << 8);
		value_dbger |= ((unsigned char)pread1_byte[1] << 16);
		value_dbger |= ((unsigned char)pread1_byte[0] << 24);
		LOG_DEBUG("%s, value_dbger=0x%x", __func__, value_dbger);
		if ((value_dbger & NDS_DBGER_ALL_SUPRS_EX) == NDS_DBGER_ALL_SUPRS_EX) {
			check_suppressed_exception(target, value_dbger);
			//return ERROR_FAIL;
		}
		if (value_dbger & NDS_DBGER_DPED) {
			return ERROR_OK;
		}
	}

	/** read DBGER.DPED */
	if (aice_check_dbger(target, NDS_DBGER_DPED) != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_EXEC_DIM,
				insts[0],
				insts[1],
				insts[2],
				insts[3]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int aice_init_edm_registers(struct target *target, bool clear_dex_use_psw)
{
	uint32_t coreid = target_to_coreid(target);
	/* enable DEH_SEL & MAX_STOP & V3_EDM_MODE & DBGI_MASK */
#if BACKUP_EDM_CTRL
	uint32_t host_edm_ctl = core_info[coreid].edm_ctl_backup | NDS_EDMCTL_DEH_SEL
																			| NDS_EDMCTL_MAX_STOP
																			| NDS_EDMCTL_LDBGACKM | NDS_EDMCTL_LDBGIM
																			| NDS_EDMCTL_DBGACKM | NDS_EDMCTL_DBGIM;
#else
	uint32_t host_edm_ctl = NDS_EDMCTL_DEH_SEL
																			| NDS_EDMCTL_MAX_STOP
																			| NDS_EDMCTL_LDBGACKM | NDS_EDMCTL_LDBGIM
																			| NDS_EDMCTL_DBGACKM | NDS_EDMCTL_DBGIM;
#endif
	if (force_edm_v3)
		host_edm_ctl |= NDS_EDMCTL_EDM_MODE;
	if (clear_dex_use_psw)
		/* After entering debug mode, OpenOCD may set
		 * DEX_USE_PSW accidentally through backup value
		 * of target EDM_CTL.
		 * So, clear DEX_USE_PSW by force. */
		host_edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;

	LOG_DEBUG("aice_init_edm_registers - EDM_CTL: 0x%08x", host_edm_ctl);

	int result = aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, host_edm_ctl);

	return result;
}

/**
 * EDM_CTL will be modified by OpenOCD as debugging. OpenOCD has the
 * responsibility to keep EDM_CTL untouched after debugging.
 *
 * There are two scenarios to consider:
 * 1. single step/running as debugging (running under debug session)
 * 2. detached from gdb (exit debug session)
 *
 * So, we need to bakcup EDM_CTL before halted and restore it after
 * running. The difference of these two scenarios is EDM_CTL.DEH_SEL
 * is on for scenario 1, and off for scenario 2.
 */
int aice_backup_edm_registers(struct target *target)
{
#if BACKUP_EDM_CTRL
	uint32_t coreid = target_to_coreid(target);
	int result = aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDM_CTL,
			&core_info[coreid].edm_ctl_backup, 1);

	/* To call aice_backup_edm_registers() after DEX on, DEX_USE_PSW
	 * may be not correct.  (For example, hit breakpoint, then backup
	 * EDM_CTL. EDM_CTL.DEX_USE_PSW will be cleared.)  Because debug
	 * interrupt will clear DEX_USE_PSW, DEX_USE_PSW is always off after
	 * DEX is on.  It only backups correct value before OpenOCD issues DBGI.
	 * (Backup EDM_CTL, then issue DBGI actively (refer aice_halt_target())) */
	if (core_info[coreid].edm_ctl_backup & NDS_EDMCTL_DEX_USE_PSW)
		core_info[coreid].dex_use_psw_on = true;
	else
		core_info[coreid].dex_use_psw_on = false;

	LOG_DEBUG("aice_backup_edm_registers - EDM_CTL: 0x%08x, DEX_USE_PSW: %s",
			core_info[coreid].edm_ctl_backup,
			core_info[coreid].dex_use_psw_on ? "on" : "off");

	return result;
#else
	return ERROR_OK;
#endif
}

int aice_restore_edm_registers(struct target *target)
{
#if BACKUP_EDM_CTRL
	uint32_t coreid = target_to_coreid(target);
	uint32_t WriteData = core_info[coreid].edm_ctl_backup | NDS_EDMCTL_DEH_SEL;
	LOG_DEBUG("aice_restore_edm_registers -");

	/* set DEH_SEL, because target still under EDM control */
	int result = aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, WriteData);
	return result;
#else
	return ERROR_OK;
#endif
}

int aice_core_init(uint32_t coreid)
{
	//core_info[coreid].memory_select = NDS_MEMORY_SELECT_AUTO;
	core_info[coreid].core_state = AICE_TARGET_UNKNOWN;

	return ERROR_OK;
}

int aice_edm_init(uint32_t coreid)
{
	uint32_t edm_cfg_value=0;
	uint32_t edm_ctl_value=0;
	uint32_t WriteData;
	int retval;

	//dump origin/reset EDM version
	retval = aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, (uint32_t*)&edm_cfg_value);

#if EDMv311_STANDBY_MODE
	if (retval != ERROR_OK) {
		if (check_processor_status(coreid) != ERROR_OK)
			return ERROR_FAIL;
		retval = aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, &edm_cfg_value);
	}
#endif
	core_info[coreid].edm_cfg = edm_cfg_value;
	core_info[coreid].edm_version = (edm_cfg_value >> 16) & 0xFFFF;
	LOG_DEBUG("After reset, EDM_CFG = 0x%08x", edm_cfg_value);
	if( core_info[coreid].edm_version == 0x0 ) {
		/* NOT NDS32 EDM, do nothing */
		return ERROR_OK;
	}

	if (aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4) != ERROR_OK)
		return ERROR_FAIL;
	WriteData = 0;
	aice_sdm_direct_select(coreid);
	aice_port->api->write_edm(coreid, JDP_W_MISC_REG, NDS_EDM_MISC_DIMIR, (uint32_t*)&WriteData, 1);

	//TODO: change to V3 mode will change target interrupt behavior
	//  find a better way to switch to V3 mode
	/* unconditionally try to turn on V3_EDM_MODE */
	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, (uint32_t*)&edm_ctl_value);
	if (force_edm_v3)
		aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_EDM_MODE);
	core_info[coreid].edm_ctl_backup = edm_ctl_value;
	LOG_DEBUG("edm_ctl_backup = 0x%08x", core_info[coreid].edm_ctl_backup);

	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, (uint32_t*)&edm_ctl_value);
	if (edm_ctl_value != core_info[coreid].edm_ctl_backup) {
		force_to_v3 = 1;
		LOG_DEBUG("edm_ctl_value = 0x%08x, force_to_v3", edm_ctl_value);
	}
	aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_DEH_SEL);
	/* clear DBGER */
	WriteData = (NDS_DBGER_DPED | NDS_DBGER_CRST | NDS_DBGER_AT_MAX);
	aice_sdm_direct_select(coreid);
	aice_port->api->write_edm(coreid, JDP_W_MISC_REG, NDS_EDM_MISC_DBGER, (uint32_t*)&WriteData, 1);

	//if (aice_set_edm_passcode(coreid, nds32->edm_passcode) != ERROR_OK)
	//	return ERROR_FAIL;
	retval = aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, (uint32_t*)&edm_cfg_value);
	core_info[coreid].edm_cfg = edm_cfg_value;
	core_info[coreid].edm_version = (edm_cfg_value >> 16) & 0xFFFF;
	LOG_DEBUG("After reset, EDM_CFG = 0x%08x", edm_cfg_value);
	return ERROR_OK;
}

bool is_v2_edm(uint32_t coreid)
{
	if ((core_info[coreid].edm_version & 0x1000) == 0)
		return true;
	else
		return false;
}

int aice_program_edm(struct target *target, char *command_sequence)
{
	char *command_str;
	char *reg_name_0;
	char *reg_name_1;
	uint32_t data_value;
	int i;

	/* init strtok() */
	command_str = strtok(command_sequence, ";");
	if (command_str == NULL)
		return ERROR_OK;

	do {
		i = 0;
		/* process one command */
		while (command_str[i] == ' ' ||
				command_str[i] == '\n' ||
				command_str[i] == '\r' ||
				command_str[i] == '\t')
			i++;

		/* skip ' ', '\r', '\n', '\t' */
		command_str = command_str + i;

		if (strncmp(command_str, "write_misc", 10) == 0) {
			reg_name_0 = strstr(command_str, "gen_port0");
			reg_name_1 = strstr(command_str, "gen_port1");

			if (reg_name_0 != NULL) {
				data_value = strtoul(reg_name_0 + 9, NULL, 0);

				if (aice_write_misc(target, NDS_EDM_MISC_GEN_PORT0, data_value) != ERROR_OK)
					return ERROR_FAIL;

			} else if (reg_name_1 != NULL) {
				data_value = strtoul(reg_name_1 + 9, NULL, 0);

				if (aice_write_misc(target, NDS_EDM_MISC_GEN_PORT1, data_value) != ERROR_OK)
					return ERROR_FAIL;
			} else {
				LOG_ERROR("program EDM, unsupported misc register: %s", command_str);
			}
		} else {
			LOG_ERROR("program EDM, unsupported command: %s", command_str);
		}

		/* update command_str */
		command_str = strtok(NULL, ";");

	} while (command_str != NULL);

	return ERROR_OK;
}

int aice_get_info(void)
{
	//version info
	if (aice_read_ctrl(AICE_READ_CTRL_GET_HARDWARE_VERSION, &aice_hardware_version) != ERROR_OK)
		return ERROR_FAIL;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FIRMWARE_VERSION, &aice_firmware_version) != ERROR_OK)
		return ERROR_FAIL;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FPGA_VERSION, &aice_fpga_version) != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("AICE version: hw_ver = 0x%x, fw_ver = 0x%x, fpga_ver = 0x%x",
		aice_hardware_version, aice_firmware_version, aice_fpga_version);
	//hardware features info
	if (aice_read_ctrl(AICE_READ_CTRL_ICE_CONFIG, &aice_ice_config) != ERROR_OK)
		return ERROR_FAIL;
	LOG_INFO("AICE hardware features: 0x%08x", aice_ice_config);
	// check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module
	if (aice_ice_config & (0x01 << 6))
		aice_sdm_support = 1;

	//batch buffer size info
	if (aice_read_ctrl(AICE_REG_BATCH_DATA_BUFFER_1_STATE, &aice_batch_data_buf1_size) != ERROR_OK)
		return ERROR_FAIL;
	aice_batch_data_buf1_size &= 0xffffu;
	LOG_INFO("AICE batch data buffer size: %d words", aice_batch_data_buf1_size);

	return ERROR_OK;
}

int aice_edm_config(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t coreid = target_to_coreid(target);
	uint32_t edm_cfg=0;
	uint32_t edm_ctl=0;

	aice_edm_init(coreid);
	//aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, &edm_cfg);
	edm_cfg = core_info[coreid].edm_cfg;
	nds32->edm.edm_cfg_reg = edm_cfg;
	nds32->edm.version = (edm_cfg >> 16) & 0xFFFF;
	LOG_OUTPUT("EDM version 0x%04" PRIx32, nds32->edm.version);

	nds32->edm.breakpoint_num = (edm_cfg & 0x7) + 1;

	if ((nds32->edm.version & 0x1000) || (0x60 <= nds32->edm.version))
		nds32->edm.access_control = true;
	else
		nds32->edm.access_control = false;

	if ((edm_cfg >> 4) & 0x1)
		nds32->edm.direct_access_local_memory = true;
	else
		nds32->edm.direct_access_local_memory = false;

	if ((edm_cfg >> 8) & 0x1) {
		nds32->profiling_support = 1;
		nds32->edm.support_probe = true;
	}
	else
		nds32->edm.support_probe = false;

	if (nds32->edm.version <= 0x20)
		nds32->edm.direct_access_local_memory = false;

	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, &edm_ctl);
	if (edm_ctl & NDS_EDMCTL_MAX_STOP)
		nds32->edm.support_max_stop = true;
	else
		nds32->edm.support_max_stop = false;

	if (aice_ice_config & AICE_ICE_CONFIG_BATCH_SUPPORT) {
		nds32->profiling_support = 1;
	}

	return ERROR_OK;
}

uint32_t aice_curr_command_mode = 0;
//extern unsigned int usb_out_packets_buffer_length;
//extern unsigned int usb_in_packets_buffer_length;
extern unsigned int aice_keep_usb_packet_append;
extern unsigned int aice_keep_usb_in_curr_packets_length;

int aice_set_command_mode(uint32_t command_mode)
{
	if ((aice_port->api->pnds32 == NULL) ||
		(aice_port->api->pnds32->set_command_mode == NULL)) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}
	if (command_mode == AICE_COMMAND_MODE_PACK) {
		usb_out_packets_buffer_length = 0;
		usb_in_packets_buffer_length = 0;
		aice_keep_usb_packet_append = 0;
		aice_keep_usb_in_curr_packets_length = 0;
	}
	int retval = aice_port->api->pnds32->set_command_mode(command_mode);
	aice_curr_command_mode = command_mode;
	return retval;
}

uint32_t aice_get_command_mode(void)
{
	return aice_curr_command_mode;
}

int aice_monitor_command(struct target *target, uint32_t nCmd, char **p_command, int *p_len, char **p_ret_data)
{
	if ((aice_port->api->pnds32 == NULL) ||
		(aice_port->api->pnds32->monitor_command == NULL)) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice_port->api->pnds32->monitor_command(nCmd, p_command, p_len, p_ret_data);
}

int aice_packbuffer_read(struct target *target, unsigned char *pReadData, unsigned int num_of_bytes)
{
	if ((aice_port->api->pnds32 == NULL) ||
		(aice_port->api->pnds32->pack_buffer_read == NULL)) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice_port->api->pnds32->pack_buffer_read(pReadData, num_of_bytes);
}

int get_debug_reason(struct target *target, uint32_t *reason)
{
	struct nds32 *nds32 = target_to_nds32(target);

	uint32_t coreid = target_to_coreid(target);
	if (is_v2_edm(coreid) == false) {
		nds32->get_debug_reason(nds32, reason);
		//LOG_DEBUG("V3 without backup R0/R1");
		return ERROR_OK;
	}
	aice_backup_edm_registers(target);
	/* backup r0 & r1, for edm-v2 only */
	aice_backup_tmp_registers(target);

	nds32->get_debug_reason(nds32, reason);

	/* restore r0 & r1 */
	aice_restore_tmp_registers(target);
	aice_restore_edm_registers(target);
	return ERROR_OK;
}

int aice_program_edm_port(struct target *target)
{
	char command_str[33];
	uint32_t code, i;

	if (nds32_edm_ops_num > 0) {
		const char *reg_name;
		for (i = 0 ; i < nds32_edm_ops_num ; i++) {
			code = nds32_edm_ops[i].value;
			if (nds32_edm_ops[i].reg_no == 6)
				reg_name = "gen_port0";
			else if (nds32_edm_ops[i].reg_no == 7)
				reg_name = "gen_port1";
			else
				return ERROR_FAIL;

			sprintf(command_str, "write_misc %s 0x%x;", reg_name, code);
			if (ERROR_OK != aice_program_edm(target, command_str))
				return ERROR_FAIL;
		}
	}
	return ERROR_OK;	
}

int aice_set_edm_passcode(struct target *target, char *edm_passcode)
{
	uint32_t passcode_length;
	uint32_t i;
	uint32_t copy_length;
	uint32_t code;
	char code_str[9];
	uint32_t coreid = target_to_coreid(target);

	if (nds32_edm_ops_num > 0) {
		return aice_program_edm_port(target);
	}

	if (!edm_passcode)
		return ERROR_OK;

	passcode_length = strlen(edm_passcode);
	for (i = 0; i < passcode_length; i += 8) {
		if (passcode_length - i < 8)
			copy_length = passcode_length - i;
		else
			copy_length = 8;

		strncpy(code_str, edm_passcode + i, copy_length);
		code_str[copy_length] = '\0';
		code = strtoul(code_str, NULL, 16);
		if (aice_write_misc(target,	NDS_EDM_MISC_GEN_PORT0, code) != ERROR_OK)
			return ERROR_FAIL;
	}

	core_info[coreid].edm_passcode = edm_passcode;
	return ERROR_OK;
}

int aice_diagnosis(struct target *target)
{
	if (aice_port->api->diagnosis == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}
	LOG_DEBUG("enter do_diagnosis...");
	aice_port->api->diagnosis(target);
	LOG_DEBUG("exit do_diagnosis...");
	nds_freerun_all_targets();
	aice_close_device();
	exit(-1);
	return ERROR_FAIL;
}

int aice_set_jtag_clock(uint32_t a_clock)
{
	if (aice_port->api->set_jtag_clock == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice_port->api->set_jtag_clock(a_clock);
}

int aice_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	if (aice_port->api->idcode == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	int retval;
	unsigned char length = 0;

	if (aice_default_use_sdm == 1) {
		retval = aice_sdm_scan_chain(idcode, &length);
		if (retval != ERROR_OK) {
			retval = aice_port->api->idcode(idcode, &length);
		} else {
			aice_current_use_sdm = 1;
			aice_usb_pack_command = 0;
			LOG_DEBUG("default use sdm mode");
		}
	} else {
		retval = aice_port->api->idcode(idcode, &length);
		if (retval != ERROR_OK) {
			retval = aice_sdm_scan_chain(idcode, &length);
			if (retval == ERROR_OK) {
				aice_current_use_sdm = 1;
				aice_usb_pack_command = 0;
				LOG_DEBUG("use sdm mode");
			}
		}
	}
	*num_of_idcode = length;

	/* ========================= */
	/* = for testing =========== */
	/* ========================= */
	//*idcode++ = 0x00001234;
	//*idcode++ = 0x00005678;
	//*idcode++ = 0x1000063d;
	//*idcode++ = 0x1000063d;
	//*num_of_idcode = 4;
	/* ========================= */

	if (ERROR_OK == retval) {
		for (int i = 0; i < *num_of_idcode; i++) {
				aice_core_init(i);
		}
	}
	return retval;
}

int aice_write_debug_reg(struct target *target, uint32_t addr, uint32_t val)
{
	uint32_t coreid = target_to_coreid(target);

	if (AICE_TARGET_HALTED == core_info[coreid].core_state) {
		if (NDS_EDM_SR_EDM_DTR == addr) {
			core_info[coreid].host_dtr_backup = val;
			core_info[coreid].edmsw_backup |= 0x2;
			core_info[coreid].host_dtr_valid = true;
		}
	}
	uint32_t WriteData = val;
	return aice_write_edm(target, JDP_W_DBG_SR, addr, (uint32_t*)&WriteData, 1);
}

int aice_read_debug_reg(struct target *target, uint32_t addr, uint32_t *val)
{
	uint32_t coreid = target_to_coreid(target);

	if (AICE_TARGET_HALTED == core_info[coreid].core_state) {
		if (NDS_EDM_SR_EDMSW == addr) {
			*val = core_info[coreid].edmsw_backup;
		} else if (NDS_EDM_SR_EDM_DTR == addr) {
			if (core_info[coreid].target_dtr_valid) {
				/* if EDM_DTR has read out, clear it. */
				*val = core_info[coreid].target_dtr_backup;
				core_info[coreid].edmsw_backup &= (~0x1);
				core_info[coreid].target_dtr_valid = false;
			} else {
				*val = 0;
			}
		}
	}
	return aice_read_edm(target, JDP_R_DBG_SR, addr, (uint32_t*)val, 1);
}

static int aice_usb_pending_DBGI(struct target *target)
{
	uint32_t value_dbger=0, val_ir0=0;
	uint32_t instructions[4] = {NOP, NOP, NOP, IRET };
	int result = ERROR_OK;

	aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);
	if (!(value_dbger & NDS_DBGER_DEX))
	{
		/* backup r0 & r1 */
		aice_backup_tmp_registers(target);
		aice_read_reg(target, IR0, &val_ir0);
		/* restore r0 & r1 */
		aice_restore_tmp_registers(target);

		LOG_DEBUG("aice_usb_pending_DBGI, val_ir0=%x", val_ir0);
		/* DBGI command is pending when PSW.DEX is set */
		if(val_ir0 & 0x400) /* read PSW-DEX */
		{
			// clear DBGER
			//aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CLEAR_ALL);
			result = aice_execute_dim(target, instructions, 4);
			if(result != ERROR_OK)
				printf("aice_usb_pending_DBGI-nop \n");
		}
	}
	return result;
}

static int aice_usb_check_DBGSPL(struct target *target)
{
	long long then = 0;
	uint32_t value_edmsw = 0, value_dbger = 0, i = 0;

	/* check Security Privilege Level (DBGSPL) */
	aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, (uint32_t*)&value_edmsw, 1);

	if ((value_edmsw & 0x30000) == 0)
		return ERROR_OK;

	while (1) {
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);
		if (value_dbger & (NDS_DBGER_HDBG | NDS_DBGER_DEX)) {
			return ERROR_OK;
		}
		if (i == 0)
			then = timeval_ms();
		else {
			if ((timeval_ms() - then) > aice_count_to_check_dbger)
				return ERROR_FAIL;
		}
		alive_sleep(1);
		i++;
	}
	return ERROR_FAIL;
}

int aice_reset_target(struct target *target, uint32_t reset_cmd)
{
	if (aice_port->api->assert_srst == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}
	return aice_port->api->assert_srst(target, reset_cmd);
}

int aice_halt_target(struct target *target)
{
	uint32_t coreid = target_to_coreid(target);
	LOG_DEBUG("=== %s ===", __func__);
	aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4);
	/** Clear EDM_CTL.DBGIM & EDM_CTL.DBGACKM */
	uint32_t clear_edm_ctl_DBGIM_DBGACKM = 0;
	uint32_t edm_ctl_value = 0;

	if (core_info[coreid].core_state == AICE_TARGET_HALTED) {
		LOG_DEBUG("aice_halt_target check halted");
		return ERROR_OK;
	}

	/** backup EDM registers */
	aice_backup_edm_registers(target);

	uint32_t reg_dbger = 0;
	core_info[coreid].debug_under_dex_on = false;

	aice_read_misc(target, NDS_EDM_MISC_DBGER, &reg_dbger);
	if (reg_dbger & NDS_DBGER_AT_MAX) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_INTLVL);
	}
#if EDMv311_STANDBY_MODE
	else if (reg_dbger & NDS_DBGER_STANDBY) {
		goto skip_Clear_EDM_CTL_DBGIM;
	}
#endif

	/** init EDM for host debugging */
	/** no need to clear dex_use_psw, because dbgi will clear it */
	aice_init_edm_registers(target, false);
	aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);

	if (edm_ctl_value & 0x3) {
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl_value &
			~(NDS_EDMCTL_DBGIM | NDS_EDMCTL_DBGACKM));
		clear_edm_ctl_DBGIM_DBGACKM = 1;
	}

skip_Clear_EDM_CTL_DBGIM:
	//1. issue DBGI unconditional
	aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0);

#if EDMv311_STANDBY_MODE
	if (check_processor_status(coreid) != ERROR_OK)
		return ERROR_FAIL;
#endif
	/* check Security Privilege Level (DBGSPL) */
	if (aice_usb_check_DBGSPL(target) != ERROR_OK)
		return ERROR_FAIL;

	//2. wait for any debug exception
#if 0
	do {
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &reg_dbger);
		if ((reg_dbger & NDS_DBGER_DEX) == 0)
			aice_usb_pending_DBGI(target);
	} while(0 == (reg_dbger & NDS_DBGER_DEX));
#else
	do {
		aice_read_misc(target, NDS_EDM_MISC_DBGER, &reg_dbger);
		if (reg_dbger & NDS_DBGER_DEX) {
			break;
		}
		else if (reg_dbger & NDS_DBGER_HDBG) {
			// Clear the pending debug interrupt, if (nds32->edm.version >= 0x1011)
			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 3);
			LOG_DEBUG("Clear the pending debug interrupt");
			break;
		}
		else {
			aice_usb_pending_DBGI(target);
		}
	} while(0 == (reg_dbger & NDS_DBGER_DEX));

#endif

#if BACKUP_EDM_CTRL
	uint32_t acc_ctl_value;
	if (NDS32_DEBUG_DEBUG_INTERRUPT != reason) {
		if (is_v2_edm(coreid) == false) {
			/** debug 'debug mode'. use force_debug to issue dbgi */
			LOG_INFO("*** debug 'debug mode'. use force_debug to issue dbgi ***");
			aice_read_misc(target, NDS_EDM_MISC_ACC_CTL, &acc_ctl_value);
			acc_ctl_value |= 0x8;
			aice_write_misc(target, NDS_EDM_MISC_ACC_CTL, acc_ctl_value);
			core_info[coreid].debug_under_dex_on = true;

			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0); //halt
			/* If CPU stalled due to AT_MAX, clear AT_MAX status. */
			if (reg_dbger & NDS_DBGER_AT_MAX)
				aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_AT_MAX);
		}
	}
#endif
	/** set EDM_CTL.DBGIM & EDM_CTL.DBGACKM after halt */
	if (clear_edm_ctl_DBGIM_DBGACKM)
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl_value);

	/* backup r0 & r1 */
	aice_backup_tmp_registers(target);
	core_info[coreid].core_state = AICE_TARGET_HALTED;

	/* Note: read IR0 will corrupt tmp register (R0), have to it after
	 *       aice_backup_tmp_registers called.
	 */
#if BACKUP_EDM_CTRL
	if (core_info[coreid].debug_under_dex_on) {
		if (core_info[coreid].dex_use_psw_on == false) {
			/* under debug 'debug mode', force $psw to 'debug mode' bahavior */
			/* !!!NOTICE!!! this is workaround for debug 'debug mode'.
			 * it is only for debugging 'debug exception handler' purpose.
			 * after openocd detaches from target, target behavior is
			 * undefined. */
			uint32_t ir0_value = 0;
			uint32_t debug_mode_ir0_value;
			aice_read_reg(target, IR0, &ir0_value);
			debug_mode_ir0_value = ir0_value | 0x408; /* turn on DEX, set POM = 1 */
			debug_mode_ir0_value &= ~(0x000000C1); /* turn off DT/IT/GIE */
			aice_write_reg(target, IR0, debug_mode_ir0_value);
		}
	}
#endif
	return ERROR_OK;
}

int aice_state(struct target *target, enum aice_target_state_s *state)
{
	uint32_t coreid = target_to_coreid(target);

	/* 1. GET_ICE_STATE */
	if (aice_port->api->state != NULL) {
		aice_port->api->state();
	}
	/* 2. read DBGER status */
	LOG_DEBUG("coreid = %d", coreid);
	if (aice_sdm_check_powered_down(coreid) == 1)
		return ERROR_OK;

	uint32_t dbger_value = 0;
	int result = aice_read_misc(target, NDS_EDM_MISC_DBGER, &dbger_value);
	if (result != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_AICE_DISCONNECT);
		exit(-1);
		return ERROR_FAIL;
	}

	/* target debugging checking */
	uint32_t edm_ctl_value = 0;
	struct nds32 *nds32 = target_to_nds32(target);
	if (nds32->attached == true) {
		if (dbger_value & NDS_DBGER_DEX) {
			aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, (uint32_t*)&edm_ctl_value);
			if ((edm_ctl_value & NDS_EDMCTL_DEH_SEL) == 0) {
				LOG_DEBUG("skip host debugging");
				*state = AICE_TARGET_RUNNING;
				core_info[coreid].core_state = AICE_TARGET_RUNNING;
				// clear NDS_DBGER_DEX bit
				aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_DEX);
				return ERROR_OK;
			}
		}
	}

	if ((dbger_value & NDS_DBGER_ILL_SEC_ACC) == NDS_DBGER_ILL_SEC_ACC) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_ILL_ACCESS);

		/* Clear ILL_SEC_ACC */
		aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_ILL_SEC_ACC);

		*state = AICE_TARGET_RUNNING;
		core_info[coreid].core_state = AICE_TARGET_RUNNING;
	} else if ((dbger_value & NDS_DBGER_AT_MAX) == NDS_DBGER_AT_MAX) {
		/* Issue DBGI to exit cpu stall */
		aice_halt_target(target);

		/* Read OIPC to find out the trigger point */
		uint32_t ir11_value = 0;
		aice_read_reg(target, IR11, &ir11_value);
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_INTLVL2, ir11_value);

		/* Clear NDS_DBGER_AT_MAX */
		aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_AT_MAX);
		*state = AICE_TARGET_HALTED;
	} else if (((dbger_value & NDS_DBGER_CRST) == NDS_DBGER_CRST) && (aice_no_crst_detect == 0)) {
		LOG_DEBUG("DBGER.CRST is on.");

		*state = AICE_TARGET_RESET;
		core_info[coreid].core_state = AICE_TARGET_RUNNING;

		/* Clear CRST */
		aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CRST);
	} else if (dbger_value & (NDS_DBGER_HDBG | NDS_DBGER_DEX)) {
		if (AICE_TARGET_RUNNING == core_info[coreid].core_state) {
			/* enter debug mode, init EDM registers */
			/* backup EDM registers */
			aice_backup_edm_registers(target);
			/* init EDM for host debugging */
			aice_init_edm_registers(target, true);
			aice_backup_tmp_registers(target);
			core_info[coreid].core_state = AICE_TARGET_HALTED;
		} else if (AICE_TARGET_UNKNOWN == core_info[coreid].core_state) {
			/* debug 'debug mode', use force debug to halt core */
			aice_halt_target(target);
		}
		*state = AICE_TARGET_HALTED;
	} else {
		*state = AICE_TARGET_RUNNING;
		core_info[coreid].core_state = AICE_TARGET_RUNNING;
	}

	return ERROR_OK;
}

//extern int nds32_pack_read_reg_into_cache(struct nds32 *nds32, uint32_t all_reg_nums,
//		uint32_t *p_reg_id, uint32_t *p_reg_value);
extern int aice_bitset_sysreg(struct target *target, uint32_t num, uint32_t bit_set);
extern int aice_bitclr_sysreg(struct target *target, uint32_t num, uint32_t bit_clr);

int aice_step(struct target *target)
{
	LOG_DEBUG("=== %s ===", __func__);

	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t coreid = target_to_coreid(target);
	long long then = 0;
	uint32_t ir0_value = 0, ir14_value = 0;

	// read IR0 & IR14 in reg-cache
	nds32_get_mapped_reg(nds32, IR14, &ir14_value);
	nds32_get_mapped_reg(nds32, IR0, &ir0_value);
	LOG_DEBUG("ir0_value = 0x%x, ir14_value = 0x%x", ir0_value, ir14_value);

	/** set DSSIM */
	if (nds32->step_isr_enable)
		ir14_value |= (0x1 << 31);
	else
		ir14_value &= ~(0x1 << 31);

	/** enable HSS */
	//if ((ir0_value & 0x800) == 0) {
		/** set PSW.HSS */
		ir0_value |= (0x01 << 11);
	//}
	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;
	}

	nds32_set_mapped_reg(nds32, IR14, ir14_value);
	nds32_set_mapped_reg(nds32, IR0, ir0_value);	
	//if (ERROR_FAIL == aice_run_target(target))
	//	return ERROR_FAIL;
	/* restore r0 & r1 before free run */
	aice_restore_tmp_registers(target);
	core_info[coreid].core_state = AICE_TARGET_RUNNING;
	/* clear DBGER */
	aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CLEAR_ALL);
	aice_restore_edm_registers(target);
	/** execute instructions in DIM */
	uint32_t instructions[4] = {NOP,NOP,NOP,IRET};
	aice_execute_dim(target, instructions, 4);
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

	uint32_t i = 0;
	enum aice_target_state_s state;
	while (1) {
		/* read DBGER */
		if (aice_state(target, &state) != ERROR_OK)
			return ERROR_FAIL;

		if (AICE_TARGET_HALTED == state)
			break;

		if (i == 0)
			then = timeval_ms();
		else {
			if ((timeval_ms() - then) > aice_count_to_check_dbger) {
				then = timeval_ms();
				LOG_DEBUG("Timeout (%dms) waiting for halt to complete", aice_count_to_check_dbger);
				//return ERROR_FAIL;
				// do NOT return, until (state == AICE_TARGET_HALTED)
			}
		}
		alive_sleep(1);
		i++;
	}

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
	}
	if (is_v2_edm(coreid) == true) {
		/* V2 EDM will push interrupt stack as debug exception */
		aice_bitclr_sysreg(target, IR1, (0x01 << 11));
	} else {
		aice_bitclr_sysreg(target, IR0, (0x01 << 11));
	}
	/* restore DSSIM */
	if (nds32->step_isr_enable) {
		aice_bitclr_sysreg(target, IR14, (0x01 << 31));
	}
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
	return ERROR_OK;
}

int aice_run_target(struct target *target)
{
	LOG_DEBUG("=== %s ===", __func__);
	assert(target);

	uint32_t dbger_value=0;
	if (aice_read_misc(target, NDS_EDM_MISC_DBGER, &dbger_value) != ERROR_OK)
		return ERROR_FAIL;

	if ((dbger_value & (NDS_DBGER_HDBG | NDS_DBGER_DEX)) == 0) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_EXIT_DEBUG);
		return ERROR_FAIL;
	}
	/* profiling initialization if runmode is profile */
	int is_profiling = false;
	struct aice_profiling_info profiling_info;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t coreid = target_to_coreid(target);

	assert(nds32 && "nds32 shall not be NULL!");
	if (RUN_MODE_PROFILE & nds32->gdb_run_mode) {
		profiling_info.profiling_type = AICE_PROFILE_MODE_INIT;
		if (ERROR_OK == aice_profiling(target, &profiling_info))
			is_profiling = true;
	}
	if (RUN_MODE_PWR_MONITOR & nds32->gdb_run_mode) {
		nds32_pwr_init(target);
	}

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;
	}

	/* restore r0 & r1 before free run */
	aice_restore_tmp_registers(target);
	core_info[coreid].core_state = AICE_TARGET_RUNNING;

	/* clear DBGER */
	aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CLEAR_ALL);

	/** restore EDM registers */
	/** OpenOCD should restore EDM_CTL **before** to exit debug state.
	 *  Otherwise, following instruction will read wrong EDM_CTL value.
	 *
	 *  pc -> mfsr $p0, EDM_CTL (single step)
	 *        slli $p0, $p0, 1
	 *        slri $p0, $p0, 31
	 */
	aice_restore_edm_registers(target);

	/** execute instructions in DIM */
	uint32_t instructions[4] = {NOP,NOP,NOP,IRET};
	int result = aice_execute_dim(target, instructions, 4);
	if (aice_usb_pack_command != 0) {
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
	}

	//ensure no immediately DBGI raised!
	do {
		ASSERTOK(aice_read_misc(target, NDS_EDM_MISC_DBGER, &dbger_value));
		if (0 == (NDS_DBGER_DEX & dbger_value)) break;

		/* if target code freerun to trigger reset&hold */
		if ((dbger_value & (NDS_DBGER_CRST | NDS_DBGER_DEX)) ==
			(NDS_DBGER_CRST | NDS_DBGER_DEX)) {
			LOG_ERROR("target code freerun to trigger reset&hold");
			return ERROR_OK;
		}

		uint32_t reason;
		get_debug_reason(target, &reason);
		if (NDS32_DEBUG_DEBUG_INTERRUPT == reason) {
			//clear EDM.DBGER.DEX
			aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CLEAR_DEX);
			result = aice_execute_dim(target, instructions, 4);

			if (0) {
				alive_sleep(100);
			}

			//TODO: verify target is running
			ASSERTOK(aice_read_misc(target, NDS_EDM_MISC_DBGER, &dbger_value));
			// assert((0 == (NDS_DBGER_CLEAR_DEX & dbger_value)) && "RUN failed!");
			LOG_DEBUG("after IRET pending DGBI, DBGER=0x%08x\n", dbger_value);
			if (0) {
				get_debug_reason(target, &reason);
				assert((NDS32_DEBUG_DEBUG_INTERRUPT != reason) && "RUN failed!");
			}
		}
	} while (0);

	/* profiling post run if profiling */
	if (is_profiling) {
		profiling_info.profiling_type = AICE_PROFILE_MODE_POSTRUN;
		if (ERROR_OK != aice_profiling(target, &profiling_info)) {
			assert(0);
		}
	}

	return result;
}

int aice_usb_set_custom_srst_script(const char *script)
{
	custom_srst_script = strdup(script);
	return ERROR_OK;
}

int aice_usb_set_custom_trst_script(const char *script)
{
	custom_trst_script = strdup(script);
	return ERROR_OK;
}

int aice_usb_set_custom_restart_script(const char *script)
{
	custom_restart_script = strdup(script);
	return ERROR_OK;
}

int aice_execute_custom_script(struct target *target, const char *script)
{
	return aice_port->api->pnds32->execute_custom_script(target, (const char *)script);
}

int aice_issue_srst(struct target *target)
{
	LOG_DEBUG("aice_issue_srst");
	uint32_t coreid = target_to_coreid(target);

	/* After issuing srst, target will be running. So we need to restore EDM_CTL. */
	aice_restore_edm_registers(target);

	if (custom_srst_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_SRST) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom srst operations */
		if (aice_execute_custom_script(target, custom_srst_script) != ERROR_OK)
			return ERROR_FAIL;
	}
	// bug-10701 workaround, N903 Weltrend Board diagnostic failed. (WIN7 64BIT)
	if ((aice_hardware_version == 0x01) &&
	    (aice_firmware_version <= 0x06) &&
	    (aice_fpga_version <= 0x06)) {
		LOG_DEBUG("aice_issue_srst-delay");
		alive_sleep(100);
	}

	/* wait CRST with timeout */
	uint32_t issue_srst_fail = 0;
	if (aice_check_dbger(target, NDS_DBGER_CRST) != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_RESET_HOLD);
		issue_srst_fail = 1;
	}

	struct nds32 *nds32 = target_to_nds32(target);
	char *edm_passcode;
	if (nds32 != NULL)
		edm_passcode = nds32->edm_passcode;
	else
		edm_passcode = nds32_edm_passcode_init;
	if (aice_set_edm_passcode(target, edm_passcode) != ERROR_OK)
		return ERROR_FAIL;

	core_info[coreid].host_dtr_valid = false;
	core_info[coreid].target_dtr_valid = false;
	core_info[coreid].core_state = AICE_TARGET_RUNNING;
	if (issue_srst_fail) {
		NDS32_LOG(NDS32_ERRMSG_SRST_FAIL);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int aice_issue_restart(struct target *target)
{
	/* issue restart again */
	if (custom_restart_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_RESTART) != ERROR_OK)
			return ERROR_FAIL;
		/* delay 500ms before EDM access (Default Debug-on-Reset Timing) */
		alive_sleep(500);
	} else {
		/* custom restart operations */
		if (aice_execute_custom_script(target, custom_restart_script) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (aice_check_edmctl(target, NDS_EDMCTL_DEH_SEL) != ERROR_OK) {
		return ERROR_FAIL;
	}
	if (aice_check_dbger(target, NDS_DBGER_CRST | NDS_DBGER_DEX) == ERROR_OK) {
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

int aice_issue_reset_hold(struct target *target)
{
	uint32_t coreid = target_to_coreid(target);
	LOG_DEBUG("aice_issue_reset_hold, coreid = %d", coreid);
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32 && "nds32 shall not be NULL!");
	uint32_t edm_ctl_value = 0;

	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);
	LOG_DEBUG("aice_issue_reset_hold %x, %x, %x", nds32->edm.version, cpu_version->revision,
				cpu_version->cpu_id_family);

	/* set no_dbgi_pin to 0 */
	uint32_t pin_status;
	uint32_t value;

	if (nds32->soft_reset_halt)
		goto soft_reset_halt;

  /* In some case, DIMBR can not be accessed
   * (like Novatek case, ticket 4126)
   * do srst first to make sure DIMBR can be accessed
   * */
	//if (nds32_reset_halt_as_init) {
	//	nds32_reset_halt_as_init = 0;
	//}
	//else {
	//	if (aice_issue_restart(target) != ERROR_OK) {
	//		LOG_DEBUG(NDS32_ERRMSG_RESTART_FAIL);
	//		NDS32_LOG(NDS32_ERRMSG_TARGET_HW_RESET_HOLD);
	//	}
	//}

	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
	aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_DEH_SEL);
	aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4);

	/* clear no_dbgi_pin */
	aice_read_ctrl(AICE_READ_CTRL_GET_JTAG_PIN_STATUS, &pin_status);
	if (pin_status | 0x4)
		aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status & (~0x4));

	if (aice_set_edm_passcode(target, nds32->edm_passcode) != ERROR_OK)
		return ERROR_FAIL;

	/* When the last command before RESTART is READ/WRITE_MISC, the chip behavior is not
	 * expected and subsequent READ/WRITE_MISC fails.
	 */
	aice_read_dtr(target, &value);

	/* issue restart */
	if (aice_issue_restart(target) == ERROR_OK) {
		goto hard_reset_halt_success;
	} else {
//hard_reset_halt_5wire:
		/* set no_dbgi_pin to 1 */
		aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status | 0x4);

		if (aice_set_edm_passcode(target, nds32->edm_passcode) != ERROR_OK)
			return ERROR_FAIL;

		aice_read_dtr(target, &value);

		/* issue restart again */
		if (aice_issue_restart(target) != ERROR_OK) {
			LOG_DEBUG(NDS32_ERRMSG_RESTART_FAIL);
			NDS32_LOG(NDS32_ERRMSG_TARGET_HW_RESET_HOLD);
			goto soft_reset_halt;
		}
		else {
			goto hard_reset_halt_success;
		}
	}

soft_reset_halt:
	/* clear timeout */
	if (aice_reset_device() != ERROR_OK)
		return ERROR_FAIL;
	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
	aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_DEH_SEL);
	aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4);
	aice_write_misc(target, NDS_EDM_MISC_DBGER,	NDS_DBGER_CLEAR_ALL);

	/* do software reset-and-hold */
	uint32_t issue_srst_fail = 0;
	if (nds32_reset_halt_as_init == 3) {
		// soft_reset_halt without issue_srst
	}
	else {
		if (aice_issue_srst(target) != ERROR_OK) {
			issue_srst_fail = 1;
		}
	}
	aice_halt_target(target);
  /* reset registers, bug-10621*/
	uint32_t mapped_regnum = 0;
	uint32_t ir3_IVB = 0;
	uint32_t cr3_MMU_CFG = 0;
	uint32_t cr4_MSC_CFG = 0;

	aice_read_reg(target, IR3, &ir3_IVB);
	aice_read_reg(target, CR3, &cr3_MMU_CFG);
	aice_read_reg(target, CR4, &cr4_MSC_CFG);

	// IPC (ir9)=IVB
	mapped_regnum = nds32->register_map(nds32, IR9);
	aice_write_reg(target, mapped_regnum, ir3_IVB & 0xFFFF0000);
	// OIPC (ir11)=IVB
	mapped_regnum = nds32->register_map(nds32, PC);
	aice_write_reg(target, mapped_regnum, ir3_IVB & 0xFFFF0000);

	// PSW (ir0)=PSW_reset_value
	uint32_t PSW_reset_value = 0x0000040a;
	if (cr3_MMU_CFG & 0x80000000)
		PSW_reset_value |= 0x1000;
	if (cr3_MMU_CFG & 0x04000000)
		PSW_reset_value |= 0x20;
	if (((cr4_MSC_CFG & 0xE000)>>13) >= 2)
		PSW_reset_value |= 0x70000;
	mapped_regnum = nds32->register_map(nds32, IR0);
	aice_write_reg(target, mapped_regnum, PSW_reset_value);

	// IPSW (ir1)=IPSW_reset_value
	PSW_reset_value &= (~0x0400);
	mapped_regnum = nds32->register_map(nds32, IR1);
	aice_write_reg(target, mapped_regnum, PSW_reset_value);

	// ITYPE (ir6)=0
	mapped_regnum = nds32->register_map(nds32, IR6);
	aice_write_reg(target, mapped_regnum, 0);
	// check MSC_CFG[12] to determine if P_ITYPE exists, P_ITYPE (ir7)=0
	if (((cr4_MSC_CFG & 0x1000)>>12) != 1) {
		mapped_regnum = nds32->register_map(nds32, IR7);
		aice_write_reg(target, mapped_regnum, 0);
  }
	// INT_PEND (ir15)=0
	mapped_regnum = nds32->register_map(nds32, IR15);
	aice_write_reg(target, mapped_regnum, 0);
	// check IVB[12:11] to determine if INT_PEND2 exists, INT_PEND2 (ir27)=0
	if (((ir3_IVB & 0x1800) >> 11) >= 1) {
		mapped_regnum = nds32->register_map(nds32, IR27);
		aice_write_reg(target, mapped_regnum, 0);
	}
	if (issue_srst_fail)
		return ERROR_OK;
	NDS32_LOG(NDS32_MSG_SW_RESET_HOLD);
	nds32_redirect_edm_v2(target);
	return ERROR_OK;

hard_reset_halt_success:
	/* backup EDM registers */
	aice_backup_edm_registers(target);

	/* init EDM for host debugging */
	aice_init_edm_registers(target, true);

	aice_backup_tmp_registers(target);
	core_info[coreid].core_state = AICE_TARGET_HALTED;

	if (force_to_v3 == 1) {
		/* check Security Privilege Level (DBGSPL) */
		uint32_t value_edmsw = 0;
		aice_read_edm(target, JDP_R_DBG_SR, NDS_EDM_SR_EDMSW, (uint32_t*)&value_edmsw, 1);
		/* ir3 only be accessed by MFSR/MTSR with ISPL==0 */
		if ((value_edmsw & 0x30000) == 0) {
			aice_read_reg(target, IR3, &ir3_IVB);
			// IPC (ir9)=IVB
			mapped_regnum = nds32->register_map(nds32, IR9);
			aice_write_reg(target, mapped_regnum, ir3_IVB & 0xFFFF0000);
			// OIPC (ir11)=IVB
			mapped_regnum = nds32->register_map(nds32, PC);
			aice_write_reg(target, mapped_regnum, ir3_IVB & 0xFFFF0000);
		}
	}
	NDS32_LOG(NDS32_MSG_HW_RESET_HOLD);
	nds32_redirect_edm_v2(target);
	return ERROR_OK;
}

int aice_issue_reset_hold_multi(struct target *active_target)
{
	uint32_t write_ctrl_value = 0;
	struct target *target;
	uint32_t coreid;
	struct nds32 *nds32;
	uint32_t edm_ctl_value = 0;

	for (target = all_targets; target; target = target->next) {
			nds32 = target_to_nds32(target);
			if (!is_nds32(nds32))
				continue;
			coreid = target_to_coreid(target);
			aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
			aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_DEH_SEL);
			aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4);
	}

	if (custom_restart_script == NULL) {
		/* set SRST */
		write_ctrl_value = AICE_CUSTOM_DELAY_SET_SRST;
		write_ctrl_value |= (0x200 << 16);
		if (aice_write_ctrl(AICE_WRITE_CTRL_CUSTOM_DELAY,
					write_ctrl_value) != ERROR_OK)
			return ERROR_FAIL;

		target = all_targets;
		for (target = all_targets; target; target = target->next) {
			nds32 = target_to_nds32(target);
			if (!is_nds32(nds32))
				continue;
			aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0);
		}
		/* clear SRST */
		write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_SRST;
		write_ctrl_value |= (0x200 << 16);
		if (aice_write_ctrl(AICE_WRITE_CTRL_CUSTOM_DELAY,
					write_ctrl_value) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom restart operations */
		if (aice_execute_custom_script(active_target, custom_restart_script) != ERROR_OK)
			return ERROR_FAIL;
	}

	for (target = all_targets; target; target = target->next) {
			nds32 = target_to_nds32(target);
			if (!is_nds32(nds32))
				continue;
			coreid = target_to_coreid(target);
			uint32_t value_dbger = 0;
			aice_read_misc(target, NDS_EDM_MISC_DBGER, &value_dbger);
			//aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
			//aice_write_edmsr(coreid, NDS_EDM_SR_EDM_CTL, edm_ctl_value | NDS_EDMCTL_DEH_SEL);
			//aice_write_edmsr(coreid, NDS_EDM_SR_DIMBR, DIMBR_D4);
	}

#if 0 // do aice_edm_init when gdb connect
	for (target = all_targets; target; target = target->next) {
		coreid = target_to_coreid(target);
		aice_edm_init(coreid);
	}
#endif
	NDS32_LOG(NDS32_MSG_HW_RESET_HOLD);
	return ERROR_OK;
}

int aice_open_device(struct aice_port_param_s *param)
{
	// setting default "target->tap->priv"
	struct target *target = (struct target *)&g_default_target;
	struct jtag_tap *tap = (struct jtag_tap *)&g_default_tap;

	target->tap = tap;
	//target->tap->abs_chain_position = 0;
	struct aice_port_s *aice = calloc(1, sizeof(struct aice_port_s));
	aice->port = aice_port;
	aice->coreid = 0;
	target->tap->priv = aice;

	if( vid_pid_array_top == -1 ) {
		NDS32_LOG(NDS32_ERRMSG_USB_VIDPID);
		return ERROR_FAIL;
	}

	if (ERROR_OK != aice_port->api->open(param)) {
		LOG_ERROR("Cannot find AICE Interface! Please check "
				"connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* attempt to reset Andes EDM */
	if (ERROR_FAIL == aice_reset_device()) {
		NDS32_LOG(NDS32_ERRMSG_AICE_RESET_BOX);
		exit(-1);
		return ERROR_FAIL;
	}
	LOG_DEBUG("nds32_reset_halt_as_init 0x%x", nds32_reset_halt_as_init);
	aice_skip_tckscan_before_restart = 0;
	if (custom_initial_script != NULL) {
		LOG_INFO("doing custom_initial_script...");
		if (aice_execute_custom_script(target, custom_initial_script) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (nds32_reset_halt_as_init) {
		if (nds32_reset_halt_as_init == 2) { // soft_reset_halt as init
			aice_issue_srst(&g_default_target);
		}
		else if (nds32_reset_halt_as_init == 3) {
			// soft_reset_halt without issue_srst
		}
		else {
			if (aice_issue_restart(&g_default_target) != ERROR_OK) {
				//LOG_DEBUG(NDS32_ERRMSG_RESTART_FAIL);
				//NDS32_LOG(NDS32_ERRMSG_TARGET_HW_RESET_HOLD);
			}
		}
	}
	LOG_INFO("AICE initialization started");
	aice_set_jtag_clock(aice_set_clk_first);

	return ERROR_OK;
}

extern void nds_dump_detail_debug_info(uint32_t);
int nds_freerun_all_targets(void)
{
	LOG_DEBUG("%s", __func__);
	struct target *target = all_targets;
	assert(target);

	for (target = all_targets; target; target = target->next) {
		LOG_DEBUG("%s", target_type_name(target));
		if (strcmp("nds32_v3", target_type_name(target)) != 0)
			continue;
		LOG_DEBUG("freerun target %s", target_type_name(target));
		uint32_t coreid = target_to_coreid(target);
		if (core_info[coreid].core_state == AICE_TARGET_HALTED)
			aice_run_target(target);

		if (aice_dis_DEH_SEL == 1) {
			uint32_t edm_ctl_reg = core_info[coreid].edm_ctl_backup;
			edm_ctl_reg &= ~(NDS_EDMCTL_DEH_SEL);
			aice_write_edm(target, JDP_W_DBG_SR, NDS_EDM_SR_EDM_CTL, (uint32_t*)&edm_ctl_reg, 1);
			LOG_DEBUG("write edm_ctl_reg 0x%x", edm_ctl_reg);
		}
	}
	nds_dump_detail_debug_info(1);
	return ERROR_OK;
}

int aice_close_device(void)
{
	LOG_DEBUG("%s", __func__);
	if (aice_port->api->close == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	if (aice_port->api->close() != ERROR_OK)
		return ERROR_FAIL;

	if (custom_srst_script)
		free(custom_srst_script);

	if (custom_trst_script)
		free(custom_trst_script);

	if (custom_restart_script)
		free(custom_restart_script);

	return ERROR_OK;
}

int aice_reset_device(void)
{
	if (aice_port->api->reset == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	if (aice_port->api->reset() != ERROR_OK)
		return ERROR_FAIL;

	/* issue TRST */
	if (custom_trst_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_TRST) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom trst operations */
		if (aice_execute_custom_script(NULL, custom_trst_script) != ERROR_OK)
			return ERROR_FAIL;
	}
	// skip tckscan before restart, for JTAG-pins pinmux
	if (aice_skip_tckscan_before_restart)
		return ERROR_OK;

	if (aice_set_jtag_clock(jtag_clock) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int aice_set_dis_DEH_SEL(uint32_t if_disable)
{
	aice_dis_DEH_SEL = if_disable;
	return ERROR_OK;
}
