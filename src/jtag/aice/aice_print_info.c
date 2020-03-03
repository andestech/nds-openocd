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

#define _ADAPTER_USE_ 0x00

#if _ADAPTER_USE_
#include "log.h"
#include "aice_usb_command.h"
#include "aice_pipe_command.h"
#include "log.h"
#include "aice_jdp.h"
#include "nds32_edm.h"
#include "nds32_insn.h"

extern enum aice_command_mode aice_command_mode;
uint32_t aice_get_command_mode(void)
{
	return (uint32_t)aice_command_mode;
}
#else

#include <helper/log.h>
#include "aice_usb.h"
#include "aice_jdp.h"
#include "aice_apis.h"
#endif

struct aice_print_insn_attr {
	unsigned int cmdmask;
	unsigned int cmdid;
	char *cmdname;
};

struct aice_print_info_attr {
	unsigned int cmdid;
	char *cmdname;
};

#define NDS_SDM_MISC_SDM_CFG            0
#define NDS_SDM_MISC_SBAR               1
#define NDS_SDM_MISC_PROCESSOR_CTL      2
#define NDS_SDM_MISC_PROCESSOR_STATUS   3
#define NDS_SDM_MISC_SDM_CTL            4

uint32_t aice_current_write_ctrl_sdm = 0;
static struct aice_print_info_attr aice_print_info_sdm_misc[] = {
	{ NDS_SDM_MISC_SDM_CFG,         "SDM_CFG", },
	{ NDS_SDM_MISC_SBAR,            "SBAR", },
	{ NDS_SDM_MISC_PROCESSOR_CTL,   "PROCESSOR_CTL", },
	{ NDS_SDM_MISC_PROCESSOR_STATUS,"PROCESSOR_STATUS", },
	{ NDS_SDM_MISC_SDM_CTL,         "SDM_CTL", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_info_attr aice_print_info_readctrl[] = {
	{ AICE_READ_CTRL_GET_ICE_STATE,         "GET_ICE_STATE", },
	{ AICE_READ_CTRL_GET_HARDWARE_VERSION,  "GET_HARDWARE_VERSION", },
	{ AICE_READ_CTRL_GET_FPGA_VERSION,      "GET_FPGA_VERSION", },
	{ AICE_READ_CTRL_GET_FIRMWARE_VERSION,  "GET_FIRMWARE_VERSION", },
	{ AICE_READ_CTRL_GET_JTAG_PIN_STATUS,   "GET_JTAG_PIN_STATUS", },
	{ AICE_READ_CTRL_BATCH_CTRL,            "BATCH_CTRL", },
	{ AICE_READ_CTRL_BATCH_ITERATION,       "BATCH_ITERATION", },
	{ AICE_READ_CTRL_BATCH_BUF_INFO,        "BATCH_BUF_INFO", },
	{ AICE_READ_CTRL_BATCH_STATUS,          "BATCH_STATUS", },
	{ AICE_READ_CTRL_BATCH_CMD_BUF0_CTRL,   "BATCH_CMD_BUF0_CTRL", },
	{ AICE_READ_CTRL_BATCH_BUF0_STATE,      "BATCH_BUF0_STATE", },
	{ AICE_READ_CTRL_BATCH_BUF4_STATE,      "BATCH_BUF4_STATE", },
	{ AICE_READ_CTRL_BATCH_BUF5_STATE,      "BATCH_BUF5_STATE", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_info_attr aice_print_info_writectrl[] = {
	{ AICE_WRITE_CTRL_TCK_CONTROL,          "TCK_CONTROL", },
	{ AICE_WRITE_CTRL_JTAG_PIN_CONTROL,     "JTAG_PIN_CONTROL", },
	{ AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, "CLEAR_TIMEOUT_STATUS", },
	{ AICE_WRITE_CTRL_JTAG_PIN_STATUS,      "JTAG_PIN_STATUS", },
	{ AICE_WRITE_CTRL_TIMEOUT,              "TIMEOUT", },
	{ AICE_WRITE_CTRL_SDMCONN,              "SDMCONN", },
	{ AICE_WRITE_CTRL_CUSTOM_DELAY,         "CUSTOM_DELAY", },
	{ AICE_WRITE_CTRL_BATCH_CTRL,           "BATCH_CTRL", },
	{ AICE_WRITE_CTRL_BATCH_ITERATION,      "BATCH_ITERATION", },
	{ AICE_WRITE_CTRL_BATCH_DIM_SIZE,       "BATCH_DIM_SIZE", },
	{ AICE_WRITE_CTRL_BATCH_CMD_BUF0_CTRL,  "BATCH_CMD_BUF0_CTRL", },
	{ AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL, "BATCH_DATA_BUF0_CTRL", },
	{ AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL, "BATCH_DATA_BUF1_CTRL", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_info_attr aice_print_info_jdp[] = {
	{ JDP_R_DBG_SR,    "READ_EDMSR", },
	{ JDP_R_DTR,       "READ_DTR", },
	{ JDP_R_MEM_W,     "READ_MEM_W", },
	{ JDP_R_MISC_REG,  "READ_MISC", },
	{ JDP_R_FAST_MEM,  "FASTREAD_MEM", },
	{ JDP_R_MEM_H,     "READ_MEM_H", },
	{ JDP_R_MEM_B,     "READ_MEM_B", },
	{ JDP_W_DIM,       "WRITE_DIM", },
	{ JDP_W_EXECUTE,   "EXECUTE", },
	{ JDP_W_DBG_SR,    "WRITE_EDMSR", },
	{ JDP_W_DTR,       "WRITE_DTR", },
	{ JDP_W_MEM_W,     "WRITE_MEM_W", },
	{ JDP_W_MISC_REG,  "WRITE_MISC", },
	{ JDP_W_FAST_MEM,  "FASTWRITE_MEM", },
	{ JDP_W_MEM_H,     "WRITE_MEM_H", },
	{ JDP_W_MEM_B,     "WRITE_MEM_B", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_info_attr aice_print_info_edm_misc[] = {
	{ NDS_EDM_MISC_DIMIR,     "DIMIR", },
	{ NDS_EDM_MISC_SBAR,      "SBAR", },
	{ NDS_EDM_MISC_EDM_CMDR,  "EDM_CMDR", },
	{ NDS_EDM_MISC_DBGER,     "DBGER", },
	{ NDS_EDM_MISC_ACC_CTL,   "ACC_CTL", },
	{ NDS_EDM_MISC_EDM_PROBE, "EDM_PROBE", },
	{ NDS_EDM_MISC_GEN_PORT0, "GEN_PORT0", },
	{ NDS_EDM_MISC_GEN_PORT1, "GEN_PORT1", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_info_attr aice_print_info_edm_sr[] = {
	{ NDS_EDM_SR_EDM_CFG, "EDM_CFG", },
	{ NDS_EDM_SR_EDMSW,   "EDMSW", },
	{ NDS_EDM_SR_EDM_CTL, "EDM_CTL", },
	{ NDS_EDM_SR_EDM_DTR, "EDM_DTR", },
	{ NDS_EDM_SR_BPMTV,   "BPMTV", },
	{ NDS_EDM_SR_DIMBR,   "DIMBR", },
	{ NDS_EDM_SR_TECR0,   "TECR0", },
	{ NDS_EDM_SR_TECR1,   "TECR1", },
	{ NDS_EDM_SR_BPC0, "BPC0", },
	{ NDS_EDM_SR_BPC1, "BPC1", },
	{ NDS_EDM_SR_BPC2, "BPC2", },
	{ NDS_EDM_SR_BPC3, "BPC3", },
	{ NDS_EDM_SR_BPC4, "BPC4", },
	{ NDS_EDM_SR_BPC5, "BPC5", },
	{ NDS_EDM_SR_BPC6, "BPC6", },
	{ NDS_EDM_SR_BPC7, "BPC7", },
	{ NDS_EDM_SR_BPA0, "BPA0", },
	{ NDS_EDM_SR_BPA1, "BPA1", },
	{ NDS_EDM_SR_BPA2, "BPA2", },
	{ NDS_EDM_SR_BPA3, "BPA3", },
	{ NDS_EDM_SR_BPA4, "BPA4", },
	{ NDS_EDM_SR_BPA5, "BPA5", },
	{ NDS_EDM_SR_BPA6, "BPA6", },
	{ NDS_EDM_SR_BPA7, "BPA7", },
	{ NDS_EDM_SR_BPV0, "BPV0", },
	{ NDS_EDM_SR_BPV1, "BPV1", },
	{ NDS_EDM_SR_BPV2, "BPV2", },
	{ NDS_EDM_SR_BPV3, "BPV3", },
	{ NDS_EDM_SR_BPV4, "BPV4", },
	{ NDS_EDM_SR_BPV5, "BPV5", },
	{ NDS_EDM_SR_BPV6, "BPV6", },
	{ NDS_EDM_SR_BPV7, "BPV7", },
	{ NDS_EDM_SR_BPAM0, "BPAM0", },
	{ NDS_EDM_SR_BPAM1, "BPAM1", },
	{ NDS_EDM_SR_BPAM2, "BPAM2", },
	{ NDS_EDM_SR_BPAM3, "BPAM3", },
	{ NDS_EDM_SR_BPAM4, "BPAM4", },
	{ NDS_EDM_SR_BPAM5, "BPAM5", },
	{ NDS_EDM_SR_BPAM6, "BPAM6", },
	{ NDS_EDM_SR_BPAM7, "BPAM7", },
	{ 0xFFFFFFFF, "", },
};

static struct aice_print_insn_attr aice_print_info_insn[] = {
	{ 0xFFFFFFFF, NOP,          "NOP", },
	{ 0xFFFFFFFF, DSB,          "DSB", },
	{ 0xFFFFFFFF, ISB,          "ISB", },
	{ 0xFFFFFFFF, BEQ_MINUS_12, "BEQ_MINUS_12", },
	{~0x01F00000, MTSR_DTR(0),  "MTSR_DTR", },
	{~0x01F00000, MFSR_DTR(0),  "MFSR_DTR", },
	{~0x01FFFFFF, SETHI(0, 0),  "SETHI", },
	{~0x01FFFFFF, ORI(0, 0, 0), "ORI", },
	{~0x01FFFFFF, 0x0C000000, "LWI_BI", },
	{~0x01FFFFFF, 0x0A000000, "LHI_BI", },
	{~0x01FFFFFF, 0x08000000, "LBI_BI", },
	{~0x01FFFFFF, 0x1C000000, "SWI_BI", },
	{~0x01FFFFFF, 0x1A000000, "SHI_BI", },
	{~0x01FFFFFF, 0x18000000, "SBI_BI", },
	{ 0xFFFFFFFF, IRET,         "IRET", },
	{~0x000F8000, L1D_IX_WB(0),    "L1D_IX_WB", },
	{~0x000F8000, L1D_IX_INVAL(0), "L1D_IX_INVAL", },
	{~0x000F8000, L1D_VA_INVAL(0), "L1D_VA_INVAL", },
	{~0x000F8000, L1D_VA_WB(0),    "L1D_VA_WB", },
	{~0x01FF8000, L1D_IX_RTAG(0),  "L1D_IX_RTAG", },
	{~0x01FF8000, L1D_IX_RWD(0),   "L1D_IX_RWD", },
	{~0x000F8000, L1I_IX_INVAL(0), "L1I_IX_INVAL", },
	{~0x000F8000, L1I_VA_INVAL(0), "L1I_VA_INVAL", },
	{~0x01FF8000, L1I_IX_RTAG(0),  "L1I_IX_RTAG", },
	{~0x01FF8000, L1I_IX_RWD(0),   "L1I_IX_RWD", },
	{~0x000F8000, L1I_VA_FILLCK(0),"L1I_VA_FILLCK", },
	{~0x01F00000, ISYNC(0),        "ISYNC", },
	{ 0xFFFFFFFF, MSYNC_STORE,     "MSYNC_STORE", },
	{ 0xFFFFFFFF, MSYNC_ALL,       "MSYNC_ALL", },
	{~0x000F8000, TLBOP_TARGET_READ(0),     "TLBOP_TARGET_READ", },
	{~0x01FF8000, TLBOP_TARGET_PROBE(0, 0), "TLBOP_TARGET_PROBE", },
	{~0x01FFFFFF, MOVI_(0, 0),     "MOVI", },
	{~0x01FF8000, MFUSR_G0(0, 0),  "MFUSR", },
	{~0x01FF8000, MTUSR_G0(0, 0),  "MTUSR", },
	{~0x01FFFC00, MFSR(0, 0),      "MFSR", },
	{~0x01FFFC00, MTSR(0, 0),      "MTSR", },
	{~0x000F801F, AMFAR(0, 0),     "AMFAR", },
	{~0x000F801F, AMTAR(0, 0),     "AMTAR", },
	{~0x000F801F, AMFAR2(0, 0),    "AMFAR2", },
	{~0x000F801F, AMTAR2(0, 0),    "AMTAR2", },
	{~0x01F00000, FMFCSR(0),       "FMFCSR", },
	{~0x01F00000, FMTCSR(0),       "FMTCSR", },
	{~0x01F00000, FMFCFG(0),       "FMFCFG", },
	{~0x01FF8000, FMFSR(0, 0),     "FMFSR", },
	{~0x01FF8000, FMTSR(0, 0),     "FMTSR", },
	{~0x01FF8000, FMFDR(0, 0),     "FMFDR", },
	{~0x01FF8000, FMTDR(0, 0),     "FMTDR", },
	{ 0xFFFFFFFF, 0xFFFFFFFF, "", },
};

static char *paice_print_info_null = "";

static char *aice_print_info_get_subcmmd(unsigned int address,
	struct aice_print_info_attr *pinfo_attr)
{
	while(1) {
		if (pinfo_attr->cmdid == 0xFFFFFFFF)
			return (pinfo_attr->cmdname);
		if (address == pinfo_attr->cmdid) {
			return (pinfo_attr->cmdname);
		}
		pinfo_attr ++;
	}
	return (paice_print_info_null);
}

static char *aice_print_info_get_insn(unsigned int instruction)
{
	struct aice_print_insn_attr *pinfo_attr = &aice_print_info_insn[0];
	while(1) {
		if (pinfo_attr->cmdid == 0xFFFFFFFF)
			return (pinfo_attr->cmdname);
		if ((instruction & pinfo_attr->cmdmask) == pinfo_attr->cmdid) {
			return (pinfo_attr->cmdname);
		}
		pinfo_attr ++;
	}
	return (paice_print_info_null);
}

void aice_print_info(unsigned int pipe_cmmd, unsigned int address,
	unsigned int *pInfoData, unsigned int target_id, unsigned int jdp_id)
{
	char *pStrUsbCmmd="", *pStrSubCmmd="";

	if ((pipe_cmmd == AICE_READ_CTRL) || (pipe_cmmd == AICE_WRITE_CTRL)) {
		if (pipe_cmmd == AICE_READ_CTRL) {
			pStrUsbCmmd = "READ_CTRL";
			pStrSubCmmd = aice_print_info_get_subcmmd(address, &aice_print_info_readctrl[0]);
			if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK) {
				LOG_DEBUG("%s: ADDR: 0x%02X [%s], DATA: N/A.",
					pStrUsbCmmd, address, pStrSubCmmd);
				return;
			}
		}
		else {
			pStrUsbCmmd = "WRITE_CTRL";
			pStrSubCmmd = aice_print_info_get_subcmmd(address, &aice_print_info_writectrl[0]);
			if ((address == AICE_WRITE_CTRL_SDMCONN) &&
				(*pInfoData & 0x80000000)) {
				aice_current_write_ctrl_sdm = 1;
			}
			else
				aice_current_write_ctrl_sdm = 0;
		}

		LOG_DEBUG("%s: ADDR: 0x%02X [%s], DATA: 0x%08X.",
			pStrUsbCmmd, address, pStrSubCmmd, *pInfoData);
		return;
	}
	else if ((pipe_cmmd == AICE_READ_EDM) || (pipe_cmmd == AICE_WRITE_EDM)) {
		pStrUsbCmmd = aice_print_info_get_subcmmd(jdp_id, &aice_print_info_jdp[0]);
		if ((jdp_id == JDP_R_MISC_REG) || (jdp_id == JDP_W_MISC_REG)) {
			if (aice_current_write_ctrl_sdm == 1)
				pStrSubCmmd = aice_print_info_get_subcmmd(address, &aice_print_info_sdm_misc[0]);
			else
				pStrSubCmmd = aice_print_info_get_subcmmd(address, &aice_print_info_edm_misc[0]);
			if ((aice_get_command_mode() == AICE_COMMAND_MODE_PACK) && (jdp_id == JDP_R_MISC_REG)) {
				LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%02X [%s], DATA: N/A.",
					pStrUsbCmmd, target_id, address, pStrSubCmmd);
				return;
			}
			LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%02X [%s], DATA: 0x%08X.",
				pStrUsbCmmd, target_id, address, pStrSubCmmd, *pInfoData);
			return;
		}
		else if ((jdp_id == JDP_R_DBG_SR) || (jdp_id == JDP_W_DBG_SR)) {
			pStrSubCmmd = aice_print_info_get_subcmmd(address, &aice_print_info_edm_sr[0]);
			if ((aice_get_command_mode() == AICE_COMMAND_MODE_PACK) && (jdp_id == JDP_R_DBG_SR)) {
				LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%02X [%s], DATA: N/A.",
					pStrUsbCmmd, target_id, address, pStrSubCmmd);
				return;
			}
			LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%02X [%s], DATA: 0x%08X.",
				pStrUsbCmmd, target_id, address, pStrSubCmmd, *pInfoData);
			return;
		}
		else if ((jdp_id == JDP_R_MEM_W) || (jdp_id == JDP_W_MEM_W)) {
			if ((aice_get_command_mode() == AICE_COMMAND_MODE_PACK) && (jdp_id == JDP_R_MEM_W)) {
				LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%08X, DATA: N/A.",
					pStrUsbCmmd, target_id, address);
				return;
			}
			LOG_DEBUG("%s: TARGET: 0x%02X, ADDR: 0x%08X, DATA: 0x%08X.",
				pStrUsbCmmd, target_id, address, *pInfoData);
			return;
		}
		else if (jdp_id == JDP_W_DIM) {
			char *pStrInsn[4];
			unsigned int i;
			for (i=0; i<4; i++) {
				pStrInsn[i] = aice_print_info_get_insn(pInfoData[i]);
			}
			LOG_DEBUG("%s: TARGET: 0x%02X, INSN: 0x%08X[%s], 0x%08X[%s], 0x%08X[%s], 0x%08X[%s].",
				pStrUsbCmmd, target_id,
				pInfoData[0], pStrInsn[0], pInfoData[1], pStrInsn[1],
				pInfoData[2], pStrInsn[2], pInfoData[3], pStrInsn[3]);
			return;
		}
		if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK) {
			LOG_DEBUG("%s: TARGET: 0x%02X, DATA: N/A.",
				pStrUsbCmmd, target_id);
			return;
		}
		LOG_DEBUG("%s: TARGET: 0x%02X, DATA: 0x%08X.",
			pStrUsbCmmd, target_id, *pInfoData);
		return;
	}
}
