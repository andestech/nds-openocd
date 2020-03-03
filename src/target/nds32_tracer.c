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

#include <helper/command.h>
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_tracer.h"
#include "nds32_log.h"
#include <jtag/aice/aice_jdp.h>
#include "nds32_v3_common.h"
#include <target/nds32_disassembler.h>

#define TRACER_TMP_BUFSIZE   0x300000  // 3MB
#define TRACER_OTB_FILESIZE  0x200000  // 2MB, 512KB
#define TRACER_MAX_OTB_PER_TRANSFER  (TRACER_TMP_BUFSIZE - 0x80000)  // 0x80000 is for remain data buffer
#define TRACER_OTB_FILENAME      "otb-pkt.txt"
#define TRACER_SYN_FILENAME      "otb-pkt.txt-sy"
#define TRACER_OTB_BAK_FILENAME  "otb-pkt-bak.txt"
#define TRACER_SYN_BAK_FILENAME  "otb-pkt-bak.txt-sy"

#define OTB_NOT_CLOSE           0
#define OTB_CLOSE_FILE          1
#define OTB_BACKUP_CLOSE_FILE   2

#define TRACER_LOG_TIME     0

static time_t seconds;
static unsigned long compare_words = 0x100000;
#if (TRACER_LOG_TIME == 1)
#define LOG_TIME    NDS32_LOG
#else
#define LOG_TIME    LOG_DEBUG
#endif

static unsigned long nds32_skip_find_1stsync = 0;
static unsigned long nds32_fifo_words_per_transfer = (TRACER_MAX_OTB_PER_TRANSFER >> 2);
unsigned long nds32_chk_tracer_support = 0;
unsigned long nds32_chk_tracer_xrw_support = 0;
unsigned long nds32_otb_id = 0, nds32_tdrx_id = 0;

FILE *pOTBPacketFile = NULL;
unsigned long nds32_etb_ori_fifo = 0;
unsigned long nds32_otb_filesize = TRACER_OTB_FILESIZE;
unsigned long read_otb_already = 0, write2file_already = 1;
unsigned long nds32_tracer_backup_file = 0;
unsigned long nds32_tracer_backup_filesize = 0;
unsigned long nds32_tracer_on = 0;
unsigned long nds32_etb_id = 0, nds32_etm_id = 0, nds32_etb_depth = 0, nds32_etb_revision = 0;
unsigned long nds32_capture_cnt = 0, nds32_trigger_ctrl = 0, nds32_realtime_mode = 1;
unsigned long nds32_sync_period = ETM_SYNC_CTL_PERIOD_1KB, nds32_etm_mode = 0, nds32_etb_mode = 0;
unsigned long *pnds32_etb_buf_start = NULL, *pnds32_etb_buf_end = NULL;
unsigned long *pnds32_etb_wptr = NULL;
static unsigned long ETB_BASE[16];
static unsigned long ETM_BASE[16];
static unsigned long TPIU_support[16];
//static int nds32_tracer_aicemini_check(struct target *target);
static int nds32_tracer_dumpfile_otb(struct target *target, unsigned long, uint32_t);
static int nds32_tracer_list_subsystem(struct target *target);
static int nds32_tracer_read_id(struct target *target);
static int nds32_tracer_search_1stsync_lastsync(char *pSrcPkt, unsigned long total_pkt_bytes, char **p1stPkt);
static int nds32_tracer_copy_lastsync(char *plast_sync);
unsigned long nds32_cycle_accurate = 0;
unsigned long nds32_ras_depth = 0;

enum nds32_tracemode {
    NDS32_TARCEMODE_CIRCULAR = 0,
    NDS32_TRACEMODE_FULL,
    NDS32_TRACEMODE_CENTER,
    NDS32_TRACEMODE_NONREALTIME,
};
unsigned char nds32_trace_mode = NDS32_TRACEMODE_FULL;

int nds32_tracer_check(struct target *target)
{
	if (nds32_chk_tracer_support != 0) {
		if (nds32_chk_tracer_support == 0x01) {
			uint32_t coreid = target_to_coreid(target);
			if (ETB_BASE[coreid] == 0xFFFFFFFF) {
				LOG_DEBUG("Do NOT support ETB !!");
				return ERROR_FAIL;
			}
			return ERROR_OK;
		} else {
			return ERROR_FAIL;
		}
	}
	// check ICE_CONFIG.SDM, Indicate ICE-Box supports system debug module
	if ((aice_ice_config & (0x01 << 6)) == 0) {
		// check old version (AICE-MINI support tracer)
		//if (nds32_tracer_aicemini_check(target) != ERROR_OK) {
			nds32_chk_tracer_support = 0xFF;
			return ERROR_FAIL;
		//}
	}
	// check ICE_CONFIG.XRW, Indicate ICE-Box supports XREAD and XWRITE
	if (aice_ice_config & (0x01 << 5))
		nds32_chk_tracer_xrw_support = 1;

	if (nds32_tracer_list_subsystem(target) != ERROR_OK) {
		nds32_chk_tracer_support = 0xFF;
		return ERROR_FAIL;
	}
	
	nds32_tracer_read_id(target);
	LOG_DEBUG("etb_id = %lx, etm_id = %lx", nds32_etb_id, nds32_etm_id);
	LOG_DEBUG("etb_revision = %lx, etb_depth = 0x%lx bytes", nds32_etb_revision, nds32_etb_depth);
	LOG_DEBUG("nds32_otb_id = %lx", nds32_otb_id);
	LOG_DEBUG("nds32_tdrx_id = %lx", nds32_tdrx_id);

	nds32_chk_tracer_support = 0x01;
	pnds32_etb_buf_start = (unsigned long *)malloc(TRACER_TMP_BUFSIZE);
	pnds32_etb_buf_end = pnds32_etb_buf_start;
	pnds32_etb_buf_end += (TRACER_TMP_BUFSIZE >> 2);

	pnds32_etb_wptr = pnds32_etb_buf_start;
	nds32_capture_cnt = (nds32_etb_depth); /* set default: full-buffer mode */
	nds32_trace_mode = NDS32_TRACEMODE_FULL;
	nds32_realtime_mode = 1;
	// non-trigger as default
	nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_TRUE | ETM_TRIGGER_A_XOR_B);
	nds32_sync_period = ETM_SYNC_CTL_PERIOD_1KB;
	nds32_etm_mode = ETM_MODE_FULL_ACTION_DBGI;
	return ERROR_OK;
}

static int nds32_tracer_reg_read(struct target *target, unsigned long address)
{
	uint32_t ReadData = 0;

	aice_read_sdm(target, JDP_R_MEM_W, address, (uint32_t*)&ReadData, 1);
	return ReadData;
}

static int nds32_tracer_reg_write(struct target *target, unsigned long address, unsigned long value)
{
	uint32_t WriteData = value;
	aice_write_sdm(target, JDP_W_MEM_W, address, (uint32_t*)&WriteData, 1);
	return ERROR_OK;
}

int aice_xread_word(uint32_t lo_addr)
{
	uint32_t getData = 0;

	if (nds32_chk_tracer_xrw_support == 0)
		return 0;

	aice_xread(lo_addr, 0, &getData, 1, 0);
	return getData;
}
/*
static int nds32_tracer_aicemini_check(struct target *target)
{
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	nds32_etb_id = nds32_tracer_reg_read(target, 0x1000);
	nds32_etm_id = nds32_tracer_reg_read(target, 0x0000);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

	if ((nds32_etb_id == NDS32_ETBID) &&
		  (nds32_etm_id == NDS32_ETMID)) {
		return ERROR_OK;
	}
	return ERROR_FAIL;
}
*/
static int aice_xread_fixed(uint32_t lo_addr, uint32_t *pGetData, uint32_t fifo_words)
{
	uint32_t read_words;
	if (nds32_chk_tracer_xrw_support == 0)
		return 0;

	while(fifo_words) {
		// The max response packet length is 16KB
		if (fifo_words > 0xF00)
			read_words = 0xF00;
		else
			read_words = fifo_words;

		// attr=0x02, fixed address
		if (aice_xread(lo_addr, 0, pGetData, read_words, 0x02) != ERROR_OK)
			return ERROR_FAIL;

		fifo_words -= read_words;
		pGetData += read_words;
	}
	return ERROR_OK;
}

int aice_xwrite_word(uint32_t lo_addr, uint32_t value)
{
	uint32_t setData = value;
	if (nds32_chk_tracer_xrw_support == 0)
		return 0;

	aice_xwrite(lo_addr, 0, &setData, 1, 0x01); // attr=0x01, last command
	return ERROR_OK;
}

static int aice_init_otb(void)
{
	if (nds32_chk_tracer_xrw_support == 0)
		return ERROR_FAIL;

	aice_xwrite_word(XMEM_TDRX_CTL, 0x0);  // disable tdrx
	aice_xwrite_word(XMEM_TDRX_CTL, 0x1);  // enable tdrx
	aice_xwrite_word(XMEM_OTB_CTL, 0x0);  // disable otb
	aice_xwrite_word(XMEM_OTB_WPTR, 0);
	aice_xwrite_word(XMEM_OTB_RPTR, 0);
	aice_xwrite_word(XMEM_OTB_CTL, 0x1);  // enable otb

	return ERROR_OK;
}

static int nds32_tracer_list_subsystem(struct target *target)
{
	unsigned long i, idcode, base_addr = 0;
	unsigned long etb_index = 0;
	unsigned long etm_index = 0;

	for (i = 0; i < 16; i++) {
		TPIU_support[i] = 0;
		ETB_BASE[i] = 0xFFFFFFFF;
		ETM_BASE[i] = 0xFFFFFFFF;
	}

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);

	while(1) {
		idcode = nds32_tracer_reg_read(target, base_addr);

		if (idcode == NDS32_ETBID) {
			ETB_BASE[etb_index] = base_addr;
			etb_index ++;
		} else if (idcode == NDS32_ETMID) {
			ETM_BASE[etm_index] = base_addr;
			etm_index ++;
		} else if (idcode == NDS32_TPIUID) {
			ETB_BASE[etb_index] = base_addr;
			TPIU_support[etb_index] = 1;
			etb_index ++;
		} else {
			break;
		}
		base_addr += 0x1000;
	}

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	if (etb_index == 0)
		return ERROR_FAIL;

	nds32_otb_id = aice_xread_word(XMEM_OTB_ID);
	nds32_tdrx_id = aice_xread_word(XMEM_TDRX_ID);
	return ERROR_OK;
}

static int nds32_tracer_etbreg_read(struct target *target, unsigned long offset)
{
	uint32_t coreid = target_to_coreid(target);
	uint32_t address = ETB_BASE[coreid] + offset;
	return nds32_tracer_reg_read(target, address);
}

static int nds32_tracer_etbreg_write(struct target *target, unsigned long offset, unsigned long value)
{
	uint32_t coreid = target_to_coreid(target);
	uint32_t address = ETB_BASE[coreid] + offset;
	return nds32_tracer_reg_write(target, address, value);
}

static int nds32_tracer_etmreg_read(struct target *target, unsigned long offset)
{
	uint32_t coreid = target_to_coreid(target);
	uint32_t address = ETM_BASE[coreid] + offset;
	return nds32_tracer_reg_read(target, address);
}

static int nds32_tracer_etmreg_write(struct target *target, unsigned long offset, unsigned long value)
{
	uint32_t coreid = target_to_coreid(target);
	uint32_t address = ETM_BASE[coreid] + offset;
	return nds32_tracer_reg_write(target, address, value);
}

static int nds32_tracer_read_id(struct target *target)
{
	unsigned long etb_cfg = 0;
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);

	nds32_etb_id = nds32_tracer_etbreg_read(target, NDS_ETB_ID);
	nds32_etm_id = nds32_tracer_etmreg_read(target, NDS_ETM_ID);
	etb_cfg = nds32_tracer_etbreg_read(target, NDS_ETB_CFG1);
	nds32_etb_revision = (etb_cfg >> ETB_CFG1_REVISION_SHIFT) & ETB_CFG1_REVISION_MASK;
	nds32_etb_depth = (1 << ((etb_cfg & ETB_CFG1_DEPTH_MASK)+2));

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	nds32_etb_ori_fifo = nds32_etb_depth;
	return ERROR_OK;
}

int nds32_tracer_disable(struct target *target)
{
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	nds32_tracer_etbreg_write(target, NDS_ETB_CTL, 0);
	nds32_tracer_etmreg_write(target, NDS_ETM_CTL, 0);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

	nds32_capture_cnt = (nds32_etb_depth);
	nds32_trace_mode = NDS32_TRACEMODE_FULL;
	nds32_realtime_mode = 1;
	// non-trigger as default
	nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_TRUE | ETM_TRIGGER_A_XOR_B);
	nds32_sync_period = ETM_SYNC_CTL_PERIOD_1KB;
	nds32_tracer_on = 0;

	/* reset Trigger Event Control register 0 */
	aice_write_debug_reg(target, NDS_EDM_SR_TECR0, 0);

	/* Reset to buffer start */
	pnds32_etb_wptr = pnds32_etb_buf_start;

	return ERROR_OK;
}

static int nds32_tracer_setting(struct target *target)
{
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	nds32_tracer_etbreg_write(target, NDS_ETB_CTL, 0);
	nds32_tracer_etmreg_write(target, NDS_ETM_CTL, 0);
	/*-- Trigger Around breakpoint 0 --*/
	if (nds32_realtime_mode)
		nds32_etb_mode |= ETB_MODE_OVERWRITE;
	else
		nds32_etb_mode &= ~ETB_MODE_OVERWRITE;
	nds32_tracer_etbreg_write(target, NDS_ETB_MODE, nds32_etb_mode);
	nds32_tracer_etbreg_write(target, NDS_ETB_CAPTURE_CNT, (nds32_capture_cnt - 16));

	/*-- enable ETB --*/
	nds32_tracer_etbreg_write(target, NDS_ETB_WPTR, 0x0);
	nds32_tracer_etbreg_write(target, NDS_ETB_RPTR, 0x0);

	/*-- enable ETM --*/
	//nds32_tracer_etmreg_write(target, NDS_ETM_CTL, ETM_CTL_DISABLE);
	nds32_tracer_etmreg_write(target, NDS_ETM_TRIGGER_CTL, nds32_trigger_ctrl);
	nds32_tracer_etmreg_write(target, NDS_ETM_SYNC_CTL, nds32_sync_period);
	if (nds32_etm_mode & ETM_MODE_CYCLE_ACCURATE)
		nds32_cycle_accurate = 1;
	else
		nds32_cycle_accurate = 0;

	if (nds32_trigger_ctrl == (ETM_TRIGGER_EVENT_A_TRUE | ETM_TRIGGER_A_XOR_B)) {  // non-trigger
		nds32_etm_mode &= ~ETM_MODE_FULL_ACTION_MASK; // 0x0: no action
	} else {
		nds32_etm_mode |= ETM_MODE_FULL_ACTION_DBGI;
	}
	nds32_tracer_etmreg_write(target, NDS_ETM_MODE, nds32_etm_mode);
	nds32_tracer_etmreg_write(target, NDS_ETM_CTL, ETM_CTL_ENABLE);
	nds32_tracer_etbreg_write(target, NDS_ETB_CTL, ETB_CTL_ENABLE);
	/*-- read CFG1.RAS_DEPTH --*/
	unsigned long etm_cfg1 = 0;
	etm_cfg1 = nds32_tracer_etmreg_read(target, NDS_ETM_CFG1);
	nds32_ras_depth = (etm_cfg1 >> ETM_CFG1_RAS_DEPTH_SHIFT) & ETM_CFG1_RANGE_NUM_MASK;

	unsigned long trigger_cnt = nds32_tracer_etbreg_read(target, NDS_ETB_CAPTURE_CNT);
	LOG_DEBUG("nds32_etm_mode = %lx, trigger_cnt = %lx", nds32_etm_mode, trigger_cnt);
	LOG_DEBUG("nds32_trigger_ctrl = %lx, nds32_etb_mode = %lx", nds32_trigger_ctrl, nds32_etb_mode);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

	/*-- enable OTB --*/
	aice_init_otb();
	nds32_tracer_on = 1;
	pOTBPacketFile = NULL;
	nds32_tracer_backup_file = 0;

	seconds = time (NULL);
	LOG_TIME("OTB log-start: %s", ctime (&seconds));
	return ERROR_OK;
}

#if 0
static int nds32_tracer_get_trigger_rptr(struct target *target)
{
	unsigned long etb_rptr = 0, etb_wptr = 0, etb_cnt = 0;
	unsigned long etb_stat = 0, etm_stat = 0;

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	etb_stat = nds32_tracer_etbreg_read(target, NDS_ETB_STATUS);
	etm_stat = nds32_tracer_etmreg_read(target, NDS_ETM_STATUS);
	LOG_DEBUG("etb_stat = %lx, etm_stat = %lx", etb_stat, etm_stat);
	if ((etm_stat & ETM_STATUS_TRIGGERED) == 0) {
		LOG_DEBUG("NOT triggered !!");
		return ERROR_FAIL;
	}

	etb_wptr = nds32_tracer_etbreg_read(target, NDS_ETB_WPTR);
	etb_cnt = nds32_tracer_etbreg_read(target, NDS_ETB_CAPTURE_CNT);
	if (etb_wptr >= (nds32_capture_cnt - etb_cnt)) {
		etb_rptr = etb_wptr - (nds32_capture_cnt - etb_cnt);
	} else {
		etb_rptr = nds32_etb_depth - ((nds32_capture_cnt - etb_cnt) - etb_wptr);
	}
	LOG_DEBUG("etb_rptr = %lx, etb_wptr = %lx", etb_rptr, etb_wptr);
	//nds32_tracer_etbreg_write(target, NDS_ETB_RPTR, etb_rptr);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	return ERROR_OK;
}
#endif

static int nds32_tracer_clear_etm_fifo(struct target *target)
{
	// clear ETB_CTL_PAUSE
	nds32_tracer_etbreg_write(target, NDS_ETB_CTL, ETB_CTL_ENABLE);
	// disable ETM
	nds32_tracer_etmreg_write(target, NDS_ETM_CTL, ETM_CTL_DISABLE);
	// wait ETM.EMPTY
	unsigned long etm_stat = nds32_tracer_etmreg_read(target, NDS_ETM_STATUS);
	if ((etm_stat & ETM_STATUS_EMPTY) == 0) {
		LOG_DEBUG("ETM FIFO NOT EMPTY");
	}
	return ERROR_OK;
}

int nds32_tracer_read_etb(struct target *target)
{
	unsigned long etb_rptr = 0, etb_wptr = 0, etb_stat = 0;
	unsigned long fifo_words, i;

	if (nds32_tracer_check(target) != ERROR_OK) {
		LOG_DEBUG("Do NOT support tracer !!");
		return ERROR_FAIL;
	}

	uint32_t coreid = target_to_coreid(target);
	if (TPIU_support[coreid] == 1) {
		LOG_DEBUG("dump OTB when dumpfile !!");
		return ERROR_OK;
	}

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	//unsigned long etm_stat = nds32_tracer_etmreg_read(target, NDS_ETM_STATUS);
	//if ((etm_stat & ETM_STATUS_TRIGGERED) == 0) {
	//	LOG_DEBUG("NOT triggered !!");
	//	return ERROR_FAIL;
	//}

	// read etb_stat before nds32_tracer_clear_etm_fifo() avoid clear ETB_STATUS_WRAP
	etb_stat = nds32_tracer_etbreg_read(target, NDS_ETB_STATUS);

	// clear ETM FIFO
	if (nds32_realtime_mode == 1) {
		nds32_tracer_clear_etm_fifo(target);
	}
	etb_wptr = nds32_tracer_etbreg_read(target, NDS_ETB_WPTR);

	if (etb_stat & ETB_STATUS_WRAP) {
		etb_rptr = etb_wptr;
		fifo_words = nds32_etb_depth;
	} else {
		etb_rptr = 0;
		fifo_words = etb_wptr;
	}
	fifo_words >>= 2;
	nds32_tracer_etbreg_write(target, NDS_ETB_RPTR, etb_rptr);

	/* copy pkt from ETB to tmp buffer */
	unsigned long *pcurr_wptr = (unsigned long *)pnds32_etb_wptr;
	pcurr_wptr += fifo_words;
	LOG_DEBUG("pnds32_etb_wptr = 0x%lx, pnds32_etb_buf_end = 0x%lx, fifo_words = 0x%lx", (unsigned long)pnds32_etb_wptr, (unsigned long)pnds32_etb_buf_end, fifo_words);
	if (pcurr_wptr >= pnds32_etb_buf_end) {
		LOG_DEBUG("PKT BUF FULL !! need to dump file");
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
		return ERROR_FAIL;
	} else {
		for (i = 0; i < fifo_words; i++) {
			*pnds32_etb_wptr ++ = nds32_tracer_etbreg_read(target, NDS_ETB_RDATA);
		}
		LOG_DEBUG("etb_wptr = 0x%lx, etb_rptr = 0x%lx, fifo_words = 0x%lx", etb_wptr, etb_rptr, fifo_words);
	}
	nds32_tracer_etbreg_write(target, NDS_ETB_WPTR, 0x0);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
	return ERROR_OK;
}

static int nds32_tracer_polling_otb(struct target *target)
{
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	unsigned long etb_stat = nds32_tracer_etbreg_read(target, NDS_ETB_STATUS);
	LOG_DEBUG("tpiu_stat = %lx", etb_stat);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

	if (etb_stat & ETB_STATUS_PAUSED) {
		seconds = time (NULL);
		LOG_TIME("OTB log-finish: %s", ctime (&seconds));

		struct nds32 *nds32 = target_to_nds32(target);
		nds32->tracer_pause = true;
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
		nds32_tracer_clear_etm_fifo(target);
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

		nds32_tracer_dumpfile_otb(target, 0, OTB_CLOSE_FILE);
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
		nds32_tracer_etbreg_write(target, NDS_ETB_CTL, 0);
		nds32_tracer_etmreg_write(target, NDS_ETM_CTL, 0);
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
		return ERROR_OK;
	}
	unsigned long otb_wptr = 0, otb_rptr = 0;
	unsigned long fifo_words;

	otb_wptr = aice_xread_word(XMEM_OTB_WPTR);
	otb_rptr = aice_xread_word(XMEM_OTB_RPTR);
	if (otb_wptr >= otb_rptr)
		fifo_words = otb_wptr - otb_rptr;
	else {
		/* overwrite 512MB */
		fifo_words = (nds32_etb_ori_fifo - otb_rptr);
		fifo_words += otb_wptr;
	}

	// dump MaxFileSize data in fifo if fifo_words > MaxFileSize
	if (fifo_words > nds32_otb_filesize) {
		if ((fifo_words + write2file_already) > (nds32_etb_depth/2)) {
			nds32_tracer_dumpfile_otb(target, 0, OTB_BACKUP_CLOSE_FILE);
		} else {
			nds32_tracer_dumpfile_otb(target, nds32_otb_filesize, OTB_NOT_CLOSE);
		}
		LOG_DEBUG("otb_wptr = %lx, otb_rptr = %lx, fifo_words = %lx",
			otb_wptr, otb_rptr, fifo_words);
	} else if (fifo_words > nds32_etb_depth) {
		/* skip packet in AICE2-T*/
		while(fifo_words > nds32_etb_depth) {
			fifo_words --;
			otb_rptr ++;
			if (otb_rptr == nds32_etb_ori_fifo)
				otb_rptr = 0;
		}
		LOG_DEBUG("skip packet in AICE2-T, otb_wptr = 0x%lx, otb_rptr = 0x%lx", otb_wptr, otb_rptr);
		aice_xwrite_word(XMEM_OTB_RPTR, otb_rptr);
	}
	return ERROR_OK;
}

int nds32_tracer_polling(struct target *target)
{
	unsigned long etb_stat = 0, etm_stat = 0;
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32_tracer_on == 0) {
		return ERROR_FAIL;
	}
	uint32_t coreid = target_to_coreid(target);
	if (TPIU_support[coreid] == 1) {
		//LOG_DEBUG("nds32_tracer_polling_otb() !!");
		return nds32_tracer_polling_otb(target);
	}

	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
	etb_stat = nds32_tracer_etbreg_read(target, NDS_ETB_STATUS);
	etm_stat = nds32_tracer_etmreg_read(target, NDS_ETM_STATUS);
	LOG_DEBUG("etb_stat = %lx, etm_stat = %lx", etb_stat, etm_stat);
	aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

	if (etb_stat & ETB_STATUS_PAUSED) {
		//unsigned long etb_wptr = 0;
		//etb_wptr = nds32_tracer_etbreg_read(target, NDS_ETB_WPTR);
		//etb_stat = nds32_tracer_etbreg_read(target, NDS_ETB_STATUS);
		//LOG_DEBUG("etb_wptr = %lx, etb_stat = %lx", etb_wptr, etb_stat);
		int retvalue = nds32_tracer_read_etb(target);
		nds32->tracer_pause = true;

		if (nds32_realtime_mode == 0) {
			aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
			nds32_tracer_etbreg_write(target, NDS_ETB_CAPTURE_CNT, nds32_capture_cnt - 16);
			/* PAUSE bit1	  0: resume trace capture */
			nds32_tracer_etbreg_write(target, NDS_ETB_WPTR, 0x0);
			nds32_tracer_etbreg_write(target, NDS_ETB_CTL, ETB_CTL_ENABLE);
			aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);

			// non-realtime mode, freerun until buffer pnds32_etb_buf_start full or trigger
			if (etm_stat & ETM_STATUS_TRIGGERED) {
				LOG_DEBUG("non-realtime mode, trigger");
				return ERROR_OK;
			}
			else if (retvalue == ERROR_OK) {
				LOG_DEBUG("pnds32_etb_wptr = %lx", (unsigned long)pnds32_etb_wptr);
				// buffer enough, freerun
				aice_run_target(target);
				nds32->tracer_pause = false;
			} else {
				LOG_DEBUG("non-realtime mode, buffer full");
			}
		}
	}
	return ERROR_OK;
}

#define TRACER_IVB_BUFSIZE   256*64  // 256(IVB.ESZ)*64(entry number)
unsigned char ivb_table[TRACER_IVB_BUFSIZE];
extern uint32_t nds32_curr_ir3_value;
#define LINEBUF_SIZE        1024
#define TITLE_IVB       "<_INTERRUPT_VECTOR_>:"
uint32_t ivb_ESZ[4] = {
	4, 16, 64, 256 };
uint32_t ivb_NIVIC[8] = {
	6, 2, 10, 16, 24, 32, 64, 128 };

static int nds32_tracer_dump_IVBinfo(struct target *target, char *pPktFileName)
{
	char ivb_filename[256];
	char *pOutFileName = (char *)&ivb_filename[0];
	FILE *pIVBFile = NULL;

	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t ir3_value = 0, ir3_ivb_base = 0, ir3_ivb_size = 32;
	uint32_t ir3_ivb_esz, ir3_ivb_nivic;

	// maybe NOT halt
	// nds32_get_mapped_reg(nds32, IR3, &ir3_value);
	ir3_value = nds32_curr_ir3_value;
	ir3_ivb_base = (ir3_value & 0xFFFF0000);
	ir3_ivb_esz = ivb_ESZ[((ir3_value & 0x0000C000) >> 14)];
	ir3_ivb_nivic = ivb_NIVIC[((ir3_value & 0x0000000E) >> 1)];
	ir3_ivb_size = ir3_ivb_esz * ir3_ivb_nivic;
	LOG_DEBUG("ir3_value = 0x%x, ir3_ivb_base = 0x%x, ir3_ivb_size = 0x%x",
		ir3_value, ir3_ivb_base, ir3_ivb_size);

	/* switch to BUS access mode */
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	aice_read_mem_bulk(target, ir3_ivb_base, TRACER_IVB_BUFSIZE, (uint8_t *)&ivb_table[0]);
	nds32->memory.access_channel = orig_channel;

	memcpy(&ivb_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&ivb_filename[strlen((char *)pPktFileName)], "-ivb", 5);
	pIVBFile = fopen(pOutFileName, "wb");
	if (pIVBFile == NULL) {
		LOG_DEBUG("%s open FAIL !!", pOutFileName);
		return ERROR_FAIL;
	}

	char line_string[LINEBUF_SIZE];
	char *pline_string = (char *)&line_string[0];
	unsigned long curr_pc = 0, next_pc = 0, max_pc = 0;
	struct nds32_instruction instruction;

	curr_pc = ir3_ivb_base;
	next_pc = ir3_ivb_base;
	max_pc = ir3_ivb_base + ir3_ivb_size;
	sprintf(pline_string, "%08x %s\n", ir3_ivb_base, TITLE_IVB);
	fwrite((char *)pline_string, 1, strlen(pline_string), pIVBFile);

	while(1) {
		nds32_evaluate_opcode(nds32, curr_pc, &instruction);
		if (instruction.instruction_size == 2) {
			next_pc += 2;
			//sprintf(pline_string, "%08lx: %02x %02x        %s\n", (curr_pc + ir3_ivb_base),
			//	p_opcode[curr_pc], p_opcode[curr_pc+1], instruction.text);
		} else {
			next_pc += 4;
			//sprintf(pline_string, "%08lx: %02x %02x %02x %02x  %s\n", (curr_pc + ir3_ivb_base),
			//	p_opcode[curr_pc], p_opcode[curr_pc+1], p_opcode[curr_pc+2], p_opcode[curr_pc+3], instruction.text);
		}
		sprintf(pline_string, " %s\n", instruction.text);
		fwrite((char *)pline_string, 1, strlen(pline_string), pIVBFile);
		curr_pc = next_pc;
		if (next_pc >= max_pc)
			break;
	}
	fclose(pIVBFile);
	return ERROR_OK;
}

unsigned long sync_pkt_counter = 0;
unsigned long last_sync_pkt_length = 0;
unsigned long sync_pkt_counter_prev = 0;
unsigned long sync_pkt_offset[0x300000];
static int nds32_tracer_dump_syncinfo(struct target *target, char *pPktFileName)
{
	char *pkt_buf;
	char sync_filename[256];
	char *pOutFileName = (char *)&sync_filename[0];
	FILE *pSyncFile = NULL;

	memcpy(&sync_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&sync_filename[strlen((char *)pPktFileName)], "-sy", 4);
	pSyncFile = fopen(pOutFileName, "wb");

	// skip the last-sync-pkt offset
	//if (sync_pkt_counter > 0)
	//	sync_pkt_counter --;

	if (pSyncFile) {
		pkt_buf = (char *)&sync_pkt_counter;
		fwrite((char *)pkt_buf, 1, 4, pSyncFile);
		pkt_buf = (char *)&sync_pkt_offset[0];
		fwrite((char *)pkt_buf, 1, (sync_pkt_counter*4), pSyncFile);
		fclose(pSyncFile);
	}
	// reset value for next time
	sync_pkt_counter_prev = sync_pkt_counter;
	sync_pkt_counter = 0;
	write2file_already = 1;  // fputc(parameter, pPacketFile);

	return ERROR_OK;
}

static int nds32_tracer_dumpfile_otb(struct target *target, unsigned long MaxFileSize,
	uint32_t if_close_file)
{
	unsigned long otb_wptr = 0, otb_rptr = 0;
	unsigned long fifo_words, read_words;
	uint32_t *pGetData;
	char *p1st_sync = NULL;
	unsigned long total_pkt_bytes = 0;
	unsigned long dump_pkt_bytes = 0;
	char *pbuf_start;

	otb_wptr = aice_xread_word(XMEM_OTB_WPTR);
	otb_rptr = aice_xread_word(XMEM_OTB_RPTR);
	if (otb_wptr >= otb_rptr)
		fifo_words = otb_wptr - otb_rptr;
	else {
		/* overwrite 512MB */
		fifo_words = (nds32_etb_ori_fifo - otb_rptr);
		fifo_words += otb_wptr;
	}

	/* skip packet in AICE2-T*/
	if (fifo_words > nds32_etb_depth) {
		while(fifo_words > nds32_etb_depth) {
			fifo_words --;
			otb_rptr ++;
			if (otb_rptr == nds32_etb_ori_fifo)
				otb_rptr = 0;
		}
		LOG_DEBUG("skip packet in AICE2-T, otb_wptr = 0x%lx, otb_rptr = 0x%lx", otb_wptr, otb_rptr);
		aice_xwrite_word(XMEM_OTB_RPTR, otb_rptr);
		if (pOTBPacketFile) {
			fclose(pOTBPacketFile);
			pOTBPacketFile = NULL;
		}
		/* reset to buffer start */
		pnds32_etb_wptr = pnds32_etb_buf_start;
	}

	// dump all data in fifo if MaxFileSize = 0
	if (MaxFileSize != 0) {
		if (fifo_words > MaxFileSize)
			fifo_words = MaxFileSize;
	}
	fifo_words >>= 2;
	LOG_DEBUG("otb_wptr = 0x%lx, otb_rptr = 0x%lx",
	  (unsigned long)otb_wptr, (unsigned long)otb_rptr);

	if (fifo_words == 0)
		return ERROR_OK;

	LOG_DEBUG("pnds32_etb_wptr = 0x%lx, pnds32_etb_buf_end = 0x%lx, fifo_words = 0x%lx",
	  (unsigned long)pnds32_etb_wptr, (unsigned long)pnds32_etb_buf_end, fifo_words);

	char *pFileName = (char *)TRACER_OTB_FILENAME;
	if (pOTBPacketFile == NULL) {
		seconds = time (NULL);
		LOG_TIME("OTB dump-start: %s", ctime (&seconds));
		compare_words = nds32_otb_filesize/8;

		pOTBPacketFile = fopen(pFileName, "wb");
		if (pOTBPacketFile == NULL)
			return ERROR_FAIL;
		LOG_DEBUG("dump otb file = %s, MaxFileSize = 0x%lx", pFileName, MaxFileSize);

		// for testing
		//for (uint32_t i = 0; i < 10; i++)
		//	fwrite(&ETB_BASE[0], 1, 0x600000, pOTBPacketFile);
		//fclose(pOTBPacketFile);
		//return ERROR_OK;

		// Prepare Parameter
		// bits[0]: cycle_accurate
		// bits[3:1]: RAS_DEPTH
		// bits[5:4]: Trace Mode
		char parameter = 0;
		parameter |= (nds32_cycle_accurate&0x1) << 0;
		parameter |= (nds32_ras_depth&0x7)      << 1;
		parameter |= (nds32_trace_mode&0x3)     << 4;
		// Write Parameter in file
		fputc(parameter, pOTBPacketFile);

		// reset value
		sync_pkt_counter = 0;
		write2file_already = 1;  // fputc(parameter, pPacketFile);
	}

	while(fifo_words) {		
		if (fifo_words > nds32_fifo_words_per_transfer)
			read_words = nds32_fifo_words_per_transfer;
		else
			read_words = fifo_words;

		/* copy pkt from OTB to tmp buffer */
		pGetData = (uint32_t *)pnds32_etb_wptr;
		aice_xread_fixed(XMEM_OTB_DATA, pGetData, read_words);
		fifo_words -= read_words;
		pnds32_etb_wptr += read_words;
		read_otb_already += (read_words << 2);

		/* only dump the 1st pkt-SYNC to last pkt-SYNC */
		total_pkt_bytes = (unsigned long)(pnds32_etb_wptr - pnds32_etb_buf_start);
		total_pkt_bytes <<= 2;
		dump_pkt_bytes = total_pkt_bytes;
 
		pbuf_start = (char *)pnds32_etb_buf_start;
		p1st_sync = pbuf_start;

		if (nds32_skip_find_1stsync == 0) {
			LOG_DEBUG("pbuf_start = 0x%lx, total_pkt_bytes = 0x%lx", (unsigned long)pbuf_start, total_pkt_bytes);
			dump_pkt_bytes = nds32_tracer_search_1stsync_lastsync(pbuf_start, total_pkt_bytes, &p1st_sync);
		}
		/* NOT found the 1st pkt-SYNC */
		if (p1st_sync == 0) {
			fclose(pOTBPacketFile);
			return ERROR_OK;
		}

		if (dump_pkt_bytes) {
			fwrite(p1st_sync, 1, dump_pkt_bytes, pOTBPacketFile);
			write2file_already += dump_pkt_bytes;
			// for testing
			//unsigned long write_cnt=0;
			//write_cnt ++;
			//if (write_cnt == 2)
			//	break;
		}
		if (nds32_skip_find_1stsync == 0) {
			p1st_sync += dump_pkt_bytes;
			nds32_tracer_copy_lastsync(p1st_sync);
		} else {
			pnds32_etb_wptr = pnds32_etb_buf_start;
		}
		if (write2file_already > compare_words) {
			compare_words += compare_words;
			LOG_TIME("write2file: 0x%08lx bytes", write2file_already);
		}
	}
	LOG_DEBUG("write2file_already = 0x%lx, read_otb_already = 0x%lx",
	  (unsigned long)write2file_already, (unsigned long)read_otb_already);

	if (if_close_file == OTB_CLOSE_FILE) {
		// write the last-sync-pkt to file
		if (nds32_skip_find_1stsync == 0) {
			if (last_sync_pkt_length)
				fwrite(p1st_sync, 1, last_sync_pkt_length, pOTBPacketFile);
			// write remain packet to file
			//unsigned long remain_bytes = (total_pkt_bytes - dump_pkt_bytes);
			//fwrite(p1st_sync, 1, remain_bytes, pOTBPacketFile);
			//LOG_DEBUG("remain_bytes = 0x%lx", remain_bytes);
		}
		fclose(pOTBPacketFile);
		pOTBPacketFile = NULL;
		nds32_tracer_dump_syncinfo(target, pFileName);

		seconds = time (NULL);
		LOG_TIME("OTB dump-end: %s", ctime (&seconds));
	}
	else if (if_close_file == OTB_BACKUP_CLOSE_FILE) {
		fclose(pOTBPacketFile);
		pOTBPacketFile = NULL;
		nds32_tracer_backup_filesize = write2file_already;
		char *pBakFileName = (char *)TRACER_OTB_BAK_FILENAME;
		int result= rename(pFileName, pBakFileName);
		if ( result == 0 ) {
			nds32_tracer_dump_syncinfo(target, pBakFileName);
			LOG_DEBUG("File successfully renamed");
			nds32_tracer_backup_file = 1;
		}

		seconds = time (NULL);
		LOG_TIME("OTB dump-end(backup): %s", ctime (&seconds));
	}
	return ERROR_OK;
}

static int nds32_tracer_mergefile_otb(char *pPktFileName)
{
	seconds = time (NULL);
	LOG_TIME("OTB mergefile-start: %s", ctime (&seconds));

	char sync_filename[256];
	char *pSyncFileName = (char *)&sync_filename[0];
	memcpy(&sync_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&sync_filename[strlen((char *)pPktFileName)], "-sy", 4);

	char *pPktBakName = (char *)TRACER_OTB_FILENAME;
	char *pSyncBakName = (char *)TRACER_SYN_FILENAME;
	if (nds32_tracer_backup_file == 1) {
		pPktBakName = (char *)TRACER_OTB_BAK_FILENAME;
		pSyncBakName = (char *)TRACER_SYN_BAK_FILENAME;
	}

	int result= rename(pPktBakName , pPktFileName);
	result= rename(pSyncBakName , pSyncFileName);
	if ( result == 0 )
		LOG_DEBUG("File successfully renamed");
	else
		LOG_DEBUG("Error renaming file");

	if (nds32_tracer_backup_file == 1) {
		char *pSrcFileName = (char *)TRACER_OTB_FILENAME;
		FILE *pSrcFile = fopen(pSrcFileName, "rb");
		FILE *pDstFile = fopen(pPktFileName, "a");
		char *curr_buf = (char *)pnds32_etb_buf_start;
		uint32_t i, buffer_size = TRACER_TMP_BUFSIZE;

		if ((pSrcFile == NULL) || (pDstFile == NULL)) {
			LOG_DEBUG("Error mergefile !!");
			return ERROR_FAIL;
		}
		/* skip the 1st-byte for the parameter */
		uint32_t read_size = fread(curr_buf, 1, 1, pSrcFile);

		while(1) {
			read_size = fread(curr_buf, 1, buffer_size, pSrcFile);
			if (read_size == 0)
				break;
			fwrite((char *)curr_buf, 1, read_size, pDstFile);
		}
		fclose(pSrcFile);
		fclose(pDstFile);

		// merge otb-pkt-bak.txt-sy & otb-pkt.txt-sy
		pDstFile = fopen(pSyncFileName, "r+b");
		if (pDstFile == NULL) {
			LOG_DEBUG("Error mergefile (sy) !!");
			return ERROR_FAIL;
		}
		LOG_DEBUG("sync_pkt_counter_prev: 0x%lx", sync_pkt_counter_prev);
		LOG_DEBUG("nds32_tracer_backup_filesize: 0x%lx", nds32_tracer_backup_filesize);

		for (i = 0; i < sync_pkt_counter_prev; i++) {
			sync_pkt_offset[i] += (nds32_tracer_backup_filesize - 1);
		}

		curr_buf = (char *)&sync_pkt_offset[0];
		fseek(pDstFile, 0, SEEK_END);
		fwrite((char *)curr_buf, 1, (sync_pkt_counter_prev*4), pDstFile);
		//fclose(pDstFile);

		// update sync_pkt_counter in xxx-sy
		//pDstFile = fopen(pSyncFileName, "r");
		uint32_t sync_pkt_cnt = 0;
		fseek(pDstFile, 0, SEEK_SET);
		read_size = fread((char *)&sync_pkt_cnt, 1, 4, pDstFile);
		LOG_DEBUG("sync_pkt_cnt: 0x%x", sync_pkt_cnt);
		sync_pkt_cnt += sync_pkt_counter_prev;
		fseek(pDstFile, 0, SEEK_SET);
		fwrite((char *)&sync_pkt_cnt, 1, 4, pDstFile);
		fclose(pDstFile);
	}
	seconds = time (NULL);
	LOG_TIME("OTB mergefile-end: %s", ctime (&seconds));
	return ERROR_OK;
}

static int nds32_tracer_dumpfile(struct target *target, char *pFileName)
{
	uint32_t coreid = target_to_coreid(target);
	if (TPIU_support[coreid] == 1) {
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_SDM);
		nds32_tracer_clear_etm_fifo(target);
		aice_write_ctrl(AICE_WRITE_CTRL_SDMCONN, NDS_SDM_SELECT_DAISY_CHAIN);
		int result = nds32_tracer_dumpfile_otb(target, 0, OTB_CLOSE_FILE);
		nds32_tracer_mergefile_otb(pFileName);

		// output IVB-table-file
		nds32_tracer_dump_IVBinfo(target, pFileName);
		return result;
	}

	unsigned long total_pkt_bytes = 0;
	FILE *pPacketFile = NULL;
	pPacketFile = fopen(pFileName, "wb");
	if (pPacketFile == NULL)
		return ERROR_FAIL;

	// Prepare Parameter
	// bits[0]: cycle_accurate 
	// bits[3:1]: RAS_DEPTH
	// bits[5:4]: Trace Mode
	char parameter = 0;
	parameter |= (nds32_cycle_accurate&0x1) << 0;
	parameter |= (nds32_ras_depth&0x7)      << 1;
	parameter |= (nds32_trace_mode&0x3)     << 4;
	// Write Parameter in file
	fputc(parameter, pPacketFile);
	
	char *pbuf_start = (char *)pnds32_etb_buf_start;
	/* get data from ETB */
	nds32_tracer_read_etb(target);

	if (pnds32_etb_wptr != pnds32_etb_buf_start) {
		total_pkt_bytes = (pnds32_etb_wptr - pnds32_etb_buf_start);
		total_pkt_bytes <<= 2;
	}

	LOG_DEBUG("pnds32_etb_wptr = 0x%lx, total_pkt_bytes = 0x%lx",
		(unsigned long)pnds32_etb_wptr, total_pkt_bytes);
	if( total_pkt_bytes == 0 ) {
		LOG_DEBUG("Buffer empty!!");
		fclose(pPacketFile);
		return ERROR_OK;
	}

#if 1
	unsigned long dump_pkt_bytes = 0;
	/* only dump the 1st pkt-SYNC to last pkt-SYNC */
	if (nds32_skip_find_1stsync == 0) {
		char *p1st_sync = NULL;
		//pbuf_start += 2;  // for testing
		dump_pkt_bytes = nds32_tracer_search_1stsync_lastsync(pbuf_start, total_pkt_bytes, &p1st_sync);
		LOG_DEBUG("pbuf_start = 0x%lx, p1st_sync = 0x%lx, dump_pkt_bytes = 0x%lx", (unsigned long)pbuf_start, (unsigned long)p1st_sync, dump_pkt_bytes);
		pbuf_start = p1st_sync;
		total_pkt_bytes = dump_pkt_bytes;
	} else {
		/* reset to buffer start */
		pnds32_etb_wptr = pnds32_etb_buf_start;
	}
#else
	/* reset to buffer start */
	pnds32_etb_wptr = pnds32_etb_buf_start;
#endif

	/* NOT found the 1st pkt-SYNC */
	if (pbuf_start == 0) {
		fclose(pPacketFile);
		return ERROR_OK;
	}

	if (total_pkt_bytes) {
		fwrite(pbuf_start, 1, total_pkt_bytes, pPacketFile);
	}

	if (nds32_skip_find_1stsync == 0) {
		pbuf_start += dump_pkt_bytes;
		nds32_tracer_copy_lastsync(pbuf_start);
	}
	fclose(pPacketFile);

	nds32_tracer_dump_syncinfo(target, pFileName);
	// output IVB-table-file
	nds32_tracer_dump_IVBinfo(target, pFileName);

	return ERROR_OK;
}

/* ============================================================================= */
/*    packet parser                                                              */
/*         input: packet file (pkt.txt)                                          */
/*         output: packet-decoded file (pkt.txt-de)                              */
/* ============================================================================= */
#define TRACER_DECODE_MSG(x)  //LOG_DEBUG x
#define PACKET_BUF_SIZE       0x100000  // 1MB
#define PKT_NUMS    7
#define PKT_SYNC    "PKT-SYNC:"
#define PKT_DIR     "PKT-DIR:"
#define PKT_RA      "PKT-RA:"
#define PKT_CID     "PKT-CID:"
#define PKT_CPC     "PKT-CPC:"
#define PKT_NPC     "PKT-NPC:"
#define PKT_CYCLE   "PKT-CYCLE:"

FILE *gpPktDecodedFile = NULL;
char *gp_curr_sync = NULL;
char *gp_1st_sync = NULL;
char str_decoded_data[256];
char pkt_decoded_filename[256];
unsigned long nds32_packet_id = 0;

static int nds32_tracer_decode_sync(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 6, more_bit = 0;
	unsigned long reason = 0, istatus = 0;

	if ((pCurrPkt[0] != (char)0x80) ||
		((pCurrPkt[1] & 0x81) != 0x81) ||
		((pCurrPkt[2] & 0x80) != 0x80) ||
		((pCurrPkt[3] & 0x80) != 0x80) ||
		((pCurrPkt[4] & 0x80) != 0x80)) {
		TRACER_DECODE_MSG(("NOT PKT SYNC"));
		return 0;
	}
	gp_curr_sync = pSrcPkt;
	// LOG_DEBUG("gp_curr_sync = 0x%lx", (unsigned long)gp_curr_sync);

	if (gp_1st_sync == NULL) {
		// the 1st SYNC pkt offset in the pkt file
		sync_pkt_offset[sync_pkt_counter] = write2file_already;
	} else {
		// the xxth SYNC pkt offset in the pkt file
		sync_pkt_offset[sync_pkt_counter] = (unsigned long)(gp_curr_sync - gp_1st_sync);
		sync_pkt_offset[sync_pkt_counter] += write2file_already;
	}
	sync_pkt_counter ++;
	//TRACER_DECODE_MSG(("%s Pkt=0x%lx, 0x%lx, 0x%lx", PKT_SYNC, pCurrPkt[0], pCurrPkt[1], pCurrPkt[2]));
	address = (pCurrPkt[1] & 0x7E);
	address |= (pCurrPkt[2] & 0x7F) << 7;
	address |= (pCurrPkt[3] & 0x7F) << 14;
	address |= (pCurrPkt[4] & 0x7F) << 21;
	address |= (pCurrPkt[5] & 0x0F) << 28;

	reason = (pCurrPkt[5] & 0x70) >> 4;
	more_bit = (pCurrPkt[5] & 0x80);
	if (more_bit) {
		istatus = (pCurrPkt[6] & 0x7F);
		pkt_length ++;
	}
	last_sync_pkt_length = pkt_length;

	sprintf(str_decoded_data, "0x%08lx %s ADDR=0x%lx, REASON=0x%lx, ISTAT=0x%lx\n",
		nds32_packet_id, PKT_SYNC, address, reason, istatus);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_cpc(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 2, more_bit = 0;
	unsigned long shift_bit = 7, le_bit = 0;

	if (pCurrPkt[0] != (char)0x82) {
		TRACER_DECODE_MSG(("NOT %s", PKT_CPC));
		return 0;
	}
	pCurrPkt ++;
	address = (pCurrPkt[0] & 0x7E);
	more_bit = (pCurrPkt[0] & 0x80);
	le_bit = (pCurrPkt[0] & 0x01);
	while (more_bit) {
		pCurrPkt ++;
		more_bit = (pCurrPkt[0] & 0x80);
		address |= (pCurrPkt[0] & 0x7F) << shift_bit;
		shift_bit += 7;
		pkt_length ++;
	}

	sprintf(str_decoded_data, "0x%08lx %s ADDR=0x%lx, BITS=0x%lx, LE=0x%lx\n",
		nds32_packet_id, PKT_CPC, address, shift_bit, le_bit);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_ra(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 2, more_bit = 0;
	unsigned long shift_bit = 7;

	if ((pCurrPkt[0] != (char)0x84) ||
		((pCurrPkt[1] & 0x01) != 0x01)) {
		TRACER_DECODE_MSG(("NOT %s", PKT_RA));
		return 0;
	}
	pCurrPkt ++;
	address = (pCurrPkt[0] & 0x7E);
	more_bit = (pCurrPkt[0] & 0x80);
	while (more_bit) {
		pCurrPkt ++;
		more_bit = (pCurrPkt[0] & 0x80);
		address |= (pCurrPkt[0] & 0x7F) << shift_bit;
		shift_bit += 7;
		pkt_length ++;
	}

	sprintf(str_decoded_data, "0x%08lx %s ADDR=0x%lx, BITS=0x%lx\n",
		nds32_packet_id, PKT_RA, address, shift_bit);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_cid(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 3;
	unsigned long shift_bit = 7;

	if ((pCurrPkt[0] != (char)0x86) ||
		((pCurrPkt[1] & 0x80) != 0x80)) {
		TRACER_DECODE_MSG(("NOT %s", PKT_CID));
		return 0;
	}
	pCurrPkt ++;
	address = (pCurrPkt[0] & 0x7F);
	address |= (pCurrPkt[1] & 0x03) << shift_bit;

	sprintf(str_decoded_data, "0x%08lx %s CID=0x%lx\n",
		nds32_packet_id, PKT_CID, address);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_npc(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 1, more_bit = 0;
	unsigned long shift_bit = 7;

	if ((pCurrPkt[0] & 0x01) != 0x01) {
		TRACER_DECODE_MSG(("NOT %s", PKT_NPC));
		return 0;
	}
	address = (pCurrPkt[0] & 0x7E);
	more_bit = (pCurrPkt[0] & 0x80);
	while (more_bit) {
		pCurrPkt ++;
		more_bit = (pCurrPkt[0] & 0x80);
		address |= (pCurrPkt[0] & 0x7F) << shift_bit;
		shift_bit += 7;
		pkt_length ++;
	}

	sprintf(str_decoded_data, "0x%08lx %s ADDR=0x%lx, BITS=0x%lx\n",
		nds32_packet_id, PKT_NPC, address, shift_bit);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_dir(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 1;
	unsigned long dir_bit = 0;

	if ((pCurrPkt[0] & 0x81) != 0x00) {
		TRACER_DECODE_MSG(("NOT %s", PKT_DIR));
		return 0;
	}
	if ((pCurrPkt[0] & 0xFD) == 0x04) {
		address = (pCurrPkt[0] & 0x02) >> 1;
		dir_bit = 1;
	} else if ((pCurrPkt[0] & 0xF9) == 0x08) {
		address = (pCurrPkt[0] & 0x06) >> 1;
		dir_bit = 2;
	} else if ((pCurrPkt[0] & 0xF1) == 0x10) {
		address = (pCurrPkt[0] & 0x0E) >> 1;
		dir_bit = 3;
	} else if ((pCurrPkt[0] & 0xE1) == 0x20) {
		address = (pCurrPkt[0] & 0x1E) >> 1;
		dir_bit = 4;
	} else if ((pCurrPkt[0] & 0xC1) == 0x40) {
		address = (pCurrPkt[0] & 0x3E) >> 1;
		dir_bit = 5;
	}

	sprintf(str_decoded_data, "0x%08lx %s DIR=0x%lx, directions=0x%lx\n",
		nds32_packet_id, PKT_DIR, address, dir_bit);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_cycle(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	unsigned long address = 0, pkt_length = 1, more_bit = 0;
	unsigned long shift_bit = 4, dir_bit = 0;

	if ((pCurrPkt[0] & 0x41) != 0x40) {
		TRACER_DECODE_MSG(("NOT %s", PKT_CYCLE));
		return 0;
	}
	address = (pCurrPkt[0] & 0x3C) >> 2;
	dir_bit = (pCurrPkt[0] & 0x02) >> 1;
	more_bit = (pCurrPkt[0] & 0x80);
	while (more_bit) {
		pCurrPkt ++;
		more_bit = (pCurrPkt[0] & 0x80);
		address |= (pCurrPkt[0] & 0x7F) << shift_bit;
		shift_bit += 7;
		pkt_length ++;
	}

	sprintf(str_decoded_data, "0x%08lx %s DIR=0x%lx, COUNT=0x%lx, BITS=0x%lx\n",
		nds32_packet_id, PKT_CYCLE, dir_bit, address, shift_bit);
	TRACER_DECODE_MSG(("%s", str_decoded_data));
	nds32_packet_id ++;
	return pkt_length;
}

static int nds32_tracer_decode_pkt(char *pSrcPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	int decoded_bytes = 0;
	//TRACER_DECODE_MSG(("pSrcPkt = 0x%lx, pSrcPkt[] = 0x%lx", (unsigned long)pSrcPkt, *pSrcPkt));
	if ((pCurrPkt[0] == (char)0x80) && ((pCurrPkt[1] & 0x81) == 0x81)) {
		decoded_bytes = nds32_tracer_decode_sync(pCurrPkt);
	} else if (pCurrPkt[0] == (char)0x82) {
		decoded_bytes = nds32_tracer_decode_cpc(pCurrPkt);
	} else if (pCurrPkt[0] == (char)0x84) {
		decoded_bytes = nds32_tracer_decode_ra(pCurrPkt);
	} else if (pCurrPkt[0] == (char)0x86) {
		decoded_bytes = nds32_tracer_decode_cid(pCurrPkt);
	} else if (pCurrPkt[0] & 0x01) {
		decoded_bytes = nds32_tracer_decode_npc(pCurrPkt);
	} else if (pCurrPkt[0] == (char)0x00) {
		sprintf(str_decoded_data, "PKT PAD: 0X00\n");
		TRACER_DECODE_MSG(("%s", str_decoded_data));
		decoded_bytes = 1;
	} else if ((nds32_cycle_accurate == 0) && ((pCurrPkt[0] & 0x81) == 0x00)) {
		decoded_bytes = nds32_tracer_decode_dir(pCurrPkt);
	} else if ((nds32_cycle_accurate == 1) && ((pCurrPkt[0] & 0x41) == 0x40)) {
		decoded_bytes = nds32_tracer_decode_cycle(pCurrPkt);
	}

	return decoded_bytes;
}

int nds32_tracer_decode_pktfile(char *pPktFileName)
{
	unsigned long buffer_size = PACKET_BUF_SIZE;
	unsigned long read_size = 0, decoded_bytes = 0;
	char *pkt_buf, *curr_buf;
	char *pOutFileName = (char *)&pkt_decoded_filename[0];
	FILE *pPacketFile = NULL;

	memcpy(&pkt_decoded_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&pkt_decoded_filename[strlen((char *)pPktFileName)], "-de", 4);
	pPacketFile = fopen(pPktFileName, "rb");
	gpPktDecodedFile = fopen(pOutFileName, "wb");
	pkt_buf = (char *)malloc(PACKET_BUF_SIZE);
	curr_buf = pkt_buf;
	nds32_packet_id = 0;
	if (pPacketFile) {
		/* get data from pkt file */
		read_size = fread(curr_buf, 1, buffer_size, pPacketFile);
		curr_buf ++; // skip the 1st byte (Parameter)
		while (read_size) {
			decoded_bytes = nds32_tracer_decode_pkt(curr_buf);
			if (decoded_bytes) {
				if (gpPktDecodedFile)
					fwrite((char *)str_decoded_data, 1, strlen((char *)str_decoded_data), gpPktDecodedFile);
			} else {
				printf("ERROR!! packet decode ERROR !!");
				break;
			}
			TRACER_DECODE_MSG(("read_size=0x%lx, decoded_bytes=0x%lx", read_size, decoded_bytes));
			if (read_size >= decoded_bytes)
				read_size -= decoded_bytes;
			else
				read_size = 0;
			curr_buf += decoded_bytes;
		}

	}

	if (pPacketFile)
		fclose(pPacketFile);
	if (gpPktDecodedFile)
		fclose(gpPktDecodedFile);
	free(pkt_buf);

	return 0;
}
/* ======================================================================================================== */

static int nds32_tracer_search_1stsync_lastsync(char *pSrcPkt, unsigned long total_pkt_bytes, char **p1stPkt)
{
	char *pCurrPkt = (char *)pSrcPkt;
	char *plast_sync = NULL;
	unsigned long decoded_bytes = 0;
	unsigned long dump_pkt_bytes = 0, find_1st_sync = 0;

	gp_curr_sync = NULL;
	gp_1st_sync = NULL;

	while (total_pkt_bytes) {
		decoded_bytes = nds32_tracer_decode_pkt(pCurrPkt);
		/* if decode error, check if begin to find 1st_sync or decode end(not enough bytes) */
		if (decoded_bytes == 0) {
			if (find_1st_sync == 1) {
				break;
			} else {
				decoded_bytes = 1;
			}
		}
		if ((gp_curr_sync) &&
			(find_1st_sync == 0)) {
			find_1st_sync = 1;
			gp_1st_sync = gp_curr_sync;
		}

		if (total_pkt_bytes >= decoded_bytes) {
			total_pkt_bytes -= decoded_bytes;
		} else {
			total_pkt_bytes = 0;
		}
		pCurrPkt += decoded_bytes;
	}
	plast_sync = gp_curr_sync;
	/* check if last words include SYNC packet "0x80" */
	if (total_pkt_bytes) {
		if ((pCurrPkt[0] == (char)0x80) && ((pCurrPkt[1] & 0x81) == 0x81)) {
			plast_sync = pCurrPkt;
		}
	}

	*p1stPkt = gp_1st_sync;
	dump_pkt_bytes = (plast_sync - gp_1st_sync);
	LOG_DEBUG("p1st_sync = 0x%lx, plast_sync = 0x%lx, dump_pkt_bytes = 0x%lx",
	  (unsigned long)gp_1st_sync, (unsigned long)plast_sync, (unsigned long)dump_pkt_bytes);
	return dump_pkt_bytes;
}

static int nds32_tracer_copy_lastsync(char *plast_sync)
{
	unsigned long copy_start = (unsigned long)plast_sync;
	/* copy remain-pkts to buffer start for next time dump */
	unsigned long *pcopy_start = (unsigned long *)(copy_start & ~0x03);
	unsigned long *pcopy_end = (unsigned long *)pnds32_etb_wptr;
	LOG_DEBUG("pcopy_start = 0x%lx, pcopy_end = 0x%lx",
	  (unsigned long)pcopy_start, (unsigned long)pcopy_end);
	pnds32_etb_wptr = pnds32_etb_buf_start;
	while (pcopy_end > pcopy_start) {
		*pnds32_etb_wptr ++ = *pcopy_start++;
	}
	//LOG_DEBUG("pnds32_etb_wptr = 0x%lx", (unsigned long)pnds32_etb_wptr);
	return ERROR_OK;
}

int nds32_tracer_add_trigger_breakpoints(struct target *target, unsigned long bp_address)
{
	if (nds32_tracer_check(target) != ERROR_OK) {
		LOG_DEBUG("Do NOT support tracer !!");
		return ERROR_FAIL;
	}
	nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_EDM | ETM_TRIGGER_EVENT_B_EDM);
	nds32_tracer_setting(target);
	return breakpoint_add(target, bp_address, 0x02|BP_WP_TRIGGER_ON, BKPT_HARD);
}

int nds32_tracer_remove_trigger_breakpoints(struct target *target, unsigned long bp_address)
{
	if (nds32_tracer_check(target) != ERROR_OK) {
		LOG_DEBUG("Do NOT support tracer !!");
		return ERROR_FAIL;
	}
	breakpoint_remove(target, bp_address);
	return ERROR_OK;
}

/*
monitor nds trace on
monitor nds trace off
monitor nds trace trigger-break-on
monitor nds trace trigger-break-off
monitor nds trace trigger-always-true
monitor nds trace circular-buffer
monitor nds trace full-buffer
monitor nds trace center-buffer
monitor nds trace dump-trace-file xxxx
monitor nds trace etm-mode 1 (CYCLE_ACCURATE mode)
*/

__COMMAND_HANDLER(handle_nds32_tracer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (nds32_tracer_check(target) != ERROR_OK) {
			LOG_DEBUG("Do NOT support tracer !!");
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "on") == 0) {
			LOG_DEBUG("trace on");
			nds32_tracer_setting(target);
		} else if (strcmp(CMD_ARGV[0], "off") == 0) {
			LOG_DEBUG("trace off");
			nds32_tracer_disable(target);
			nds32_tracer_on = 0;
		} else if (strcmp(CMD_ARGV[0], "id") == 0) {
			NDS32_LOG("etb_id = %lx, etm_id = %lx", nds32_etb_id, nds32_etm_id);
			NDS32_LOG("etb_revision = %lx, etb_depth = 0x%lx bytes", nds32_etb_revision, nds32_etb_depth);
		} else if (strcmp(CMD_ARGV[0], "polling") == 0) {
			nds32_tracer_polling(target);
		} else if (strcmp(CMD_ARGV[0], "read") == 0) {
			nds32_tracer_read_etb(target);
		} else if (strcmp(CMD_ARGV[0], "trigger-always-true") == 0) {
			nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_TRUE);
			LOG_DEBUG("trace trigger-always-true");
			nds32_tracer_setting(target);
		} else if (strcmp(CMD_ARGV[0], "circular-buffer") == 0) {
			/* from program start to (CTRL+C or break) */
			nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_TRUE | ETM_TRIGGER_A_XOR_B);  // non-trigger
			nds32_realtime_mode = 1;
			nds32_capture_cnt = nds32_etb_depth;
			LOG_DEBUG("trace circular-buffer");
			nds32_tracer_setting(target);
			nds32_trace_mode = NDS32_TARCEMODE_CIRCULAR;
		} else if (strcmp(CMD_ARGV[0], "full-buffer") == 0) {
			nds32_trigger_ctrl = (ETM_TRIGGER_EVENT_A_TRUE);
			nds32_realtime_mode = 1;
			nds32_capture_cnt = (nds32_etb_depth);
			LOG_DEBUG("trace full-buffer");
			nds32_tracer_setting(target);
			nds32_trace_mode = NDS32_TRACEMODE_FULL;
		} else if (strcmp(CMD_ARGV[0], "center-buffer") == 0) {
			nds32_realtime_mode = 1;
			nds32_capture_cnt = (nds32_etb_depth >> 1);
			LOG_DEBUG("trace center-buffer");
			nds32_tracer_setting(target);
			nds32_trace_mode = NDS32_TRACEMODE_CENTER;
		} else if (strcmp(CMD_ARGV[0], "non-realtime") == 0) {
			nds32_realtime_mode = 0;
			nds32_capture_cnt = (nds32_etb_depth);
			LOG_DEBUG("non-realtime");
			nds32_tracer_setting(target);
			nds32_trace_mode = NDS32_TRACEMODE_NONREALTIME;
		} else if (strcmp(CMD_ARGV[0], "dump-trace-file") == 0) {
			LOG_DEBUG("trace dump-trace-file %s", CMD_ARGV[1]);
			nds32_tracer_dumpfile(target, (char *)CMD_ARGV[1]);
		} else if (strcmp(CMD_ARGV[0], "decode-trace-file") == 0) {
			LOG_DEBUG("trace decode-trace-file %s", CMD_ARGV[1]);
			nds32_tracer_decode_pktfile((char *)CMD_ARGV[1]);
		} else if (strcmp(CMD_ARGV[0], "sync-period") == 0) {
			unsigned int sync_period = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], sync_period);
			LOG_DEBUG("sync-period 0x%x", sync_period);
			if (sync_period <= 3) {
				nds32_sync_period = sync_period;
			}
		} else if (strcmp(CMD_ARGV[0], "etm-mode") == 0) {
			unsigned int etm_mode = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], etm_mode);
			LOG_DEBUG("etm-mode 0x%x", etm_mode);
			nds32_etm_mode = (etm_mode & 0x07) | ETM_MODE_FULL_ACTION_DBGI;
			nds32_tracer_setting(target);
		} else if (strcmp(CMD_ARGV[0], "fifosize") == 0) {
			unsigned int fifosize = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], fifosize);
			LOG_DEBUG("fifosize 0x%x", fifosize);
			if (fifosize < nds32_etb_ori_fifo)
				nds32_etb_depth = fifosize;
		} else if (strcmp(CMD_ARGV[0], "otbsize") == 0) {
			unsigned int otbsize = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], otbsize);
			LOG_DEBUG("otbsize 0x%x", otbsize);
			nds32_otb_filesize = otbsize;
		} else if (strcmp(CMD_ARGV[0], "skip-1stsync") == 0) {
			unsigned int skip_find_1stsync = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], skip_find_1stsync);
			LOG_DEBUG("skip_find_1stsync 0x%x", skip_find_1stsync);
			nds32_skip_find_1stsync = skip_find_1stsync;
		} else if (strcmp(CMD_ARGV[0], "words-per-trans") == 0) {
			unsigned int words_per_transfer = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], words_per_transfer);
			LOG_DEBUG("words_per_transfer 0x%x", words_per_transfer);
			if (words_per_transfer <= (TRACER_MAX_OTB_PER_TRANSFER >> 2))
				nds32_fifo_words_per_transfer = words_per_transfer;
		}
	}

	return ERROR_OK;
}
