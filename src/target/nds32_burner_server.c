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
#include <server/server.h>
#include <helper/log.h>
//#include <server/gdb_fileio.h>
#include "nds32.h"
#include "nds32_aice.h"
#include <jtag/aice/aice_jdp.h>

const char *burner_port = "2354";
unsigned int burner_coreid = 0;
int burner_debug_mode = 0;
#define BURNER_BUFFER_SIZE 4096

extern unsigned int aice_max_retry_times;
extern uint32_t runtest_num_clocks;
//extern int aice_read_edmsr(uint8_t target_id, uint32_t address, uint32_t *data);
//extern int aice_write_edmsr(uint8_t target_id, uint32_t address, uint32_t data);
//extern bool jtag_poll_get_enabled(void);
//extern void jtag_poll_set_enabled(bool value);
//extern struct target* coreid_to_target(uint32_t coreid);
extern int abs_chain_table[];

extern int nds32_select_memory_mode(struct target *target, uint32_t address,
		uint32_t length, uint32_t *end_address);

//extern int debug_level;
// Command code
#define WRITE_WORD           0x1A
#define READ_WORD            0x1B
#define WRITE_BYTE           0x2A
#define READ_BYTE            0x2B
#define WRITE_HALF           0x4A
#define READ_HALF            0x4B
#define FAST_READ            0x1C
#define WRITE_IO             0x1F
#define FAST_WRITE           0x20
#define BURNER_QUIT          0x04
#define MULTIPLE_WRITE_WORD  0x5A
#define MULTIPLE_WRITE_HALF  0x5B
#define MULTIPLE_WRITE_BYTE  0x5C
#define MULTIPLE_READ_WORD   0x5D
#define MULTIPLE_READ_HALF   0x5E
#define MULTIPLE_READ_BYTE   0x5F
#define RESET_TARGET         0x3A
#define RESET_HOLD           0x3B
#define RESET_AICE           0x3C
#define HOLD_CORE            0x1D
#define READ_EDM_SR          0x60
#define WRITE_EDM_SR         0x61
#define READ_EDM_JDP         0x6E
#define WRITE_EDM_JDP        0x6F
#define BURNER_INIT          0x01
#define BURNER_SELECT_CORE   0x05
#define MONITOR_CMD          0x06
#define BURNER_DEBUG         0x07
#define BURNER_SELECT_TARGET 0x08
#define READ_REG             0x70
#define WRITE_REG            0x71

#define get_u32(buffer) le_to_h_u32((const uint8_t *)buffer)

static const int NDS32_LM_SIZE_TABLE[16] = {
	4 * 1024,
	8 * 1024,
	16 * 1024,
	32 * 1024,
	64 * 1024,
	128 * 1024,
	256 * 1024,
	512 * 1024,
	1024 * 1024,
	1 * 1024,
	2 * 1024,
	2048 * 1024,
	4096 * 1024,
	0,
	0,
	0,
};

extern uint32_t nds32_custom_def_idlm_base;
static int burner_update_lm_info(struct target *target)
{
	if (nds32_custom_def_idlm_base) {
		LOG_DEBUG("nds32_custom_def_idlm_base: %x", nds32_custom_def_idlm_base);
		return ERROR_OK;
	}
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	uint32_t value_cr1; /* ICM_CFG */
	uint32_t value_cr2; /* DCM_CFG */

	aice_read_register(target, CR1, &value_cr1);
	memory->ilm_base = (value_cr1 >> 10) & 0x7;
	memory->ilm_align_ver = (value_cr1 >> 13) & 0x3;

	aice_read_register(target, CR2, &value_cr2);
	memory->dlm_base = (value_cr2 >> 10) & 0x7;
	memory->dlm_align_ver = (value_cr2 >> 13) & 0x3;
	LOG_DEBUG("value_cr1: %x, value_cr2: %x", value_cr1, value_cr2);

	uint32_t value_mr6 = 0;
	uint32_t value_mr7 = 0;
	uint32_t size_index = 0;

	if ((memory->ilm_base != 0) &&
		(nds32->privilege_level == 0))
		aice_read_register(target, MR6, &value_mr6);
	size_index = (value_mr6 >> 1) & 0xF;
	nds32->memory.ilm_size = NDS32_LM_SIZE_TABLE[size_index];

	if (value_mr6 & 0x1)
		memory->ilm_enable = true;
	else
		memory->ilm_enable = false;

	if (memory->ilm_align_ver == 0) { /* 1MB aligned */
		memory->ilm_start = value_mr6 & 0xFFF00000;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else if (memory->ilm_align_ver == 1) { /* aligned to local memory size */
		memory->ilm_start = value_mr6 & 0xFFFFFC00;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else {
		memory->ilm_start = -1;
		memory->ilm_end = -1;
	}

	if ((memory->dlm_base != 0) &&
		(nds32->privilege_level == 0))
		aice_read_register(target, MR7, &value_mr7);
	size_index = (value_mr7 >> 1) & 0xF;
	nds32->memory.dlm_size = NDS32_LM_SIZE_TABLE[size_index];

	if (value_mr7 & 0x1)
		memory->dlm_enable = true;
	else
		memory->dlm_enable = false;

	if (memory->dlm_align_ver == 0) { /* 1MB aligned */
		memory->dlm_start = value_mr7 & 0xFFF00000;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else if (memory->dlm_align_ver == 1) { /* aligned to local memory size */
		memory->dlm_start = value_mr7 & 0xFFFFFC00;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else {
		memory->dlm_start = -1;
		memory->dlm_end = -1;
	}
	LOG_DEBUG("value_mr6: %x, value_mr7: %x", value_mr6, value_mr7);

	return ERROR_OK;
}

static int burner_halt_target(struct target *target, uint32_t if_reset_hold)
{
	int retval;

	if (if_reset_hold)
		retval = aice_reset_target(target, AICE_RESET_HOLD);
	else
		retval = aice_halt_target(target);

	/* update lm information */
	//aice_edm_config(target);
	burner_update_lm_info(target);
	return retval;
}

int burner_aice_write(struct target *target, unsigned char *packet, int *response_len)
{
	target_addr_t end_address;
	unsigned int WriteAddr, WriteData, AiceCmmdType, num_of_pairs, i;
	int retval = ERROR_OK;
	char *data_ptr = (char *)(packet + 2);

	AiceCmmdType = packet[0];
	if (AiceCmmdType == MULTIPLE_WRITE_WORD) {
		num_of_pairs = (unsigned int)packet[1];
		AiceCmmdType = WRITE_WORD;
	}
	else if (AiceCmmdType == MULTIPLE_WRITE_HALF) {
		num_of_pairs = (unsigned int)packet[1];
		AiceCmmdType = WRITE_HALF;
	}
	else if (AiceCmmdType == MULTIPLE_WRITE_BYTE) {
		num_of_pairs = (unsigned int)packet[1];
		AiceCmmdType = WRITE_BYTE;
	}
	else {
		num_of_pairs = 1;
	}

	for (i = 0; i < num_of_pairs; i++) {
		WriteAddr = ((((unsigned int)data_ptr[3]<< 24) & 0xFF000000) |
				(((unsigned int)data_ptr[2]<< 16) & 0x00FF0000) |
				(((unsigned int)data_ptr[1]<< 8) & 0x0000FF00) |
				(((unsigned int)data_ptr[0]<< 0) & 0x000000FF));
		WriteData = ((((unsigned int)data_ptr[7]<< 24) & 0xFF000000) |
				(((unsigned int)data_ptr[6]<< 16) & 0x00FF0000) |
				(((unsigned int)data_ptr[5]<< 8) & 0x0000FF00) |
				(((unsigned int)data_ptr[4]<< 0) & 0x000000FF));
		data_ptr += 8;

		if (AiceCmmdType == WRITE_WORD) {
			nds32_select_memory_mode(target, WriteAddr, 4, (uint32_t *)&end_address);
			retval = aice_write_edm(target, JDP_W_MEM_W, WriteAddr, (uint32_t*)&WriteData, 1);
		}
		else if (AiceCmmdType == WRITE_HALF) {
			nds32_select_memory_mode(target, WriteAddr, 2, (uint32_t *)&end_address);
			retval = aice_write_edm(target, JDP_W_MEM_H, WriteAddr, (uint32_t*)&WriteData, 1);
		}
		else if (AiceCmmdType == WRITE_BYTE) {
			nds32_select_memory_mode(target, WriteAddr, 1, (uint32_t *)&end_address);
			retval = aice_write_edm(target, JDP_W_MEM_B, WriteAddr, (uint32_t*)&WriteData, 1);
		}
		else if (AiceCmmdType == WRITE_EDM_SR) {
			retval = aice_write_edm(target, JDP_W_DBG_SR, WriteAddr, (uint32_t*)&WriteData, 1);
		}
		else if (AiceCmmdType == WRITE_EDM_JDP) {
			retval = aice_write_edm(target,
				(WriteAddr & 0xFF), (WriteAddr & 0xFF00) >> 8, (uint32_t*)&WriteData, 1);
		}
	}
	*response_len = 2;
	return retval;
}

int burner_aice_read(struct target *target, unsigned char *packet, int *response_len)
{
	target_addr_t end_address;
	unsigned int ReadAddr, ReadData, AiceCmmdType;
	int retval = ERROR_OK, resp_length = 0;

	ReadData = 0xFFFFFFFF;
	AiceCmmdType = packet[0];
	ReadAddr = ((((unsigned int)packet[5]<< 24) & 0xFF000000) |
			(((unsigned int)packet[4]<< 16) & 0x00FF0000) |
			(((unsigned int)packet[3]<< 8) & 0x0000FF00) |
			(((unsigned int)packet[2]<< 0) & 0x000000FF));
	if (AiceCmmdType == READ_WORD) {
		nds32_select_memory_mode(target, ReadAddr, 4, (uint32_t *)&end_address);
		retval = aice_read_edm(target, JDP_R_MEM_W, ReadAddr, (uint32_t*)&ReadData, 1);
		resp_length = 6;
	}
	else if (AiceCmmdType == READ_HALF) {
		nds32_select_memory_mode(target, ReadAddr, 2, (uint32_t *)&end_address);
		retval = aice_read_edm(target, JDP_R_MEM_H, ReadAddr, (uint32_t*)&ReadData, 1);
		ReadData = (ReadData << 16);
		resp_length = 4;
	}
	else if (AiceCmmdType == READ_BYTE) {
		nds32_select_memory_mode(target, ReadAddr, 1, (uint32_t *)&end_address);
		retval = aice_read_edm(target, JDP_R_MEM_B, ReadAddr, (uint32_t*)&ReadData, 1);
		ReadData = (ReadData << 24);
		resp_length = 3;
	}
	else if (AiceCmmdType == READ_EDM_SR) {
		retval = aice_read_edm(target, JDP_R_DBG_SR, ReadAddr, (uint32_t*)&ReadData, 1);
		resp_length = 6;
	}
	else if (AiceCmmdType == READ_EDM_JDP) {
		retval = aice_read_edm(target,
			(ReadAddr & 0xFF), (ReadAddr & 0xFF00) >> 8, (uint32_t*)&ReadData, 1);
		resp_length = 6;
	}

	packet[2] = (char)((ReadData & 0xFF000000) >> 24);
	packet[3] = (char)((ReadData & 0x00FF0000) >> 16);
	packet[4] = (char)((ReadData & 0x0000FF00) >> 8);
	packet[5] = (char)((ReadData & 0x000000FF) >> 0);
	*response_len = resp_length;
	return retval;
}

int burner_aice_multi_read(struct target *target, unsigned char *packet, int *response_len)
{
	unsigned int i, ReadAddr, ReadData, AiceCmmdType, num_of_pairs, total_of_pairs, end_address;
	int retval = ERROR_OK, resp_length = 2;
	char *data_ptr = (char *)(packet + 2);

	ReadData = 0xFFFFFFFF;
	AiceCmmdType = packet[0];
	total_of_pairs = (unsigned int)packet[1];
	char *dst_ptr = (char *)(packet + 2);

	while(total_of_pairs) {
		/* for AICE-mini zero-packet issue */
		num_of_pairs = total_of_pairs;
		if (num_of_pairs >= (aice_usb_tx_max_packet/8)) {
			num_of_pairs = (aice_usb_tx_max_packet/8) - 1;
			//printf("num_of_pairs = %d \n", num_of_pairs);
		}
		total_of_pairs -= num_of_pairs;

		ReadAddr = ((((unsigned int)data_ptr[3]<< 24) & 0xFF000000) |
					(((unsigned int)data_ptr[2]<< 16) & 0x00FF0000) |
					(((unsigned int)data_ptr[1]<< 8) & 0x0000FF00) |
					(((unsigned int)data_ptr[0]<< 0) & 0x000000FF));
		nds32_select_memory_mode(target, ReadAddr, num_of_pairs*4, &end_address);

		// usb pack command start
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);

		for (i = 0; i < num_of_pairs; i++) {
			ReadAddr = ((((unsigned int)data_ptr[3]<< 24) & 0xFF000000) |
					(((unsigned int)data_ptr[2]<< 16) & 0x00FF0000) |
					(((unsigned int)data_ptr[1]<< 8) & 0x0000FF00) |
					(((unsigned int)data_ptr[0]<< 0) & 0x000000FF));
			data_ptr += 4;
			if (AiceCmmdType == MULTIPLE_READ_WORD) {
				retval = aice_read_edm(target, JDP_R_MEM_W, ReadAddr, (uint32_t*)&ReadData, 1);
			}
			else if (AiceCmmdType == MULTIPLE_READ_HALF) {
				retval = aice_read_edm(target, JDP_R_MEM_H, ReadAddr, (uint32_t*)&ReadData, 1);
			}
			else if (AiceCmmdType == MULTIPLE_READ_BYTE) {
				retval = aice_read_edm(target, JDP_R_MEM_B, ReadAddr, (uint32_t*)&ReadData, 1);
			}
		}
		// flush usb pack command
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		// parse RX data in usb_in_packets_buffer
		//char *src_ptr = (char *)&usb_in_packets_buffer[4];
		char usbin_data[2048];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], 4 + num_of_pairs*8);
		char *src_ptr = (char *)&usbin_data[4];

		if (AiceCmmdType == MULTIPLE_READ_WORD) {
			for (i = 0; i < num_of_pairs; i++) {
				*dst_ptr++ = src_ptr[3];
				*dst_ptr++ = src_ptr[2];
				*dst_ptr++ = src_ptr[1];
				*dst_ptr++ = src_ptr[0];
				src_ptr += 8;  // sizeof aice_usb_cmmd_dthma
				resp_length += 4;
			}
		}
		else if (AiceCmmdType == MULTIPLE_READ_HALF) {
			for (i = 0; i < num_of_pairs; i++) {
				*dst_ptr++ = src_ptr[1];
				*dst_ptr++ = src_ptr[0];
				src_ptr += 8;  // sizeof aice_usb_cmmd_dthma
				resp_length += 2;
			}
		}
		else if (AiceCmmdType == MULTIPLE_READ_BYTE) {
			for (i = 0; i < num_of_pairs; i++) {
				*dst_ptr++ = src_ptr[0];
				src_ptr += 8;  // sizeof aice_usb_cmmd_dthma
				resp_length += 1;
			}
		}
	}
	*response_len = resp_length;
	return retval;
}

int burner_aice_bulkmode(struct target *target, unsigned char *packet, int *response_len)
{
	unsigned int ReadWriteAddr, ReadWriteCnt, AiceCmmdType, end_address;
	int retval = ERROR_OK;
	uint8_t *data_ptr;

	AiceCmmdType = packet[0];
	ReadWriteAddr = ((((unsigned int)packet[5]<< 24) & 0xFF000000) |
			(((unsigned int)packet[4]<< 16) & 0x00FF0000) |
			(((unsigned int)packet[3]<< 8) & 0x0000FF00) |
			(((unsigned int)packet[2]<< 0) & 0x000000FF));
	ReadWriteCnt = ((((unsigned int)packet[9]<< 24) & 0xFF000000) |
			(((unsigned int)packet[8]<< 16) & 0x00FF0000) |
			(((unsigned int)packet[7]<< 8) & 0x0000FF00) |
			(((unsigned int)packet[6]<< 0) & 0x000000FF));
	
	nds32_select_memory_mode(target, ReadWriteAddr, ReadWriteCnt, &end_address);
	if (AiceCmmdType == FAST_READ) {
		data_ptr = (uint8_t *)&packet[2];
		// aice_usb_bulk_read_mem
		retval = aice_read_mem_bulk(target, ReadWriteAddr, ReadWriteCnt, (uint8_t *)data_ptr);
		*response_len = (ReadWriteCnt + 2);
	}
	else if (AiceCmmdType == FAST_WRITE) {
		data_ptr = (uint8_t *)&packet[12];
		// aice_usb_bulk_write_mem
		retval = aice_write_mem_bulk(target, ReadWriteAddr, ReadWriteCnt, (uint8_t *)data_ptr);
		*response_len = 2;
	}
	return retval;
}

static int burner_aice_read_reg(struct target *target, unsigned char *packet, int *response_len)
{
	unsigned int ReadReg, ReadData;
	int retval = ERROR_OK;

	ReadData = 0xFFFFFFFF;
	ReadReg  = ((((unsigned int)packet[5]<< 24) & 0xFF000000) |
		    (((unsigned int)packet[4]<< 16) & 0x00FF0000) |
		    (((unsigned int)packet[3]<< 8)  & 0x0000FF00) |
		    (((unsigned int)packet[2]<< 0)  & 0x000000FF)  );

	retval = aice_read_reg(target, ReadReg, (uint32_t*)&ReadData); 

	packet[2] = (char)((ReadData & 0xFF000000) >> 24);
	packet[3] = (char)((ReadData & 0x00FF0000) >> 16);
	packet[4] = (char)((ReadData & 0x0000FF00) >> 8);
	packet[5] = (char)((ReadData & 0x000000FF) >> 0);
	*response_len = 6;
	return retval;

}

static int burner_aice_write_reg(struct target *target, unsigned char *packet, int *response_len)
{
	unsigned int WriteReg, WriteData;
	int retval = ERROR_OK;
	char *data_ptr = (char *)(packet + 2);

	WriteReg = ((((unsigned int)data_ptr[3]<< 24) & 0xFF000000) |
		    (((unsigned int)data_ptr[2]<< 16) & 0x00FF0000) |
		    (((unsigned int)data_ptr[1]<< 8)  & 0x0000FF00) |
		    (((unsigned int)data_ptr[0]<< 0)  & 0x000000FF)  );

	WriteData = ((((unsigned int)data_ptr[7]<< 24) & 0xFF000000) |
		     (((unsigned int)data_ptr[6]<< 16) & 0x00FF0000) |
		     (((unsigned int)data_ptr[5]<< 8)  & 0x0000FF00) |
		     (((unsigned int)data_ptr[4]<< 0)  & 0x000000FF)  );

	retval = aice_write_reg(target, WriteReg, WriteData);
	*response_len = 2;
	return retval;
}

static int burner_write(struct connection *connection, const void *data, int len)
{
	if (connection_write(connection, data, len) == len)
		return ERROR_OK;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int burner_input(struct connection *connection)
{
	unsigned int coreid = abs_chain_table[burner_coreid], AiceCmmdType;
	if (abs_chain_table[burner_coreid] == -1)
		coreid = 0;
	unsigned char cmmd_buffer[BURNER_BUFFER_SIZE] = {0};
	unsigned char *buf_p = (unsigned char *)&cmmd_buffer[0];
	char *target_name;
	int retval, bytes_read, res_length, name_len;
	struct target *target = coreid_to_target(coreid);
	//struct command_context *command_context = connection->cmd_ctx;
	char *ret_data = NULL;
	int ret_size = 0;
	int debug_level_bak;
	unsigned int aice_max_retry_times_bak = aice_max_retry_times;
	uint32_t runtest_num_clocks_bak = runtest_num_clocks;
	char **aice_custom_monitor_command;
	int* len;

	bytes_read = connection_read(connection, buf_p, BURNER_BUFFER_SIZE);

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1) {
		LOG_ERROR("error during burner_input");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if( burner_debug_mode == 0 ) {
		debug_level_bak = debug_level;
		debug_level = LOG_LVL_WARNING;
	}
	else {
		debug_level_bak = debug_level;
		debug_level = LOG_LVL_DEBUG; 
	}

	if (aice_max_retry_times < 50)
		aice_max_retry_times = 50; // for slower FLASH-ROM
	if (runtest_num_clocks < 200)
		runtest_num_clocks = 200;  // for slower FLASH-ROM
	LOG_DEBUG("burner_input=%x, bytes_read=%x, coreid=%d", buf_p[0], bytes_read, coreid);

	res_length = 0;
	AiceCmmdType = buf_p[0];
	switch (AiceCmmdType)
	{
		case WRITE_WORD:
		case WRITE_HALF:
		case WRITE_BYTE:
		case WRITE_EDM_SR:
		case WRITE_EDM_JDP:
			retval = burner_aice_write(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		case READ_WORD:
		case READ_HALF:
		case READ_BYTE:
		case READ_EDM_SR:
		case READ_EDM_JDP:
			retval = burner_aice_read(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break; 
		case FAST_READ:
		case FAST_WRITE:
			retval = burner_aice_bulkmode(target, buf_p, &res_length);
			// restore the 1st byte buf_p[0]
			buf_p[0] = AiceCmmdType;
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;

		case MULTIPLE_WRITE_WORD:
		case MULTIPLE_WRITE_HALF:
		case MULTIPLE_WRITE_BYTE:
			aice_set_command_mode(AICE_COMMAND_MODE_PACK);
			retval = burner_aice_write(target, buf_p, &res_length);
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;

		case MULTIPLE_READ_WORD:
		case MULTIPLE_READ_HALF:
		case MULTIPLE_READ_BYTE:
			retval = burner_aice_multi_read(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;

		case READ_REG:
			retval = burner_aice_read_reg(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		
		case WRITE_REG:
			retval = burner_aice_write_reg(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;

		case HOLD_CORE:
			burner_halt_target(target, 0);
			res_length = 2;
			break;
		case RESET_HOLD:
			burner_halt_target(target, 1);
			res_length = 2;
			break;
		case RESET_TARGET:
			aice_reset_target(target, AICE_SRST);
			res_length = 2;
			break;
		case RESET_AICE:
			aice_reset_device();
			res_length = 2;
			break;

		case BURNER_INIT:
		case WRITE_IO:
			// do nothing
			res_length = 2;
			break;
		case BURNER_QUIT:
			debug_level = debug_level_bak;
			return -1;

		case BURNER_SELECT_CORE:
			burner_coreid = get_u32(buf_p+1);
			LOG_DEBUG("Change Burn Coreid to %d", burner_coreid );
			target = coreid_to_target(burner_coreid);
			burner_halt_target(target, 0);
			res_length = 2;
			break;

		case BURNER_SELECT_TARGET:
			name_len = get_u32(buf_p+1);
			// use calloc can initial to 0 and let length = name_len + 1(the end of string need be 0), target_name should be a string for generic function get_target()
			target_name = (char*)calloc(name_len+1, sizeof(char));
			memcpy(target_name, buf_p+5, name_len);
			target = get_target(target_name);
			if (target == NULL) {
				LOG_ERROR("Target: %s is unknown\n", target_name);
				buf_p[0] |= 0x80;
			} else {
				burner_coreid = target->target_number;
				target = coreid_to_target(burner_coreid);
				burner_halt_target(target, 0);
				LOG_DEBUG("Change Burn Coreid to %d", burner_coreid);
			}
			res_length = 2;
			free(target_name);
			break;

		case BURNER_DEBUG:
			burner_debug_mode = 1;
			res_length = 2;
			break;

        case MONITOR_CMD:

            // Find command length
            len = (int*)malloc(2*sizeof(int));
            len[0] = get_u32(buf_p+1);

            // Prepare monitor command
            aice_custom_monitor_command = (char**)malloc(1*sizeof(char*));
            aice_custom_monitor_command[0] = (char*)malloc(len[0]*sizeof(char)+1);
            memcpy(aice_custom_monitor_command[0], buf_p+5, len[0]);

            // Issue monitor command
            retval = aice_monitor_command(target, 1, aice_custom_monitor_command, len, &ret_data);
            free(aice_custom_monitor_command[0]);
            free(aice_custom_monitor_command);
            free(len);

            if( retval == ERROR_OK ) {
                ret_size = get_u32(ret_data);
                memcpy( buf_p+2, ret_data, 4+ret_size ); //copy data including length(ret_size:4Bytes)
                res_length = 2+4+ret_size;   // buf_p[0]=CMD, buf_p[1]=0, buf_p[2~...]=data 

                free(ret_data);
            }
            else {
                buf_p[1] = 1;       //issue error
                res_length = 2;
                goto INPUT_END;
            }
            break;
		default:
			break;
	}
	buf_p[1] = 0;

INPUT_END:
	burner_write(connection, buf_p, res_length);
	debug_level = debug_level_bak;
	aice_max_retry_times = aice_max_retry_times_bak;
	runtest_num_clocks = runtest_num_clocks_bak;
	return ERROR_OK;
}

static int burner_new_connection(struct connection *connection)
{
	unsigned int coreid = abs_chain_table[burner_coreid];
	if (abs_chain_table[burner_coreid] == -1)
		coreid = 0;
	struct target *target = coreid_to_target(coreid);
	struct nds32 *nds32 = target_to_nds32(target);

	LOG_DEBUG("burner_new_connection");
	LOG_DEBUG("halt target...");
	/* set passcode for secure MCU */
	nds32_edm_config(nds32);
	burner_halt_target(target, 0);

	//send_cmd(TARGET_CMD_BUS_MODE);
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	LOG_DEBUG("access_channel 0x%lx, 0x%lx", (long)&nds32->memory.access_channel, (long)nds32->memory.access_channel);
	LOG_DEBUG("access_control 0x%lx, direct_access_local_memory 0x%lx", (long)nds32->edm.access_control, (long)nds32->edm.direct_access_local_memory);

	uint32_t WriteACCCtrl = 0; // NDS_MEMORY_SELECT_MEM
	aice_write_edm(target, JDP_W_MISC_REG, NDS_EDM_MISC_ACC_CTL, (uint32_t*)&WriteACCCtrl, 1);

	//send_cmd(TARGET_CMD_POLL_OFF);
	jtag_poll_set_enabled(false);

	/* Debug Mode */
	extern unsigned int MaxLogFileSize;
	if( MaxLogFileSize == 0x20000000 )
		burner_debug_mode = 1;

	return ERROR_OK;
}

static int burner_connection_closed(struct connection *connection)
{
	unsigned int coreid = abs_chain_table[burner_coreid];
	if (abs_chain_table[burner_coreid] == -1)
		coreid = 0;
	struct target *target = coreid_to_target(coreid);
	struct nds32 *nds32 = target_to_nds32(target);

	LOG_DEBUG("burner_connection_closed");
	nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
	jtag_poll_set_enabled(true);
	return ERROR_OK;
}
unsigned int burner_server_init_done = 0;
int burner_server_init(char *banner)
{
	int ret_value;

	if (burner_server_init_done)
		return ERROR_OK;
	ret_value = add_service("burner",
		burner_port,
		1,
		&burner_new_connection,
		&burner_input,
		&burner_connection_closed,
		NULL);
	burner_server_init_done = 1;
	return ret_value;
}

