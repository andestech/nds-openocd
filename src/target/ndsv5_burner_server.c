/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <server/server.h>
#include <helper/log.h>
#include "target.h"
#include "riscv/riscv.h"
#include "riscv/ndsv5.h"
#include "jtag/jtag.h"

const char *ndsv5_burner_port = "2354";
static struct target *ndsv5_burner_target;
static int burner_debug_mode;
static uint32_t select_burner_coreid;
static uint32_t select_burner_rtos_hartid;

#define BURNER_BUFFER_SIZE 4096

/* Command code table */
#define WRITE_WORD           0x1A
#define READ_WORD            0x1B
#define WRITE_BYTE           0x2A
#define READ_BYTE            0x2B
#define WRITE_HALF           0x4A
#define READ_HALF            0x4B
#define FAST_READ            0x1C
#define FAST_READ_BYTE       0x1E
#define WRITE_IO             0x1F
#define FAST_WRITE           0x20
#define FAST_WRITE_BYTE      0x21
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

extern int ndsv5_reset_halt_as_examine(struct target *target);
extern int ndsv5_srst_reset_target(struct target *target);
extern int ndsv5_write_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
extern int ndsv5_read_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
extern int ndsv5_access_memory_pack_batch_run(struct target *target, uint32_t free_after_run);

static int burner_write(struct connection *connection, const void *data, int len)
{
	if (connection_write(connection, data, len) == len)
		return ERROR_OK;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int burner_cmd_read(struct target *target, unsigned char *packet, int *response_len)
{
	uint32_t read_addr, read_data, cmd_type;
	int retval = ERROR_OK, resp_length = 0;

	ndsv5_access_memory_pack_batch_run(target, 1);
	read_data = 0xFFFFFFFF;
	cmd_type = packet[0];
	read_addr = ((((unsigned int)packet[5] << 24) & 0xFF000000) |
		     (((unsigned int)packet[4] << 16) & 0x00FF0000) |
		     (((unsigned int)packet[3] << 8)  & 0x0000FF00) |
		     (((unsigned int)packet[2] << 0)  & 0x000000FF));

	if (cmd_type == READ_WORD) {
		retval = ndsv5_read_memory_progbuf_pack(target, read_addr, 4, 1, (uint8_t *)&read_data);
		resp_length = 6;
	} else if (cmd_type == READ_HALF) {
		retval = ndsv5_read_memory_progbuf_pack(target, read_addr, 2, 1, (uint8_t *)&read_data);
		read_data = (read_data << 16);
		resp_length = 4;
	} else if (cmd_type == READ_BYTE) {
		retval = ndsv5_read_memory_progbuf_pack(target, read_addr, 1, 1, (uint8_t *)&read_data);
		read_data = (read_data << 24);
		resp_length = 3;
	}

	packet[2] = (char)((read_data & 0xFF000000) >> 24);
	packet[3] = (char)((read_data & 0x00FF0000) >> 16);
	packet[4] = (char)((read_data & 0x0000FF00) >> 8);
	packet[5] = (char)((read_data & 0x000000FF) >> 0);
	*response_len = resp_length;
	return retval;
}

static int burner_cmd_write(struct target *target, unsigned char *packet, int *response_len)
{
	uint32_t write_addr, write_data, cmd_type;
	int retval = ERROR_OK;
	char *data_ptr = (char *)(packet + 2);

	ndsv5_access_memory_pack_batch_run(target, 1);
	cmd_type = packet[0];
	write_addr = ((((unsigned int)data_ptr[3] << 24) & 0xFF000000) |
		      (((unsigned int)data_ptr[2] << 16) & 0x00FF0000) |
		      (((unsigned int)data_ptr[1] << 8)  & 0x0000FF00) |
		      (((unsigned int)data_ptr[0] << 0)  & 0x000000FF));
	write_data = ((((unsigned int)data_ptr[7] << 24) & 0xFF000000) |
		      (((unsigned int)data_ptr[6] << 16) & 0x00FF0000) |
		      (((unsigned int)data_ptr[5] << 8)  & 0x0000FF00) |
		      (((unsigned int)data_ptr[4] << 0)  & 0x000000FF));
	data_ptr += 8;

	if (cmd_type == WRITE_WORD)
		retval = ndsv5_write_memory_progbuf_pack(target, write_addr, 4, 1, (uint8_t *)&write_data);
	else if (cmd_type == WRITE_HALF)
		retval = ndsv5_write_memory_progbuf_pack(target, write_addr, 2, 1, (uint8_t *)&write_data);
	else if (cmd_type == WRITE_BYTE)
		retval = ndsv5_write_memory_progbuf_pack(target, write_addr, 1, 1, (uint8_t *)&write_data);

	*response_len = 2;
	return retval;
}

static int burner_fast_readwrite(struct target *target, unsigned char *packet, int *response_len)
{
	uint32_t readwrite_addr, readwritecnt, cmd_type, final_addr, word_num;
	int retval = ERROR_OK;
	uint32_t i;
	char *data_ptr;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
	struct nds32_v5_memory *memory = &(nds32->memory);
	uint32_t bak_access_channel = (uint32_t)memory->access_channel;

	ndsv5_access_memory_pack_batch_run(target, 1);

	memory->access_channel = NDS_MEMORY_ACC_CPU;
	nds32->nds_va_to_pa_off = 1;

	cmd_type = packet[0];
	readwrite_addr = ((((unsigned int)packet[5] << 24) & 0xFF000000) |
			  (((unsigned int)packet[4] << 16) & 0x00FF0000) |
			  (((unsigned int)packet[3] << 8)  & 0x0000FF00) |
			  (((unsigned int)packet[2] << 0)  & 0x000000FF));
	readwritecnt = ((((unsigned int)packet[9] << 24) & 0xFF000000) |
			(((unsigned int)packet[8] << 16) & 0x00FF0000) |
			(((unsigned int)packet[7] << 8)  & 0x0000FF00) |
			(((unsigned int)packet[6] << 0)  & 0x000000FF));

	if (readwrite_addr & 0x02) {
		nds32->nds_const_addr_mode = 1;
		readwrite_addr &= ~0x02;
	}

	final_addr = readwrite_addr + readwritecnt;
	word_num = readwritecnt / 4;
	if (readwritecnt % 4)
		word_num = word_num + 1;

	if (cmd_type == FAST_READ) {
		i = 2;
		data_ptr = (char *)(packet + i);
		while (readwrite_addr < final_addr) {
			retval = target_read_memory(target, readwrite_addr, 4, word_num, (uint8_t *)data_ptr);
			if (retval != ERROR_OK)
				break;
			readwrite_addr = readwrite_addr + readwritecnt;
			data_ptr = data_ptr + readwritecnt;
		}
		*response_len = readwritecnt + 2;
	} else if (cmd_type == FAST_WRITE) {
		i = 12;
		data_ptr = (char *)(packet + i);
		while (readwrite_addr < final_addr) {
			retval = target_write_memory(target, readwrite_addr, 4, word_num, (uint8_t *)data_ptr);
			if (retval != ERROR_OK)
				break;
			readwrite_addr = readwrite_addr + readwritecnt;
			data_ptr = data_ptr + readwritecnt;
		}
		*response_len = 2;
	}

	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	nds32->nds_const_addr_mode = 0;
	memory->access_channel = bak_access_channel;
	return retval;
}

static int burner_fast_byte_readwrite(struct target *target, unsigned char *packet, int *response_len)
{
	uint32_t readwrite_addr, readwritecnt, cmd_type, final_addr;
	int retval = ERROR_OK;
	uint32_t i;
	char *data_ptr, if_const_addr;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
	struct nds32_v5_memory *memory = &(nds32->memory);
	uint32_t bak_access_channel = (uint32_t)memory->access_channel;

	ndsv5_byte_access_from_burn = 1;
	ndsv5_access_memory_pack_batch_run(target, 1);

	memory->access_channel = NDS_MEMORY_ACC_CPU;
	nds32->nds_va_to_pa_off = 1;

	cmd_type = packet[0];
	readwrite_addr = ((((unsigned int)packet[5] << 24) & 0xFF000000) |
			  (((unsigned int)packet[4] << 16) & 0x00FF0000) |
			  (((unsigned int)packet[3] << 8)  & 0x0000FF00) |
			  (((unsigned int)packet[2] << 0)  & 0x000000FF));
	readwritecnt = ((((unsigned int)packet[9] << 24) & 0xFF000000) |
			(((unsigned int)packet[8] << 16) & 0x00FF0000) |
			(((unsigned int)packet[7] << 8)  & 0x0000FF00) |
			(((unsigned int)packet[6] << 0)  & 0x000000FF));
	if_const_addr = packet[10];

	if (if_const_addr != 0)
		nds32->nds_const_addr_mode = 1;

	final_addr = readwrite_addr + readwritecnt;

	LOG_DEBUG("readwrite_addr=0x%08x, readwritecnt=0x%08x, const_addr_mode=%x",
			readwrite_addr, readwritecnt, nds32->nds_const_addr_mode);

	if (cmd_type == FAST_READ_BYTE) {
		i = 2;
		data_ptr = (char *)(packet + i);
		while (readwrite_addr < final_addr) {
			retval = target_read_memory(target, readwrite_addr, 1, readwritecnt, (uint8_t *)data_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("const addr read byte data failed\n");
				break;
			}
			readwrite_addr = readwrite_addr + readwritecnt;
			data_ptr = data_ptr + readwritecnt;
		}
		*response_len = readwritecnt + 2;
	} else if (cmd_type == FAST_WRITE_BYTE) {
		i = 12;
		data_ptr = (char *)(packet + i);
		while (readwrite_addr < final_addr) {
			retval = target_write_memory(target, readwrite_addr, 1, readwritecnt, (uint8_t *)data_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("const addr write byte data failed\n");
				break;
			}
			readwrite_addr = readwrite_addr + readwritecnt;
			data_ptr = data_ptr + readwritecnt;
		}
		*response_len = 2;
	}

	ndsv5_byte_access_from_burn = 0;
	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	nds32->nds_const_addr_mode = 0;
	memory->access_channel = bak_access_channel;
	return retval;
}

static int set_rtos_hartid(uint32_t target_num)
{
	struct target *target = NULL;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == (int)target_num)
			break;
	}
	if (target == NULL) {
		LOG_ERROR("Target is NULL\n");
		return ERROR_FAIL;
	}

	if (!riscv_rtos_enabled(target))
		return ERROR_OK;

	int hartid = -1;
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (riscv_hart_enabled(target, i)) {
			LOG_DEBUG("target core num n %d", i);
			hartid = i;
			break;
		}
	}
	if (hartid < 0) {
		LOG_ERROR("hartid < 0, no hart enable");
		return ERROR_FAIL;
	}
	select_burner_rtos_hartid = hartid;
	LOG_DEBUG("select_burner_rtos_hartid: %d\n", select_burner_rtos_hartid);
	return ERROR_OK;
}

static int ndsv5_burner_input(struct connection *connection)
{
	unsigned char cmmd_buffer[BURNER_BUFFER_SIZE] = {0};
	unsigned char *buf_p = (unsigned char *)&cmmd_buffer[0];
	char *target_name;
	int retval = 0, bytes_read, res_length, name_len;

	/* first connection for send BURNER_SELECT_TARGET or BURNER_SELECT_CORE, used current target to execute */
	if (ndsv5_burner_target != NULL) {
		select_burner_coreid = ndsv5_burner_target->target_number;
		if (set_rtos_hartid(select_burner_coreid) != ERROR_OK)
			return ERROR_FAIL;
		ndsv5_burner_target = NULL;
	}
	bool select = false;
	uint32_t burner_coreid = select_burner_coreid, cmd_type;
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == (int)burner_coreid) {
			select = true;
			if (riscv_rtos_enabled(target))
				riscv_set_current_hartid(target, select_burner_rtos_hartid);
			break;
		}
	}
	if (!select)
		return ERROR_FAIL;

	bytes_read = connection_read(connection, buf_p, BURNER_BUFFER_SIZE);

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1) {
		LOG_ERROR("error during burner_input");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	int debug_level_bak;
	if (burner_debug_mode == 0) {
		debug_level_bak = debug_level;
		debug_level = LOG_LVL_WARNING;
	} else {
		debug_level_bak = debug_level;
		debug_level = LOG_LVL_DEBUG;
	}

	res_length = 0;
	cmd_type = buf_p[0];
	LOG_DEBUG("target_num=%d: cmd_type=0x%x, bytes_read=0x%x", target->target_number, cmd_type, bytes_read);
	switch (cmd_type) {
		case WRITE_WORD:
		case WRITE_HALF:
		case WRITE_BYTE:
			retval = burner_cmd_write(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		case READ_WORD:
		case READ_HALF:
		case READ_BYTE:
			retval = burner_cmd_read(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		case FAST_READ:
		case FAST_WRITE:
			retval = burner_fast_readwrite(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		case FAST_READ_BYTE:
		case FAST_WRITE_BYTE:
			retval = burner_fast_byte_readwrite(target, buf_p, &res_length);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			break;
		case HOLD_CORE:
			retval = target_halt(target);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			res_length = 2;
			break;
		case RESET_HOLD:
			/* reset and halt all harts(AMP) */
			retval = ndsv5_reset_halt_as_examine(target);
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			res_length = 2;
			break;
		case RESET_TARGET:
			/* TODO: Use SRST as default!! */
			ndsv5_reset_target(target, RESET_HALT);

			/* free run in debug mode */
			retval = target_resume(target, 1, 0, 0, 0);
			/* //retval = ndsv5_srst_reset_target(target); */
			if (retval != ERROR_OK)
				buf_p[0] |= 0x80;
			res_length = 2;
			break;

		case RESET_AICE:
		case BURNER_INIT:
		case WRITE_IO:
		case READ_REG:
		case WRITE_REG:
			/* do nothing */
			res_length = 2;
			break;

		case BURNER_QUIT:
			debug_level = debug_level_bak;
			return -1;

		case BURNER_SELECT_CORE:
			burner_coreid = get_u32(buf_p+1);
			LOG_DEBUG("Change Burn Coreid to %d", burner_coreid);
			select_burner_coreid = burner_coreid;
			if (set_rtos_hartid(select_burner_coreid) != ERROR_OK)
				buf_p[0] |= 0x80;
			res_length = 2;
			break;

		case BURNER_SELECT_TARGET:
			name_len = get_u32(buf_p+1);
			/* use calloc can initial to 0 and let length = name_len + 1(the end of string need be 0),
			 * target_name should be a string for generic function get_target() */
			target_name = (char *) calloc(name_len+1, sizeof(char));
			memcpy(target_name, buf_p+5, name_len);
			target = get_target(target_name);
			if (target == NULL) {
				LOG_ERROR("Target: %s is unknown\n", target_name);
				buf_p[0] |= 0x80;
			} else {
				select_burner_coreid = target->target_number;
				LOG_DEBUG("Change Burn Coreid to %d", select_burner_coreid);
				if (set_rtos_hartid(select_burner_coreid) != ERROR_OK)
					buf_p[0] |= 0x80;
			}
			res_length = 2;
			free(target_name);
			break;

		case BURNER_DEBUG:
			burner_debug_mode = 1;
			res_length = 2;
			break;

		default:
			break;
	}
	buf_p[1] = 0;
	retval = burner_write(connection, buf_p, res_length);

	/* restore debug_level at the end of every burner command for eticket 16504 */
	debug_level = debug_level_bak;

	return retval;
}

static int ndsv5_burner_new_connection(struct connection *connection)
{
	LOG_DEBUG("%s", __func__);
	/* halt all targets */
	bool first_target = false;
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		/* setting ndsv5_burner_target:fix bug for ndsv5_burner_input failed,
		 * but restart SPI_burn/PAR_burn to try again */
		if (!first_target) {
			ndsv5_burner_target = target;
			first_target = true;
		}
		target_halt(target);
	}
	jtag_poll_set_enabled(false);

	/* Debug Mode */
	extern unsigned int MaxLogFileSize;
	if (MaxLogFileSize == 0x20000000)
		burner_debug_mode = 1;

	return ERROR_OK;
}

static int ndsv5_burner_connection_closed(struct connection *connection)
{
	LOG_DEBUG("%s", __func__);
	jtag_poll_set_enabled(true);
	return ERROR_OK;
}

static uint32_t ndsv5_burner_server_init_done;
int ndsv5_burner_server_init(struct target *target)
{
	if (ndsv5_burner_server_init_done)
		return ERROR_OK;
	LOG_DEBUG("%s", __func__);

	int ret_value = add_service("burner",
		ndsv5_burner_port,
		1,
		&ndsv5_burner_new_connection,
		&ndsv5_burner_input,
		&ndsv5_burner_connection_closed,
		NULL);
	ndsv5_burner_server_init_done = 1;
	return ret_value;
}

