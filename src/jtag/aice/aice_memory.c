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

extern uint32_t aice_usb_pack_command;
extern uint32_t nds_ftdi_jtag_opt;
typedef int (*read_mem_func_t)(struct target *target, uint32_t address, uint32_t *data);
typedef int (*write_mem_func_t)(struct target *target, uint32_t address, uint32_t data);

static int aice_usb_fastread_mem(struct target *target, uint32_t addr, unsigned char *pReadData, unsigned int num_of_words)
{
	return aice_read_edm(target, JDP_R_FAST_MEM, addr, (uint32_t*)pReadData, num_of_words);
}

static int aice_usb_fastwrite_mem(struct target *target, uint32_t addr, const unsigned char *pWriteData, unsigned int num_of_words)
{
	return aice_write_edm(target, JDP_W_FAST_MEM, addr, (uint32_t*)pWriteData, num_of_words);
}

static int aice_usb_set_address_dim(struct target *target, uint32_t address)
{
	uint32_t instructions[4] = {
		SETHI(R0, address >> 12),
		ORI(R0, R0, address & 0x00000FFF),
		NOP,
		BEQ_MINUS_12
	};

	return aice_execute_dim(target, instructions, 4);
}

static int aice_usb_write_mem_b_bus(struct target *target, uint32_t address, uint32_t val)
{
	uint32_t WriteData = val;
	return aice_write_edm(target, JDP_W_MEM_B, address, (uint32_t*)&WriteData, 1);
}

static int aice_usb_write_mem_h_bus(struct target *target, uint32_t address, uint32_t val)
{
	uint32_t WriteData = val;
	return aice_write_edm(target, JDP_W_MEM_H, address, (uint32_t*)&WriteData, 1);
}

static int aice_usb_write_mem_w_bus(struct target *target, uint32_t address, uint32_t val)
{
	uint32_t WriteData = val;
	return aice_write_edm(target, JDP_W_MEM_W, address, (uint32_t*)&WriteData, 1);
}

static int aice_usb_read_mem_b_bus(struct target *target, uint32_t address, uint32_t *pReadData)
{
	return aice_read_edm(target, JDP_R_MEM_B, address, (uint32_t*)pReadData, 1);
}

static int aice_usb_read_mem_h_bus(struct target *target, uint32_t address, uint32_t *pReadData)
{
	return aice_read_edm(target, JDP_R_MEM_H, address, (uint32_t*)pReadData, 1);
}

static int aice_usb_read_mem_w_bus(struct target *target, uint32_t address, uint32_t *pReadData)
{
	return aice_read_edm(target, JDP_R_MEM_W, address, (uint32_t*)pReadData, 1);
}

static int aice_usb_read_mem_b_dim(struct target *target, uint32_t address, uint32_t *data)
{
	uint32_t value;
	uint32_t instructions[4] = {
		LBI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	int result = aice_execute_dim(target, instructions, 4);
	if (result != ERROR_OK)
		return result;

	aice_read_dtr(target, &value);
	*data = value & 0xFF;

	return ERROR_OK;
}

static int aice_usb_read_mem_h_dim(struct target *target, uint32_t address, uint32_t *data)
{
	uint32_t value;
	uint32_t instructions[4] = {
		LHI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	int result = aice_execute_dim(target, instructions, 4);
	if (result != ERROR_OK)
		return result;

	aice_read_dtr(target, &value);
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->data_endian == TARGET_BIG_ENDIAN) {
		value &= 0xFFFF;
		*data = (((value >> 8) & 0xFF) |
		((value & 0xFF) << 8) );
	}
	else {
		*data = value & 0xFFFF;
	}
	return ERROR_OK;
}

static int aice_usb_read_mem_w_dim(struct target *target, uint32_t address, uint32_t *data)
{
	uint32_t value;
	uint32_t instructions[4] = {
		LWI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	int result = aice_execute_dim(target, instructions, 4);
	if (result != ERROR_OK)
		return result;

	aice_read_dtr(target, &value);
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->data_endian == TARGET_BIG_ENDIAN) {
		*data = (((value >> 24) & 0x00FF) |
			((value >> 8) & 0xFF00) |
			((value & 0xFF00) << 8) |
			((value & 0x00FF) << 24) );
	}
	else {
		*data = value;
	}

	return ERROR_OK;
}

static int aice_usb_write_mem_b_dim(struct target *target, uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SBI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};

	if (aice_write_dtr(target, data & 0xFF) != ERROR_OK)
		return ERROR_FAIL;
	return aice_execute_dim(target, instructions, 4);
}

static int aice_usb_write_mem_h_dim(struct target *target, uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SHI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};
	uint32_t write_dtrData;
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->data_endian == TARGET_BIG_ENDIAN) {
		write_dtrData = (((data >> 8) & 0x00FF) |
			((data & 0x00FF) << 8) );
	}
	else {
		write_dtrData = data & 0xFFFF;
	}
	if (aice_write_dtr(target, write_dtrData) != ERROR_OK)
		return ERROR_FAIL;
	return aice_execute_dim(target, instructions, 4);
}

static int aice_usb_write_mem_w_dim(struct target *target, uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SWI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};
	uint32_t write_dtrData;
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->data_endian == TARGET_BIG_ENDIAN) {
		write_dtrData = (((data >> 24) & 0x00FF) |
			((data >> 8) & 0xFF00) |
			((data & 0xFF00) << 8) |
			((data & 0x00FF) << 24) );
	}
	else {
		write_dtrData = data;
	}
	if (aice_write_dtr(target, write_dtrData) != ERROR_OK)
		return ERROR_FAIL;
	return aice_execute_dim(target, instructions, 4);
}

#if 0
static int aice_usb_read_mem_dim_pack(struct target *target, uint32_t address, uint32_t size,
		uint32_t count, uint8_t *pbuffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t count_per_pack, i, j, data_value;
	uint32_t offset = size;
	uint8_t *pbyte_data;

	uint32_t instructions_address[4] = {
		SETHI(R0, address >> 12),
		ORI(R0, R0, address & 0x00000FFF),
		NOP,
		BEQ_MINUS_12
	};
	uint32_t instructions[4] = {
		LWI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	if (size == 1) {
		instructions[0] = LBI_BI(R1, R0);
	} else if (size == 2) {
		instructions[0] = LHI_BI(R1, R0);
	}

	while(count) {
		if (count >= 9)
			count_per_pack = 9;
		else
			count_per_pack = count;
		count -= count_per_pack;

		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;

		instructions_address[0] = SETHI(R0, address >> 12);
		instructions_address[1] = ORI(R0, R0, address & 0x00000FFF);
		int result = aice_execute_dim(target, instructions_address, 4);
		if (result != ERROR_OK)
			return result;

		for (i = 0; i < count_per_pack; i++) {
			result = aice_execute_dim(target, instructions, 4);
			if (result != ERROR_OK)
				return result;
			aice_read_dtr(target, &data_value);
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		char usbin_data[512];
		aice_packbuffer_read(target, (uint8_t *)&usbin_data[0], aice_packet_append_size);
		unsigned char *pread_byte = (unsigned char *)&usbin_data[52];  // (execute_dim)20 +20 +12

		for (i = 0; i < count_per_pack; i++) {
			data_value = (unsigned char)pread_byte[3];
			data_value |= ((unsigned char)pread_byte[2] << 8);
			data_value |= ((unsigned char)pread_byte[1] << 16);
			data_value |= ((unsigned char)pread_byte[0] << 24);
			pread_byte += 36;  // (execute_dim)20 + (read_dtr)16
			LOG_DEBUG("DTR: 0x%08x", data_value);

			pbyte_data = (uint8_t *)&data_value;
			if (nds32->data_endian == TARGET_BIG_ENDIAN) {
				for (j = offset; j > 0; j--) {
					*pbuffer ++ = pbyte_data[j-1];
				}
			} else {
				for (j = 0; j < offset; j++) {
					*pbuffer ++ = pbyte_data[j];
				}
			}
		}
		address += (offset * count_per_pack);
	}
	return ERROR_OK;
}
#endif

int aice_read_mem_unit(struct target *target, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access access_channel = nds32->memory.access_channel;

	LOG_DEBUG("aice_read_mem_unit, addr: 0x%08x, size: %d, count: %d",
			addr, size, count);
#if 0
	if ((aice_usb_pack_command == 1) &&
		  (NDS_MEMORY_ACC_CPU == access_channel)){
		return aice_usb_read_mem_dim_pack(target, addr, size, count, buffer);
	}
#endif

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(target, addr);

	uint32_t value;
	size_t i;
	read_mem_func_t read_mem_func;

	switch (size) {
		case 1:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_b_bus;
			else
				read_mem_func = aice_usb_read_mem_b_dim;

			for (i = 0; i < count; i++) {
				if (read_mem_func(target, addr, &value) != ERROR_OK)
					return ERROR_FAIL;
				*buffer++ = (uint8_t)value;
				addr++;
			}
			break;
		case 2:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_h_bus;
			else
				read_mem_func = aice_usb_read_mem_h_dim;

			for (i = 0; i < count; i++) {
				if (read_mem_func(target, addr, &value) != ERROR_OK)
					return ERROR_FAIL;
				uint16_t svalue = value;
				memcpy(buffer, &svalue, sizeof(uint16_t));
				buffer += 2;
				addr += 2;
			}
			break;
		case 4:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_w_bus;
			else
				read_mem_func = aice_usb_read_mem_w_dim;

			for (i = 0; i < count; i++) {
				if (read_mem_func(target, addr, &value) != ERROR_OK)
					return ERROR_FAIL;
				memcpy(buffer, &value, sizeof(uint32_t));
				buffer += 4;
				addr += 4;
			}
			break;
	}

	return ERROR_OK;
}

int aice_write_mem_unit(struct target *target, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access access_channel = nds32->memory.access_channel;

	LOG_DEBUG("aice_write_mem_unit, addr: 0x%08x, size: %d, count: %d",
			addr, size, count);

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(target, addr);

	size_t i;
	write_mem_func_t write_mem_func;

	switch (size) {
		case 1:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_b_bus;
			else
				write_mem_func = aice_usb_write_mem_b_dim;

			for (i = 0; i < count; i++) {
				if (write_mem_func(target, addr, *buffer) != ERROR_OK)
					return ERROR_FAIL;
				buffer++;
				addr++;
			}
			break;
		case 2:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_h_bus;
			else
				write_mem_func = aice_usb_write_mem_h_dim;

			for (i = 0; i < count; i++) {
				uint16_t value;
				memcpy(&value, buffer, sizeof(uint16_t));

				if (write_mem_func(target, addr, value) != ERROR_OK)
					return ERROR_FAIL;
				buffer += 2;
				addr += 2;
			}
			break;
		case 4:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_w_bus;
			else
				write_mem_func = aice_usb_write_mem_w_dim;

			for (i = 0; i < count; i++) {
				uint32_t value;
				memcpy(&value, buffer, sizeof(uint32_t));

				if (write_mem_func(target, addr, value) != ERROR_OK) {
					LOG_DEBUG("aice_write_mem_unit ERROR !!");
					return ERROR_FAIL;
				}
				buffer += 4;
				addr += 4;
			}
			break;
	}

	return ERROR_OK;
}

static int aice_bulk_read_mem(struct target *target, uint32_t addr, uint32_t count,
		uint8_t *buffer)
{
	uint32_t packet_size;
	uint32_t const_mode = (addr & 0x02);

	/** set address */
	addr &= 0xFFFFFFFC;
	addr |= const_mode;
	if (aice_write_misc(target, NDS_EDM_MISC_SBAR, addr) != ERROR_OK)
		return ERROR_FAIL;

	if ((aice_port->type == AICE_PORT_FTDI) && (nds_ftdi_jtag_opt == 2)) {
		LOG_DEBUG("VENDOR aice_usb_fastread_mem, addr: 0x%08x, count: 0x%08x", addr, count);
		if (aice_usb_fastread_mem(target, addr, buffer, count) != ERROR_OK)
			return ERROR_FAIL;
		return ERROR_OK;
	}

	while (count > 0) {
		packet_size = (count >= 0x100) ? 0x100 : count;

		if (aice_usb_fastread_mem(target, addr, buffer, packet_size) != ERROR_OK)
			return ERROR_FAIL;

		buffer += (packet_size * 4);
		if (const_mode == 0)
			addr += (packet_size * 4);
		count -= packet_size;
	}

	return ERROR_OK;
}

static int aice_bulk_write_mem(struct target *target, uint32_t addr, uint32_t count,
		uint8_t *buffer)
{
	uint32_t packet_size;
	uint32_t const_mode = (addr & 0x02);

	/** set address */
	addr &= 0xFFFFFFFC;
	addr |= const_mode;
	if (aice_write_misc(target, NDS_EDM_MISC_SBAR, addr | 1) != ERROR_OK)
		return ERROR_FAIL;

	if ((aice_port->type == AICE_PORT_FTDI) && (nds_ftdi_jtag_opt == 2)) {
		LOG_DEBUG("VENDOR aice_usb_fastwrite_mem, addr: 0x%08x, count: 0x%08x", addr, count);
		if (aice_usb_fastwrite_mem(target, addr, buffer, count) != ERROR_OK)
			return ERROR_FAIL;
		return ERROR_OK;
	}

	while (count > 0) {
		packet_size = (count >= 0x100) ? 0x100 : count;
		/* for AICE-mini zero-packet issue, +8 => AICE_FORMAT_HTDMD-4  */
		if ( (((packet_size*4)+8) % 64) == 0) {
			packet_size -= 1;
			LOG_DEBUG("aice_bulk_write_mem-zero-packet, addr: 0x%08x, packet_size: 0x%08x", addr, packet_size);
		}

		if (aice_usb_fastwrite_mem(target, addr, buffer, packet_size) != ERROR_OK)
			return ERROR_FAIL;

		buffer += (packet_size * 4);
		if (const_mode == 0)
			addr += (packet_size * 4);
		count -= packet_size;
	}

	return ERROR_OK;
}

int aice_read_mem_bulk(struct target *target, uint32_t addr, uint32_t length, uint8_t *buffer)
{
	LOG_DEBUG("aice_read_mem_bulk, addr: 0x%08x, length: 0x%08x", addr, length);

	int retval, auto_switch_bus = 0;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	enum nds_memory_access access_channel = nds32->memory.access_channel;

	/* auto switch to BUS bulk mode for performance */
	if (access_channel == NDS_MEMORY_ACC_CPU) {
		if (((memory->ilm_base != 0) && (memory->ilm_enable == true)) ||
				((memory->dlm_base != 0) && (memory->dlm_enable == true))) {
				auto_switch_bus = 0;
		}
		else if (nds32->memory.dcache.enable == false) {
				auto_switch_bus = 1;
		}
		LOG_DEBUG("memory->ilm_enable: %x, memory->dlm_enable: %x", memory->ilm_enable, memory->dlm_enable);
		LOG_DEBUG("dcache.enable: %x, access_channel: %x", nds32->memory.dcache.enable, access_channel);
	}
	if ((auto_switch_bus) || (access_channel != NDS_MEMORY_ACC_CPU)) {
			retval = aice_bulk_read_mem(target, addr, length / 4, buffer);
	}
	else {
		aice_usb_set_address_dim(target, addr);
		retval = aice_read_mem_unit(target, addr, 4, length / 4, buffer);
	}

	return retval;
}

int aice_write_mem_bulk(struct target *target, uint32_t addr, uint32_t length, uint8_t *buffer)
{
	LOG_DEBUG("aice_write_mem_bulk, addr: 0x%08x, length: 0x%08x", addr, length);

	int retval, auto_switch_bus = 0;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	enum nds_memory_access access_channel = nds32->memory.access_channel;

	/* auto switch to BUS bulk mode for performance */
	if (access_channel == NDS_MEMORY_ACC_CPU) {
		if (((memory->ilm_base != 0) && (memory->ilm_enable == true)) ||
				((memory->dlm_base != 0) && (memory->dlm_enable == true))) {
				auto_switch_bus = 0;
		}
		else if (nds32->memory.dcache.enable == false) {
				auto_switch_bus = 1;
		}
		LOG_DEBUG("memory->ilm_enable: %x, memory->dlm_enable: %x", memory->ilm_enable, memory->dlm_enable);
		LOG_DEBUG("dcache.enable: %x, access_channel: %x", nds32->memory.dcache.enable, access_channel);
	}
	if ((auto_switch_bus) || (access_channel != NDS_MEMORY_ACC_CPU)) {
			retval = aice_bulk_write_mem(target, addr, length / 4, buffer);
	}
	else {
		aice_usb_set_address_dim(target, addr);
		retval = aice_write_mem_unit(target, addr, 4, length / 4, buffer);
	}

	return retval;
}

char *pStringMemSelect[] = {
	"MEMORY_SELECT_AUTO",
	"MEMORY_SELECT_MEM",
	"MEMORY_SELECT_ILM",
	"MEMORY_SELECT_DLM",
};

int aice_acc_memory_mode(struct target *target, enum nds_memory_select mem_select)
{
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->memory.set_acc_mode == mem_select)
		return ERROR_OK;
	else if (mem_select > NDS_MEMORY_SELECT_DLM)
		return ERROR_FAIL;

	LOG_DEBUG("aice_usb_memory_mode, memory select: %d (%s)", mem_select, (char*)pStringMemSelect[mem_select]);

	if (mem_select == NDS_MEMORY_SELECT_AUTO)
		mem_select = NDS_MEMORY_SELECT_MEM;

	nds32->memory.set_acc_mode = mem_select;
	aice_write_misc(target, NDS_EDM_MISC_ACC_CTL,
				nds32->memory.set_acc_mode - 1);

	return ERROR_OK;
}
