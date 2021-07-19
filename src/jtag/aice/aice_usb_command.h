/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
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
#ifndef __AICE_USB_COMMAND_H__
#define __AICE_USB_COMMAND_H__

#include <target/target.h>

enum aice_usb_cmmd_index {
	AICE_CMDIDX_WRITE_CTRL = 0,
	AICE_CMDIDX_WRITE_MISC,
	AICE_CMDIDX_WRITE_EDMSR,
	AICE_CMDIDX_WRITE_DTR,
	AICE_CMDIDX_WRITE_DTR_FROM_BUFFER,
	AICE_CMDIDX_READ_DTR_TO_BUFFER,
	AICE_CMDIDX_WRITE_DIM,
	AICE_CMDIDX_EXECUTE,
	AICE_CMDIDX_WRITE_MEM,
	AICE_CMDIDX_WRITE_MEM_H,
	AICE_CMDIDX_WRITE_MEM_B,
	AICE_CMDIDX_FASTWRITE_MEM,
	AICE_CMDIDX_BATCH_BUFFER_WRITE,
	AICE_CMDIDX_WRITE_PINS,

	AICE_CMDIDX_SCAN_CHAIN,
	AICE_CMDIDX_READ_CTRL,
	AICE_CMDIDX_READ_MISC,
	AICE_CMDIDX_READ_EDMSR,
	AICE_CMDIDX_READ_DTR,
	AICE_CMDIDX_READ_MEM,
	AICE_CMDIDX_READ_MEM_H,
	AICE_CMDIDX_READ_MEM_B,
	AICE_CMDIDX_FASTREAD_MEM,
	AICE_CMDIDX_BATCH_BUFFER_READ,
	//AICE_CMDIDX_SELECT_TARGET,
};

enum aice_command_mode {
	AICE_COMMAND_MODE_NORMAL = 0x00,
	AICE_COMMAND_MODE_PACK,
	AICE_COMMAND_MODE_BATCH,
};

/* Constants for AICE command READ_CTRL */
#define AICE_READ_CTRL_INTF_FREQ            (0x00)
#define AICE_READ_CTRL_HARDWARE_VERSION     (0x01)
#define AICE_READ_CTRL_FPGA_VERSION         (0x02)
#define AICE_READ_CTRL_FIRMWARE_VERSION     (0x03)
#define AICE_READ_CTRL_ICE_CONFIG           (0x04)

#define AICE_READ_CTRL_GET_ICE_STATE         0x00
#define AICE_READ_CTRL_GET_HARDWARE_VERSION  0x01
#define AICE_READ_CTRL_GET_FPGA_VERSION      0x02
#define AICE_READ_CTRL_GET_FIRMWARE_VERSION  0x03
#define AICE_READ_CTRL_GET_JTAG_PIN_STATUS   0x04
#define AICE_READ_CTRL_BATCH_CTRL            0x20
#define AICE_READ_CTRL_BATCH_ITERATION       0x21
#define AICE_READ_CTRL_BATCH_BUF_INFO        0x22
#define AICE_READ_CTRL_BATCH_STATUS          0x23
#define AICE_READ_CTRL_BATCH_CMD_BUF0_CTRL   0x30
#define AICE_READ_CTRL_BATCH_BUF0_STATE      0x31
#define AICE_READ_CTRL_BATCH_BUF4_STATE      0x39
#define AICE_READ_CTRL_BATCH_BUF5_STATE      0x3B

/* Constants for AICE command WRITE_CTRL */
#define AICE_WRITE_CTRL_TCK_CONTROL          0x00
#define AICE_WRITE_CTRL_JTAG_PIN_CONTROL     0x01
#define AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS 0x02
#define AICE_WRITE_CTRL_RESERVED             0x03
#define AICE_WRITE_CTRL_JTAG_PIN_STATUS      0x04
#define AICE_WRITE_CTRL_TIMEOUT              0x07
#define AICE_WRITE_CTRL_SDMCONN              0x08
#define AICE_WRITE_CTRL_CUSTOM_DELAY         0x0D
#define AICE_WRITE_CTRL_BATCH_CTRL           0x20
#define AICE_WRITE_CTRL_BATCH_ITERATION      0x21
#define AICE_WRITE_CTRL_BATCH_DIM_SIZE       0x22
#define AICE_WRITE_CTRL_BATCH_CMD_BUF0_CTRL  0x30
#define AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL 0x38
#define AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL 0x3A
#define AICE_WRITE_CTRL_EFREQ                0x40
#define AICE_REG_BATCH_DATA_BUFFER_1_STATE   (AICE_READ_CTRL_BATCH_BUF5_STATE)

#define AICE_BATCH_COMMAND_BUFFER_0  0x00
#define AICE_BATCH_COMMAND_BUFFER_1  0x01
#define AICE_BATCH_COMMAND_BUFFER_2  0x02
#define AICE_BATCH_COMMAND_BUFFER_3  0x03
#define AICE_BATCH_DATA_BUFFER_0     0x04
#define AICE_BATCH_DATA_BUFFER_1     0x05
#define AICE_BATCH_DATA_BUFFER_2     0x06
#define AICE_BATCH_DATA_BUFFER_3     0x07

/* Custom SRST/DBGI/TRST */
#define AICE_CUSTOM_DELAY_SET_SRST		0x01
#define AICE_CUSTOM_DELAY_CLEAN_SRST	0x02
#define AICE_CUSTOM_DELAY_SET_DBGI		0x04
#define AICE_CUSTOM_DELAY_CLEAN_DBGI	0x08
#define AICE_CUSTOM_DELAY_SET_TRST		0x10
#define AICE_CUSTOM_DELAY_CLEAN_TRST	0x20
/* Constants for AICE command WRITE_CTRL:TCK_CONTROL */
#define AICE_TCK_CONTROL_TCK_SCAN		0x10

extern unsigned char usb_out_packets_buffer[];
extern unsigned char usb_in_packets_buffer[];
extern unsigned int usb_out_packets_buffer_length;
extern unsigned int usb_in_packets_buffer_length;
extern enum aice_command_mode aice_command_mode;
extern unsigned int aice_usb_rx_max_packet;
extern unsigned int aice_usb_tx_max_packet;
extern uint32_t jtag_clock;
extern uint32_t aice_count_to_check_dbger;
extern uint32_t aice_set_usb_timeout;

int aice_usb_open(unsigned int usb_vid, unsigned int usb_pid);
int aice_usb_close(void);
int aice_usb_write(unsigned char *out_buffer, unsigned int out_length);
int aice_usb_read(unsigned char *in_buffer, unsigned int expected_size);
int aice_usb_reset_box(void);
int aice_scan_chain(unsigned int *id_codes, unsigned char *num_of_ids);
int aice_usb_write_ctrl(uint32_t address, uint32_t WriteData);
int aice_usb_read_ctrl(uint32_t address, uint32_t *pReadData);
int aice_usb_read_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words );
int aice_usb_write_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words );
int aice_read_dtr_to_buffer(uint32_t target_id, uint32_t buffer_idx);
int aice_write_dtr_from_buffer(uint32_t target_id, uint32_t buffer_idx);

int aice_batch_buffer_write(uint32_t buf_index);
int aice_batch_buffer_read(uint32_t buf_index, unsigned char *pReadData, uint32_t num_of_words);
int aice_pack_buffer_read(unsigned char *pReadData, uint32_t num_of_bytes);

int aice_usb_set_command_mode(enum aice_command_mode command_mode);
int aice_usb_execute_custom_script(struct target *target, const char *script);
int aice_reset_aice_as_startup(void);
int aice_usb_set_clock(uint32_t set_clock);
int aice_set_write_pins_support(uint32_t if_support);
int aice_icemem_xread(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr);
int aice_icemem_xwrite(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr);

#endif
