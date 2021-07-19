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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jtag/drivers/libusb_common.h>
#include <log.h>
#include "aice_usb_pack_format.h"
#include "aice_usb_command.h"
#include "aice_jdp.h"
#include <target/target.h>
#include <target/nds32.h>
#include <target/nds32_log.h>
#include <helper/time_support.h>

#define _ADAPTER_USE_ 0x00

#if _ADAPTER_USE_
#include "aice_pipe_command.h"
#else
#include "aice_usb.h"
#endif

#define AICE_USBCMMD_MSG	LOG_DEBUG

extern struct command_context *global_cmd_ctx;
extern uint32_t aice_usb_pack_command;
extern void aice_print_info(unsigned int pipe_cmmd, unsigned int address,
	unsigned int *pInfoData, unsigned int target_id, unsigned int jdp_id);
extern int aice_write_ctrl(uint32_t address, uint32_t WriteData);
extern int aice_write_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
extern int aice_read_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);

/* AICE USB timeout value */
#define AICE_USB_TIMEOUT				5000
/* Global USB buffers */
#define AICE_IN_BUFFER_SIZE            0x4100 // for XREAD, bulk-in: 9+0x2000+9+0x2000
#define AICE_OUT_BUFFER_SIZE           0x4000
#define AICE_IN_PACKETS_BUFFER_SIZE    1032
#define AICE_OUT_PACKETS_BUFFER_SIZE   1032
#define AICE_IN_BATCH_COMMAND_SIZE     1032
#define AICE_OUT_BATCH_COMMAND_SIZE    1032
#define AICE_IN_PACK_COMMAND_SIZE      511
#define AICE_OUT_PACK_COMMAND_SIZE     511
static unsigned char usb_in_buffer[AICE_IN_BUFFER_SIZE];
static unsigned char usb_out_buffer[AICE_OUT_BUFFER_SIZE];
unsigned char usb_out_packets_buffer[AICE_OUT_PACKETS_BUFFER_SIZE];
unsigned char usb_in_packets_buffer[AICE_IN_PACKETS_BUFFER_SIZE];
unsigned int usb_out_packets_buffer_length = 0;
unsigned int usb_in_packets_buffer_length = 0;
unsigned int aice_keep_usb_packet_append = 0;
unsigned int aice_keep_usb_in_curr_packets_length = 0;
enum aice_command_mode aice_command_mode = AICE_COMMAND_MODE_NORMAL;
unsigned int aice_max_retry_times = 2;//50;
unsigned int aice_usb_rx_max_packet = 512;
unsigned int aice_usb_tx_max_packet = 512;
uint32_t aice_write_pins_support = 0;
unsigned int log_usb_packets = 0;

struct aice_usb_cmmd_info usb_cmmd_pack_info = {
		.pusb_buffer = &usb_out_buffer[0],
		.cmdtype = 0,
		.access_little_endian = 1,
		.cmd = 0,
		.target = 0,
		.length = 0,
		.addr = 0,
		.pword_data = 0,
};

struct aice_usb_cmmd_info usb_cmmd_unpack_info = {
		.pusb_buffer = &usb_in_buffer[0],
		.cmdtype = 0,
		.access_little_endian = 1,
		.cmd = 0,
		.target = 0,
		.length = 0,
		.addr = 0,
		.pword_data = 0,
};

struct aice_usb_cmmd_attr {
	const char *cmdname;
	unsigned char cmd;
	unsigned char h2d_type;
	unsigned char d2h_type;
};

/* Constants for AICE command */
#define AICE_CMD_SCAN_CHAIN       0x00
#define AICE_CMD_SELECT_TARGET    0x01
#define AICE_CMD_READ_DIM         0x02
#define AICE_CMD_READ_EDMSR       0x03
#define AICE_CMD_READ_DTR         0x04
#define AICE_CMD_READ_MEM         0x05
#define AICE_CMD_READ_MISC        0x06
#define AICE_CMD_FASTREAD_MEM     0x07
#define AICE_CMD_WRITE_DIM        0x08
#define AICE_CMD_WRITE_EDMSR      0x09
#define AICE_CMD_WRITE_DTR        0x0A
#define AICE_CMD_WRITE_MEM        0x0B
#define AICE_CMD_WRITE_MISC       0x0C
#define AICE_CMD_FASTWRITE_MEM    0x0D
#define AICE_CMD_EXECUTE          0x0E
#define AICE_CMD_XREAD            0x10
#define AICE_CMD_XWRITE           0x11

#define AICE_CMD_READ_MEM_B       0x14
#define AICE_CMD_READ_MEM_H       0x15
#define AICE_CMD_WRITE_PINS       0x18
#define AICE_CMD_T_READ_MISC      0x20
#define AICE_CMD_T_READ_EDMSR     0x21
#define AICE_CMD_T_READ_DTR       0x22
#define AICE_CMD_T_READ_DIM       0x23
#define AICE_CMD_T_READ_MEM_B     0x24
#define AICE_CMD_T_READ_MEM_H     0x25
#define AICE_CMD_T_READ_MEM       0x26
#define AICE_CMD_T_FASTREAD_MEM   0x27
#define AICE_CMD_T_WRITE_MISC     0x28
#define AICE_CMD_T_WRITE_EDMSR    0x29
#define AICE_CMD_T_WRITE_DTR      0x2A
#define AICE_CMD_T_WRITE_DIM      0x2B
#define AICE_CMD_T_WRITE_MEM_B    0x2C
#define AICE_CMD_T_WRITE_MEM_H    0x2D
#define AICE_CMD_T_WRITE_MEM      0x2E
#define AICE_CMD_T_FASTWRITE_MEM  0x2F
#define AICE_CMD_T_GET_TRACE_STATUS    0x36
#define AICE_CMD_T_EXECUTE             0x3E
#define AICE_CMD_AICE_PROGRAM_READ     0x40
#define AICE_CMD_AICE_PROGRAM_WRITE    0x41
#define AICE_CMD_AICE_PROGRAM_CONTROL  0x42
#define AICE_CMD_READ_CTRL             0x50
#define AICE_CMD_WRITE_CTRL            0x51
#define AICE_CMD_BATCH_BUFFER_READ     0x60
#define AICE_CMD_READ_DTR_TO_BUFFER    0x61
#define AICE_CMD_BATCH_BUFFER_WRITE    0x68
#define AICE_CMD_WRITE_DTR_FROM_BUFFER 0x69

struct aice_usb_cmmd_attr usb_all_cmmd_attr[] = {
	{ "WRITE_CTRL", AICE_CMD_WRITE_CTRL, AICE_CMDTYPE_HTDC, AICE_CMDTYPE_DTHB, },
	{ "WRITE_MISC", AICE_CMD_T_WRITE_MISC, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_EDMSR", AICE_CMD_T_WRITE_EDMSR, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_DTR", AICE_CMD_T_WRITE_DTR, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_DTR_FROM_BUFFER", AICE_CMD_WRITE_DTR_FROM_BUFFER, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMB, },
	{ "READ_DTR_TO_BUFFER", AICE_CMD_READ_DTR_TO_BUFFER, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_DIM", AICE_CMD_T_WRITE_DIM, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "EXECUTE", AICE_CMD_T_EXECUTE, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_MEM", AICE_CMD_T_WRITE_MEM, AICE_CMDTYPE_HTDMD, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_MEM_H", AICE_CMD_T_WRITE_MEM_H, AICE_CMDTYPE_HTDMD, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_MEM_B", AICE_CMD_T_WRITE_MEM_B, AICE_CMDTYPE_HTDMD, AICE_CMDTYPE_DTHMB, },
	{ "FASTWRITE_MEM", AICE_CMD_T_FASTWRITE_MEM, AICE_CMDTYPE_HTDMD, AICE_CMDTYPE_DTHMB, },
	{ "BATCH_BUFFER_WRITE", AICE_CMD_BATCH_BUFFER_WRITE, AICE_CMDTYPE_HTDMC, AICE_CMDTYPE_DTHMB, },
	{ "WRITE_PINS", AICE_CMD_WRITE_PINS, AICE_CMDTYPE_HTDC, AICE_CMDTYPE_DTHB, },

	{ "SCAN_CHAIN", AICE_CMD_SCAN_CHAIN, AICE_CMDTYPE_HTDA, AICE_CMDTYPE_DTHA, },
	{ "READ_CTRL", AICE_CMD_READ_CTRL, AICE_CMDTYPE_HTDA, AICE_CMDTYPE_DTHA, },
	{ "READ_MISC", AICE_CMD_T_READ_MISC, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMA, },
	{ "READ_EDMSR", AICE_CMD_T_READ_EDMSR, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMA, },
	{ "READ_DTR", AICE_CMD_T_READ_DTR, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMA, },
	{ "READ_MEM", AICE_CMD_T_READ_MEM, AICE_CMDTYPE_HTDMB, AICE_CMDTYPE_DTHMA, },
	{ "READ_MEM_H", AICE_CMD_T_READ_MEM_H, AICE_CMDTYPE_HTDMB, AICE_CMDTYPE_DTHMA, },
	{ "READ_MEM_B", AICE_CMD_T_READ_MEM_B, AICE_CMDTYPE_HTDMB, AICE_CMDTYPE_DTHMA, },
	{ "FASTREAD_MEM", AICE_CMD_T_FASTREAD_MEM, AICE_CMDTYPE_HTDMB, AICE_CMDTYPE_DTHMA, },
	{ "BATCH_BUFFER_READ", AICE_CMD_BATCH_BUFFER_READ, AICE_CMDTYPE_HTDMA, AICE_CMDTYPE_DTHMA, },
};

unsigned int aice_usb_read_ep, aice_usb_write_ep;
struct jtag_libusb_device_handle *aice_usb_handle = NULL;

char NullName[] = {""};
char *pdescp_Manufacturer = (char *)&NullName[0];
char *pdescp_Product = (char *)&NullName[0];
unsigned int descp_bcdDevice = 0x0;

static int aice_access_cmmd(unsigned char cmdidx, unsigned char target_id,
		unsigned int address, unsigned char *pdata, unsigned int length);
static int aice_usb_packet_flush(void);
static int aice_usb_write_pins(unsigned int num_of_words, unsigned int *pWriteData);

int aice_usb_open(unsigned int usb_vid, unsigned int usb_pid)
{
	const uint16_t vids[] = { usb_vid, 0 };
	const uint16_t pids[] = { usb_pid, 0 };
	struct jtag_libusb_device_handle *devh = NULL;

	if (jtag_libusb_open(vids, pids, NULL, &devh) != ERROR_OK) {
		return ERROR_FAIL;
	}
	/* BE ***VERY CAREFUL*** ABOUT MAKING CHANGES IN THIS
	 * AREA!!!!!!!!!!!  The behavior of libusb is not completely
	 * consistent across Windows, Linux, and Mac OS X platforms.
	 * The actions taken in the following compiler conditionals may
	 * not agree with published documentation for libusb, but were
	 * found to be necessary through trials and tribulations.  Even
	 * little tweaks can break one or more platforms, so if you do
	 * make changes test them carefully on all platforms before
	 * committing them!
	 */

#if !defined(__MINGW32__) && !defined(__CYGWIN__)
	jtag_libusb_reset_device(devh);

#if IS_DARWIN == 0
	if (devh)
		jtag_libusb_close(devh);
	int timeout = 5;
	/* reopen jlink after usb_reset
	 * on win32 this may take a second or two to re-enumerate */
	int retval;
	while ((retval = jtag_libusb_open(vids, pids, NULL, &devh)) != ERROR_OK) {
		usleep(1000);
		timeout--;
		if (!timeout)
			break;
	}
	if (ERROR_OK != retval)
		return ERROR_FAIL;
#endif

#endif

	/* usb_set_configuration required under win32 */
	/* move set_configuration() into jtag_libusb_open() */
	//jtag_libusb_set_configuration(devh, 0);
	//jtag_libusb_claim_interface(devh, 0);
	unsigned int aice_read_ep = 0x83;
	unsigned int aice_write_ep = 0x02;
#if _NDS32_ONLY_
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	jtag_libusb_get_endpoints(udev, &aice_read_ep, &aice_write_ep,
			&aice_usb_rx_max_packet, &aice_usb_tx_max_packet);
#endif

	AICE_USBCMMD_MSG("aice_read_ep=0x%x, aice_write_ep=0x%x", aice_read_ep, aice_write_ep);
	AICE_USBCMMD_MSG("aice_usb_rx_max_packet=%d, aice_usb_tx_max_packet=%d",aice_usb_rx_max_packet, aice_usb_tx_max_packet);

	/* do NOT support usb_pack_command mode if USB1.1 */
	/*
	if (aice_usb_tx_max_packet == 64) {
		aice_usb_pack_command = 0;
	}*/

	aice_usb_read_ep = aice_read_ep;
	aice_usb_write_ep = aice_write_ep;
	aice_usb_handle = devh;

#if _NDS32_ONLY_
	jtag_libusb_get_descriptor_string(devh, udev,
		&pdescp_Manufacturer,
		&pdescp_Product,
		&descp_bcdDevice);
#endif
	AICE_USBCMMD_MSG("pdescp_Manufacturer = %s \n", pdescp_Manufacturer);
	AICE_USBCMMD_MSG("pdescp_Product = %s \n", pdescp_Product);
	AICE_USBCMMD_MSG("descp_bcdDevice = %x \n", descp_bcdDevice);

	// dummy read, bug-10444, workaround for USB Data Toggling problem under Linux
	unsigned int hardware_version;
	aice_usb_read_ctrl(AICE_READ_CTRL_GET_HARDWARE_VERSION, &hardware_version);
	aice_usb_read_ctrl(AICE_READ_CTRL_GET_HARDWARE_VERSION, &hardware_version);

	return ERROR_OK;
}

int aice_usb_close(void)
{
	if (aice_usb_handle) {
		jtag_libusb_close(aice_usb_handle);
		aice_usb_handle = NULL;
	}
	return ERROR_OK;
}

/* calls the given usb_bulk_* function, allowing for the data to
 * trickle in with some timeouts  */
static int usb_bulk_with_retries(
			int (*f)(jtag_libusb_device_handle *, int, char *, int, int),
			jtag_libusb_device_handle *dev, int ep,
			char *bytes, int size, int timeout)
{
	int tries = 3, count = 0;
	unsigned char *cur_bytes = (unsigned char *)bytes;
	unsigned int cur_attr = 0, cur_length = 0, rx_length = 0;

	while (tries && (count < size)) {
		int result = f(dev, ep, bytes + count, size - count, timeout);
		if (result > 0) {
			/*
			 * The length of SCAN_CHAIN and FASTREAD_MEM are variable,
			 * we should determine the real length by check LENGTH byte.
			 */
			if (!count && (ep & LIBUSB_ENDPOINT_IN) && result > 3) {
				switch (bytes[0]) {
					case AICE_CMD_FASTREAD_MEM:
					case AICE_CMD_SCAN_CHAIN:
						size = AICE_CMDSIZE_DTHA + 4*bytes[1];
						break;
					case AICE_CMD_T_FASTREAD_MEM:
						size = AICE_CMDSIZE_DTHMA + 4*bytes[2];
						break;

					case AICE_CMD_XREAD:
						while (1) {
							cur_attr = cur_bytes[4];
							cur_length = cur_bytes[8];
							cur_length |= (cur_bytes[7] << 8);
							rx_length += 9;
							rx_length += (cur_length << 2);
							//AICE_USBCMMD_MSG("cur_bytes=0x%lx, result=0x%x, cur_attr=0x%x, cur_length=0x%x",
							//  (long)cur_bytes, result, cur_attr, cur_length);
							if (cur_attr & 0x01) {
								return rx_length;
							}
							cur_bytes += rx_length;
							result = f(dev, ep, (char *)cur_bytes, size - (cur_length << 2), timeout);
						}

				}
			}
			count += result;
		}
		else if ((-ETIMEDOUT != result) || !--tries)
			return result;
	}
	return count;
}

static int wrap_usb_bulk_write(jtag_libusb_device_handle *dev, int ep,
		char *buff, int size, int timeout)
{
	/* usb_bulk_write() takes const char *buff */
	return jtag_libusb_bulk_write(dev, ep, buff, size, timeout);
}

static inline int usb_bulk_write_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	if (log_usb_packets == 1) {
		unsigned int i;
		char *pOutData = bytes;
		char msgbuffer[4096] = {0};
		char *pmsgbuffer = (char *)&msgbuffer[0];

		sprintf ( pmsgbuffer, "bulk_out:");
		pmsgbuffer += strlen(pmsgbuffer);
		for (i=0; i<(unsigned int)size; i++) {
			sprintf ( pmsgbuffer, " %02x", (unsigned char)*pOutData++);
			pmsgbuffer += 3;
		}
		*pmsgbuffer = 0x0;
		pmsgbuffer = (char *)&msgbuffer[0];
		AICE_USBCMMD_MSG("%s", msgbuffer);
	}
	return usb_bulk_with_retries(&wrap_usb_bulk_write,
			dev, ep, bytes, size, timeout);
}

static inline int usb_bulk_read_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	int result;
	char zero_buffer[512];

	result = usb_bulk_with_retries(&jtag_libusb_bulk_read,
			dev, ep, bytes, size, timeout);

	if ((size % aice_usb_rx_max_packet) == 0) {
		AICE_USBCMMD_MSG("usb_bulk_read_ex: zero packet!!\n");
		jtag_libusb_bulk_read(dev, ep, &zero_buffer[0], aice_usb_rx_max_packet, timeout);
	}
	if (log_usb_packets == 1) {
		unsigned int i;
		char msgbuffer[4096] = {0};
		char *pmsgbuffer = (char *)&msgbuffer[0];
		sprintf ( pmsgbuffer, "bulk_in: size=0x%02x,", size);
		pmsgbuffer += strlen(pmsgbuffer);
		if (size > 128)
			size = 128;
		for (i=0; i<(unsigned int)size; i++) {
			sprintf ( pmsgbuffer, " %02x", (unsigned char)bytes[i]);
			pmsgbuffer += 3;
		}
		*pmsgbuffer = 0x0;
		pmsgbuffer = (char *)&msgbuffer[0];
		AICE_USBCMMD_MSG("%s", msgbuffer);
	}
	return result;
}

/* Write data from out_buffer to USB. */
int aice_usb_write(unsigned char *out_buffer, unsigned int out_length)
{
	int result;

	if (out_length > AICE_OUT_BUFFER_SIZE) {
		AICE_USBCMMD_MSG("aice_write illegal out_length=%d (max=%d)",
				out_length, AICE_OUT_BUFFER_SIZE);
		return -1;
	}
	/* for AICE-mini zero-packet issue */
	assert( (out_length % aice_usb_tx_max_packet) != 0);
	result = usb_bulk_write_ex(aice_usb_handle, aice_usb_write_ep,
			(char *)out_buffer, out_length, aice_set_usb_timeout);

	return result;
}

/* Read data from USB into in_buffer. */
int aice_usb_read(unsigned char *in_buffer, unsigned int expected_size)
{
	int result = usb_bulk_read_ex(aice_usb_handle, aice_usb_read_ep,
			(char *)in_buffer, expected_size, aice_set_usb_timeout);

	return result;
}

int aice_scan_chain(unsigned int *id_codes, unsigned char *num_of_ids)
{
	int result;
	struct aice_usb_cmmd_info *pusb_rx_cmmd_info = &usb_cmmd_unpack_info;

	result = aice_access_cmmd(AICE_CMDIDX_SCAN_CHAIN, 0, 0, (unsigned char *)id_codes, 16);
	if (result != ERROR_OK)
		return result;
	*num_of_ids = pusb_rx_cmmd_info->length;
	return ERROR_OK;
}

// also define in nds32_edm.h
#define NDS_EDM_SR_EDMSW  0x30
#define NDS_EDMSW_WDV		(1 << 0)
#define NDS_EDMSW_RDV		(1 << 1)

int aice_read_dtr_to_buffer(uint32_t target_id, uint32_t buffer_idx)
{
	unsigned int dtr_data = 0;
	return aice_access_cmmd(AICE_CMDIDX_READ_DTR_TO_BUFFER, target_id, buffer_idx, (unsigned char *)&dtr_data, 1);
}

int aice_write_dtr_from_buffer(uint32_t target_id, uint32_t buffer_idx)
{
	unsigned int dtr_data = 0;
	return aice_access_cmmd(AICE_CMDIDX_WRITE_DTR_FROM_BUFFER, target_id, buffer_idx, (unsigned char *)&dtr_data, 1);
}

int aice_batch_buffer_write(uint32_t buf_index)
{
	int result;
	//enum aice_command_mode aice_command_mode_ori = aice_command_mode;

	unsigned char *pWriteData = (unsigned char *)&usb_out_packets_buffer[0];
	unsigned int num_of_words = ((usb_out_packets_buffer_length + 3) / 4);
	aice_command_mode = AICE_COMMAND_MODE_NORMAL;
	usb_cmmd_pack_info.access_little_endian = 0;  // AICE_BIG_ENDIAN
	result = aice_access_cmmd(AICE_CMDIDX_BATCH_BUFFER_WRITE, 0, buf_index, (unsigned char *)pWriteData, num_of_words);
	usb_cmmd_pack_info.access_little_endian = 1;
	//aice_command_mode = aice_command_mode_ori;
	usb_out_packets_buffer_length = 0;
	usb_in_packets_buffer_length = 0;
	return result;
}

int aice_batch_buffer_read(uint32_t buf_index, unsigned char *pReadData, uint32_t num_of_words)
{
	int result;
	enum aice_command_mode aice_command_mode_ori = aice_command_mode;
	aice_command_mode = AICE_COMMAND_MODE_NORMAL;
	result = aice_access_cmmd(AICE_CMDIDX_BATCH_BUFFER_READ, 0, buf_index, (unsigned char *)pReadData, num_of_words);
	aice_command_mode = aice_command_mode_ori;
	return result;
}

int aice_pack_buffer_read(unsigned char *pReadData, uint32_t num_of_bytes)
{
	unsigned int i;

	for (i = 0 ; i < num_of_bytes ; i++) {
		*pReadData++ = usb_in_packets_buffer[i];
	}

	return ERROR_OK;
}

int aice_usb_reset_box(void)
{
	// AICE-MCU clear timeout is enough for reboot, not necessary others.
	if (aice_usb_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	/* turn off FASTMODE for AICE-MCU 1.3.2 - */
	unsigned int pin_status = 0;
	if (aice_usb_read_ctrl(AICE_READ_CTRL_GET_JTAG_PIN_STATUS, &pin_status)
			!= ERROR_OK)
		return ERROR_FAIL;

	if (aice_usb_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status & (~0x2))
			!= ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_usb_write_pins(unsigned int num_of_words, unsigned int *pWriteData)
{
	int result;

	usb_cmmd_pack_info.access_little_endian = 0;  // AICE_BIG_ENDIAN
	result = aice_access_cmmd(AICE_CMDIDX_WRITE_PINS, 0, 0, (unsigned char *)pWriteData, num_of_words);
	usb_cmmd_pack_info.access_little_endian = 1;
	return result;
}

int aice_usb_write_ctrl(uint32_t address, uint32_t WriteData)
{
	unsigned int ctrl_data = WriteData;
	int result;
	aice_print_info(AICE_WRITE_CTRL, address, (unsigned int *)&ctrl_data, 0, 0);
	result = aice_access_cmmd(AICE_CMDIDX_WRITE_CTRL, 0, address, (unsigned char *)&ctrl_data, 1);
	return result;
}

int aice_usb_read_ctrl(uint32_t address, uint32_t *pReadData)
{
	int result = ERROR_FAIL;
	result = aice_access_cmmd(AICE_CMDIDX_READ_CTRL, 0, address, (unsigned char *)pReadData, 1);
	if (result == ERROR_OK) {
		aice_print_info(AICE_READ_CTRL, address, (unsigned int *)pReadData, 0, 0);
	}
	return result;
}

int aice_usb_read_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	int result = ERROR_FAIL;
	uint32_t bak_addr = address;

	if ( (JDPInst & 0x80) != 0 ) {
		LOG_ERROR( "AICE Read EDM JDPInst Error: does not Read inst, inst code: 0x%02X", JDPInst );
		return ERROR_FAIL;
	}

	switch ( JDPInst ) {
		case JDP_R_DBG_SR:
			result = aice_access_cmmd(AICE_CMDIDX_READ_EDMSR, target_id, address, (unsigned char *)EDMData, 1);
			break;
		case JDP_R_DTR:
			result = aice_access_cmmd(AICE_CMDIDX_READ_DTR, target_id, 0, (unsigned char *)EDMData, 1);
			break;
		case JDP_R_MEM_W:
			address = ((address >> 2) & 0x3FFFFFFF);
			result = aice_access_cmmd(AICE_CMDIDX_READ_MEM, target_id, address, (unsigned char *)EDMData, 1);
			address = bak_addr;
			break;
		case JDP_R_MISC_REG:
			result = aice_access_cmmd(AICE_CMDIDX_READ_MISC, target_id, address, (unsigned char *)EDMData, 1);
			break;
		case JDP_R_FAST_MEM:
			result = aice_access_cmmd(AICE_CMDIDX_FASTREAD_MEM, target_id, 0, (unsigned char *)EDMData, num_of_words);
			break;
		case JDP_R_MEM_H:
			address = ((address >> 1) & 0x7FFFFFFF);
			result = aice_access_cmmd(AICE_CMDIDX_READ_MEM_H, target_id, address, (unsigned char *)EDMData, 1);
			address = bak_addr;
			break;
		case JDP_R_MEM_B:
			result = aice_access_cmmd(AICE_CMDIDX_READ_MEM_B, target_id, address, (unsigned char *)EDMData, 1);
			break;
		case JDP_R_DIM:
		case JDP_R_DBG_EVENT:
		case JDP_R_IDCODE:
		default:
			LOG_ERROR( "AICE Read EDM JDPInst Error: inst error, inst code: 0x%02X", JDPInst );
			return ERROR_FAIL;
	};
	if (result == ERROR_OK) {
		aice_print_info(AICE_READ_EDM, address, (unsigned int *)EDMData, target_id, JDPInst);
	}
	return result;
}

int aice_usb_write_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	int result = 0;
	unsigned int write_data  = 0;

	//    if( num_of_words == 1 )
	//        LOG_DEBUG("AICE Write EDM USB: target_id=0x%08X, cmd=0x%02X, addr=0x%08X, date=0x%08X", target_id, JDPInst, address, *EDMData);
	//    else
	//        LOG_DEBUG("AICE Write EDM USB: target_id=0x%08X, cmd=0x%02X, addr=0x%08X", target_id, JDPInst, address);

	if ( (JDPInst & 0x80) != 0x80 ) {
		LOG_ERROR( "AICE Write EDM JDPInst Error: does not Write inst, inst code: 0x%02X", JDPInst );
		return ERROR_FAIL;
	}
	aice_print_info(AICE_WRITE_EDM, address, (unsigned int *)EDMData, target_id, JDPInst);

	switch ( JDPInst ) {
		case JDP_W_DIM:
			usb_cmmd_pack_info.access_little_endian = 0;  // AICE_BIG_ENDIAN
			result = aice_access_cmmd(AICE_CMDIDX_WRITE_DIM, target_id, 0,
		                            (unsigned char *)EDMData, num_of_words);
			usb_cmmd_pack_info.access_little_endian = 1;
			return result;

		case JDP_W_DBG_SR:
			return aice_access_cmmd(AICE_CMDIDX_WRITE_EDMSR, target_id, address, (unsigned char *)EDMData, 1);

		case JDP_W_DTR:
			return aice_access_cmmd(AICE_CMDIDX_WRITE_DTR, target_id, 0, (unsigned char *)EDMData, 1);

		case JDP_W_MEM_W:
			address = ((address >> 2) & 0x3FFFFFFF);
			return aice_access_cmmd(AICE_CMDIDX_WRITE_MEM, target_id, address, (unsigned char *)EDMData, 1);

		case JDP_W_MISC_REG:
			return aice_access_cmmd(AICE_CMDIDX_WRITE_MISC, target_id, address, (unsigned char *)EDMData, 1);

		case JDP_W_FAST_MEM:
			return aice_access_cmmd(AICE_CMDIDX_FASTWRITE_MEM, target_id, 0, (unsigned char *)EDMData, num_of_words);

		case JDP_W_EXECUTE:
			return aice_access_cmmd(AICE_CMDIDX_EXECUTE, target_id, 0, (unsigned char *)&write_data, 1);

		case JDP_W_MEM_H:
			write_data = (*EDMData & 0x0000FFFF);
			address = ((address >> 1) & 0x7FFFFFFF);
			return aice_access_cmmd(AICE_CMDIDX_WRITE_MEM_H, target_id, address, (unsigned char *)&write_data, 1);

		case JDP_W_MEM_B:
			write_data = (*EDMData & 0x000000FF);
			return aice_access_cmmd(AICE_CMDIDX_WRITE_MEM_B, target_id, address, (unsigned char *)&write_data, 1);

		default:
			LOG_ERROR( "AICE Write EDM JDPInst Error: inst error, inst code: 0x%02X", JDPInst );
			return ERROR_FAIL;
	};
	return ERROR_FAIL;
}

static int aice_usb_packet_flush(void)
{
	if (usb_out_packets_buffer_length == 0)
		return 0;

	if (AICE_COMMAND_MODE_PACK == aice_command_mode) {
		AICE_USBCMMD_MSG("Flush usb packets (AICE_COMMAND_MODE_PACK)");

		if (aice_usb_write(usb_out_packets_buffer, usb_out_packets_buffer_length) < 0) {
			AICE_USBCMMD_MSG("Flush FAIL: aice_usb_write");
			return ERROR_FAIL;
		}
		if (aice_usb_read(&usb_in_packets_buffer[aice_keep_usb_in_curr_packets_length], usb_in_packets_buffer_length) < 0) {
			AICE_USBCMMD_MSG("Flush FAIL: aice_usb_read");
			return ERROR_FAIL;
		}
		if (aice_keep_usb_packet_append == 1) {
			AICE_USBCMMD_MSG("aice_keep_usb_in_curr_packets_length: 0x%x", aice_keep_usb_in_curr_packets_length);
			AICE_USBCMMD_MSG("usb_in_packets_buffer_length: 0x%x", usb_in_packets_buffer_length);
			aice_keep_usb_in_curr_packets_length += usb_in_packets_buffer_length;
		}

		usb_out_packets_buffer_length = 0;
		usb_in_packets_buffer_length = 0;
	} else if (AICE_COMMAND_MODE_BATCH == aice_command_mode) {
		AICE_USBCMMD_MSG("Flush usb packets (AICE_COMMAND_MODE_BATCH)");

		/* use BATCH_BUFFER_WRITE to fill command-batch-buffer */
		if (aice_batch_buffer_write(AICE_BATCH_COMMAND_BUFFER_0) != ERROR_OK)
			return ERROR_FAIL;

		usb_out_packets_buffer_length = 0;
		usb_in_packets_buffer_length = 0;

		/* enable BATCH command */
		aice_command_mode = AICE_COMMAND_MODE_NORMAL;
		if (aice_usb_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0x80000000) != ERROR_OK)
			return ERROR_FAIL;
		aice_command_mode = AICE_COMMAND_MODE_BATCH;

		/* wait 1 second (AICE bug, workaround) */
		alive_sleep(1000);

		/* check status */
		unsigned int i;
		unsigned int batch_status;

		i = 0;
		while (1) {
			aice_usb_read_ctrl(AICE_READ_CTRL_BATCH_STATUS, &batch_status);

			if (batch_status & 0x1)
				return ERROR_OK;
			else if (batch_status & 0xE)
				return ERROR_FAIL;

			if ((i % 30) == 0) {
				keep_alive();
			}
			i++;
		}
	}
	return ERROR_OK;
}

static int aice_usb_packet_append(unsigned char *out_buffer, unsigned int out_length, unsigned int in_length)
{
	/* for AICE-mini zero-packet issue */
	unsigned int check_packet_size, hit_max_packet_size=0, hit_zero_packet_issue=0;

	check_packet_size = ((usb_out_packets_buffer_length + 3) & 0xFFFFFFFC);
	check_packet_size += out_length;

	if (AICE_COMMAND_MODE_PACK == aice_command_mode) {
		if (check_packet_size > AICE_OUT_PACK_COMMAND_SIZE)
			hit_max_packet_size = 1;
		else if ( (check_packet_size % aice_usb_tx_max_packet) == 0)
			hit_zero_packet_issue = 1;
	} else if (AICE_COMMAND_MODE_BATCH == aice_command_mode) {
		if (check_packet_size > AICE_OUT_BATCH_COMMAND_SIZE)
			hit_max_packet_size = 1;
		else if ( ((check_packet_size + 4) % aice_usb_tx_max_packet) == 0)
			hit_zero_packet_issue = 1;
	} else {
		/* AICE_COMMAND_MODE_NORMAL */
		if (aice_usb_packet_flush() != ERROR_OK)
			return ERROR_FAIL;
	}

	if ((hit_max_packet_size) || (hit_zero_packet_issue)) {
		AICE_USBCMMD_MSG("check=0x%x, flush=0x%x", check_packet_size, usb_out_packets_buffer_length);
		aice_keep_usb_packet_append = 1;
		if (aice_usb_packet_flush() != ERROR_OK) {
			AICE_USBCMMD_MSG("Flush usb packets failed");
			return ERROR_FAIL;
		}
	}
	//AICE_USBCMMD_MSG("Append usb packets 0x%02x", out_buffer[0]);

	memcpy(usb_out_packets_buffer + usb_out_packets_buffer_length, out_buffer, out_length);
	usb_out_packets_buffer_length += out_length;
	usb_in_packets_buffer_length += in_length;
	return ERROR_OK;
}

int aice_usb_set_command_mode(enum aice_command_mode command_mode)
{
	int retval = ERROR_OK;

	/* flush usb_packets_buffer as users change mode */
	retval = aice_usb_packet_flush();

	if (AICE_COMMAND_MODE_BATCH == command_mode) {
		/* reset batch buffer */
		aice_command_mode = AICE_COMMAND_MODE_NORMAL;
		retval = aice_usb_write_ctrl(AICE_WRITE_CTRL_BATCH_CMD_BUF0_CTRL, 0x40000);
	}

	aice_command_mode = command_mode;

	return retval;
}

int aice_clear_timeout(void)
{
	int result = 0;
	unsigned char tmp_usbbuf[8];
	tmp_usbbuf[0] = AICE_CMD_WRITE_CTRL;
	tmp_usbbuf[1] = 0x00;
	tmp_usbbuf[2] = AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS;
	tmp_usbbuf[3] = 0x00;
	tmp_usbbuf[4] = 0x00;
	tmp_usbbuf[5] = 0x00;
	tmp_usbbuf[6] = 0x01;
	// Host to device, send usb packet to device
	aice_usb_write(&tmp_usbbuf[0], 7);
	// Device to host, receive usb packet from device
	result = aice_usb_read(&tmp_usbbuf[0], 2);
	if (result < 0)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int aice_access_cmmd(unsigned char cmdidx, unsigned char target_id,
		unsigned int address, unsigned char *pdata, unsigned int length)
{
	struct aice_usb_cmmd_info *pusb_tx_cmmd_info = &usb_cmmd_pack_info;
	struct aice_usb_cmmd_info *pusb_rx_cmmd_info = &usb_cmmd_unpack_info;
	int result;
	struct aice_usb_cmmd_attr *pusb_cmmd_attr = &usb_all_cmmd_attr[cmdidx];
	unsigned int h2d_size, d2h_size, retry_times = 0;
	unsigned char cmd_ack_code; //extra_length, res_target_id;
	//unsigned int *pWordData = (unsigned int*)pdata;

	h2d_size = aice_get_usb_cmd_size(pusb_cmmd_attr->h2d_type);
	d2h_size = aice_get_usb_cmd_size(pusb_cmmd_attr->d2h_type);
	pusb_tx_cmmd_info->cmdtype = pusb_cmmd_attr->h2d_type;
	pusb_tx_cmmd_info->cmd = pusb_cmmd_attr->cmd;
	pusb_tx_cmmd_info->target = (unsigned char)target_id;
	pusb_tx_cmmd_info->length = (length - 1);
	pusb_tx_cmmd_info->addr = address;
	pusb_tx_cmmd_info->pword_data = (unsigned char *)pdata;
	aice_pack_usb_cmd(pusb_tx_cmmd_info);

	pusb_rx_cmmd_info->cmdtype = pusb_cmmd_attr->d2h_type;
	pusb_rx_cmmd_info->length = (length - 1);
	pusb_rx_cmmd_info->pword_data = (unsigned char *)pdata;

	// read command
	if (cmdidx >= AICE_CMDIDX_SCAN_CHAIN) {
		d2h_size += ((length - 1) * 4);
	}
	// write command
	else {
		h2d_size += ((length - 1) * 4);
	}
	if ((aice_command_mode == AICE_COMMAND_MODE_PACK) ||
			(aice_command_mode == AICE_COMMAND_MODE_BATCH)) {
			result = aice_usb_packet_append(pusb_tx_cmmd_info->pusb_buffer,
							h2d_size, d2h_size);
			//AICE_USBCMMD_MSG("%s(pack), COREID: %d, address: 0x%x, data: 0x%x",
			//	pusb_cmmd_attr->cmdname, target_id, address, (unsigned int)*pWordData);
			return result;
	}
	do {
		// Host to device, send usb packet to device
		aice_usb_write(pusb_tx_cmmd_info->pusb_buffer, h2d_size);

		// Device to host, receive usb packet from device
		result = aice_usb_read(pusb_rx_cmmd_info->pusb_buffer, d2h_size);
		if ((result < 0) ||
			  ((result != (int)d2h_size) &&
			  (cmdidx != AICE_CMDIDX_SCAN_CHAIN))) {  // scan_chain received data maybe < d2h_size
				AICE_USBCMMD_MSG("%s, aice_usb_read failed (requested=%d, result=%d)", pusb_cmmd_attr->cmdname, d2h_size, result);
				return ERROR_FAIL;
		}
		aice_pack_usb_cmd(pusb_rx_cmmd_info);
		cmd_ack_code = pusb_rx_cmmd_info->cmd;

		//AICE_USBCMMD_MSG("%s, COREID: %d, address: 0x%x, data: 0x%x",
		//	pusb_cmmd_attr->cmdname, target_id, address, (unsigned int)*pWordData);

		if (cmd_ack_code == pusb_cmmd_attr->cmd) {
			//AICE_USBCMMD_MSG("%s response", pusb_cmmd_attr->cmdname);
			break;
		} else {
			if (retry_times > aice_max_retry_times) {
				AICE_USBCMMD_MSG("aice command timeout (command=0x%x, response=0x%x)",
					pusb_cmmd_attr->cmd, cmd_ack_code);

				return ERROR_FAIL;
			}

			/* clear timeout and retry */
			if (aice_clear_timeout() != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

#define LINE_BUFFER_SIZE 1024
enum AICE_CUSTOM_CMD {
	AICE_CUSTOM_CMD_SET_SRST = 0,
	AICE_CUSTOM_CMD_CLEAR_SRST,
	AICE_CUSTOM_CMD_SET_DBGI,
	AICE_CUSTOM_CMD_CLEAR_DBGI,
	AICE_CUSTOM_CMD_SET_TRST,
	AICE_CUSTOM_CMD_CLEAR_TRST,
	AICE_CUSTOM_CMD_DELAY,
	AICE_CUSTOM_CMD_WRITE_PINS,
	AICE_CUSTOM_CMD_T_WRITE_MISC,
	AICE_CUSTOM_CMD_WRITE_CTRL,
	AICE_CUSTOM_CMD_SCAN_CHAIN,
	AICE_CUSTOM_CMD_TCK_SCAN,
	AICE_CUSTOM_CMD_T_WRITE_EDMSR,
	AICE_CUSTOM_CMD_T_WRITE_EDMREG,
	AICE_CUSTOM_CMD_T_READ_EDMREG,
	AICE_CUSTOM_CMD_READ_CTRL,
	AICE_CUSTOM_CMD_SET_CURR_TARGET,
	AICE_CUSTOM_CMD_MAX,
};

static char *custom_script_cmmd[AICE_CUSTOM_CMD_MAX]={
	"set srst",
	"clear srst",
	"set dbgi",
	"clear dbgi",
	"set trst",
	"clear trst",
	"delay",
	"write_pins",
	"t_write_misc",
	"write_ctrl",
	"scan_chain",
	"tck_scan",
	"t_write_edmsr",
	"t_write_edmreg",
	"t_read_edmreg",
	"read_ctrl",
	"set_current_target",
};

int aice_usb_execute_custom_script(struct target *target, const char *script)
{
	FILE *script_fd;
	uint32_t word_buffer[LINE_BUFFER_SIZE/4];
	char tmp_buffer[LINE_BUFFER_SIZE];
	char *line_buffer = (char *)&word_buffer[0];
	char *curr_str, *compare_str;
	uint32_t delay, i, num_of_words;
	uint32_t Nibble1 = 0, Nibble2 = 0, write_pins_num = 0;
	uint32_t write_ctrl_value = 0, idx = 0;
	uint32_t target_id = 0, argv[5];
	uint32_t write_ctrl_addr, write_ctrl_data;
	uint32_t write_misc_addr, write_misc_data;
	uint32_t write_edmsr_addr, write_edmsr_data;
	uint32_t write_edm_addr, write_edm_data, write_edm_reg;
	uint32_t read_ctrl_addr, read_ctrl_exp_data, read_ctrl_mask;
	uint32_t read_edm_addr, read_edm_exp_data, read_edm_mask, read_edm_reg;
	long long then = 0;
	int result = ERROR_OK;
	struct command_context *cmd_ctx = global_cmd_ctx;
	uint32_t skip_execute_cmd = 0;
	char *cur_target_name;

	script_fd = fopen(script, "r");
	if (script_fd == NULL) {
		NDS32_LOG("custom_script open file fail: %s", script);
		return ERROR_FAIL;
	}
	while (fgets(line_buffer, LINE_BUFFER_SIZE, script_fd) != NULL) {
		if ((line_buffer[0] == '#') || (line_buffer[0] == '\r'))
			continue;

		for (i = 0; i < AICE_CUSTOM_CMD_MAX; i ++) {
			compare_str = custom_script_cmmd[i];
			curr_str = strstr(line_buffer, compare_str);
			if (curr_str != NULL)
				break;
		}
		if (skip_execute_cmd)
			goto aice_execute_custom_script_skip_cmd;

		if (i < AICE_CUSTOM_CMD_MAX) {
			LOG_DEBUG("custom_script %s", curr_str);
		}
		if (i <= AICE_CUSTOM_CMD_DELAY) {
			result = sscanf(curr_str + strlen(compare_str), " %d", &delay);
			if (result != 1) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x", &delay);
				if (result != 1)
					LOG_ERROR("expected exactly one argument");
			}
			if (i == AICE_CUSTOM_CMD_DELAY) {
				alive_sleep(delay);
			} else {
			if (i == AICE_CUSTOM_CMD_SET_SRST)
				write_ctrl_value = AICE_CUSTOM_DELAY_SET_SRST;
			else if (i == AICE_CUSTOM_CMD_CLEAR_SRST)
				write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_SRST;
			else if (i == AICE_CUSTOM_CMD_SET_DBGI)
				write_ctrl_value = AICE_CUSTOM_DELAY_SET_DBGI;
			else if (i == AICE_CUSTOM_CMD_CLEAR_DBGI)
				write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_DBGI;
			else if (i == AICE_CUSTOM_CMD_SET_TRST)
				write_ctrl_value = AICE_CUSTOM_DELAY_SET_TRST;
			else if (i == AICE_CUSTOM_CMD_CLEAR_TRST)
				write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_TRST;
				

			write_ctrl_value |= (1 << 16);
			LOG_DEBUG("custom_script aice_write_ctrl = 0x%x, delay = 0x%x", write_ctrl_value, delay);
			result = aice_write_ctrl(AICE_WRITE_CTRL_CUSTOM_DELAY, write_ctrl_value);
			if (result != ERROR_OK)
				goto aice_execute_custom_script_error;
			alive_sleep(delay);
		}
		}
		else if ((i == AICE_CUSTOM_CMD_WRITE_PINS) && (aice_port->type == AICE_PORT_FTDI)) {
			LOG_DEBUG("AICE_CUSTOM_CMD_WRITE_PINS: %s", curr_str);
			command_run_line(cmd_ctx, curr_str);
		}
		else if (i == AICE_CUSTOM_CMD_WRITE_PINS) {
			sscanf(curr_str + strlen(compare_str), " %s", &tmp_buffer[0]);
			write_pins_num = strlen(&tmp_buffer[0]);
			LOG_DEBUG("custom_script write_pins, %d %s", write_pins_num, &tmp_buffer[0]);
			idx = 0;
			for (i = 0; i < ((write_pins_num + 1) >> 1); i ++) {
				Nibble1 = 0;
				Nibble2 = 0;
				sscanf(&tmp_buffer[idx++], "%01x", &Nibble1);
				sscanf(&tmp_buffer[idx++], "%01x", &Nibble2);
				LOG_DEBUG("custom_script write_pins, %x %x", Nibble1, Nibble2);
				line_buffer[2 + i] = (char)(Nibble1 | (Nibble2 << 4));
			}
			num_of_words = 2 + ((write_pins_num + 1) >> 1);
			num_of_words = (num_of_words + 3) >> 2;
			line_buffer[0] = (char)(write_pins_num >> 8);
			line_buffer[1] = (char)(write_pins_num & 0xFF);

			LOG_DEBUG("aice_write_pins_support = %x", aice_write_pins_support);

			if (aice_write_pins_support == 0) {
				LOG_DEBUG("write_pins unsupported !!");
				goto aice_execute_custom_script_error;
			}
			result = aice_usb_write_pins(num_of_words, &word_buffer[0]);
			if (result != ERROR_OK)
				goto aice_execute_custom_script_error;
		}
		else if ((i == AICE_CUSTOM_CMD_T_WRITE_MISC) || (i == AICE_CUSTOM_CMD_T_WRITE_EDMSR)) {
			result = sscanf(curr_str + strlen(compare_str), " %d %d %d", &argv[0], &argv[1], &argv[2]);
			if (result != 3) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x 0x%x 0x%x", &argv[0], &argv[1], &argv[2]);
				if (result != 3)
					LOG_ERROR("expected exactly 3 argument");
			}
			if (result == 3) {
				if (i == AICE_CUSTOM_CMD_T_WRITE_MISC) {
					target_id = argv[0];
					write_misc_addr = argv[1];
					write_misc_data = argv[2];
					LOG_DEBUG("custom_script aice_write_misc, 0x%x, 0x%x, 0x%x", target_id, write_misc_addr, write_misc_data);
					result = aice_write_edm_by_coreid(target_id, JDP_W_MISC_REG, write_misc_addr, (uint32_t*)&write_misc_data, 1);
					if (result != ERROR_OK)
						goto aice_execute_custom_script_error;
				} else if (i == AICE_CUSTOM_CMD_T_WRITE_EDMSR) {
					target_id = argv[0];
					write_edmsr_addr = argv[1];
					write_edmsr_data = argv[2];
					LOG_DEBUG("custom_script aice_write_edmsr, 0x%x, 0x%x, 0x%x", target_id, write_edmsr_addr, write_edmsr_data);
					result = aice_write_edm_by_coreid(target_id, JDP_W_DBG_SR, write_edmsr_addr, (uint32_t*)&write_edmsr_data, 1);
					if (result != ERROR_OK)
						goto aice_execute_custom_script_error;
				}
			}
		}
		else if (i == AICE_CUSTOM_CMD_WRITE_CTRL) {
			result = sscanf(curr_str + strlen(compare_str), " %d %d", &argv[0], &argv[1]);
			if (result != 2) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x 0x%x", &argv[0], &argv[1]);
				if (result != 2)
					LOG_ERROR("expected exactly 2 argument");
			}
			if (result == 2) {
				write_ctrl_addr = argv[0];
				write_ctrl_data = argv[1];
				LOG_DEBUG("custom_script aice_write_ctrl, 0x%x, 0x%x", write_ctrl_addr, write_ctrl_data);
				result = aice_write_ctrl(write_ctrl_addr, write_ctrl_data);
				if (result != ERROR_OK)
					goto aice_execute_custom_script_error;
			}
		}
		else if (i == AICE_CUSTOM_CMD_T_WRITE_EDMREG) {
			result = sscanf(curr_str + strlen(compare_str), " %d %d %d %d", &argv[0], &argv[1], &argv[2], &argv[3]);
			if (result != 4) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x 0x%x 0x%x 0x%x", &argv[0], &argv[1], &argv[2], &argv[3]);
				if (result != 4)
					LOG_ERROR("expected exactly 4 argument");
			}
			if (result == 4) {
				target_id = argv[0];
				write_edm_reg = argv[1];
				write_edm_addr = argv[2];
				write_edm_data = argv[3];
				LOG_DEBUG("custom_script aice_write_edmreg, 0x%x, 0x%x, 0x%x", write_edm_reg, write_edm_addr, write_edm_data);
				result = aice_write_edm_by_coreid(target_id, write_edm_reg|JDP_WRITE, write_edm_addr, (uint32_t*)&write_edm_data, 1);
				if (result != ERROR_OK)
					goto aice_execute_custom_script_error;
			}
		}
		else if (i == AICE_CUSTOM_CMD_T_READ_EDMREG) {
			result = sscanf(curr_str + strlen(compare_str), " %d %d %d %d %d", &argv[0], &argv[1], &argv[2], &argv[3], &argv[4]);
			if (result != 5) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x 0x%x 0x%x 0x%x 0x%x", &argv[0], &argv[1], &argv[2], &argv[3], &argv[4]);
				if (result != 5)
					LOG_ERROR("expected exactly 5 argument");
			}
			if (result == 5) {
				target_id = argv[0];
				read_edm_reg = argv[1];
				read_edm_addr = argv[2];
				read_edm_mask = argv[3];
				read_edm_exp_data = argv[4];
				LOG_DEBUG("custom_script aice_read_edm, 0x%x, 0x%x, 0x%x, 0x%x", read_edm_reg, read_edm_addr, read_edm_mask, read_edm_exp_data);
				uint32_t get_edm_data=0;
				i = 0;
				while (1) {
					aice_read_edm_by_coreid(target_id, read_edm_reg, read_edm_addr, (uint32_t*)&get_edm_data, 1);
					if ((get_edm_data & read_edm_mask) == read_edm_exp_data) {
						break;
					}
					if (i == 0)
						then = timeval_ms();
					else {
						if ((timeval_ms() - then) > aice_count_to_check_dbger)
							break;
					}
					alive_sleep(1);
					i++;
				}
			}
		}
		else if (i == AICE_CUSTOM_CMD_READ_CTRL) {
			result = sscanf(curr_str + strlen(compare_str), " %d %d %d", &argv[0], &argv[1], &argv[2]);
			if (result != 3) {
				result = sscanf(curr_str + strlen(compare_str), " 0x%x 0x%x 0x%x", &argv[0], &argv[1], &argv[2]);
				if (result != 3)
					LOG_ERROR("expected exactly 3 argument");
			}
			if (result == 3) {
				read_ctrl_addr = argv[0];
				read_ctrl_mask = argv[1];
				read_ctrl_exp_data = argv[2];
				LOG_DEBUG("custom_script aice_read_ctrl, 0x%x, 0x%x, 0x%x", read_ctrl_addr, read_ctrl_mask, read_ctrl_exp_data);
				uint32_t get_ctrl_data=0;
				i = 0;
				while (1) {
					aice_usb_read_ctrl(read_ctrl_addr, &get_ctrl_data);
					if ((get_ctrl_data & read_ctrl_mask) == read_ctrl_exp_data) {
						break;
					}
					if (i == 0)
						then = timeval_ms();
					else {
						if ((timeval_ms() - then) > aice_count_to_check_dbger)
							break;
					}
					alive_sleep(1);
					i++;
				}
			}
		}
		else if (i == AICE_CUSTOM_CMD_SCAN_CHAIN) {
			unsigned int id_codes[16];
			aice_access_cmmd(AICE_CMDIDX_SCAN_CHAIN, 0, 0, (unsigned char *)&id_codes[0], 16);
			LOG_DEBUG("custom_script scan_chain, id_codes = %x", id_codes[0]);
			//alive_sleep(100);
		}
		else if (i == AICE_CUSTOM_CMD_TCK_SCAN) {
			aice_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL, AICE_TCK_CONTROL_TCK_SCAN);
			LOG_DEBUG("custom_script tck_scan");
			//alive_sleep(100);
		}
		else if (i >= AICE_CUSTOM_CMD_MAX) {
			//NDS32_LOG("custom_script unknown command: %s", line_buffer);
			LOG_DEBUG("cmd: %s", line_buffer);
			command_run_line(cmd_ctx, line_buffer);
		}

aice_execute_custom_script_skip_cmd:
		if (i == AICE_CUSTOM_CMD_SET_CURR_TARGET) {
			result = sscanf(curr_str + strlen(compare_str), " %s", &tmp_buffer[0]);
			cur_target_name = (char *)&tmp_buffer[0];
			if (result == 1) {
				if (strcmp(target_name(target), cur_target_name) == 0) {
					skip_execute_cmd = 0;
				} else {
					skip_execute_cmd = 1;
				}
				LOG_DEBUG("cur_target_name: %s, %s, skip_exe: %d", cur_target_name, target_name(target), skip_execute_cmd);
			}
		}
	}
	fclose(script_fd);
	return ERROR_OK;

aice_execute_custom_script_error:
	NDS32_LOG("<-- Issue custom_script '%s' failed, abandon continue -->", curr_str);
	fclose(script_fd);
	return result;
}

// reset c51 through ep0
// The reset sequence is as following.
// 1. send ep0 packet: bmRequestType 0x40, bRequest 0xA0, wValue 0xe600, index 0x0, byte 1, length 1
// 2. delay several milliseconds
// 3. send ep0 packet: bmRequestType 0x40, bRequest 0xA0, wValue 0xe600, index 0x0, byte 0, length 1
// 4. wait c51 reload FPGA
// The reset sequence can work on AICE 1.6.6 or above.
// Bug 7568 - AICE could not be reset by ICEman
int aice_reset_aice_as_startup(void)
{
	int ret;
	unsigned char reset_bit;
	jtag_libusb_device_handle *dev = aice_usb_handle;

	reset_bit = 1;
	ret = jtag_libusb_control_transfer(dev, 0x40, 0xa0, 0xe600, 0x00, (char *)&reset_bit, 1, 5000);
	alive_sleep(100);
	reset_bit = 0;
	ret = jtag_libusb_control_transfer(dev, 0x40, 0xa0, 0xe600, 0x00, (char *)&reset_bit, 1, 5000);
	alive_sleep(1500);
	/* usb_control_msg() returns the number of bytes transferred during the
	 * DATA stage of the control transfer - must be exactly 1 in this case! */
	if (ret != 1)
		return ERROR_FAIL;
	return ERROR_OK;
}

uint32_t jtag_clock = 16;
uint32_t aice_count_to_check_dbger = 5000;
uint32_t aice_set_usb_timeout = AICE_USB_TIMEOUT;
extern int aice_efreq_support;
extern unsigned int aice_efreq_value;
int aice_usb_set_clock(uint32_t set_clock)
{
	// had set_clock before
	//if ((jtag_clock != 16) && (jtag_clock == (uint32_t)set_clock))
	//	return ERROR_OK;
	unsigned int  read_efreq = 0;
	if( aice_efreq_value != 0 ) {
		if( aice_efreq_support == 0 ) {
			NDS32_LOG("<-- This AICE doesn't support extended TCK frequency range(EFREQ)!! -->");
			return ERROR_FAIL;
		}

		LOG_DEBUG("efreq: %d", aice_efreq_value);
		if(aice_usb_write_ctrl(AICE_WRITE_CTRL_EFREQ, aice_efreq_value) != ERROR_OK) {
			NDS32_LOG("<-- Write extended TCK frequency range failed!! -->");
			return ERROR_FAIL;
		}

		if(aice_usb_read_ctrl(AICE_WRITE_CTRL_EFREQ, &read_efreq) != ERROR_OK)
			return ERROR_FAIL;

		if( read_efreq != aice_efreq_value ) {
			NDS32_LOG("<-- Unspport extended TCK frequency, set: %dHz, get: %dHz -->", aice_efreq_value, read_efreq);
			aice_efreq_value = read_efreq;
			return ERROR_FAIL;
		}

		return ERROR_OK;
	}



	if (set_clock == 16) {
		// Users do NOT specify the jtag clock, use scan-freq
		LOG_DEBUG("Use scan-freq");

		if (aice_usb_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL,
				AICE_TCK_CONTROL_TCK_SCAN) != ERROR_OK)
			goto aice_usb_set_clock_ERR;

		/* Read out TCK_SCAN clock value */
		uint32_t scan_clock=0;
		if (aice_usb_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &scan_clock) != ERROR_OK)
			goto aice_usb_set_clock_ERR;

		set_clock = (scan_clock & 0x0F);
		// If scan-freq = 48MHz, use 24MHz by default
		if (set_clock == 8)
			set_clock = 9;
	}

	if (aice_usb_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL, set_clock) != ERROR_OK)
		goto aice_usb_set_clock_ERR;

	uint32_t check_speed;
	if (aice_usb_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &check_speed) != ERROR_OK)
		goto aice_usb_set_clock_ERR;

	if ((check_speed & 0x20) == 0) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
	}

	if (((int)check_speed & 0x0F) != set_clock) {
		LOG_ERROR("Set jtag clock failed");
		goto aice_usb_set_clock_ERR;
	}
	aice_usb_write_ctrl(AICE_WRITE_CTRL_TIMEOUT, aice_count_to_check_dbger);
//aice_usb_set_clock_OK:
	jtag_clock = set_clock;
	return ERROR_OK;
aice_usb_set_clock_ERR:
	return ERROR_FAIL;
}

int aice_set_write_pins_support(uint32_t if_support)
{
	aice_write_pins_support = if_support;
	return ERROR_OK;
}

int aice_icemem_xread(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr)
{
	struct aice_usb_cmmd_info *pusb_tx_cmmd_info = &usb_cmmd_pack_info;
	struct aice_usb_cmmd_info *pusb_rx_cmmd_info = &usb_cmmd_unpack_info;
	int result;
	unsigned int h2d_size, d2h_size;

	h2d_size = aice_get_usb_cmd_size(AICE_CMDTYPE_HTDXR);
	d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHXR);

	pusb_tx_cmmd_info->cmdtype = AICE_CMDTYPE_HTDXR;
	pusb_tx_cmmd_info->cmd = AICE_CMD_XREAD;
	pusb_tx_cmmd_info->length = (num_of_words);
	pusb_tx_cmmd_info->lo_addr = lo_addr;
	pusb_tx_cmmd_info->hi_addr = hi_addr;
	pusb_tx_cmmd_info->attr = attr;
	//pusb_tx_cmmd_info->pword_data = (unsigned char *)pdata;
	aice_pack_usb_cmd(pusb_tx_cmmd_info);

	pusb_rx_cmmd_info->cmdtype = AICE_CMDTYPE_DTHXR;
	pusb_rx_cmmd_info->length = (num_of_words);
	pusb_rx_cmmd_info->pword_data = (unsigned char *)pGetData;

	// read command
	d2h_size += ((num_of_words - 1) * 4);

	// Host to device, send usb packet to device
	aice_usb_write(pusb_tx_cmmd_info->pusb_buffer, h2d_size);

	// Device to host, receive usb packet from device
	result = aice_usb_read(pusb_rx_cmmd_info->pusb_buffer, d2h_size);

	aice_pack_usb_cmd(pusb_rx_cmmd_info);
	if (result < 0) {
			AICE_USBCMMD_MSG("XREAD, aice_usb_read failed (requested=%d, result=%d)", d2h_size, result);
			return ERROR_FAIL;
	}

	AICE_USBCMMD_MSG("XREAD, lo_addr: 0x%x, hi_addr: 0x%x, data: 0x%x",
			lo_addr, hi_addr, (unsigned int)*pGetData);
	AICE_USBCMMD_MSG("XREAD response");
	return ERROR_OK;
}

int aice_icemem_xwrite(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr)
{
	struct aice_usb_cmmd_info *pusb_tx_cmmd_info = &usb_cmmd_pack_info;
	struct aice_usb_cmmd_info *pusb_rx_cmmd_info = &usb_cmmd_unpack_info;
	int result;
	unsigned int h2d_size, d2h_size, retry_times = 0;

	h2d_size = aice_get_usb_cmd_size(AICE_CMDTYPE_HTDXW);
	d2h_size = aice_get_usb_cmd_size(AICE_CMDTYPE_DTHXW);

	pusb_tx_cmmd_info->cmdtype = AICE_CMDTYPE_HTDXW;
	pusb_tx_cmmd_info->cmd = AICE_CMD_XWRITE;
	pusb_tx_cmmd_info->length = (num_of_words);
	pusb_tx_cmmd_info->lo_addr = lo_addr;
	pusb_tx_cmmd_info->hi_addr = hi_addr;
	pusb_tx_cmmd_info->attr = attr;
	pusb_tx_cmmd_info->pword_data = (unsigned char *)pSetData;
	aice_pack_usb_cmd(pusb_tx_cmmd_info);

	pusb_rx_cmmd_info->cmdtype = AICE_CMDTYPE_DTHXW;
	pusb_rx_cmmd_info->length = (num_of_words);
	//pusb_rx_cmmd_info->pword_data = (unsigned char *)pSetData;

	// write command
	h2d_size += ((num_of_words - 1) * 4);

	do {
		// Host to device, send usb packet to device
		aice_usb_write(pusb_tx_cmmd_info->pusb_buffer, h2d_size);

		// Device to host, receive usb packet from device
		result = aice_usb_read(pusb_rx_cmmd_info->pusb_buffer, d2h_size);
		if ((result < 0) ||
			  (result != (int)d2h_size) ) { 
				AICE_USBCMMD_MSG("XWRITE, aice_usb_read failed (requested=%d, result=%d)", d2h_size, result);
				return ERROR_FAIL;
		}
		aice_pack_usb_cmd(pusb_rx_cmmd_info);

		AICE_USBCMMD_MSG("XWRITE, lo_addr: 0x%x, hi_addr: 0x%x, data: 0x%x",
			lo_addr, hi_addr, (unsigned int)*pSetData);
		if (pusb_rx_cmmd_info->cmd == AICE_CMD_XWRITE) {
			AICE_USBCMMD_MSG("XWRITE response");
			break;
		} else {
			if (retry_times > aice_max_retry_times) {
				AICE_USBCMMD_MSG("aice command timeout (command=0x%x, response=0x%x)",
					pusb_tx_cmmd_info->cmd, pusb_rx_cmmd_info->cmd);

				return ERROR_FAIL;
			}

			/* clear timeout and retry */
			if (aice_clear_timeout() != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}


