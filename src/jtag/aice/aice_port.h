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
#ifndef _AICE_PORT_H_
#define _AICE_PORT_H_

#include <target/target.h>
#include <target/nds32.h>
#include "aice_usb_command.h"
#define AICE_MAX_NUM_CORE      (0x10)
#define AICE_MAX_VID_NUM       (0xFF)

#define ERROR_AICE_DISCONNECT  (-200)
#define ERROR_AICE_TIMEOUT     (-201)

enum aice_target_state_s {
	AICE_DISCONNECT = 0,
	AICE_TARGET_DETACH,
	AICE_TARGET_UNKNOWN,
	AICE_TARGET_RUNNING,
	AICE_TARGET_HALTED,
	AICE_TARGET_RESET,
	AICE_TARGET_DEBUG_RUNNING,
};

enum aice_srst_type_s {
	AICE_SRST = 0x1,
	AICE_RESET_HOLD = 0x8,
};

enum aice_target_endian {
	AICE_LITTLE_ENDIAN = 0,
	AICE_BIG_ENDIAN,
};

enum aice_error_s {
	AICE_OK,
	AICE_ACK,
	AICE_ERROR,
};

enum aice_cache_ctl_type {
	AICE_CACHE_CTL_L1D_INVALALL = 0,
	AICE_CACHE_CTL_L1D_VA_INVAL,
	AICE_CACHE_CTL_L1D_WBALL,
	AICE_CACHE_CTL_L1D_VA_WB,
	AICE_CACHE_CTL_L1I_INVALALL,
	AICE_CACHE_CTL_L1I_VA_INVAL,
	AICE_CACHE_CTL_LOOPCACHE_ISYNC,
	AICE_CACHE_CTL_L1I_IX_INVAL,
};

enum aice_profiling_mode {
	AICE_PROFILE_MODE_NORMAL,
	AICE_PROFILE_MODE_INIT,
	AICE_PROFILE_MODE_POSTRUN,
	AICE_PROFILE_MODE_POST,
	AICE_PROFILE_MODE_STATE,
	AICE_PROFILE_MODE_BENCH,
};

struct aice_profiling_info {
	uint32_t profiling_type;
	uint32_t interval;
	uint32_t iteration;
	uint32_t reg_no;
	uint32_t *psamples;
	uint32_t *pnum_samples;
	enum aice_target_state_s *pstate;
	struct command_invocation *bench_cmd;
};

struct aice_port_param_s {
	/** */
	char *device_desc;
	/** */
	char *serial;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** */
	char *adapter_name;
};

struct aice_port_s {
	/** */
	uint32_t coreid;
	/** */
	const struct aice_port *port;
};

struct aice_nds32_api_s {
	/** write ICE box registers */
	int (*write_ctrl)(uint32_t address, uint32_t WriteData);
	/** read ICE box registers */
	int (*read_ctrl)(uint32_t address, uint32_t *pReadData);
	/** AICE read_dtr */
	int (*read_dtr_to_buffer)(uint32_t coreid, uint32_t buffer_idx);
	/** AICE write_dtr */
	int (*write_dtr_from_buffer)(uint32_t coreid, uint32_t buffer_idx);
	/** AICE batch_buffer_write */
	int (*batch_buffer_write)(uint32_t buf_index);
	/** AICE batch_buffer_read */
	int (*batch_buffer_read)(uint32_t buf_index, unsigned char *pReadData, uint32_t num_of_words);
	/** ICE box custom script */
	int (*execute_custom_script)(struct target *target, const char *script);
	/** */
	int (*set_command_mode)(enum aice_command_mode command_mode);
	/** AICE pack_buffer_read */
	int (*pack_buffer_read)(unsigned char *pReadData, uint32_t num_of_bytes);
	/** AICE Monitor command */
	int (*monitor_command)(uint32_t nCmd, char **command, int *len, char **ret_data );
	/** write ICE-Box Peripheral Memory */
	int (*xwrite)(uint32_t lo_addr, uint32_t hi_addr,
	  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr);
	/** read ICE-Box Peripheral Memory */
	int (*xread)(uint32_t lo_addr, uint32_t hi_addr,
	  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr);
};


/** */
struct aice_port_api_s {
	/** */
	int (*open)(struct aice_port_param_s *param);
	/** */
	int (*close)(void);
	/** */
	int (*reset)(void);
	/** */
	int (*idcode)(uint32_t *idcode, uint8_t *num_of_idcode);
	/** */
	int (*set_jtag_clock)(uint32_t a_clock);
	/** */
	int (*assert_srst)(struct target *target, enum aice_srst_type_s srst);
	/** */
	int (*state)(void);
	/** read edm */
	int (*read_edm)(uint32_t coreid, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words );
	/** write edm */
	int (*write_edm)(uint32_t coreid, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words );
	/** */
	int (*profiling)(struct target *target, struct aice_profiling_info *profiling_info);
	/** */
	int (*diagnosis)(struct target *target);
	/** */
	struct aice_nds32_api_s  *pnds32;
};

#define AICE_PORT_UNKNOWN     0
#define AICE_PORT_AICE_USB    1
#define AICE_PORT_AICE_PIPE   2
#define AICE_PORT_FTDI        3
#define AICE_PORT_VENDOR      4

#define AICE_ICE_CONFIG_BATCH_SUPPORT (1 << 31)

/** */
struct aice_port {
	/** */
	char *name;
	/** */
	int type;
	/** */
	struct aice_port_api_s *const api;
};

struct vid_pid_s {
  	/** */
    uint16_t vid;
	/** */
	uint16_t pid;
};

extern const struct aice_port *aice_port_get_list(void);
extern const struct aice_port *aice_port;

#endif
