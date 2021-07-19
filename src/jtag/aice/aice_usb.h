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
#ifndef __AICE_USB_H__
#define __AICE_USB_H__

#include "aice_port.h"

/* Constants for AICE command WRITE_CTRL:TCK_CONTROL */
#define AICE_TCK_CONTROL_TCK3048		0x08

enum aice_api_s {
    AICE_OPEN = 0x00,
    AICE_CLOSE,
    AICE_RESET,
    AICE_IDCODE,
    AICE_SET_JTAG_CLOCK,
    AICE_WRITE_EDM,
    AICE_READ_EDM,
    AICE_CUSTOM_SCRIPT,
    AICE_CUSTOM_MONITOR_CMD,

    AICE_WRITE_CTRL = 0x10,
    AICE_READ_CTRL,
    AICE_SET_CMD_MODE,
    AICE_WRITE_DTR_FROM_BUFFER,
    AICE_READ_DTR_TO_BUFFER,
    AICE_BATCH_BUFFER_WRITE,
    AICE_BATCH_BUFFER_READ,
    AICE_PACK_BUFFER_READ,
    AICE_XWRITE,
    AICE_XREAD,
};

// API

int aice_usb_assert_srst(struct target *target, enum aice_srst_type_s srst);
int aice_get_state(void);
int aice_usb_profile_entry(struct target *target, struct aice_profiling_info *profiling_info);
int aice_usb_do_diagnosis(struct target *target);

extern struct aice_port_api_s aice_usb_api;

#endif

