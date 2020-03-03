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

#include "aice_usb.h"
#include "aice_pipe.h"
#include "aice_port.h"

extern struct aice_port_api_s aice_vendor_api;
extern struct aice_port_api_s nds_ftdi_api;

static const struct aice_port aice_ports[] = {
	{
		.name = "aice_usb",
		.type = AICE_PORT_AICE_USB,
		.api = &aice_usb_api,
	},
	{
		.name = "aice_pipe",
		.type = AICE_PORT_AICE_PIPE,
		.api = &aice_pipe_api,
	},
	{
		.name = "aice_vendor",
		.type = AICE_PORT_VENDOR,
		.api = &aice_vendor_api,
	},
	{
		.name = "nds_ftdi",
		.type = AICE_PORT_FTDI,
		.api = &nds_ftdi_api,
	},
	{.name = NULL, /* END OF TABLE */ },
};

const struct aice_port *aice_port = (const struct aice_port *)&aice_ports[3];

/** */
const struct aice_port *aice_port_get_list(void)
{
	return aice_ports;
}
