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
#ifndef __NDS32_ACE_H__
#define __NDS32_ACE_H__

#define MAX_COP_COUNT	4
#define ACE_INDEX	MAX_COP_COUNT

typedef struct ACR_info {
    char name[1024];
    unsigned width;
    unsigned num;
} ACR_INFO_T;


int
nds32_ace_init(const char *aceconf,
                   unsigned *acr_type_count, unsigned *total_acr_reg_nums);
uint32_t
nds32_copilot_version(void);

#endif
