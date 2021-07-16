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
#ifndef __NDS32_ACE_V5_H__
#define __NDS32_ACE_V5_H__

#define MAX_COP_COUNT	4
#define ACE_INDEX	MAX_COP_COUNT

/* ACR_info records essential information of ACR and SRAM-type ACM */
typedef struct ACR_info_v5 {
    char name[1024];
    unsigned width;
    unsigned num;
} ACR_INFO_T_V5;

/* INSN_TYPE_V5 represents what kind of utility instruction
   acr_io1 : ACR utility instruction and exists one GPR for din/dout
   acr_io2 : ACR utility instruction and exists two GPRs for din/dout
             (din_high, din_low/dout_high, din_low)
   acm_io1 : ACM utility instruction and exists one GPR for din/dout
   acm_io2 : ACM utility instruction and exists two GPRs for din/dout
             (din_high, din_low/dout_high, din_low) */
typedef enum {
  acr_io1 = 0, acr_io2 = 1,
  acm_io1 = 2, acm_io2 = 3
} INSN_TYPE_V5;

typedef struct Util_Insn {
  INSN_TYPE_V5 version;
  unsigned insn;
} UTIL_INSN_T_V5;

typedef struct Insn_Code {
  unsigned num;
  UTIL_INSN_T_V5* code;
} INSN_CODE_T_V5;

int32_t nds32_ace_init_v5(const char *aceconf);
int32_t get_ace_file_name_for_gdb_v5(const char *aceconf, const char *platform, char **name);

#endif
