/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

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
 * acr_io1 : ACR utility instruction and exists one GPR for din/dout
 * acr_io2 : ACR utility instruction and exists two GPRs for din/dout
 *           (din_high, din_low/dout_high, din_low)
 * acm_io1 : ACM utility instruction and exists one GPR for din/dout
 * acm_io2 : ACM utility instruction and exists two GPRs for din/dout
 *           (din_high, din_low/dout_high, din_low)
 */
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
	UTIL_INSN_T_V5 *code;
} INSN_CODE_T_V5;

int32_t nds32_ace_init_v5(const char *aceconf);
int32_t get_ace_file_name_for_gdb_v5(const char *aceconf, const char *platform, char **name);

#endif
