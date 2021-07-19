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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <server/gdb_fileio.h>
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_tlb.h"
#include "nds32_disassembler.h"
#include "nds32_log.h"
#include "bytecode-ax.h"

#define ENABLE_DEX_USE_PSW   1
#if ENABLE_DEX_USE_PSW
uint32_t nds32_backup_psw = 0;
uint32_t nds32_use_psw_mode = 0;
#endif

#define USER_DEF_COP_REG   1
#if USER_DEF_COP_REG
extern uint32_t nds32_cop_reg_ena[4];
extern uint32_t nds32_cop_reg_base_id[4];
#endif
extern uint32_t nds32_reg_total_cop_reg_nums(void);
extern uint32_t nds32_reg_total_ace_reg_nums(void);

uint32_t nds32_curr_ir3_value = 0;
const int NDS32_BREAK_16 = 0x00EA;      /* 0xEA00 */
const int NDS32_BREAK_32 = 0x0A000064;  /* 0x6400000A */

int nds32_auto_change_memory_mode(struct target *target, uint32_t address, uint32_t length);
extern int burner_server_init(char *banner);
char *nds32_edm_passcode_init = NULL;
extern uint32_t nds32_security_compat_display;
extern uint32_t nds32_bytecode_parsing;
extern uint32_t aice_usb_pack_command;

const char *nds32_debug_type_name[11] = {
	"SOFTWARE BREAK",
	"SOFTWARE BREAK_16",
	"HARDWARE BREAKPOINT",
	"DATA ADDR WATCHPOINT PRECISE",
	"DATA VALUE WATCHPOINT PRECISE",
	"DATA VALUE WATCHPOINT IMPRECISE",
	"DEBUG INTERRUPT",
	"HARDWARE SINGLE STEP",
	"DATA ADDR WATCHPOINT NEXT PRECISE",
	"DATA VALUE WATCHPOINT NEXT PRECISE",
	"LOAD STORE GLOBAL STOP",
};

static const int NDS32_LM_SIZE_TABLE[16] = {
	4     * 1024,
	8     * 1024,
	16    * 1024,
	32    * 1024,
	64    * 1024,
	128   * 1024,
	256   * 1024,
	512   * 1024,
	1024  * 1024,
	1     * 1024,
	2     * 1024,
	2048  * 1024,
	4096  * 1024,
	8192  * 1024,
	16384 * 1024,
	0,
};

static const int NDS32_LINE_SIZE_TABLE[6] = {
	0,
	8,
	16,
	32,
	64,
	128,
};

static int nds32_get_core_reg(struct reg *reg)
{
	int retval;
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg->valid) {
		LOG_DEBUG("reading register(cached) %i(%s), value: 0x%8.8" PRIx32,
				reg_arch_info->num, reg->name, reg_arch_info->value);
		return ERROR_OK;
	}

	int mapped_regnum = nds32->register_map(nds32, reg_arch_info->num);
	if (reg_arch_info->enable == false) {
		reg_arch_info->value = NDS32_REGISTER_DISABLE;
		retval = ERROR_FAIL;
	} else {
		uint32_t reg_type = nds32_reg_type(mapped_regnum);
		if ((nds32->fpu_enable == false &&
				NDS32_REG_TYPE_FPU == reg_type) ||
			(nds32->audio_enable == false &&
				NDS32_REG_TYPE_AUMR == reg_type) ||
			(nds32->cop_enable[0] == false &&
				NDS32_REG_TYPE_COP0 == reg_type) ||
			(nds32->cop_enable[1] == false &&
				NDS32_REG_TYPE_COP1 == reg_type) ||
			(nds32->cop_enable[2] == false &&
				NDS32_REG_TYPE_COP2 == reg_type) ||
			(nds32->cop_enable[3] == false &&
				NDS32_REG_TYPE_COP3 == reg_type)
				) {

			reg_arch_info->value = 0;
			retval = ERROR_OK;
		} else {
			retval = aice_read_register(target, mapped_regnum, &(reg_arch_info->value));
			LOG_DEBUG("reading register %i(%s), value: 0x%8.8" PRIx32,
					reg_arch_info->num, reg->name, reg_arch_info->value);
		}

	}

	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}


static int nds32_get_core_reg_64(struct reg *reg)
{
  int retval;
  struct nds32_reg *reg_arch_info = reg->arch_info;
  struct target *target = reg_arch_info->target;
  struct nds32 *nds32 = target_to_nds32(target);

  if (target->state != TARGET_HALTED) {
      LOG_ERROR("Target not halted @ %s", __func__);
      return ERROR_TARGET_NOT_HALTED;
  }

  if (reg->valid)
    return ERROR_OK;

  if (reg_arch_info->enable == false) {
      reg_arch_info->value_64 = NDS32_REGISTER_DISABLE;
      retval = ERROR_FAIL;
  } else {
      uint32_t reg_type = nds32_reg_type(reg_arch_info->num);
      if ((nds32->fpu_enable == false &&
           NDS32_REG_TYPE_FPU == reg_type) ||
          (nds32->cop_enable[0] == false &&
           NDS32_REG_TYPE_COP0 == reg_type) ||
          (nds32->cop_enable[1] == false &&
           NDS32_REG_TYPE_COP1 == reg_type) ||
          (nds32->cop_enable[2] == false &&
           NDS32_REG_TYPE_COP2 == reg_type) ||
          (nds32->cop_enable[3] == false &&
           NDS32_REG_TYPE_COP3 == reg_type) 
           ) {

          reg_arch_info->value_64 = 0;
          retval = ERROR_OK;
      } else {
          retval = aice_read_reg_64(target, reg_arch_info->num,
                                    &(reg_arch_info->value_64));
      }
  }

  if (retval == ERROR_OK) {
      reg->valid = true;
      reg->dirty = false;
  }

  return retval;
}

static void nds_ace_enable(struct nds32 *nds32) {                                                             
	uint32_t idr1_value = 0;                                                                                  
	nds32_get_mapped_reg(nds32, IDR1, &idr1_value);                                                           
	if ((idr1_value & 0x100) == 0x100) {                                                                          
		nds32_set_mapped_reg(nds32, IDR1, idr1_value ^ 0x100);                                                
	}                                                                                                         
}   

static int nds32_get_ace_reg(struct reg *reg)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target); 
	uint32_t regnum = reg_arch_info->num;
	int retval;

	if (nds32->cpu_version.ace_enable == 0) {                                                                 
		nds_ace_enable(nds32);                                                                                
		nds32->cpu_version.ace_enable = 1;                                                                    
	} 
	
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg_arch_info->enable == false) {
		return ERROR_FAIL;
	}

	if (reg->valid) {
		LOG_DEBUG("reading register(cached) %i(%s), value: 0x%s",
				reg_arch_info->num, reg->name, reg_arch_info->value_acr);
		return ERROR_OK;
	}

	if (NDS32_REG_TYPE_ACE == nds32_reg_type(regnum)) {
		retval = aice_read_acr(target, regnum, reg_arch_info->value_acr);
		LOG_DEBUG("reading register %i(%s), value: 0x%s",
				regnum, reg->name, reg_arch_info->value_acr);
	} else {
		retval = ERROR_FAIL;
	}


	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}

static int nds32_update_psw(struct nds32 *nds32)
{
	uint32_t value_ir0;

	nds32_get_mapped_reg(nds32, IR0, &value_ir0);

	/* Save data memory endian */
	if ((value_ir0 >> 5) & 0x1) {
		nds32->data_endian = TARGET_BIG_ENDIAN;
	} else {
		nds32->data_endian = TARGET_LITTLE_ENDIAN;
	}

	/* Save translation status */
	nds32->memory.address_translation = ((value_ir0 >> 7) & 0x1) ? true : false;
	if (nds32->memory.va_to_pa_off)
		nds32->memory.address_translation = false;
	//LOG_DEBUG("address_translation: %d, va_to_pa_off: %d", (int)nds32->memory.address_translation, nds32->memory.va_to_pa_off);
	return ERROR_OK;
}

static int nds32_update_mmu_info(struct nds32 *nds32)
{
	uint32_t value;

	/* Update MMU control status */
	nds32_get_mapped_reg(nds32, MR0, &value);
	nds32->mmu_config.default_min_page_size = value & 0x1;
	nds32->mmu_config.multiple_page_size_in_use = (value >> 10) & 0x1;

	return ERROR_OK;
}

static int nds32_update_cache_info(struct nds32 *nds32)
{
	uint32_t value;

	if (ERROR_OK == nds32_get_mapped_reg(nds32, MR8, &value)) {
		if (value & 0x1)
			nds32->memory.icache.enable = true;
		else
			nds32->memory.icache.enable = false;

		if (value & 0x2)
			nds32->memory.dcache.enable = true;
		else
			nds32->memory.dcache.enable = false;
	} else {
		nds32->memory.icache.enable = false;
		nds32->memory.dcache.enable = false;
	}

	return ERROR_OK;
}
extern uint32_t nds32_custom_def_idlm_base;
static int nds32_update_lm_info(struct nds32 *nds32)
{
	if (nds32_custom_def_idlm_base) {
		LOG_DEBUG("nds32_custom_def_idlm_base: %x", nds32_custom_def_idlm_base);
		return ERROR_OK;
	}

	struct nds32_memory *memory = &(nds32->memory);
	uint32_t value_mr6;
	uint32_t value_mr7;

	nds32_get_mapped_reg(nds32, MR6, &value_mr6);
	if (value_mr6 & 0x1)
		memory->ilm_enable = true;
	else
		memory->ilm_enable = false;

	if (memory->ilm_align_ver == 0) { /* 1MB aligned */
		memory->ilm_start = value_mr6 & 0xFFF00000;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else if (memory->ilm_align_ver == 1) { /* aligned to local memory size */
		memory->ilm_start = value_mr6 & 0xFFFFFC00;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else {
		memory->ilm_start = -1;
		memory->ilm_end = -1;
	}

	nds32_get_mapped_reg(nds32, MR7, &value_mr7);
	if (value_mr7 & 0x1)
		memory->dlm_enable = true;
	else
		memory->dlm_enable = false;

	if (memory->dlm_align_ver == 0) { /* 1MB aligned */
		memory->dlm_start = value_mr7 & 0xFFF00000;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else if (memory->dlm_align_ver == 1) { /* aligned to local memory size */
		memory->dlm_start = value_mr7 & 0xFFFFFC00;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else {
		memory->dlm_start = -1;
		memory->dlm_end = -1;
	}

	return ERROR_OK;
}

/**
 * If fpu/audio/cop# is disabled, to access fpu/audio/cop# registers will cause
 * exceptions. So, we need to check if fpu/audio is enabled or not as
 * target is halted. If fpu/audio is disabled, as users access fpu/audio
 * registers, OpenOCD will return fake value 0 instead of accessing
 * registers through DIM.
 */
static int nds32_check_extension(struct nds32 *nds32)
{
	uint32_t value = 0, i;

	if (nds32->cpu_version.cop_fpu_extension == 0)
		return ERROR_OK;

	nds32_get_mapped_reg(nds32, FUCPR, &value);
	LOG_DEBUG("fucpr_value: 0x%08x", value);

	if (value == NDS32_REGISTER_DISABLE) {
		nds32->fpu_enable = false;
		nds32->audio_enable = false;
		for (i = 0; i < MAX_COP_COUNT; i++)
			nds32->cop_enable[i] = false;
		return ERROR_OK;
	}

	if (value & 0x1)
		nds32->fpu_enable = true;
	else
		nds32->fpu_enable = false;

	if (value & 0x80000000)
		nds32->audio_enable = true;
	else
		nds32->audio_enable = false;

	for (i = 0; i < MAX_COP_COUNT; i++) {
		if (value & (0x1 << i))
			nds32->cop_enable[i] = true;
		else
			nds32->cop_enable[i] = false;
	}

	return ERROR_OK;
}

static struct nds32_relative_core_reg NDS32_RELATIVE_REG_TABLE[] = {
	{ FS0, FD0}, { FS1, FD0},
	{ FS2, FD1}, { FS3, FD1},
	{ FS4, FD2}, { FS5, FD2},
	{ FS6, FD3}, { FS7, FD3},
	{ FS8, FD4}, { FS9, FD4},
	{ FS10, FD5}, { FS11, FD5},
	{ FS12, FD6}, { FS13, FD6},
	{ FS14, FD7}, { FS15, FD7},
	{ FS16, FD8}, { FS17, FD8},
	{ FS18, FD9}, { FS19, FD9},
	{ FS20, FD10}, { FS21, FD10},
	{ FS22, FD11}, { FS23, FD11},
	{ FS24, FD12}, { FS25, FD12},
	{ FS26, FD13}, { FS27, FD13},
	{ FS28, FD14}, { FS29, FD14},
	{ FS30, FD15}, { FS31, FD15},
};

static int nds32_invalidate_relative_core_reg(uint32_t chk_reg_num, struct nds32 *nds32)
{
	struct nds32_relative_core_reg *p_relative_reg = &NDS32_RELATIVE_REG_TABLE[0];
	uint32_t i, loopcnt, reg_num1, reg_num2;

	loopcnt = sizeof(NDS32_RELATIVE_REG_TABLE) / sizeof(struct nds32_relative_core_reg);
	for (i=0; i<loopcnt; i++) {
		reg_num1 = p_relative_reg->core_reg_num;
		reg_num2 = p_relative_reg->relative_reg_num;
		if (chk_reg_num == reg_num1) {
			// invalidate relative reg
			nds32->core_cache->reg_list[reg_num2].valid = false;
		}
		else if (chk_reg_num == reg_num2) {
			// invalidate relative reg
			nds32->core_cache->reg_list[reg_num1].valid = false;
		}
		p_relative_reg ++;
	}
	return ERROR_OK;
}

static int nds32_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t value = buf_get_u32(buf, 0, 32);
	uint32_t reg_type;
	uint32_t append_mode = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	int mapped_regnum = nds32->register_map(nds32, reg_arch_info->num);

	/* ignore values that will generate exception */
	if (nds32_reg_exception(mapped_regnum, value))
		return ERROR_OK;

	LOG_DEBUG("writing register %i(%s) with value 0x%8.8" PRIx32,
			reg_arch_info->num, reg->name, value);

	reg_type = nds32_reg_type(mapped_regnum);
	if ((nds32->fpu_enable == false &&
			NDS32_REG_TYPE_FPU == reg_type) ||
		(nds32->audio_enable == false &&
			NDS32_REG_TYPE_AUMR == reg_type) ||
		(nds32->cop_enable[0] == false &&
			NDS32_REG_TYPE_COP0 == reg_type) ||
		(nds32->cop_enable[1] == false &&
			NDS32_REG_TYPE_COP1 == reg_type) ||
		(nds32->cop_enable[2] == false &&
			NDS32_REG_TYPE_COP2 == reg_type) ||
		(nds32->cop_enable[3] == false &&
			NDS32_REG_TYPE_COP3 == reg_type))
			{
		buf_set_u32(reg->value, 0, 32, 0);

		reg->valid = false;
		reg->dirty = false;
	} else {
		if (aice_usb_pack_command == 2) {
			/* without DBGER.DPED checking */
			if (aice_get_command_mode() == AICE_COMMAND_MODE_PACK)
				append_mode = 1;
			else {
				aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
				aice_set_command_mode(AICE_COMMAND_MODE_PACK);
				aice_packet_append_size = 0;
			}
		}
		buf_set_u32(reg->value, 0, 32, value);
		aice_write_register(target, mapped_regnum, reg_arch_info->value);

		if (append_mode == 1) {
			// skip read register & status update
			reg->valid = false;
			reg->dirty = false;
			return ERROR_OK;
		}
		/* After set value to registers, read the value from target
		 * to avoid W1C inconsistency. */
		aice_read_register(target, mapped_regnum, &(reg_arch_info->value));
		reg->valid = true;
		reg->dirty = false;
		if (aice_usb_pack_command == 2) {
			/* without DBGER.DPED checking */
			aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
			char usbin_data[512];
			aice_packbuffer_read(nds32->target, (uint8_t *)&usbin_data[0], aice_packet_append_size);
			unsigned char *pread0_byte = (unsigned char *)&usbin_data[32+32];  // 8*2+4*4, 4*3+8*2+4
			uint32_t value_reg = (unsigned char)pread0_byte[3];
			value_reg |= ((unsigned char)pread0_byte[2] << 8);
			value_reg |= ((unsigned char)pread0_byte[1] << 16);
			value_reg |= ((unsigned char)pread0_byte[0] << 24);
			reg_arch_info->value = value_reg;
		}
	}

	/* update registers to take effect right now */
	if (IR0 == mapped_regnum) {
		nds32_update_psw(nds32);
	} else if (MR0 == mapped_regnum) {
		nds32_update_mmu_info(nds32);
	} else if ((MR6 == mapped_regnum) || (MR7 == mapped_regnum)) {
		/* update lm information */
		nds32_update_lm_info(nds32);
	} else if (MR8 == mapped_regnum) {
		nds32_update_cache_info(nds32);
	} else if (FUCPR == mapped_regnum) {
		/* update audio/fpu/cop setting */
		nds32_check_extension(nds32);
	} else if ((mapped_regnum >= FS0) && (mapped_regnum <= FS31)) {
		nds32_invalidate_relative_core_reg(mapped_regnum, nds32);
	}
	else if (mapped_regnum == IDR1) {
		/* update ace_enable info. */
		uint32_t idr1_value = 0;
		nds32_get_mapped_reg(nds32, IDR1, &idr1_value);
		if ((idr1_value & 0x100) == 0) {
			nds32->cpu_version.ace_enable = 1;
		}
	}


	return ERROR_OK;
}

static int nds32_set_core_reg_64(struct reg *reg, uint8_t *buf)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t low_part = buf_get_u32(buf, 0, 32);
	uint32_t high_part = buf_get_u32(buf, 32, 32);
	uint32_t reg_type;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	reg_type = nds32_reg_type(reg_arch_info->num);
	if ((nds32->fpu_enable == false &&
			NDS32_REG_TYPE_FPU == reg_type) ||
		(nds32->cop_enable[0] == false &&
			NDS32_REG_TYPE_COP0 == reg_type) ||
		(nds32->cop_enable[1] == false &&
			NDS32_REG_TYPE_COP1 == reg_type) ||
		(nds32->cop_enable[2] == false &&
			NDS32_REG_TYPE_COP2 == reg_type) ||
		(nds32->cop_enable[3] == false &&
			NDS32_REG_TYPE_COP3 == reg_type))
			{

		buf_set_u32(reg->value, 0, 32, 0);
		buf_set_u32(reg->value, 32, 32, 0);

		reg->valid = false;
		reg->dirty = false;
	} else {
		buf_set_u32(reg->value, 0, 32, low_part);
		buf_set_u32(reg->value, 32, 32, high_part);
		aice_write_reg_64(target, reg_arch_info->num, reg_arch_info->value_64);
		reg->valid = true;
		reg->dirty = false;
	}

	int mapped_regnum = reg_arch_info->num;
	if ((mapped_regnum >= FD0) && (mapped_regnum <= FD15)) {
		nds32_invalidate_relative_core_reg(mapped_regnum, nds32);
	}

	return ERROR_OK;
}

static int nds32_set_ace_reg(struct reg *reg, uint8_t *buf)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target); 
	uint32_t regnum = reg_arch_info->num;
	uint32_t size_in_byte = 0;
	uint32_t reg_r2_value = 0;
	int retval;

	if (nds32->cpu_version.ace_enable == 0) {                                                                 
		nds_ace_enable(nds32);                                                                                
		nds32->cpu_version.ace_enable = 1;                                                                    
	} 

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg_arch_info->enable == false) {
		return ERROR_FAIL;
	}

	LOG_DEBUG("writing register %i(%s)",
			reg_arch_info->num, reg->name);

	if (NDS32_REG_TYPE_ACE == nds32_reg_type(regnum)) {
		size_in_byte = DIV_ROUND_UP(reg->size, 8);
		memset(reg->value, 0, size_in_byte);
		memcpy(reg->value, buf, size_in_byte);

		aice_read_reg(target, R2, &reg_r2_value);
		retval = aice_write_acr(target, regnum, reg->value);
		aice_write_reg(target, R2, reg_r2_value);

	} else {
		retval = ERROR_FAIL;
	}

	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}

static const struct reg_arch_type nds32_reg_access_type = {
	.get = nds32_get_core_reg,
	.set = nds32_set_core_reg,
};

static const struct reg_arch_type nds32_reg_access_type_64 = {
	.get = nds32_get_core_reg_64,
	.set = nds32_set_core_reg_64,
};

static const struct reg_arch_type nds32_ace_reg_access_type = {
	.get = nds32_get_ace_reg,
	.set = nds32_set_ace_reg,
};

#define NDS32_FEATURE(fname) \
static struct reg_feature feature_nds32_##fname = { .name = "org.gnu.gdb.nds32." #fname };

NDS32_FEATURE(core)
NDS32_FEATURE(system)
NDS32_FEATURE(audio)
NDS32_FEATURE(fpu)
NDS32_FEATURE(cop)
NDS32_FEATURE(ace)

static struct reg_cache *nds32_build_reg_cache(struct target *target,
		struct nds32 *nds32)
{
	uint32_t total_ace_reg_nums = nds32_reg_total_ace_reg_nums();
	LOG_DEBUG("ACE: total_ace_reg_nums: %d", total_ace_reg_nums);

	uint32_t total_cop_reg_nums = nds32_reg_total_cop_reg_nums();
	LOG_DEBUG("COP: total_cop_reg_nums: %d", total_cop_reg_nums);
	struct reg_cache *cache = calloc(sizeof(struct reg_cache), 1);
	struct reg *reg_list = calloc(TOTAL_REG_NUM + total_ace_reg_nums + total_cop_reg_nums, sizeof(struct reg));
	struct nds32_reg *reg_arch_info = calloc(TOTAL_REG_NUM + total_ace_reg_nums + total_cop_reg_nums, sizeof(struct nds32_reg));
	enum nds32_reg_type_s type;
	uint32_t i;

	if (!cache || !reg_list || !reg_arch_info) {
		free(cache);
		free(reg_list);
		free(reg_arch_info);
		return NULL;
	}

	cache->name = "Andes registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = 0;

	for (i = 0; i < TOTAL_REG_NUM + total_ace_reg_nums + total_cop_reg_nums; i++) {
		reg_arch_info[i].num = i;
		reg_arch_info[i].target = target;
		reg_arch_info[i].nds32 = nds32;
		reg_arch_info[i].enable = false;


		reg_list[i].name = nds32_reg_simple_name(i);
		reg_list[i].number = reg_arch_info[i].num;
		reg_list[i].size = nds32_reg_size(i);
		reg_list[i].arch_info = &reg_arch_info[i];

		reg_list[i].reg_data_type = calloc(sizeof(struct reg_data_type), 1);
		type = nds32_reg_type(i);

		if (type == NDS32_REG_TYPE_ACE) {
			/* ACR */
			unsigned acr_size_in_byte = DIV_ROUND_UP(nds32_reg_size(i), 8);
			reg_arch_info[i].value_acr = (char *)calloc(acr_size_in_byte, sizeof(char));
			reg_list[i].value = reg_arch_info[i].value_acr;
			reg_list[i].type = &nds32_ace_reg_access_type;
			reg_list[i].group = "ace";
			if (nds32_copilot_version() != 0x40000) {
				reg_list[i].reg_data_type->type = REG_TYPE_UINT8;
				reg_list[i].reg_data_type->id = "uint8";
			}
			//((struct nds32_reg*)(reg_list[i].arch_info))->enable = true;
		}
#if 1
		else if (type >= NDS32_REG_TYPE_COP0 && type <= NDS32_REG_TYPE_COP3) {
			/* cop register */
			if (reg_list[i].size == 32U) {
				reg_list[i].value = &(reg_arch_info[i].value);
				reg_list[i].type = &nds32_reg_access_type;
				reg_list[i].reg_data_type->type = REG_TYPE_UINT32;
				reg_list[i].reg_data_type->id = "uint32";
			} else {
				reg_list[i].value = &(reg_arch_info[i].value_64);
				reg_list[i].type = &nds32_reg_access_type_64;
				reg_list[i].reg_data_type->type = REG_TYPE_UINT64;
				reg_list[i].reg_data_type->id = "uint64";
			}
			reg_list[i].group = "cop";
#if USER_DEF_COP_REG
			uint32_t cop_idx = nds32_reg_type(i) - NDS32_REG_TYPE_COP0;
			if (cop_idx <= 3) {
				if ((nds32_cop_reg_base_id[cop_idx] == 0) ||
					(nds32_cop_reg_base_id[cop_idx] > i)){
					nds32_cop_reg_base_id[cop_idx] = i;
				}
				/* enable by user define */
				if (nds32_cop_reg_ena[cop_idx] == 1) {
					((struct nds32_reg*)(reg_list[i].arch_info))->enable = true;
					//nds32->cop_enable[cop_idx] = 1;
				}
			}
#endif
		}
#endif
		else if (FD0 <= reg_arch_info[i].num && reg_arch_info[i].num <= FD31) {
			reg_list[i].value = &(reg_arch_info[i].value_64);
			reg_list[i].type = &nds32_reg_access_type_64;

			reg_list[i].reg_data_type->type = REG_TYPE_IEEE_DOUBLE;
			reg_list[i].reg_data_type->id = "ieee_double";
			reg_list[i].group = "float";
		} else {
			reg_list[i].value = &(reg_arch_info[i].value);
			reg_list[i].type = &nds32_reg_access_type;
			reg_list[i].group = "general";

			if ((FS0 <= reg_arch_info[i].num) && (reg_arch_info[i].num <= FS31)) {
				reg_list[i].reg_data_type->type = REG_TYPE_IEEE_SINGLE;
				reg_list[i].reg_data_type->id = "ieee_single";
				reg_list[i].group = "float";
			} else if ((reg_arch_info[i].num == FPCSR) ||
				   (reg_arch_info[i].num == FPCFG)) {
				reg_list[i].group = "float";
			} else if ((reg_arch_info[i].num == R28) ||
				   (reg_arch_info[i].num == R29) ||
				   (reg_arch_info[i].num == R31)) {
				reg_list[i].reg_data_type->type = REG_TYPE_DATA_PTR;
				reg_list[i].reg_data_type->id = "data_ptr";
			} else if ((reg_arch_info[i].num == R30) ||
				   (reg_arch_info[i].num == PC)) {
				reg_list[i].reg_data_type->type = REG_TYPE_CODE_PTR;
				reg_list[i].reg_data_type->id = "code_ptr";
			} else {
				reg_list[i].reg_data_type->type = REG_TYPE_UINT32;
				reg_list[i].reg_data_type->id = "uint32";
			}
		}

		/* restore all registers except read-only registers */
		if (CR0 <= reg_arch_info[i].num && reg_arch_info[i].num <= CR7)
			reg_list[i].caller_save = false;
		else
			reg_list[i].caller_save = true;

		if (type == NDS32_REG_TYPE_GPR || type == NDS32_REG_TYPE_SPR)
			reg_list[i].feature = &feature_nds32_core;
		else if (type == NDS32_REG_TYPE_AUMR)
			reg_list[i].feature = &feature_nds32_audio;
		else if (type == NDS32_REG_TYPE_FPU)
			reg_list[i].feature = &feature_nds32_fpu;
#if 1
		else if (type >= NDS32_REG_TYPE_COP0 && type <= NDS32_REG_TYPE_COP3)
			reg_list[i].feature = &feature_nds32_cop;
#endif
		else if (type == NDS32_REG_TYPE_ACE)
			reg_list[i].feature = &feature_nds32_ace;
		else
			reg_list[i].feature = &feature_nds32_system;
		cache->num_regs++;
	}

	nds32->core_cache = cache;
	LOG_DEBUG("nds32->core_cache->num_regs: %d", nds32->core_cache->num_regs);
	return cache;
}

static int nds32_reg_cache_init(struct target *target, struct nds32 *nds32)
{
	struct reg_cache *cache;

	cache = nds32_build_reg_cache(target, nds32);
	if (!cache)
		return ERROR_FAIL;

	*register_get_last_cache_p(&target->reg_cache) = cache;

	return ERROR_OK;
}

static struct reg *nds32_reg_current(struct nds32 *nds32, unsigned regnum)
{
	struct reg *r;

	r = nds32->core_cache->reg_list + regnum;

	return r;
}

int nds32_pack_read_reg_into_cache(struct nds32 *nds32, uint32_t all_reg_nums,
		uint32_t *p_reg_id, uint32_t *p_reg_value)
{
	uint32_t value_reg[50], i, reg_nums;
	uint32_t *p_regid, *p_value_reg;
	struct reg *r;
	struct reg_cache *reg_cache = nds32->core_cache;

	LOG_DEBUG("read %d regs in pack command", all_reg_nums);
	if (p_reg_value != NULL)
		p_value_reg = p_reg_value;
	else
		p_value_reg = (uint32_t *)&value_reg[0];

	while (all_reg_nums) {
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);
		aice_set_command_mode(AICE_COMMAND_MODE_PACK);
		aice_packet_append_size = 0;

		if (all_reg_nums > 10)
			reg_nums = 10;
		else
			reg_nums = all_reg_nums;

		p_regid = p_reg_id;
		for (i = 0; i < reg_nums; i++) {
			nds32->core_cache->reg_list[*p_regid].valid = false;
			if (((struct nds32_reg *)reg_cache->reg_list[*p_regid].arch_info)->enable == true) {
				nds32_get_mapped_reg(nds32, *p_regid, &value_reg[i]);
			}
			p_regid ++;
		}
		aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

		char usbin_data[512];
		aice_packbuffer_read(nds32->target, (uint8_t *)&usbin_data[0], aice_packet_append_size);
		unsigned char *pread0_byte = (unsigned char *)&usbin_data[32];  // 4*3 + 8*2 +4

		p_regid = p_reg_id;
		for (i = 0; i < reg_nums; i++) {
			if (((struct nds32_reg *)reg_cache->reg_list[*p_regid].arch_info)->enable == true) {
				p_value_reg[i] = (unsigned char)pread0_byte[3];
				p_value_reg[i] |= ((unsigned char)pread0_byte[2] << 8);
				p_value_reg[i] |= ((unsigned char)pread0_byte[1] << 16);
				p_value_reg[i] |= ((unsigned char)pread0_byte[0] << 24);
				pread0_byte += 36;
				r = nds32_reg_current(nds32, *p_regid);
				buf_set_u32(r->value, 0, 32, p_value_reg[i]);
			}
			p_regid ++;
		}

		all_reg_nums -= reg_nums;
		p_value_reg += reg_nums;
		p_reg_id += reg_nums;
	}
	return ERROR_OK;
}

int nds32_full_context(struct nds32 *nds32)
{
	uint32_t value_ir0, i;
	nds32->core_cache->reg_list[R0].valid = true;
	nds32->core_cache->reg_list[R1].valid = true;

	for (i = CR0; i < CR7; i++) {
		nds32->core_cache->reg_list[i].valid = true;
	}

	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		uint32_t value_reg[20], reg_id[20], reg_nums = 11;
		for (i = 0; i < reg_nums; i ++)
			value_reg[i] = 0;
		reg_id[0] = PC;
		reg_id[1] = R28;  // FP
		reg_id[2] = R31;  // SP
		reg_id[3] = IR0;
		reg_id[4] = IR4;
		reg_id[5] = IR6;
		reg_id[6] = IR14;
		reg_id[7] = MR0;
		reg_id[8] = MR8;
		reg_id[9] = MR6;
		reg_id[10] = MR7;
		//reg_id[10] = FUCPR;
		nds32_pack_read_reg_into_cache(nds32, reg_nums,
			&reg_id[0], &value_reg[0]);

		value_ir0 = value_reg[3];
		LOG_DEBUG("pc=0x%x, fp=0x%x, sp=0x%x", value_reg[0], value_reg[1], value_reg[2]);
		LOG_DEBUG("ir0=0x%x, ir4=0x%x, ir6=0x%x, ir14=0x%x",
				value_reg[3], value_reg[4], value_reg[5], value_reg[6]);
		LOG_DEBUG("mr0=0x%x, mr8=0x%x, mr6=0x%x, mr7=0x%x",
				value_reg[7], value_reg[8], value_reg[9], value_reg[10]);
	} else {
		/* save $pc & $psw */
		nds32_get_mapped_reg(nds32, IR0, &value_ir0);
	}


#if ENABLE_DEX_USE_PSW
	nds32_backup_psw = value_ir0;
#endif
	uint32_t ir3_value;
	nds32_get_mapped_reg(nds32, IR3, &ir3_value);
	nds32_curr_ir3_value = ir3_value;

	nds32_update_psw(nds32);
	nds32_update_mmu_info(nds32);
	nds32_update_cache_info(nds32);
	nds32_update_lm_info(nds32);

	nds32_check_extension(nds32);

	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);


	///   \ \        / /\   |  __ \| \ | |_   _| \ | |/ ____|
	///    \ \  /\  / /  \  | |__) |  \| | | | |  \| | |  __ 
	///     \ \/  \/ / /\ \ |  _  /| . ` | | | | . ` | | |_ |
	///      \  /\  / ____ \| | \ \| |\  |_| |_| |\  | |__| |
	///       \/  \/_/    \_\_|  \_\_| \_|_____|_| \_|\_____|
	/// !!WARNING!! Please also check aice_read_acr() in src/jtag/aice
	/// !!WARNING!! Please also check nds32_set_core_reg()
	if (cpu_version->ace_support == 1) {
		uint32_t idr1_value = 0;
		nds32_get_mapped_reg(nds32, IDR1, &idr1_value);
		/* idr1(MISC_CTL) bit-8 for ACE disable */
		if ((idr1_value & 0x100) == 0) {
			cpu_version->ace_enable = 1;
		} else {
			cpu_version->ace_enable = 0;
		}
	}
	LOG_DEBUG("ace_support=%d, ace_enable=%d",
			cpu_version->ace_support, cpu_version->ace_enable);

	uint32_t acc_ctl_value = 0;
	uint32_t set_acc_mode_before = nds32->memory.set_acc_mode;
	aice_read_misc(nds32->target, NDS_EDM_MISC_ACC_CTL, &acc_ctl_value);
	nds32->memory.set_acc_mode = (acc_ctl_value & 0x07) + 1;
	LOG_DEBUG("before=0x%x, set_acc_mode=0x%x", set_acc_mode_before, nds32->memory.set_acc_mode);
	return ERROR_OK;
}

/* get register value internally */
int nds32_get_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t *value)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *r;

	if (regnum > reg_cache->num_regs)
		return ERROR_FAIL;

	r = nds32_reg_current(nds32, regnum);
	if (ERROR_OK != r->type->get(r))
		return ERROR_FAIL;

	*value = buf_get_u32(r->value, 0, 32);

	return ERROR_OK;
}

/** set register internally */
int nds32_set_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t value)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *r;
	uint8_t set_value[4];

	if (regnum > reg_cache->num_regs)
		return ERROR_FAIL;

	r = nds32_reg_current(nds32, regnum);

	buf_set_u32(set_value, 0, 32, value);

	return r->type->set(r, set_value);
}

/** get general register list */
static int nds32_get_general_reg_list(struct nds32 *nds32,
		struct reg **reg_list[], int *reg_list_size)
{
	struct reg *reg_current;
	int i;
	int current_idx;

	/** freed in gdb_server.c */
	*reg_list = malloc(sizeof(struct reg *) * (R0 - R0 + 1));
	current_idx = 0;

	for (i = R0; i < R0 + 1; i++) {
		reg_current = nds32_reg_current(nds32, i);
		if (((struct nds32_reg *)reg_current->arch_info)->enable) {
			(*reg_list)[current_idx] = reg_current;
			current_idx++;
		}
	}
	*reg_list_size = current_idx;

	return ERROR_OK;
}

/** get all register list */
static int nds32_get_all_reg_list(struct nds32 *nds32,
		struct reg **reg_list[], int *reg_list_size)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *reg_current;
	unsigned int i;

	*reg_list_size = reg_cache->num_regs;

	/** freed in gdb_server.c */
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < reg_cache->num_regs; i++) {
		reg_current = nds32_reg_current(nds32, i);
		reg_current->exist = ((struct nds32_reg *)
				reg_current->arch_info)->enable;
		(*reg_list)[i] = reg_current;
	}

	return ERROR_OK;
}

/** get all register list */
int nds32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	struct nds32 *nds32 = target_to_nds32(target);

	switch (reg_class) {
		case REG_CLASS_ALL:
			return nds32_get_all_reg_list(nds32, reg_list, reg_list_size);
		case REG_CLASS_GENERAL:
			return nds32_get_general_reg_list(nds32, reg_list, reg_list_size);
		default:
			return ERROR_FAIL;
	}

	return ERROR_FAIL;
}

int nds32_select_memory_mode(struct target *target, uint32_t address,
		uint32_t length, uint32_t *end_address)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	struct nds32_edm *edm = &(nds32->edm);
	uint32_t dlm_start, dlm_end;
	uint32_t ilm_start, ilm_end;
	uint32_t address_end = address + length;

	/* init end_address */
	*end_address = address_end;

	if (NDS_MEMORY_ACC_CPU == memory->access_channel)
		return ERROR_OK;

	/* set default mode */
	if (target->state != TARGET_HALTED)
		aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);

	if (edm->access_control == false) {
		LOG_DEBUG("EDM does not support ACC_CTL");
		aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		nds32_auto_change_memory_mode(target, address, length);
		return ERROR_OK;
	}

	if (edm->direct_access_local_memory == false) {
		LOG_DEBUG("EDM does not support DALM");
		aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		nds32_auto_change_memory_mode(target, address, length);
		return ERROR_OK;
	}

	if (NDS_MEMORY_SELECT_AUTO != memory->select_acc_mode) {
		LOG_DEBUG("Memory mode is not AUTO");
		aice_acc_memory_mode(target, memory->select_acc_mode);
		return ERROR_OK;
	}

	if ((memory->ilm_base != 0) && (memory->ilm_enable == true)) {
		ilm_start = memory->ilm_start;
		ilm_end = memory->ilm_end;

		/* case 1, address < ilm_start */
		if (address < ilm_start) {
			if (ilm_start < address_end) {
				/* update end_address to split non-ILM from ILM */
				*end_address = ilm_start;
			}
			/* MEM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		} else if ((ilm_start <= address) && (address < ilm_end)) {
			/* case 2, ilm_start <= address < ilm_end */
			if (ilm_end < address_end) {
				/* update end_address to split non-ILM from ILM */
				*end_address = ilm_end;
			}
			/* ILM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_ILM);
			return ERROR_OK;
		} else { /* case 3, ilm_end <= address */
			/* MEM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		}
	} else {
		LOG_DEBUG("ILM is not enabled");
	}

	if ((memory->dlm_base != 0) && (memory->dlm_enable == true)) {
		dlm_start = memory->dlm_start;
		dlm_end = memory->dlm_end;

		/* case 1, address < dlm_start */
		if (address < dlm_start) {
			if (dlm_start < address_end) {
				/* update end_address to split non-DLM from DLM */
				*end_address = dlm_start;
			}
			/* MEM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		} else if ((dlm_start <= address) && (address < dlm_end)) {
			/* case 2, dlm_start <= address < dlm_end */
			if (dlm_end < address_end) {
				/* update end_address to split non-DLM from DLM */
				*end_address = dlm_end;
			}
			/* DLM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_DLM);
			return ERROR_OK;
		} else { /* case 3, dlm_end <= address */
			/* MEM mode */
			aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);
		}
	} else {
		LOG_DEBUG("DLM is not enabled");
	}
	/* set default mode */
	aice_acc_memory_mode(target, NDS_MEMORY_SELECT_MEM);

	return ERROR_OK;
}

#define MEM_ACCESS_ATTR_MAX		128
struct nds32_mem_access_attr all_mem_access_attr[MEM_ACCESS_ATTR_MAX+1];
uint32_t curr_mem_access_attr_index = 0;

void nds32_reset_buffer_access_size(void)
{
	uint32_t i;
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr*)&all_mem_access_attr[0];
	for (i=0; i<=MEM_ACCESS_ATTR_MAX; i++) {
		pmem_attr->lowAddr = 0;
		pmem_attr->highAddr = 0;
		pmem_attr->access_size = 0;
		pmem_attr ++;
	}
	curr_mem_access_attr_index = 0;
}

int nds32_set_buffer_access_size(uint64_t start_addr, uint64_t end_addr,
		uint32_t access_size)
{
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr*)&all_mem_access_attr[0];
	if (curr_mem_access_attr_index >= MEM_ACCESS_ATTR_MAX) {
		LOG_ERROR("nds32_set_buffer_access_size error!");
		return ERROR_FAIL;
	}

	curr_mem_access_attr_index ++;  // start from (idx = 1)
	pmem_attr += curr_mem_access_attr_index;
	pmem_attr->lowAddr = start_addr;
	pmem_attr->highAddr = end_addr;
	pmem_attr->access_size = access_size;
	LOG_DEBUG("nds32_set_buffer_access_size %d: 0x%016" PRIx64 ", 0x%016" PRIx64 ", %d\n",
		curr_mem_access_attr_index, pmem_attr->lowAddr, pmem_attr->highAddr, pmem_attr->access_size);
	return ERROR_OK;
}

int nds32_get_buffer_access_size(uint64_t start_addr, uint32_t bufsize,
		uint32_t *pcurr_size, uint32_t *paccess_size)
{
	uint32_t access_size = 4;  // default word access
	uint32_t curr_size = bufsize;

	if (curr_mem_access_attr_index == 0) {
		*pcurr_size = curr_size;
		*paccess_size = access_size;
		return ERROR_OK;
	}
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr*)&all_mem_access_attr[0];
	// search from tail to head
	pmem_attr += curr_mem_access_attr_index;
	//while(1) {
	for (uint32_t i = 0; i < curr_mem_access_attr_index; i++) {
		if (pmem_attr->access_size == 0)
			break;
		if ((start_addr >= pmem_attr->lowAddr) &&
				(start_addr <= pmem_attr->highAddr)){
			if ((start_addr + (bufsize - 1)) >= pmem_attr->highAddr) {
				curr_size = (pmem_attr->highAddr - start_addr) + 1;
			}
			access_size = pmem_attr->access_size;
			break;
		}
		pmem_attr--;
	}

	*pcurr_size = curr_size;
	*paccess_size = access_size;
	return ERROR_OK;
}

int nds32_rw_buffer_unaligned(uint32_t ifwrite, struct target *target, uint32_t address,
		uint32_t size, uint8_t *buffer, uint32_t access_size)
{
	uint32_t start_address, end_address;
	int retval;
	uint32_t tmp_buffer, i;
	uint8_t *ptmp_buffer = (uint8_t *)&tmp_buffer;

	start_address = address - (address % access_size);
	ptmp_buffer += (address % access_size);
	nds32_select_memory_mode(target, start_address, access_size, &end_address);
	retval = aice_read_mem_unit(target, start_address, access_size, 1, (uint8_t *)&tmp_buffer);
	if (retval != ERROR_OK)
		return retval;
	// write buffer
	if (ifwrite) {
		for (i = 0; i < size; i++) {
			*ptmp_buffer++ = *buffer++;
		}
		retval = aice_write_mem_unit(target, start_address, access_size, 1, (uint8_t *)&tmp_buffer);
		if (retval != ERROR_OK)
			return retval;
	}
	// read buffer
	else {
		for (i = 0; i < size; i++) {
			*buffer++ = *ptmp_buffer++;
		}
	}
	return ERROR_OK;
}

int nds32_rw_buffer_with_access_size(uint32_t ifwrite, struct target *target, uint32_t address,
		uint32_t size, uint8_t *buffer, uint32_t access_size)
{
	if (size == 0)
		return ERROR_OK;
	struct nds32 *nds32 = target_to_nds32(target);

	if ((NDS_MEMORY_ACC_CPU == nds32->memory.access_channel) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = ERROR_OK;
	uint32_t end_address;

	LOG_DEBUG("nds32_rw_buffer_with_access_size, address=%x, access_size=%x", address, access_size);
	/* handle unaligned head bytes */
	if (address % access_size) {
		uint32_t unaligned_size = access_size - (address % access_size);

		if (unaligned_size > size)
			unaligned_size = size;
		LOG_DEBUG("PRECEDE RW BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address, unaligned_size);
		retval = nds32_rw_buffer_unaligned(ifwrite, target, address, unaligned_size, buffer, access_size);
		if (retval != ERROR_OK)
			return retval;

		buffer += unaligned_size;
		address += unaligned_size;
		size -= unaligned_size;
	}

	/* handle aligned words */
	if (size >= access_size) {
		uint32_t aligned_size = size - (size % access_size);
		uint32_t rw_length;

		do {
			nds32_select_memory_mode(target, address, aligned_size, &end_address);

			rw_length = end_address - address;
			if (ifwrite) {
				if ((rw_length > 8) && (access_size == 4))
					retval = aice_write_mem_bulk(target, address, rw_length, buffer);
				else
					retval = aice_write_mem_unit(target, address, access_size, rw_length / access_size, buffer);
			}
			else {
				if ((rw_length > 8) && (access_size == 4))
					retval = aice_read_mem_bulk(target, address, rw_length, buffer);
				else
					retval = aice_read_mem_unit(target, address, access_size, rw_length / access_size, buffer);
			}
			if (retval != ERROR_OK)
				return retval;

			buffer += rw_length;
			address += rw_length;
			size -= rw_length;
			aligned_size -= rw_length;

		} while (aligned_size != 0);
	}

	/* handle tail writes of less than 4 bytes */
	if (size > 0) {
		LOG_DEBUG("POSTPONE RW BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address, size);
		retval = nds32_rw_buffer_unaligned(ifwrite, target, address, size, buffer, access_size);
		if (retval != ERROR_OK)
			return retval;
	}
	return ERROR_OK;
}

int nds32_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;
	retval = nds32_rw_buffer_with_access_size(0, target, address,
			size*count, (uint8_t *)buffer, size);

#if ENABLE_DEX_USE_PSW
	if (nds32_use_psw_mode == 1) {
		uint32_t edm_ctl = 0;
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl);
		aice_write_register(target, IR0, nds32_backup_psw);
	}
#endif

	return retval;
}

int nds32_read_buffer(struct target *target, uint32_t address,
		uint32_t readsize, uint8_t *buffer)
{
	uint32_t access_size = 4;
	int retval = ERROR_OK;

	LOG_DEBUG("READ BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address,
			readsize);
	if (readsize == 0)
		return ERROR_OK;

	uint32_t start_addr = address, total_size = readsize;
	while(total_size) {
		nds32_get_buffer_access_size(start_addr, total_size,
			&readsize, &access_size);

		retval = nds32_rw_buffer_with_access_size(0, target, start_addr,
			readsize, (uint8_t *)buffer, access_size);
		if (retval != ERROR_OK)
			break;
		total_size -= readsize;
		start_addr += readsize;
		buffer += readsize;
	}

#if ENABLE_DEX_USE_PSW
	if (nds32_use_psw_mode == 1) {
		uint32_t edm_ctl = 0;
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl);
		aice_write_register(target, IR0, nds32_backup_psw);
	}
#endif

	return retval;
}

int nds32_read_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	int result;

	/* write_back & invalidate dcache & invalidate icache */
	if (nds32->memory.dcache.enable == true)
		nds32_cache_sync(target, address, size*count);
	/* switch to BUS access mode to skip MMU */
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;

	/* The input address is physical address.  No need to do address translation. */
	result = nds32_read_memory(target, address,
			size, count, buffer);

	/* restore to origin access mode */
	nds32->memory.access_channel = orig_channel;
	return result;
}

int nds32_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;
	retval = nds32_rw_buffer_with_access_size(1, target, address,
			size*count, (uint8_t *)buffer, size);

#if ENABLE_DEX_USE_PSW
	if (nds32_use_psw_mode == 1) {
		uint32_t edm_ctl = 0;
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl);
		aice_write_register(target, IR0, nds32_backup_psw);
	}
#endif

	return retval;
}

int nds32_write_buffer(struct target *target, uint32_t address,
		uint32_t writesize, const uint8_t *buffer)
{
	uint32_t access_size = 4;
	int retval = ERROR_OK;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t start_addr = address, total_size = writesize;
	uint8_t *write_buffer = (uint8_t *)buffer;

	LOG_DEBUG("WRITE BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address,
			writesize);
	if (writesize == 0)
		return ERROR_OK;

	if (nds32->hit_syscall) {
		nds32_gdb_fileio_write_memory(nds32, address, &total_size, &write_buffer);
		LOG_DEBUG("gdb_fileio_write_memory, total_size=%d", total_size);
	}

	while(total_size) {
		nds32_get_buffer_access_size(start_addr, total_size,
			&writesize, &access_size);

		retval = nds32_rw_buffer_with_access_size(1, target, start_addr,
			writesize, (uint8_t *)write_buffer, access_size);
		if (retval != ERROR_OK)
			break;
		total_size -= writesize;
		start_addr += writesize;
		write_buffer += writesize;
	}

#if ENABLE_DEX_USE_PSW
	if (nds32_use_psw_mode == 1) {
		uint32_t edm_ctl = 0;
		aice_read_debug_reg(target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;
		aice_write_debug_reg(target, NDS_EDM_SR_EDM_CTL, edm_ctl);
		aice_write_register(target, IR0, nds32_backup_psw);
	}
#endif

	return retval;
}

int nds32_write_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	int result;

	/* write_back & invalidate dcache & invalidate icache */
	if (nds32->memory.dcache.enable == true)
		nds32_cache_sync(target, address, size*count);
	/* switch to BUS access mode to skip MMU */
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;

	/* The input address is physical address.  No need to do address translation. */
	result = nds32_write_memory(target, address,
			size, count, buffer);

	/* restore to origin access mode */
	nds32->memory.access_channel = orig_channel;
	return result;
}

int nds32_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_INVALID;
	}

	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);

	if ((mmu_config->memory_protection == 2) && (memory->address_translation == true))
		*enabled = 1;
	else
		*enabled = 0;

	return ERROR_OK;
}

int nds32_arch_state(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->common_magic != NDS32_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-Andes target");
		return ERROR_FAIL;
	}

	uint32_t value_pc, value_psw;

	nds32_get_mapped_reg(nds32, PC, &value_pc);
	nds32_get_mapped_reg(nds32, IR0, &value_psw);

	LOG_DEBUG("target halted due to %s\n"
			"psw: 0x%8.8" PRIx32 " pc: 0x%8.8" PRIx32,
			debug_reason_name(target),
			value_psw,
			value_pc);

	/* save pc value to pseudo register pc */
	struct reg *reg = register_get_by_name(target->reg_cache, "pc", 1);
	buf_set_u32(reg->value, 0, 32, value_pc);

	return ERROR_OK;
}

static void nds32_init_must_have_registers(struct nds32 *nds32)
{
	struct reg_cache *reg_cache = nds32->core_cache;

	/** MUST have general registers */
	((struct nds32_reg *)reg_cache->reg_list[R0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R2].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R4].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R5].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R6].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R7].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R8].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R9].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R10].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R15].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R28].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R29].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R30].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R31].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[PC].arch_info)->enable = true;

	/** MUST have configuration system registers */
	((struct nds32_reg *)reg_cache->reg_list[CR0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR2].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR4].arch_info)->enable = true;

	/** MUST have interrupt system registers */
	((struct nds32_reg *)reg_cache->reg_list[IR0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR4].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR6].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR9].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR11].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR14].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR15].arch_info)->enable = true;

	/** MUST have MMU system registers */
	((struct nds32_reg *)reg_cache->reg_list[MR0].arch_info)->enable = true;

	/** MUST have EDM system registers */
	((struct nds32_reg *)reg_cache->reg_list[DR40].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[DR41].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[DR42].arch_info)->enable = true;
}

static int nds32_init_memory_config(struct nds32 *nds32)
{
	uint32_t value_cr1; /* ICM_CFG */
	uint32_t value_cr2; /* DCM_CFG */
	struct nds32_memory *memory = &(nds32->memory);

	/* read $cr1 to init instruction memory information */
	nds32_get_mapped_reg(nds32, CR1, &value_cr1);
	memory->icache.set = value_cr1 & 0x7;
	memory->icache.way = (value_cr1 >> 3) & 0x7;
	memory->icache.line_size = (value_cr1 >> 6) & 0x7;
	memory->icache.lock_support = (value_cr1 >> 9) & 0x1;

	memory->ilm_base = (value_cr1 >> 10) & 0x7;
	memory->ilm_align_ver = (value_cr1 >> 13) & 0x3;

	memory->icache_ecc = (value_cr1 >> 17) & 0x3;
	memory->ilm_ecc = (value_cr1 >> 19) & 0x3;
	/* read $cr2 to init data memory information */
	nds32_get_mapped_reg(nds32, CR2, &value_cr2);
	memory->dcache.set = value_cr2 & 0x7;
	memory->dcache.way = (value_cr2 >> 3) & 0x7;
	memory->dcache.line_size = (value_cr2 >> 6) & 0x7;
	memory->dcache.lock_support = (value_cr2 >> 9) & 0x1;

	memory->dlm_base = (value_cr2 >> 10) & 0x7;
	memory->dlm_align_ver = (value_cr2 >> 13) & 0x3;

	memory->dcache_ecc = (value_cr2 >> 17) & 0x3;
	memory->dlm_ecc = (value_cr2 >> 19) & 0x3;
	return ERROR_OK;
}

static void nds32_init_config(struct nds32 *nds32)
{
	uint32_t value_cr0;
	uint32_t value_cr3;
	uint32_t value_cr4;
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	struct nds32_misc_config *misc_config = &(nds32->misc_config);

	nds32_get_mapped_reg(nds32, CR0, &value_cr0);
	nds32_get_mapped_reg(nds32, CR3, &value_cr3);
	nds32_get_mapped_reg(nds32, CR4, &value_cr4);

	/* config cpu version */
	cpu_version->performance_extension = value_cr0 & 0x1;
	cpu_version->_16bit_extension = (value_cr0 >> 1) & 0x1;
	cpu_version->performance_extension_2 = (value_cr0 >> 2) & 0x1;
	cpu_version->cop_fpu_extension = (value_cr0 >> 3) & 0x1;
	cpu_version->string_extension = (value_cr0 >> 4) & 0x1;
	cpu_version->secure = (value_cr0 >> 7) & 0x1;
	cpu_version->revision = (value_cr0 >> 16) & 0xFF;
	cpu_version->cpu_id_family = (value_cr0 >> 24) & 0xF;
	cpu_version->cpu_id_version = (value_cr0 >> 28) & 0xF;
	cpu_version->ace_enable = 0;
	cpu_version->ace_support = 0;
	if (value_cr0 & 0x40)
		cpu_version->ace_support = 1;

	/* config MMU */
	mmu_config->memory_protection = value_cr3 & 0x3;
	mmu_config->memory_protection_version = (value_cr3 >> 2) & 0x1F;
	mmu_config->fully_associative_tlb = (value_cr3 >> 7) & 0x1;
	if (mmu_config->fully_associative_tlb) {
		mmu_config->tlb_size = (value_cr3 >> 8) & 0x7F;
	} else {
		mmu_config->tlb_ways = (value_cr3 >> 8) & 0x7;
		mmu_config->tlb_sets = (value_cr3 >> 11) & 0x7;
	}
	mmu_config->_8k_page_support = (value_cr3 >> 15) & 0x1;
	mmu_config->extra_page_size_support = (value_cr3 >> 16) & 0xFF;
	mmu_config->tlb_lock = (value_cr3 >> 24) & 0x1;
	mmu_config->hardware_page_table_walker = (value_cr3 >> 25) & 0x1;
	mmu_config->default_endian = (value_cr3 >> 26) & 0x1;
	mmu_config->partition_num = (value_cr3 >> 27) & 0x1;
	mmu_config->invisible_tlb = (value_cr3 >> 28) & 0x1;
	mmu_config->vlpt = (value_cr3 >> 29) & 0x1;
	mmu_config->ntme = (value_cr3 >> 30) & 0x1;
	mmu_config->drde = (value_cr3 >> 31) & 0x1;

	/* config misc */
	misc_config->edm = value_cr4 & 0x1;
	misc_config->local_memory_dma = (value_cr4 >> 1) & 0x1;
	misc_config->performance_monitor = (value_cr4 >> 2) & 0x1;
	misc_config->high_speed_memory_port = (value_cr4 >> 3) & 0x1;
	misc_config->debug_tracer = (value_cr4 >> 4) & 0x1;
	misc_config->div_instruction = (value_cr4 >> 5) & 0x1;
	misc_config->mac_instruction = (value_cr4 >> 6) & 0x1;
	misc_config->audio_isa = (value_cr4 >> 7) & 0x3;
	misc_config->L2_cache = (value_cr4 >> 9) & 0x1;
	misc_config->reduce_register = (value_cr4 >> 10) & 0x1;
	misc_config->addr_24 = (value_cr4 >> 11) & 0x1;
	misc_config->interruption_level = (value_cr4 >> 12) & 0x1;
	misc_config->baseline_instruction = (value_cr4 >> 13) & 0x7;
	misc_config->no_dx_register = (value_cr4 >> 16) & 0x1;
	misc_config->implement_dependant_register = (value_cr4 >> 17) & 0x1;
	misc_config->implement_dependant_sr_encoding = (value_cr4 >> 18) & 0x1;
	misc_config->ifc = (value_cr4 >> 19) & 0x1;
	misc_config->mcu = (value_cr4 >> 20) & 0x1;
	misc_config->shadow = (value_cr4 >> 21) & 0x7;
	misc_config->ex9 = (value_cr4 >> 24) & 0x1;
	misc_config->pft = (value_cr4 >> 26) & 0x1;
	misc_config->hsp = (value_cr4 >> 27) & 0x1;
	misc_config->msc_ext = (value_cr4 >> 30) & 0x3;

	nds32_init_memory_config(nds32);

	uint32_t value_ir0=0;
	nds32_get_mapped_reg(nds32, IR0, &value_ir0);
	if ((value_ir0 >> 5) & 0x1) {
		nds32->target->endianness = TARGET_BIG_ENDIAN;
		nds32->data_endian = TARGET_BIG_ENDIAN;
	} else {
		nds32->target->endianness = TARGET_LITTLE_ENDIAN;
		nds32->data_endian = TARGET_LITTLE_ENDIAN;
	}
}

static int nds32_init_option_registers(struct nds32 *nds32)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	struct nds32_misc_config *misc_config = &(nds32->misc_config);
	struct nds32_memory *memory_config = &(nds32->memory);
	uint32_t msc_cfg2_value = 0, i;
	bool no_cr5;
	bool mr10_exist;
	bool no_racr0;
	LOG_DEBUG("ACE init nds32_init_option_registers");

	if (((cpu_version->cpu_id_family == 0xC) || (cpu_version->cpu_id_family == 0xD)) &&
			((cpu_version->revision & 0xFC) == 0)) {
		no_cr5 = true;
		mr10_exist = true;
		no_racr0 = true;
	} else {
		no_cr5 = false;
		mr10_exist = false;
		no_racr0 = false;
	}

	if (misc_config->reduce_register == false) {
		((struct nds32_reg *)reg_cache->reg_list[R11].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R12].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R13].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R14].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R16].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R17].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R18].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R19].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R20].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R21].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R22].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R23].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R24].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R25].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R26].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R27].arch_info)->enable = true;
	}

	if (misc_config->no_dx_register == false) {
		((struct nds32_reg *)reg_cache->reg_list[D0LO].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D0HI].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D1LO].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D1HI].arch_info)->enable = true;
	}

	if (misc_config->ex9)
		((struct nds32_reg *)reg_cache->reg_list[ITB].arch_info)->enable = true;

	if (no_cr5 == false) {
		((struct nds32_reg *)reg_cache->reg_list[CR5].arch_info)->enable = true;
		uint32_t value_cr5 = 0;
		nds32_get_mapped_reg(nds32, CR5, &value_cr5);
	}

	uint32_t reg_type, cop_reg_num, cop_reg_num_bound;
	uint32_t total_ace_reg_nums = nds32_reg_total_ace_reg_nums();
	uint32_t total_cop_reg_nums = nds32_reg_total_cop_reg_nums();
	/* ACE */
	uint32_t value_cr0 = 0;
	nds32_get_mapped_reg(nds32, CR0, &value_cr0);

	if (value_cr0 & 0x40) {
		/* enable ACE registers if CEXT is set */
		LOG_DEBUG("ACE: has cop extension : %d", total_ace_reg_nums);
		cop_reg_num = TOTAL_REG_NUM;
		cop_reg_num_bound = TOTAL_REG_NUM + total_ace_reg_nums;
		LOG_DEBUG("ACE: value_cr0=0x%x", value_cr0);
		reg_type = NDS32_REG_TYPE_ACE;

		while (cop_reg_num < cop_reg_num_bound) {
			if (nds32_reg_type(cop_reg_num) == reg_type) {
				LOG_DEBUG("ACE reg_num: %d", cop_reg_num);
				((struct nds32_reg *)reg_cache->reg_list[cop_reg_num].arch_info)->enable = true;
				cop_reg_num++;
			} else
				break;
		}
	}
	if (cpu_version->cop_fpu_extension) {

		LOG_DEBUG("COP: has cop extension");
		((struct nds32_reg *)reg_cache->reg_list[CR6].arch_info)->enable = true;
		//((struct nds32_reg *)reg_cache->reg_list[FPCSR].arch_info)->enable = true;
		//((struct nds32_reg *)reg_cache->reg_list[FPCFG].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[FUCPR].arch_info)->enable = true;
#if 1
		/* COP */
		/* enable coprocessor registers if CPxEX is set */
		uint32_t value_cr6 = 0, value_ex = 0;
		nds32_get_mapped_reg(nds32, CR6, &value_cr6);
		cop_reg_num = TOTAL_REG_NUM + total_ace_reg_nums;
		cop_reg_num_bound = TOTAL_REG_NUM + total_ace_reg_nums + total_cop_reg_nums;
		for (i = 0; i < MAX_COP_COUNT; i++) {
			value_ex = (0x01 << i);
			if (value_cr6 & value_ex) {
				reg_type = NDS32_REG_TYPE_COP0 + i;
				cop_reg_num = TOTAL_REG_NUM + total_ace_reg_nums;
				while (cop_reg_num < cop_reg_num_bound) {
					if (nds32_reg_type(cop_reg_num) == reg_type) {
						((struct nds32_reg *)reg_cache->reg_list[cop_reg_num].arch_info)->enable = true;
					}
					cop_reg_num++;
				}

			}
		}
#endif
		if (value_cr6 & 0x80000000) { // CP0ISFPU
			((struct nds32_reg *)reg_cache->reg_list[FPCSR].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[FPCFG].arch_info)->enable = true;

		uint32_t fpcfg_value = 0;
		uint32_t fucpr_backup = 0;
		uint32_t num_of_sp_reg = 0;
		uint32_t num_of_dp_reg = 0;

		nds32_get_mapped_reg(nds32, FUCPR, &fucpr_backup);

		if ((fucpr_backup & 0x1) == 0)
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup | 0x1);

		nds32->fpu_enable = true;
		nds32_get_mapped_reg(nds32, FPCFG, &fpcfg_value);

		if ((fucpr_backup & 0x1) == 0){
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup);
			nds32->fpu_enable = false;
		}

		switch ((fpcfg_value & 0xc) >> 2){
			case 0:
				num_of_sp_reg = 8;
				num_of_dp_reg = 4;
				break;
			case 1:
				num_of_sp_reg = 16;
				num_of_dp_reg = 8;
				break;
			case 2:
				num_of_sp_reg = 32;
				num_of_dp_reg = 16;
				break;
			case 3:
				num_of_sp_reg = 32;
				num_of_dp_reg = 32;
				break;
		}

		for(i = 0; i < num_of_sp_reg; i++)
			((struct nds32_reg *)reg_cache->reg_list[FS0+i].arch_info)->enable = true;
		for(i = 0; i < num_of_dp_reg; i++)
			((struct nds32_reg *)reg_cache->reg_list[FD0+i].arch_info)->enable = true;
	}
	}

	if (nds32->privilege_level != 0)
		((struct nds32_reg *)reg_cache->reg_list[IR3].arch_info)->enable = false;

	if (misc_config->mcu == true)
		((struct nds32_reg *)reg_cache->reg_list[IR4].arch_info)->enable = false;

	if (misc_config->interruption_level == false) {
		((struct nds32_reg *)reg_cache->reg_list[IR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR10].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR12].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR13].arch_info)->enable = true;

		/* Secure MPU has no P_ITYPE */
		if (mmu_config->memory_protection != 1)
			((struct nds32_reg *)reg_cache->reg_list[IR7].arch_info)->enable = true;
	}

	//if ((cpu_version->cpu_id_family == 0x9) ||
	//		(cpu_version->cpu_id_family == 0xA) ||
	//		(cpu_version->cpu_id_family == 0xC) ||
	//		(cpu_version->cpu_id_family == 0xD))
		((struct nds32_reg *)reg_cache->reg_list[IR8].arch_info)->enable = true;

	if (misc_config->shadow == 1) {
		((struct nds32_reg *)reg_cache->reg_list[IR16].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR17].arch_info)->enable = true;
	}

	if (misc_config->ifc)
		((struct nds32_reg *)reg_cache->reg_list[IFC_LP].arch_info)->enable = true;

	if (nds32->privilege_level != 0)
		((struct nds32_reg *)reg_cache->reg_list[MR0].arch_info)->enable = false;

	if (mmu_config->memory_protection == 1) {
		if (mmu_config->memory_protection_version == 24)
			((struct nds32_reg *)reg_cache->reg_list[MR4].arch_info)->enable = true;

		if (nds32->privilege_level == 0) {
			if ((mmu_config->memory_protection_version == 16) ||
				(mmu_config->memory_protection_version == 24)) {

				((struct nds32_reg *)reg_cache->reg_list[MR11].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[SECUR0].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR20].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR22].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR24].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR30].arch_info)->enable = true;

				if (misc_config->shadow == 1) {
					((struct nds32_reg *)reg_cache->reg_list[IR21].arch_info)->enable = true;
					((struct nds32_reg *)reg_cache->reg_list[IR23].arch_info)->enable = true;
					((struct nds32_reg *)reg_cache->reg_list[IR25].arch_info)->enable = true;
				}
			}
		}
		else {
			LOG_DEBUG("IR1 IR9 disable, nds32_security_compat_display = %d", nds32_security_compat_display);
			if (nds32_security_compat_display == 0) {
				((struct nds32_reg *)reg_cache->reg_list[IR1].arch_info)->enable = false;
				((struct nds32_reg *)reg_cache->reg_list[IR9].arch_info)->enable = false;
			}
		}
	} else if (mmu_config->memory_protection == 2) {
		((struct nds32_reg *)reg_cache->reg_list[MR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MR4].arch_info)->enable = true;
	}

	if (mmu_config->memory_protection > 0) {
		((struct nds32_reg *)reg_cache->reg_list[MR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MR3].arch_info)->enable = true;
	}

	if (memory_config->ilm_base != 0)
		if (nds32->privilege_level == 0)
			((struct nds32_reg *)reg_cache->reg_list[MR6].arch_info)->enable = true;

	if (memory_config->dlm_base != 0)
		if (nds32->privilege_level == 0)
			((struct nds32_reg *)reg_cache->reg_list[MR7].arch_info)->enable = true;

	if ((memory_config->icache.line_size != 0) || (memory_config->dcache.line_size != 0))
		((struct nds32_reg *)reg_cache->reg_list[MR8].arch_info)->enable = true;

	if (misc_config->high_speed_memory_port)
		((struct nds32_reg *)reg_cache->reg_list[MR9].arch_info)->enable = true;

	if (mr10_exist)
		((struct nds32_reg *)reg_cache->reg_list[MR10].arch_info)->enable = true;

	if (misc_config->edm) {
		uint32_t dr_reg_n = nds32->edm.breakpoint_num * 5;

		for (i = 0 ; i < dr_reg_n ; i++)
			((struct nds32_reg *)reg_cache->reg_list[DR0 + i].arch_info)->enable = true;

		((struct nds32_reg *)reg_cache->reg_list[DR41].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR43].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR44].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR45].arch_info)->enable = true;
	}

	if (misc_config->debug_tracer) {
		((struct nds32_reg *)reg_cache->reg_list[DR46].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR47].arch_info)->enable = true;
	}

	if (misc_config->performance_monitor) {
		((struct nds32_reg *)reg_cache->reg_list[PFR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR3].arch_info)->enable = true;
	}
	if (misc_config->pft) {
		((struct nds32_reg *)reg_cache->reg_list[PFR4].arch_info)->enable = true;
	}

	if (cpu_version->secure) {
		((struct nds32_reg *)reg_cache->reg_list[SECUR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SECUR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SECUR3].arch_info)->enable = true;
	}

	if (misc_config->msc_ext >= 0x01) {
		((struct nds32_reg *)reg_cache->reg_list[CR7].arch_info)->enable = true;
		nds32_get_mapped_reg(nds32, CR7, &msc_cfg2_value);
		LOG_DEBUG("msc_cfg2_value = %x", msc_cfg2_value);

		/* MSC_CFG2.TLB_ECC = 1 or 2 */
		if (((msc_cfg2_value & 0x03) == 0x01) || ((msc_cfg2_value & 0x03) == 0x02)) {
			((struct nds32_reg *)reg_cache->reg_list[IDR2].arch_info)->enable = true;
		}
		/* MSC_CFG2.ECC = 1 */
		if ((msc_cfg2_value & 0x04) == 0x04 ) {
			((struct nds32_reg *)reg_cache->reg_list[IDR2].arch_info)->enable = true;
		}
		/* MSC_CFG2.INT64 = 1 */
		if ((msc_cfg2_value & 0x40) == 0x40 ) {
			((struct nds32_reg *)reg_cache->reg_list[IR31].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[IR32].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[IR33].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[IR34].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[IR35].arch_info)->enable = true;
		}
		/* MSC_CFG2.EINSN = 1 */
		if ((msc_cfg2_value & 0x80) == 0x80 ) {
			((struct nds32_reg *)reg_cache->reg_list[IR36].arch_info)->enable = true;
		}
	}
	if ((memory_config->icache_ecc == 0x01) ||
		  (memory_config->icache_ecc == 0x02) ||
		  (memory_config->ilm_ecc == 0x01) ||
		  (memory_config->ilm_ecc == 0x02) ||
		  (memory_config->dcache_ecc == 0x01) ||
		  (memory_config->dcache_ecc == 0x02) ||
		  (memory_config->dlm_ecc == 0x01) ||
		  (memory_config->dlm_ecc == 0x02)) {
			((struct nds32_reg *)reg_cache->reg_list[IDR2].arch_info)->enable = true;
	}

	if (misc_config->local_memory_dma) {
		((struct nds32_reg *)reg_cache->reg_list[DMAR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR4].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR8].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR9].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR10].arch_info)->enable = true;
		uint32_t dma_cfg_value = 0;
		nds32_get_mapped_reg(nds32, DMAR0, &dma_cfg_value);
		LOG_DEBUG("dma_cfg_value = %x", dma_cfg_value);
		if ((dma_cfg_value >> 16) >= 0x200) {
			((struct nds32_reg *)reg_cache->reg_list[DMAR11].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[DMAR12].arch_info)->enable = true;
		}
	}

	if ((misc_config->local_memory_dma || misc_config->performance_monitor) &&
			(no_racr0 == false))
		((struct nds32_reg *)reg_cache->reg_list[RACR].arch_info)->enable = true;

	if (cpu_version->cop_fpu_extension || (misc_config->audio_isa != 0))
		((struct nds32_reg *)reg_cache->reg_list[FUCPR].arch_info)->enable = true;

	/* DSP ZOL, (cr7[5](MSC_CFG2.ZOL) == 1) */
	if (msc_cfg2_value & 0x20) {
		((struct nds32_reg *)reg_cache->reg_list[DSP_LB].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DSP_LE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DSP_LC].arch_info)->enable = true;
	}
	else if (misc_config->audio_isa != 0) {
		if (misc_config->audio_isa > 1) {
			((struct nds32_reg *)reg_cache->reg_list[D0L24].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[D1L24].arch_info)->enable = true;
		}

		((struct nds32_reg *)reg_cache->reg_list[I0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I4].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MOD].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LBE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LC].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[ADM_VBASE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SHFT_CTL0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SHFT_CTL1].arch_info)->enable = true;

		uint32_t value_mod;
		uint32_t fucpr_backup;
		/* enable fpu and get configuration */
		nds32_get_mapped_reg(nds32, FUCPR, &fucpr_backup);
		if ((fucpr_backup & 0x80000000) == 0)
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup | 0x80000000);
		nds32_get_mapped_reg(nds32, MOD, &value_mod);
		/* restore origin fucpr value */
		if ((fucpr_backup & 0x80000000) == 0)
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup);

		if ((value_mod >> 6) & 0x1) {
			((struct nds32_reg *)reg_cache->reg_list[CB_CTL].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB0].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB1].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB2].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB3].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE0].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE1].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE2].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE3].arch_info)->enable = true;
		}
	}

	if ((cpu_version->cpu_id_family == 0x09) ||
			(cpu_version->cpu_id_family == 0x19) ||
			(cpu_version->cpu_id_family == 0x0a) ||
			(cpu_version->cpu_id_family == 0x1a) ||
			(cpu_version->cpu_id_family == 0x0c)) {
		// some n13 netlist or TC01 platform(CONFIG_CPU_VER_ = 0x0C0C001F) do not have idr0.
		//if (CONFIG_CPU_VER_ != 0x0C0C001F)
			((struct nds32_reg *)reg_cache->reg_list[IDR0].arch_info)->enable = true;
	}
	((struct nds32_reg *)reg_cache->reg_list[IDR1].arch_info)->enable = true;

	if (misc_config->hsp) {
		((struct nds32_reg *)reg_cache->reg_list[HSPR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[HSPR1].arch_info)->enable = true;
		if (misc_config->shadow == 1 ) {
			((struct nds32_reg *)reg_cache->reg_list[HSPR2].arch_info)->enable = true;
		}
		uint32_t hspr0_value = 0;
		nds32_get_mapped_reg(nds32, HSPR0, &hspr0_value);
		/* HSPR3 exist if HSP_CTL.UDF==1 (bit 7 of hspr0) */
		if (hspr0_value & 0x80) {
			((struct nds32_reg *)reg_cache->reg_list[HSPR3].arch_info)->enable = true;
		}
		uint32_t idr1_value = 0;
		nds32_get_mapped_reg(nds32, IDR1, &idr1_value);
		/* HSPR4 exist if MISC_CTL.SP_SHADOW_EN==1 (bit 4 of idr1) */
		if (idr1_value & 0x10) {
			((struct nds32_reg *)reg_cache->reg_list[HSPR4].arch_info)->enable = true;
		}
	}

	uint32_t ir3_value;
	uint32_t ivb_prog_pri_lvl;
	uint32_t ivb_ivic_ver;

	nds32_get_mapped_reg(nds32, IR3, &ir3_value);
	ivb_prog_pri_lvl = ir3_value & 0x1;
	ivb_ivic_ver = (ir3_value >> 11) & 0x3;
	nds32_curr_ir3_value = ir3_value;

	if ((ivb_prog_pri_lvl == 1) || (ivb_ivic_ver >= 1)) {
		((struct nds32_reg *)reg_cache->reg_list[IR18].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR19].arch_info)->enable = true;
	}

	if (ivb_ivic_ver >= 1) {
		((struct nds32_reg *)reg_cache->reg_list[IR26].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR27].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR28].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR29].arch_info)->enable = true;
	}
#if 0
	/* IVB.NIVIC = 6, 64 interrupt input sources */
	if (((ir3_value & 0x0E) >> 1) == 6) {
		((struct nds32_reg *)reg_cache->reg_list[IR31].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR32].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR33].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR34].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR35].arch_info)->enable = true;
	}
#endif
	return ERROR_OK;
}

int nds32_init_register_table(struct nds32 *nds32)
{
	nds32_init_must_have_registers(nds32);

	return ERROR_OK;
}

int nds32_add_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	uint32_t data;
	uint32_t check_data;
	uint32_t break_insn;
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	/* switch to BUS access mode to write memory,
		 fix bug-9828 TARGET WARNING! Exception ..., when write data into ROM via CPU mode  */
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	LOG_DEBUG("access_channel: NDS_MEMORY_ACC_BUS");

	/* Because PSW.IT is turned off under debug exception, address MUST
	 * be physical address.  L1I_VA_INVALIDATE uses PSW.IT to decide
	 * address translation or not. */
	/* already do virt2phys() in read_buffer()
	uint32_t physical_addr;
	if (ERROR_FAIL == target->type->virt2phys(target, breakpoint->address,
				&physical_addr))
		return ERROR_FAIL;
	*/
	uint32_t physical_addr = breakpoint->address;
	uint32_t sw_breakpoint_length;
#if 0
	/* check the breakpoint size */
	target->type->read_buffer(target, physical_addr, 4, (uint8_t *)&data);

	/* backup origin instruction
	 * instruction is big-endian */
	if (*(char *)&data & 0x80) { /* 16-bits instruction */
		sw_breakpoint_length = 2;
		break_insn = NDS32_BREAK_16;
	} else { /* 32-bits instruction */
		sw_breakpoint_length = 4;
		break_insn = NDS32_BREAK_32;
	}
#else
	int result = target->type->read_buffer(target, physical_addr, 2, (uint8_t *)&data);
	/* just return ERROR_FAIL if read_buffer ERROR_FAIL, (virt2phys FAIL)
	   auto convert to hardware breakpoint  */
	if (result != ERROR_OK)
		return ERROR_FAIL;
	sw_breakpoint_length = 2;
	break_insn = NDS32_BREAK_16;
#endif

	if (breakpoint->orig_instr != NULL)
		free(breakpoint->orig_instr);

	breakpoint->orig_instr = malloc(sw_breakpoint_length);
	memcpy(breakpoint->orig_instr, &data, sw_breakpoint_length);
	LOG_DEBUG("breakpoint->orig_instr: %x", data);

	/* write_back & invalidate dcache & invalidate icache */
	nds32_cache_sync(target, physical_addr, sw_breakpoint_length);

	/* self-modified code */
	target->type->write_buffer(target, physical_addr, sw_breakpoint_length, (const uint8_t *)&break_insn);

	/* read back to check */
	target->type->read_buffer(target, physical_addr, sw_breakpoint_length, (uint8_t *)&check_data);

	/* restore to origin access mode */
	nds32->memory.access_channel = orig_channel;

	if (memcmp(&check_data, &break_insn, sw_breakpoint_length) == 0)
		return ERROR_OK;

	return ERROR_FAIL;
}

int nds32_remove_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	uint32_t check_data;
	uint32_t break_insn;
	struct nds32 *nds32 = target_to_nds32(target);
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	/* switch to BUS access mode to write memory,
		 fix bug-9828 TARGET WARNING! Exception ..., when write data into ROM via CPU mode  */
	nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
	LOG_DEBUG("access_channel: NDS_MEMORY_ACC_BUS");

	uint32_t sw_breakpoint_length = 2;
	if (sw_breakpoint_length == 2)
		break_insn = NDS32_BREAK_16;
	else if (sw_breakpoint_length == 4)
		break_insn = NDS32_BREAK_32;
	else
		return ERROR_FAIL;

	/* Because PSW.IT is turned off under debug exception, address MUST
	 * be physical address.  L1I_VA_INVALIDATE uses PSW.IT to decide
	 * address translation or not. */
	/* already do virt2phys() in read_buffer()
	uint32_t physical_addr;
	if (ERROR_FAIL == target->type->virt2phys(target, breakpoint->address,
				&physical_addr))
		return ERROR_FAIL;
	*/
	uint32_t physical_addr = breakpoint->address;

	int result = target->type->read_buffer(target, physical_addr, sw_breakpoint_length,
			(uint8_t *)&check_data);
	if (result != ERROR_OK)
		return ERROR_FAIL;

	/* break instruction is modified */
	if (memcmp(&check_data, &break_insn, sw_breakpoint_length) != 0)
		return ERROR_FAIL;

	/* write_back & invalidate dcache & invalidate icache */
	nds32_cache_sync(target, physical_addr, sw_breakpoint_length);

	/* self-modified code */
	target->type->write_buffer(target, physical_addr, sw_breakpoint_length,
			breakpoint->orig_instr);

	/* restore to origin access mode */
	nds32->memory.access_channel = orig_channel;
	return ERROR_OK;
}

/**
 * Restore the processor context on an Andes target.  The full processor
 * context is analyzed to see if any of the registers are dirty on this end, but
 * have a valid new value.  If this is the case, the processor is changed to the
 * appropriate mode and the new register values are written out to the
 * processor.  If there happens to be a dirty register with an invalid value, an
 * error will be logged.
 *
 * @param target Pointer to the Andes target to have its context restored
 * @return Error status if the target is not halted.
 */
int nds32_restore_context(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *reg;
	struct nds32_reg *reg_arch_info;
	unsigned int i;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* check if there are dirty registers */
	for (i = 0; i < reg_cache->num_regs; i++) {
		reg = &(reg_cache->reg_list[i]);
		if (reg->dirty == true) {
			if (reg->valid == true) {

				LOG_DEBUG("examining dirty reg: %s", reg->name);
				LOG_DEBUG("writing register %i "
						"with value 0x%8.8" PRIx32, i, buf_get_u32(reg->value, 0, 32));

				reg_arch_info = reg->arch_info;
				if (FD0 <= reg_arch_info->num && reg_arch_info->num <= FD31)
					aice_write_reg_64(target, reg_arch_info->num, reg_arch_info->value_64);
				else
					aice_write_register(target, reg_arch_info->num, reg_arch_info->value);
				reg->valid = true;
				reg->dirty = false;
			}
		}
	}
	return ERROR_OK;
}

extern int nds32_update_edm_config(struct nds32 *nds32);
int nds32_edm_config(struct nds32 *nds32)
{
	struct target *target = nds32->target;

	aice_edm_config(target);
	nds32_update_edm_config(nds32);
	/* set passcode for secure MCU */
	nds32_login(nds32);

	return ERROR_OK;
}

int nds32_config(struct nds32 *nds32)
{
	if (aice_usb_pack_command == 2) {
		/* without DBGER.DPED checking */
		uint32_t value_reg[20], reg_id[20], reg_nums = 6;
		reg_id[0] = CR0;
		reg_id[1] = CR1;
		reg_id[2] = CR2;
		reg_id[3] = CR3;
		reg_id[4] = CR4;
		reg_id[5] = IR0;
		nds32_pack_read_reg_into_cache(nds32, reg_nums,
			&reg_id[0], &value_reg[0]);
	}

	nds32_init_config(nds32);

	/* init optional system registers according to config registers */
	nds32_init_option_registers(nds32);

	/* get max interrupt level */
	if (nds32->misc_config.interruption_level)
		nds32->max_interrupt_level = 2;
	else
		nds32->max_interrupt_level = 3;

	/* get ILM/DLM size from MR6/MR7 */
	uint32_t value_mr6, value_mr7;
	uint32_t size_index;
	nds32_get_mapped_reg(nds32, MR6, &value_mr6);
	size_index = (value_mr6 >> 1) & 0xF;
	nds32->memory.ilm_size = NDS32_LM_SIZE_TABLE[size_index];

	nds32_get_mapped_reg(nds32, MR7, &value_mr7);
	size_index = (value_mr7 >> 1) & 0xF;
	nds32->memory.dlm_size = NDS32_LM_SIZE_TABLE[size_index];

	return ERROR_OK;
}

int nds32_init_arch_info(struct target *target, struct nds32 *nds32)
{
	target->arch_info = nds32;
	nds32->target = target;

	nds32->common_magic = NDS32_COMMON_MAGIC;
	nds32->init_arch_info_after_halted = false;
	nds32->auto_convert_hw_bp = true;
	nds32->global_stop = false;
	nds32->soft_reset_halt = false;
	nds32->edm_passcode = nds32_edm_passcode_init;
	nds32->privilege_level = 0;
	nds32->boot_time = 1500;
	nds32->reset_time = 1000;
	nds32->reset_halt_as_examine = false;
	nds32->keep_target_edm_ctl = false;
	nds32->word_access_mem = false;
	nds32->hit_syscall = false;
	nds32->active_syscall_id = NDS32_SYSCALL_UNDEFINED;
	nds32->virtual_hosting_errno = 0;
	nds32->virtual_hosting_ctrl_c = false;
	nds32->attached = false;
	nds32->aceconf = target->variant;

	nds32_reg_init(nds32);

	if (ERROR_FAIL == nds32_reg_cache_init(target, nds32))
		return ERROR_FAIL;

	if (ERROR_OK != nds32_init_register_table(nds32))
		return ERROR_FAIL;

	return ERROR_OK;
}

int nds32_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical)
{
	struct nds32 *nds32 = target_to_nds32(target);

#if ENABLE_DEX_USE_PSW
	nds32_use_psw_mode = 0;
#endif

	if ((nds32->memory.address_translation == false) ||
		(nds32->memory.va_to_pa_off == 1)) {
		*physical = address;
		LOG_DEBUG("va_to_pa_off:%d, address_translation:%d", nds32->memory.va_to_pa_off, nds32->memory.address_translation);
		return ERROR_OK;
	}

#if ENABLE_DEX_USE_PSW
	uint32_t edm_ctl = 0;
	uint32_t value_ir0 = nds32_backup_psw;
	/* DEX_USE_PSW enable */
	LOG_DEBUG("value_ir0=%x, access_channel=%x", value_ir0, nds32->memory.access_channel);

	if ((nds32->memory.access_channel == NDS_MEMORY_ACC_CPU) &&
		(nds32->memory.dcache.enable == true) && // if dcache disable, auto switch to BUS bulk mode later
		(nds32->memory.address_translation == true)) {
		value_ir0 &= ~(0x41); /* disable GIE & IT, enable DT: Data address translation*/
		value_ir0 |= 0x08;  /* POM[4-3] 1->Superuser */

		aice_write_register(target, IR0, value_ir0); /* write real IR0, not mapped */
		aice_read_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		aice_write_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, edm_ctl | NDS_EDMCTL_DEX_USE_PSW);
		nds32_use_psw_mode = 1;
		*physical = address;
		LOG_DEBUG("nds32_use_psw_mode:%d, addr: 0x%" TARGET_PRIxADDR, nds32_use_psw_mode, *physical);
		return ERROR_OK;
	}
#endif

	if (ERROR_OK == aice_read_tlb(target, address, (uint32_t *)physical))
		return ERROR_OK;

	if (ERROR_OK == nds32_walk_page_table(nds32, address, physical))
		return ERROR_OK;

	return ERROR_FAIL;
}

int nds32_cache_sync(struct target *target, target_addr_t address, uint32_t length)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	struct nds32_cache *icache = &(nds32->memory.icache);
	uint32_t dcache_line_size = NDS32_LINE_SIZE_TABLE[dcache->line_size];
	uint32_t icache_line_size = NDS32_LINE_SIZE_TABLE[icache->line_size];
	uint32_t cur_address;
	int result;
	uint32_t start_line, end_line;
	uint32_t cur_line;

#if 0
	if ((dcache->line_size != 0) && (dcache->enable == true)) {
		/* address / dcache_line_size */
		start_line = address >> (dcache->line_size + 2);
		/* (address + length - 1) / dcache_line_size */
		end_line = (address + length - 1) >> (dcache->line_size + 2);

		for (cur_address = address, cur_line = start_line ;
				cur_line <= end_line ;
				cur_address += dcache_line_size, cur_line++) {

			/* D$ write back */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_VA_WB, cur_address);
			if (result != ERROR_OK)
				return result;

			/* D$ invalidate */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_VA_INVAL, cur_address);
			if (result != ERROR_OK)
				return result;
		}
	}

	if ((icache->line_size != 0) && (icache->enable == true)) {
		/*  address / icache_line_size */
		start_line = address >> (icache->line_size + 2);
		/* (address + length - 1) / icache_line_size */
		end_line = (address + length - 1) >> (icache->line_size + 2);

		for (cur_address = address, cur_line = start_line ;
				cur_line <= end_line ;
				cur_address += icache_line_size, cur_line++) {
			/* Because PSW.IT is turned off under debug exception, address MUST
			 * be physical address.  L1I_VA_INVALIDATE uses PSW.IT to decide
			 * address translation or not. */
			target_addr_t physical_addr;
			if (ERROR_FAIL == target->type->virt2phys(target, cur_address,
						&physical_addr))
				return ERROR_FAIL;

			/* I$ invalidate */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_L1I_VA_INVAL, physical_addr);
			if (result != ERROR_OK)
				return result;
		}
	}
#endif
#if ENABLE_DEX_USE_PSW
	uint32_t edm_ctl = 0;
	uint32_t curr_use_psw_mode = 0;
	uint32_t value_ir0 = nds32_backup_psw;
	/* DEX_USE_PSW enable */
	LOG_DEBUG("value_ir0=%x, access_channel=%x", value_ir0, nds32->memory.access_channel);

	if ((nds32->memory.address_translation == true) &&
		  (nds32->mmu_config.memory_protection == 2)) {
		value_ir0 &= ~(0x01); /* disable GIE, enable DT & IT */
		value_ir0 |= 0x08;  /* POM[4-3] 1->Superuser */

		aice_write_register(target, IR0, value_ir0); /* write real IR0, not mapped */
		aice_read_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		aice_write_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, edm_ctl | NDS_EDMCTL_DEX_USE_PSW);
		curr_use_psw_mode = 1;
		LOG_DEBUG("curr_use_psw_mode:%d, addr: 0x%"TARGET_PRIxADDR, curr_use_psw_mode, address);
	}
#endif

	uint32_t cache_line_size;
	uint32_t line_size_index;
	if (dcache_line_size >= icache_line_size) {
		cache_line_size = icache_line_size;
		line_size_index = icache->line_size;
	}
	else { //if (dcache_line_size < icache_line_size)
		cache_line_size = dcache_line_size;
		line_size_index = dcache->line_size;
	}
	if (line_size_index == 0)
		cache_line_size = 4;

	/* (1). ISYNC = D-cache write_back & I-cache invalidate */
	start_line = address >> (line_size_index + 2);
	end_line = (address + length - 1) >> (line_size_index + 2);
	for (cur_address = address, cur_line = start_line ;
			cur_line <= end_line ;
			cur_address += cache_line_size, cur_line++) {
		/* Fix Bug 9979 - invalid loop cache when disable or delete breakpoint.
		 * ISYNC (Instruction Data Coherence Synchronization), the address is don't care */
		result = aice_cache_ctl(target, AICE_CACHE_CTL_LOOPCACHE_ISYNC, cur_address);
		if (result != ERROR_OK)
			return result;
	}

	if (nds32->memory.address_translation == true) {
		if ((icache->line_size != 0) && (icache->enable == true)) {
			/*  address / icache_line_size */
			start_line = address >> (icache->line_size + 2);
			/* (address + length - 1) / icache_line_size */
			end_line = (address + length - 1) >> (icache->line_size + 2);

			for (cur_address = address, cur_line = start_line ;
					cur_line <= end_line ;
					cur_address += icache_line_size, cur_line++) {
				/* Because PSW.IT is turned off under debug exception, address MUST
				 * be physical address.  L1I_VA_INVALIDATE uses PSW.IT to decide
				 * address translation or not. */

				/* enable PSW.IT & use_psw_mode to do L1I_VA_INVAL */
				//uint32_t physical_addr;
				//if (ERROR_FAIL == target->type->virt2phys(target, cur_address,
				//			&physical_addr))
				//	return ERROR_FAIL;
				LOG_DEBUG("AICE_CACHE_CTL_L1I_IX_INVAL cur_address=%x", cur_address);
				/* I$ invalidate */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1I_IX_INVAL, cur_address);
				if (result != ERROR_OK)
					return result;
			}
		}
	}

	/* (2). D-cache invalidate */
	if ((dcache->line_size != 0) && (dcache->enable == true)) {
		start_line = address >> (dcache->line_size + 2);
		end_line = (address + length - 1) >> (dcache->line_size + 2);
		for (cur_address = address, cur_line = start_line ;
				cur_line <= end_line ;
				cur_address += dcache_line_size, cur_line++) {

			/* D$ write back */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_VA_WB, cur_address);
			if (result != ERROR_OK)
				return result;
			/* D$ invalidate */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_VA_INVAL, cur_address);
			if (result != ERROR_OK)
				return result;
		}
	}
#if ENABLE_DEX_USE_PSW
	if (curr_use_psw_mode == 1) {
		aice_read_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl &= ~NDS_EDMCTL_DEX_USE_PSW;
		aice_write_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, edm_ctl);
		aice_write_register(nds32->target, IR0, nds32_backup_psw);
	}
#endif
	return ERROR_OK;
}

uint32_t nds32_nextpc(struct nds32 *nds32, int current, uint32_t address)
{
	if (!current)
		nds32_set_mapped_reg(nds32, PC, address);
	else
		nds32_get_mapped_reg(nds32, PC, &address);

	return address;
}

int nds32_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct nds32 *nds32 = target_to_nds32(target);

	address = nds32_nextpc(nds32, current, address);

	LOG_DEBUG("STEP PC %08" TARGET_PRIxADDR "%s", address, !current ? "!" : "");

	/* check hit_syscall before leave_debug_state() because
	 * leave_debug_state() may clear hit_syscall flag */
	bool no_step = false;
	if (nds32->hit_syscall)
		/* step after hit_syscall should be ignored because
		 * leave_debug_state will step implicitly to skip the
		 * syscall */
		no_step = true;

	/********* TODO: maybe create another function to handle this part */
	CHECK_RETVAL(nds32->leave_debug_state(nds32, true));
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));

	if (no_step == false) {
		if (ERROR_OK != aice_step(target))
			return ERROR_FAIL;
	}

	/* save state */
	CHECK_RETVAL(nds32->enter_debug_state(nds32, true));
	/********* TODO: maybe create another function to handle this part */

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static int nds32_step_without_watchpoint(struct nds32 *nds32)
{
	struct target *target = nds32->target;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/********* TODO: maybe create another function to handle this part */
	CHECK_RETVAL(nds32->leave_debug_state(nds32, false));
	if (ERROR_OK != aice_step(target))
		return ERROR_FAIL;

	/* save state */
	CHECK_RETVAL(nds32->enter_debug_state(nds32, false));
	/********* TODO: maybe create another function to handle this part */

	return ERROR_OK;
}

int nds32_target_state(struct nds32 *nds32, enum target_state *state)
{
	enum aice_target_state_s nds32_state;

	if ((RUN_MODE_PROFILE & nds32->gdb_run_mode) && (true == nds32->gdb_run_mode_acting)) {
		if (aice_profile_state(nds32->target, &nds32_state) != ERROR_OK)
			return ERROR_FAIL;
	}
	if ((RUN_MODE_PWR_MONITOR & nds32->gdb_run_mode) && (true == nds32->gdb_pwr_mode_acting)) {
		if (nds32_pwr_state(nds32->target, &nds32_state) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		if (aice_state(nds32->target, &nds32_state) != ERROR_OK)
			return ERROR_FAIL;
	}

	switch (nds32_state) {
		case AICE_DISCONNECT:
			LOG_INFO("USB is disconnected");
			return ERROR_FAIL;
		case AICE_TARGET_DETACH:
			LOG_INFO("Target is disconnected");
			return ERROR_FAIL;
		case AICE_TARGET_UNKNOWN:
			*state = TARGET_UNKNOWN;
			break;
		case AICE_TARGET_RUNNING:
			*state = TARGET_RUNNING;
			break;
		case AICE_TARGET_HALTED:
			*state = TARGET_HALTED;
			break;
		case AICE_TARGET_RESET:
			*state = TARGET_RESET;
			break;
		case AICE_TARGET_DEBUG_RUNNING:
			*state = TARGET_DEBUG_RUNNING;
			break;
		default:
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

int nds32_check_break_condition(struct nds32 *nds32)
{
	LOG_DEBUG(" %s", __func__);
	struct target *target = nds32->target;
	struct breakpoint *bp = target->breakpoints;
	if (bp == NULL)
		return ERROR_OK;
	if (nds32_bytecode_parsing == 0)
		return ERROR_OK;

	uint32_t val_pc = 0, if_condition_bp = 0, hit_condition = 0;
	uint32_t break_start, break_end;
	nds32_get_mapped_reg(nds32, PC, &val_pc);

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->length & BP_WP_CONDITIONAL) {
			break_start = bp->address;
			break_end = bp->address + (bp->length & BP_WP_LENGTH_MASK);
			if ((val_pc >= break_start) && (val_pc <= break_end)) {
				LOG_DEBUG(" %s", bp->bytecode);
				hit_condition = gdb_bytecode_parsing(target, (char *)bp->bytecode);
				if_condition_bp = 1;
				break;
			}
		}
	}

#if 0
	uint32_t test_memory_val = 0;
	uint32_t physical_addr = 0x0077fff4;
	target->type->read_buffer(target, physical_addr, 4, (uint8_t *)&test_memory_val);
	if (test_memory_val == 50) {
		hit_condition = 1;
	}
	if_condition_bp = 1;
#endif

	if ((if_condition_bp == 1) && (hit_condition == 0)) {
		/* do single step */
		aice_step(target);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int nds32_examine_debug_reason(struct nds32 *nds32)
{
	uint32_t reason;
	struct target *target = nds32->target;

	if (nds32->hit_syscall == true) {
		LOG_DEBUG("Hit syscall breakpoint");
		// target->debug_reason = DBG_REASON_BREAKPOINT;
		// return ERROR_OK;
	}

	nds32->get_debug_reason(nds32, &reason);
	LOG_DEBUG("nds32 examines debug reason: %s", nds32_debug_type_name[reason]);

	/* Examine debug reason */
	switch (reason) {
		case NDS32_DEBUG_BREAK:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case NDS32_DEBUG_BREAK_16:
		case NDS32_DEBUG_INST_BREAK:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			return nds32_check_break_condition(nds32);
			break;
		case NDS32_DEBUG_DATA_ADDR_WATCHPOINT_PRECISE:
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_PRECISE:
		case NDS32_DEBUG_LOAD_STORE_GLOBAL_STOP: /* GLOBAL_STOP is precise exception */
			{
				int result;
				nds32->hit_user_def_wp = false;
				result = nds32->get_watched_address(nds32,
						&(nds32->watched_address), reason);
				/* do single step(without watchpoints) to skip the "watched" instruction */
				nds32_step_without_watchpoint(nds32);

				/* before single_step, save exception address */
				if (ERROR_OK != result)
					return ERROR_FAIL;

				target->debug_reason = DBG_REASON_WATCHPOINT;
				/* for monitor command set_watch_value, NOT gdb watch/rwatch */
				if (reason == NDS32_DEBUG_DATA_VALUE_WATCHPOINT_PRECISE)
					target->debug_reason = DBG_REASON_BREAKPOINT;
				if (nds32->hit_user_def_wp == true)
					target->debug_reason = DBG_REASON_HIT_MONITOR_WATCH;
			}
			break;
		case NDS32_DEBUG_DEBUG_INTERRUPT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case NDS32_DEBUG_HARDWARE_SINGLE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_IMPRECISE:
		case NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE:
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE:
			nds32->hit_user_def_wp = false;
			if (ERROR_OK != nds32->get_watched_address(nds32,
						&(nds32->watched_address), reason))
				return ERROR_FAIL;

			target->debug_reason = DBG_REASON_WATCHPOINT;
			/* for monitor command set_watch_value, NOT gdb watch/rwatch */
			if ((reason == NDS32_DEBUG_DATA_VALUE_WATCHPOINT_IMPRECISE) ||
					(reason == NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE))
					target->debug_reason = DBG_REASON_BREAKPOINT;

			if (nds32->hit_user_def_wp == true)
				target->debug_reason = DBG_REASON_HIT_MONITOR_WATCH;
			break;
		default:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
	}
#if _NDS32_ONLY_
	if (nds32->tracer_pause == true) {
		target->debug_reason = DBG_REASON_TRACE_BUFFULL;
		nds32->tracer_pause = false;
	}
#endif
	LOG_DEBUG("target->debug_reason = %d\n", target->debug_reason);

	return ERROR_OK;
}

int nds32_login(struct nds32 *nds32)
{
	struct target *target = nds32->target;
	uint32_t value_edmsw=0;

	LOG_DEBUG("nds32_login");
	aice_set_edm_passcode(target, nds32->edm_passcode);
	/* get current privilege level */
	aice_read_debug_reg(target, NDS_EDM_SR_EDMSW, &value_edmsw);
	nds32->privilege_level = (value_edmsw >> 16) & 0x3;
	LOG_INFO("edm_passcode =>Current privilege level: %d", nds32->privilege_level);
	return ERROR_OK;
}

static int nds32_halt_profile(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	nds32->gdb_run_mode_halt = true;

	enum target_state state;
	do {
		ASSERTOK(nds32_target_state(nds32, &state));
	} while(TARGET_HALTED != state);

	nds32->gdb_run_mode_halt = false;

	return ERROR_OK;
}

int nds32_halt(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum target_state state;

	if ((RUN_MODE_PROFILE & nds32->gdb_run_mode) && (true == nds32->gdb_run_mode_acting))
		nds32_halt_profile(target);

	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (nds32_target_state(nds32, &state) != ERROR_OK)
		return ERROR_FAIL;

	if (TARGET_HALTED != state)
		/* TODO: if state == TARGET_HALTED, check ETYPE is DBGI or not */
		if (ERROR_OK != aice_halt_target(target))
			return ERROR_FAIL;

	CHECK_RETVAL(nds32->enter_debug_state(nds32, true));

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	if ((RUN_MODE_PROFILE & nds32->gdb_run_mode) && (true == nds32->gdb_run_mode_acting))
		aice_profile_post(target);
	if ((RUN_MODE_PWR_MONITOR & nds32->gdb_run_mode) && (true == nds32->gdb_pwr_mode_acting))
		nds32_pwr_post(target);

	return ERROR_OK;
}

extern int nds32_tracer_polling(struct target *target);
extern uint32_t nds_skip_dmi;
/* poll current target status */
int nds32_poll(struct target *target)
{
	if (nds_skip_dmi == 1)
		return ERROR_OK;

	struct nds32 *nds32 = target_to_nds32(target);
	enum target_state state;

	nds32_tracer_polling(target);
	if (nds32_target_state(nds32, &state) != ERROR_OK)
		return ERROR_FAIL;

	if (state == TARGET_HALTED) {
		if (target->state != TARGET_HALTED) {
			/* if false_hit, continue free_run */
			if (ERROR_OK != nds32->enter_debug_state(nds32, true)) {
				aice_run_target(target);
				return ERROR_OK;
			}

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	} else if (state == TARGET_RESET) {
		if (target->state == TARGET_HALTED) {
			/* similar to assert srst */
			register_cache_invalidate(nds32->core_cache);
			target->state = TARGET_RESET;

			/* TODO: deassert srst */
		} else if (target->state == TARGET_RUNNING) {
			/* reset as running */
			NDS32_LOG(NDS32_ERRMSG_TARGET_RESET);
		}
	} else {
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}

	if ((RUN_MODE_PROFILE & nds32->gdb_run_mode) && (true == nds32->gdb_run_mode_acting))
		aice_profile_post(target);
	if ((RUN_MODE_PWR_MONITOR & nds32->gdb_run_mode) && (true == nds32->gdb_pwr_mode_acting))
		nds32_pwr_post(target);

	return ERROR_OK;
}

int nds32_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("current %d address %08" TARGET_PRIxADDR
			" handle_breakpoints %d"
			" debug_execution %d",
			current, address, handle_breakpoints, debug_execution);

	struct nds32 *nds32 = target_to_nds32(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted @ %s", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	address = nds32_nextpc(nds32, current, address);

	LOG_DEBUG("RESUME PC %08" TARGET_PRIxADDR "%s", address, !current ? "!" : "");

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* Disable HSS to avoid users misuse HSS */
	if (nds32_reach_max_interrupt_level(nds32) == false) {
		uint32_t value_ir0;
		nds32_get_mapped_reg(nds32, IR0, &value_ir0);
		value_ir0 &= ~(0x1 << 11);
		nds32_set_mapped_reg(nds32, IR0, value_ir0);

		/*
		 * If INT_MASK.DSSIM = 1, the "stepi" command
		 * will step into exception handler if exception occur.
		 * In this case, IPSW.HSS or P_IPSW.HSS should be cleared
		 * if "continue" command is issued.
		 */
		if(nds32->current_interrupt_level > 0){
			uint32_t value_ir1;
			nds32_get_mapped_reg(nds32, IR1, &value_ir1);
			value_ir1 &= ~(0x1 << 11);
			nds32_set_mapped_reg(nds32, IR1, value_ir1);
		}
		if(nds32->current_interrupt_level > 1){
			uint32_t value_ir2;
			nds32_get_mapped_reg(nds32, IR2, &value_ir2);
			value_ir2 &= ~(0x1 << 11);
			nds32_set_mapped_reg(nds32, IR2, value_ir2);
		}
	}

	CHECK_RETVAL(nds32->leave_debug_state(nds32, true));
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));

	if (nds32->virtual_hosting_ctrl_c == false) {
		aice_run_target(target);
	} else
		nds32->virtual_hosting_ctrl_c = false;

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;

	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	return ERROR_OK;
}

static int nds32_soft_reset_halt(struct target *target)
{
	/* TODO: test it */
	struct nds32 *nds32 = target_to_nds32(target);
	aice_reset_target(target, AICE_SRST);

	/* halt core and set pc to 0x0 */
	int retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	/* start fetching from IVB */
	/* reset registers, bug-10621*/
	uint32_t ir3_IVB = 0;
	uint32_t cr3_MMU_CFG = 0;
	uint32_t cr4_MSC_CFG = 0;

	nds32_get_mapped_reg(nds32, IR3, &ir3_IVB);
	nds32_get_mapped_reg(nds32, CR3, &cr3_MMU_CFG);
	nds32_get_mapped_reg(nds32, CR4, &cr4_MSC_CFG);

	// IPC (ir9)=IVB
	nds32_set_mapped_reg(nds32, IR9, ir3_IVB & 0xFFFF0000);
	// OIPC (ir11)=IVB
	nds32_set_mapped_reg(nds32, PC, ir3_IVB & 0xFFFF0000);

	// PSW (ir0)=PSW_reset_value
	uint32_t PSW_reset_value = 0x0000040a;
	if (cr3_MMU_CFG & 0x80000000)
		PSW_reset_value |= 0x1000;
	if (cr3_MMU_CFG & 0x04000000)
		PSW_reset_value |= 0x20;
	if (((cr4_MSC_CFG & 0xE000)>>13) >= 2)
		PSW_reset_value |= 0x70000;

	nds32_set_mapped_reg(nds32, IR0, PSW_reset_value);

	// IPSW (ir1)=IPSW_reset_value
	PSW_reset_value &= (~0x0400);
	nds32_set_mapped_reg(nds32, IR1, PSW_reset_value);

	// ITYPE (ir6)=0
	nds32_set_mapped_reg(nds32, IR6, 0);
	// check MSC_CFG[12] to determine if P_ITYPE exists, P_ITYPE (ir7)=0
	if (((cr4_MSC_CFG & 0x1000)>>12) != 1) {
			nds32_set_mapped_reg(nds32, IR7, 0);
  }
	// INT_PEND (ir15)=0
	nds32_set_mapped_reg(nds32, IR15, 0);
	// check IVB[12:11] to determine if INT_PEND2 exists, INT_PEND2 (ir27)=0
	if (((ir3_IVB & 0x1800) >> 11) >= 1) {
		nds32_set_mapped_reg(nds32, IR27, 0);
	}

	return ERROR_OK;
}

int nds32_assert_reset(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);

	LOG_DEBUG("nds32_assert_reset %x, %x, %x", nds32->edm.version, cpu_version->revision, cpu_version->cpu_id_family);
	if (target->reset_halt) {
		#if 0
		if ((nds32->soft_reset_halt)
			|| (nds32->edm.version < 0x51)
			|| ((nds32->edm.version == 0x51)
				&& (cpu_version->revision == 0x1C)
				&& (cpu_version->cpu_id_family == 0xC)
				&& (cpu_version->cpu_id_version == 0x0)))
		#endif
		if (nds32->soft_reset_halt)
			nds32_soft_reset_halt(target);
		else
			aice_reset_target(target, AICE_RESET_HOLD);
		alive_sleep(nds32->reset_time);
	} else {
		aice_reset_target(target, AICE_SRST);
		alive_sleep(nds32->boot_time);
	}

	/* set passcode for secure MCU after core reset */
	nds32_login(nds32);

	/* registers are now invalid */
	register_cache_invalidate(nds32->core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

#define FILE_BIT_FIELD_V3      "nds32_tdesc_bitfield.xml"
extern char *gpBitFieldFileName;
extern int nds32_tracer_check(struct target *target);
extern int nds32_tracer_disable(struct target *target);
static int nds32_gdb_attach(struct nds32 *nds32)
{
	LOG_DEBUG("nds32_gdb_attach, target coreid: %d", nds32->target->coreid);
	gpBitFieldFileName = (char *)FILE_BIT_FIELD_V3;
	LOG_DEBUG("gpBitFieldFileName: %s", gpBitFieldFileName);
	if (nds32->attached == false) {
		nds32_edm_config(nds32);

		if (nds32->keep_target_edm_ctl) {
			/* backup target EDM_CTL */
			aice_read_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, &nds32->backup_edm_ctl);
		}
		if (nds32_tracer_check(nds32->target) == ERROR_OK) {
				LOG_DEBUG("nds32_tracer_disable !!");
				nds32_tracer_disable(nds32->target);
		}
		target_halt(nds32->target);

		nds32->attached = true;
		nds32->hit_syscall = false;
		//reset to debug mode in case abnormal operations
		nds32->gdb_run_mode = RUN_MODE_DEBUG;
		//nds32->target->is_program_exit = false;
		//LOG_OUTPUT("%s: target->is_program_exit = %d\n", __func__, nds32->target->is_program_exit);
	}

	return ERROR_OK;
}

static int nds32_gdb_detach(struct nds32 *nds32)
{
	LOG_DEBUG("nds32_gdb_detach");

	if (nds32->attached) {
		if (nds32_tracer_check(nds32->target) == ERROR_OK) {
				LOG_DEBUG("nds32_tracer_disable !!");
				nds32_tracer_disable(nds32->target);
		}
		//free run in debug mode
		//NOTE: might exit soon => "gdb_last_signal()" complains.
		enum target_run_mode run_mode_backup = nds32->gdb_run_mode;
		nds32->gdb_run_mode = RUN_MODE_DEBUG;
		// clear all breakpoints & watchpoints
		breakpoint_clear_target(nds32->target);
		watchpoint_clear_target(nds32->target);

		target_resume(nds32->target, 1, 0, 0, 0);

		if (nds32->keep_target_edm_ctl) {
			/* restore target EDM_CTL */
			aice_write_debug_reg(nds32->target, NDS_EDM_SR_EDM_CTL, nds32->backup_edm_ctl);
		}

		nds32->attached = false;

		//restore run mode settings
		nds32->gdb_run_mode = run_mode_backup;
	}

	return ERROR_OK;
}

int nds32_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	int retval = ERROR_OK;
	int target_number = *(int *)priv;
	LOG_DEBUG("nds32_callback_event_handler: target_number=%d, target->target_number=%d, event=0x%08x",
		target_number, target->target_number, event);
	if (target_number != target->target_number)
		return ERROR_OK;

	struct nds32 *nds32 = target_to_nds32(target);

	switch (event) {
		case TARGET_EVENT_GDB_ATTACH:
			retval = nds32_gdb_attach(nds32);
			break;
		case TARGET_EVENT_GDB_DETACH:
			retval = nds32_gdb_detach(nds32);
			break;
		default:
			break;
	}

	return retval;
}
unsigned int MaxLogFileSize	= 0xA00000; // default: 10MB
extern char* log_output_path;
FILE *pLogFile = NULL;
//#define MaxLogFileSize	0x100000
unsigned int LogFileIdx = 0;
const char *Log_File_Name[2]={
	"iceman_debug0.log",
	"iceman_debug1.log",
};

int nds32_log_callback(void *priv)
{
    char log_buffer[2048];
	unsigned int FileSize = 0;
    char *c;

    c = strstr(log_output_path, "iceman_debug0.log");
    if( c ) {
        *c = '\0';
    }

	if (pLogFile) {
		//fseek(pLogFile, 0, SEEK_END);
		FileSize = ftell(pLogFile);
	}
	#if _NDS32_ONLY_
	else {
		FILE *pStartLogFile = get_log_output();
		if (pStartLogFile)
			fclose(pStartLogFile);
	}
	#endif

	if ((FileSize >= MaxLogFileSize) ||
		 (pLogFile == NULL)) {
		if (pLogFile)
			fclose(pLogFile);
		LogFileIdx ^= 0x01;

        memset(log_buffer, 0, sizeof(log_buffer));
        strncpy(log_buffer, log_output_path, strlen(log_output_path));
        strncat(log_buffer, Log_File_Name[LogFileIdx], strlen(Log_File_Name[LogFileIdx])); 
		pLogFile = fopen(log_buffer, "w");
		set_log_output(NULL, pLogFile);
	}
	//LOG_DEBUG("nds32_log_callback: FileSize = 0x%x\n", FileSize);
	return ERROR_OK;
}

unsigned int nds32_do_once = 0;
unsigned int nds32_set_log_callback = 0;
void nds32_do_log_callback(struct target *target)
{
	if (nds32_set_log_callback)
		return;
	target_register_timer_callback(nds32_log_callback, 5000, 1, target);
	nds32_set_log_callback = 1;
}

void nds32_do_once_time(struct target *target)
{
	if (nds32_do_once)
		return;

	nds32_do_log_callback(target);
	burner_server_init(NULL);
	nds32_do_once = 1;	
}

int nds32_init(struct nds32 *nds32)
{
	/* Initialize anything we can set up without talking to the target */
	nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
	nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_AUTO;
	nds32->memory.set_acc_mode = NDS_MEMORY_SELECT_AUTO;
	nds32->memory.va_to_pa_off = 0;

	/* register event callback */
	target_register_event_callback(nds32_callback_event_handler,
			&(nds32->target->target_number));
	LOG_DEBUG("nds32_init: target_number=0x%x\n", nds32->target->target_number);
	nds32_do_once_time(nds32->target);
	return ERROR_OK;
}

int nds32_is_syscall_handled(int syscall_id)
{
	switch (syscall_id) {
		case NDS32_SYSCALL_OPEN:
		case NDS32_SYSCALL_CLOSE:
		case NDS32_SYSCALL_READ:
		case NDS32_SYSCALL_WRITE:
		case NDS32_SYSCALL_LSEEK:
		case NDS32_SYSCALL_UNLINK:
		case NDS32_SYSCALL_RENAME:
		case NDS32_SYSCALL_FSTAT:
		case NDS32_SYSCALL_STAT:
		case NDS32_SYSCALL_GETTIMEOFDAY:
		case NDS32_SYSCALL_ISATTY:
		case NDS32_SYSCALL_SYSTEM:
		case NDS32_SYSCALL_ERRNO:
		case NDS32_SYSCALL_FOPEN:
		case NDS32_SYSCALL_FREOPEN:
		case NDS32_SYSCALL_FCLOSE:
		case NDS32_SYSCALL_FFLUSH:
		case NDS32_SYSCALL_FREAD:
		case NDS32_SYSCALL_FWRITE:
		case NDS32_SYSCALL_FGETC:
		case NDS32_SYSCALL_FGETS:
		case NDS32_SYSCALL_FPUTC:
		case NDS32_SYSCALL_FPUTS:
		case NDS32_SYSCALL_UNGETC:
		case NDS32_SYSCALL_FTELL:
		case NDS32_SYSCALL_FSEEK:
		case NDS32_SYSCALL_REWIND:
		case NDS32_SYSCALL_CLEARERR:
		case NDS32_SYSCALL_FEOF:
		case NDS32_SYSCALL_FERROR:
		case NDS32_SYSCALL_REMOVE:
		case NDS32_SYSCALL_TMPFILE:
			return 1;
		default:
			return 0;
	}

	return 0;
}

int nds32_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	/* fill syscall parameters to file-I/O info */
	if (NULL == fileio_info) {
		LOG_ERROR("Target has not initial file-I/O data structure");
		return ERROR_FAIL;
	}

	fileio_info->param_1 = 0;
	fileio_info->param_2 = 0;
	fileio_info->param_3 = 0;
	fileio_info->param_4 = 0;
	fileio_info->param_5 = 0;

	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->hit_syscall == false)
		return ERROR_FAIL;

	LOG_DEBUG("hit syscall ID: 0x%x\n", nds32->active_syscall_id);

	/* free previous identifier storage */
	if (NULL != fileio_info->identifier) {
		free(fileio_info->identifier);
		fileio_info->identifier = NULL;
	}

	switch (nds32->active_syscall_id) {
		case NDS32_SYSCALL_EXIT:
		case NDS32_VIRTUAL_EXIT:
			fileio_info->identifier = (char *)malloc(5);
			sprintf(fileio_info->identifier, "exit");
			if (nds32->active_syscall_id == NDS32_VIRTUAL_EXIT)
				fileio_info->param_1 = 0;  // always return value 0, for nds virtual-exit(break 0x7FFFF)
			else
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			//target->is_program_exit = true;
			break;
		case NDS32_SYSCALL_OPEN:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(5);
				sprintf(fileio_info->identifier, "open");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of path */
				nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_3));
				nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_4));

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_CLOSE:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "close");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_READ:
			fileio_info->identifier = (char *)malloc(5);
			sprintf(fileio_info->identifier, "read");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			break;
		case NDS32_SYSCALL_WRITE:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "write");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			break;
		case NDS32_SYSCALL_LSEEK:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "lseek");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			break;
		case NDS32_SYSCALL_UNLINK:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "unlink");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of path */

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_RENAME:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "rename");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of old path */
				nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_3));
				/* reserve fileio_info->param_4 for length of new path */

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;

				target->type->read_buffer(target, fileio_info->param_3,
						256, filename);
				fileio_info->param_4 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_FSTAT:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fstat");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			break;
		case NDS32_SYSCALL_STAT:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(5);
				sprintf(fileio_info->identifier, "stat");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of old path */
				nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_3));

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_GETTIMEOFDAY:
			fileio_info->identifier = (char *)malloc(13);
			sprintf(fileio_info->identifier, "gettimeofday");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			break;
		case NDS32_SYSCALL_ISATTY:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "isatty");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_SYSTEM:
			{
				uint8_t command[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "system");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of old path */

				target->type->read_buffer(target, fileio_info->param_1,
						256, command);
				fileio_info->param_2 = strlen((char *)command) + 1;
			}
			break;
		case NDS32_SYSCALL_ERRNO:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "errno");
			nds32_set_mapped_reg(nds32, R0, nds32->virtual_hosting_errno);
			break;
		case NDS32_SYSCALL_FOPEN:
			{
				uint8_t filename[256];
				uint8_t mode[32];

				fileio_info->identifier = (char *)malloc(6);
				sprintf(fileio_info->identifier, "fopen");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of path */
				nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_3));
				/* reserve fileio_info->param_4 for length of mode */

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;

				target->type->read_buffer(target, fileio_info->param_3,
						32, mode);
				fileio_info->param_4 = strlen((char *)mode) + 1;
			}
			break;
		case NDS32_SYSCALL_FREOPEN:
			{
				uint8_t filename[256];
				uint8_t mode[32];

				fileio_info->identifier = (char *)malloc(8);
				sprintf(fileio_info->identifier, "freopen");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of path */
				nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_3));
				/* reserve fileio_info->param_4 for length of mode */
				nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_5));

				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;

				target->type->read_buffer(target, fileio_info->param_3,
						32, mode);
				fileio_info->param_4 = strlen((char *)mode) + 1;
			}
			break;
		case NDS32_SYSCALL_FCLOSE:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "fclose");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FFLUSH:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "fflush");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FREAD:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fread");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			nds32_get_mapped_reg(nds32, R3, (uint32_t *)&(fileio_info->param_4));
			break;
		case NDS32_SYSCALL_FWRITE:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "fwrite");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			nds32_get_mapped_reg(nds32, R3, (uint32_t *)&(fileio_info->param_4));
			break;
		case NDS32_SYSCALL_FGETC:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fgetc");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FGETS:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fgets");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			break;
		case NDS32_SYSCALL_FPUTC:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fputc");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			break;
		case NDS32_SYSCALL_FPUTS:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fputs");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			break;
		case NDS32_SYSCALL_UNGETC:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "ungetc");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			break;
		case NDS32_SYSCALL_FTELL:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "ftell");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FSEEK:
			fileio_info->identifier = (char *)malloc(6);
			sprintf(fileio_info->identifier, "fseek");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			nds32_get_mapped_reg(nds32, R1, (uint32_t *)&(fileio_info->param_2));
			nds32_get_mapped_reg(nds32, R2, (uint32_t *)&(fileio_info->param_3));
			break;
		case NDS32_SYSCALL_REWIND:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "rewind");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_CLEARERR:
			fileio_info->identifier = (char *)malloc(9);
			sprintf(fileio_info->identifier, "clearerr");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FEOF:
			fileio_info->identifier = (char *)malloc(5);
			sprintf(fileio_info->identifier, "feof");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_FERROR:
			fileio_info->identifier = (char *)malloc(7);
			sprintf(fileio_info->identifier, "ferror");
			nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
			break;
		case NDS32_SYSCALL_REMOVE:
			{
				uint8_t filename[256];
				fileio_info->identifier = (char *)malloc(7);
				sprintf(fileio_info->identifier, "remove");
				nds32_get_mapped_reg(nds32, R0, (uint32_t *)&(fileio_info->param_1));
				/* reserve fileio_info->param_2 for length of path */
				target->type->read_buffer(target, fileio_info->param_1,
						256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_TMPFILE:
			fileio_info->identifier = (char *)malloc(8);
			sprintf(fileio_info->identifier, "tmpfile");
			break;
		default:
			fileio_info->identifier = (char *)malloc(8);
			sprintf(fileio_info->identifier, "unknown");
			break;
	}

	return ERROR_OK;
}

static int nds32_convert_to_target_errno(int rsp_errno)
{
	switch (rsp_errno) {
		case FILEIO_EPERM:
			return NDS32_EPERM;
		case FILEIO_ENOENT:
			return NDS32_ENOENT;
		case FILEIO_EINTR:
			return NDS32_EINTR;
		case FILEIO_EIO:
			return NDS32_EIO;
		case FILEIO_EBADF:
			return NDS32_EBADF;
		case FILEIO_EACCES:
			return NDS32_EACCES;
		case FILEIO_EFAULT:
			return NDS32_EFAULT;
		case FILEIO_EBUSY:
			return NDS32_EBUSY;
		case FILEIO_EEXIST:
			return NDS32_EEXIST;
		case FILEIO_ENODEV:
			return NDS32_ENODEV;
		case FILEIO_ENOTDIR:
			return NDS32_ENOTDIR;
		case FILEIO_EISDIR:
			return NDS32_EISDIR;
		case FILEIO_EINVAL:
			return NDS32_EINVAL;
		case FILEIO_ENFILE:
			return NDS32_ENFILE;
		case FILEIO_EMFILE:
			return NDS32_EMFILE;
		case FILEIO_EFBIG:
			return NDS32_EFBIG;
		case FILEIO_ENOSPC:
			return NDS32_ENOSPC;
		case FILEIO_ESPIPE:
			return NDS32_ESPIPE;
		case FILEIO_EROFS:
			return NDS32_EROFS;
		case FILEIO_ENOSYS:
			return NDS32_ENOSYS;
		case FILEIO_ENAMETOOLONG:
			return NDS32_ENAMETOOLONG;
	}

	return NDS32_EUNKNOWN;
}

int nds32_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c)
{
	LOG_DEBUG("syscall return code: 0x%x, errno: 0x%x, ctrl_c: %s",
			retcode, fileio_errno, ctrl_c ? "true" : "false");

	struct nds32 *nds32 = target_to_nds32(target);

	/* For virtual hosting, GDB always returns -1 as error.  OpenOCD should
	   fill correct return value to $r0 according to function calls.  */
	if (retcode == -1) {
		switch (nds32->active_syscall_id) {
			/* Return NULL if error.  */
			case NDS32_SYSCALL_FOPEN:
			case NDS32_SYSCALL_FREOPEN:
			case NDS32_SYSCALL_FGETS:
			case NDS32_SYSCALL_TMPFILE:
				nds32_set_mapped_reg(nds32, R0, (uint32_t)NDS32_TARGET_NULL);
				break;

			/* Return EOF if error.  */
			case NDS32_SYSCALL_FCLOSE:
			case NDS32_SYSCALL_FFLUSH:
			case NDS32_SYSCALL_FGETC:
			case NDS32_SYSCALL_FPUTC:
			case NDS32_SYSCALL_FPUTS:
			case NDS32_SYSCALL_UNGETC:
				nds32_set_mapped_reg(nds32, R0, (uint32_t)NDS32_TARGET_EOF);
				break;

			/* Return 0 if error.  */
			case NDS32_SYSCALL_FREAD:
			case NDS32_SYSCALL_FWRITE:
			case NDS32_SYSCALL_ISATTY:
				nds32_set_mapped_reg(nds32, R0, (uint32_t)0);
				break;

			/* Return -1 if error.  */
			case NDS32_SYSCALL_FTELL:
			case NDS32_SYSCALL_FSEEK:
			case NDS32_SYSCALL_REMOVE:

			case	NDS32_SYSCALL_OPEN:
			case	NDS32_SYSCALL_CLOSE:
			case	NDS32_SYSCALL_READ:
			case	NDS32_SYSCALL_WRITE:
			case	NDS32_SYSCALL_LSEEK:
			case	NDS32_SYSCALL_UNLINK:
			case	NDS32_SYSCALL_RENAME:
			case	NDS32_SYSCALL_FSTAT:
			case	NDS32_SYSCALL_STAT:
			case	NDS32_SYSCALL_GETTIMEOFDAY:
			case	NDS32_SYSCALL_SYSTEM:

				nds32_set_mapped_reg(nds32, R0, (uint32_t)-1);
				break;
		}
	} else {
		nds32_set_mapped_reg(nds32, R0, (uint32_t)retcode);
	}

	nds32->virtual_hosting_errno = nds32_convert_to_target_errno(fileio_errno);
	nds32->virtual_hosting_ctrl_c = ctrl_c;
	nds32->active_syscall_id = NDS32_SYSCALL_UNDEFINED;

	return ERROR_OK;
}

int nds32_profiling(struct target *target, uint32_t *samples,
			uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	//TODO: unregister profile command would be better
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);
	if (nds32->profiling_support == 0) {
		LOG_ERROR("*** Profiling is not supported! ***");
		return ERROR_FAIL;
	}

	/* sample $PC every 10 milliseconds */
	uint32_t iteration = seconds * 100;

	if (max_num_samples < iteration)
		iteration = max_num_samples;

	int pc_regnum = nds32->register_map(nds32, PC);
	struct aice_profiling_info profiling_info;

	profiling_info.profiling_type = AICE_PROFILE_MODE_NORMAL;
	profiling_info.interval = 9; // interval 9 for best effort 100 samples/second
	profiling_info.iteration = iteration;
	profiling_info.reg_no = pc_regnum;
	profiling_info.psamples = samples;
	profiling_info.pnum_samples = num_samples;
	if (aice_profiling(target, &profiling_info) != ERROR_OK) {
		return ERROR_FAIL;
	}

	register_cache_invalidate(nds32->core_cache);

	return ERROR_OK;
}

uint8_t stat_buffer[NDS32_STRUCT_STAT_SIZE];
uint8_t timeval_buffer[NDS32_STRUCT_TIMEVAL_SIZE];

int nds32_gdb_fileio_write_memory(struct nds32 *nds32, uint32_t address,
		uint32_t *psize, uint8_t **pbuffer)
{
	uint8_t *buffer = (uint8_t *)*pbuffer;

	if ((NDS32_SYSCALL_FSTAT == nds32->active_syscall_id) ||
			(NDS32_SYSCALL_STAT == nds32->active_syscall_id)) {
		/* If doing GDB file-I/O, target should convert 'struct stat'
		 * from gdb-format to target-format */
		/* st_dev 2 */
		stat_buffer[0] = buffer[3];
		stat_buffer[1] = buffer[2];
		/* st_ino 2 */
		stat_buffer[2] = buffer[7];
		stat_buffer[3] = buffer[6];
		/* st_mode 4 */
		stat_buffer[4] = buffer[11];
		stat_buffer[5] = buffer[10];
		stat_buffer[6] = buffer[9];
		stat_buffer[7] = buffer[8];
		/* st_nlink 2 */
		stat_buffer[8] = buffer[15];
		stat_buffer[9] = buffer[16];
		/* st_uid 2 */
		stat_buffer[10] = buffer[19];
		stat_buffer[11] = buffer[18];
		/* st_gid 2 */
		stat_buffer[12] = buffer[23];
		stat_buffer[13] = buffer[22];
		/* st_rdev 2 */
		stat_buffer[14] = buffer[27];
		stat_buffer[15] = buffer[26];
		/* st_size 4 */
		stat_buffer[16] = buffer[35];
		stat_buffer[17] = buffer[34];
		stat_buffer[18] = buffer[33];
		stat_buffer[19] = buffer[32];
		/* st_atime 4 */
		stat_buffer[20] = buffer[55];
		stat_buffer[21] = buffer[54];
		stat_buffer[22] = buffer[53];
		stat_buffer[23] = buffer[52];
		/* st_spare1 4 */
		stat_buffer[24] = 0;
		stat_buffer[25] = 0;
		stat_buffer[26] = 0;
		stat_buffer[27] = 0;
		/* st_mtime 4 */
		stat_buffer[28] = buffer[59];
		stat_buffer[29] = buffer[58];
		stat_buffer[30] = buffer[57];
		stat_buffer[31] = buffer[56];
		/* st_spare2 4 */
		stat_buffer[32] = 0;
		stat_buffer[33] = 0;
		stat_buffer[34] = 0;
		stat_buffer[35] = 0;
		/* st_ctime 4 */
		stat_buffer[36] = buffer[63];
		stat_buffer[37] = buffer[62];
		stat_buffer[38] = buffer[61];
		stat_buffer[39] = buffer[60];
		/* st_spare3 4 */
		stat_buffer[40] = 0;
		stat_buffer[41] = 0;
		stat_buffer[42] = 0;
		stat_buffer[43] = 0;
		/* st_blksize 4 */
		stat_buffer[44] = buffer[43];
		stat_buffer[45] = buffer[42];
		stat_buffer[46] = buffer[41];
		stat_buffer[47] = buffer[40];
		/* st_blocks 4 */
		stat_buffer[48] = buffer[51];
		stat_buffer[49] = buffer[50];
		stat_buffer[50] = buffer[49];
		stat_buffer[51] = buffer[48];
		/* st_spare4 8 */
		stat_buffer[52] = 0;
		stat_buffer[53] = 0;
		stat_buffer[54] = 0;
		stat_buffer[55] = 0;
		stat_buffer[56] = 0;
		stat_buffer[57] = 0;
		stat_buffer[58] = 0;
		stat_buffer[59] = 0;

		*psize = NDS32_STRUCT_STAT_SIZE;
		*pbuffer = (uint8_t*)&stat_buffer[0];
	} else if (NDS32_SYSCALL_GETTIMEOFDAY == nds32->active_syscall_id) {
		/* If doing GDB file-I/O, target should convert 'struct timeval'
		 * from gdb-format to target-format */
		timeval_buffer[0] = buffer[3];
		timeval_buffer[1] = buffer[2];
		timeval_buffer[2] = buffer[1];
		timeval_buffer[3] = buffer[0];
		timeval_buffer[4] = buffer[11];
		timeval_buffer[5] = buffer[10];
		timeval_buffer[6] = buffer[9];
		timeval_buffer[7] = buffer[8];

		*psize = NDS32_STRUCT_TIMEVAL_SIZE;
		*pbuffer = (uint8_t*)&timeval_buffer[0];
	}

	return ERROR_OK;
}

int nds32_reset_halt(struct nds32 *nds32)
{
	LOG_INFO("reset halt as init");
	aice_reset_target(nds32->target, AICE_RESET_HOLD);

	return ERROR_OK;
}

int nds32_auto_change_memory_mode(struct target *target, uint32_t address, uint32_t length)
{
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t dlm_start   = nds32->memory.dlm_start;
	uint32_t dlm_end     = nds32->memory.dlm_end;
	uint32_t ilm_start   = nds32->memory.ilm_start;
	uint32_t ilm_end     = nds32->memory.ilm_end;
	uint32_t address_end = address + length;

	/* for BUS=>CPU */
	if (nds32->memory.access_channel == NDS_MEMORY_ACC_BUS) {
		if (((nds32->memory.ilm_base != 0) && (nds32->memory.ilm_enable == true)) ||
			((nds32->memory.dlm_base != 0) && (nds32->memory.dlm_enable == true))) {
			nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
		}
		LOG_DEBUG("ilm:0x%08X ~ 0x%08X\n dlm:0x%08X ~ 0x%08X\n   address:0x%08X, length:0x%X",
	    ilm_start, ilm_end, dlm_start, dlm_end, address, length);

		/* if address is out of ILM */
		if( (address_end < ilm_start) || (address > ilm_end) ) {
			/* if address is out of ILM & DLM */
			if( (address_end < dlm_start) || (address > dlm_end) ) {
				nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
				return ERROR_OK;
			}
		}
	}
	/* for CPU=>BUS */
	else {
	}
	return ERROR_OK;
}

