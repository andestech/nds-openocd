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

#include <helper/command.h>
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_reg.h"
#include "nds32_disassembler.h"
#include <target/breakpoints.h>
#include "nds32_v3_common.h"
#include <jtag/aice/aice_apis.h>
#include "target/nds32_log.h"

// for v5 command functions
extern uint32_t nds_no_crst_detect;
extern bool is_ndsv5(struct target *target);
extern int ndsv5cmd_set_va_to_pa_off(struct target *target, uint32_t va_to_pa_off);
extern int ndsv5cmd_set_boot_time(struct target *target, uint32_t boot_time);
extern int ndsv5cmd_set_reset_time(struct target *target, uint32_t reset_time);
extern int ndsv5cmd_set_reset_halt_as_examine(struct target *target, bool reset_halt_as_examine);
extern int ndsv5cmd_set_auto_convert_hw_bp(struct target *target, bool auto_convert_hw_bp);

extern __COMMAND_HANDLER(handle_ndsv5_memory_access_command);
extern __COMMAND_HANDLER(handle_ndsv5_cache_command);
extern __COMMAND_HANDLER(handle_ndsv5_icache_command);
extern __COMMAND_HANDLER(handle_ndsv5_dcache_command);
extern __COMMAND_HANDLER(handle_ndsv5_query_target_command);
extern __COMMAND_HANDLER(handle_ndsv5_query_endian_command);
extern __COMMAND_HANDLER(handle_ndsv5_query_cpuid_command);
extern __COMMAND_HANDLER(handle_ndsv5_query_capability_command);
extern __COMMAND_HANDLER(handle_ndsv5_configure_command);
extern __COMMAND_HANDLER(handle_ndsv5_ace_command);
extern __COMMAND_HANDLER(handle_ndsv5_reset_and_hold);

extern char *user_algorithm_path;
extern bool algorithm_bin_read;
extern char *dump_pwr_path;
extern void nds32_pwr_write_file(struct nds32 *nds32);
extern unsigned int MaxLogFileSize;
extern int nds32_set_buffer_access_size(uint64_t, uint64_t, uint32_t);
extern void nds32_reset_buffer_access_size(void);
#if NDS32_TRACER_SUPPORT
extern __COMMAND_HANDLER(handle_nds32_tracer_command);
#endif
uint32_t nds32_dis_global_stop_warning = 0;
uint32_t nds32_custom_def_idlm_base = 0;
uint32_t nds32_security_compat_display = 0;
uint32_t nds32_bytecode_parsing = 1;

static const char *const NDS_MEMORY_ACCESS_NAME[] = {
	"BUS",
	"CPU",
};

static const char *const NDS_MEMORY_SELECT_NAME[] = {
	"AUTO",
	"MEM",
	"ILM",
	"DLM",
};

COMMAND_HANDLER(handle_nds32_dssim_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->step_isr_enable = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->step_isr_enable = false;
	}

	command_print(CMD, "%s: $INT_MASK.DSSIM: %d", target_name(target),
			nds32->step_isr_enable);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_memory_access_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_memory_access_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "bus") == 0)
			memory->access_channel = NDS_MEMORY_ACC_BUS;
		else if (strcmp(CMD_ARGV[0], "cpu") == 0)
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		else /* default access channel is NDS_MEMORY_ACC_CPU */
			memory->access_channel = NDS_MEMORY_ACC_CPU;

		LOG_DEBUG("memory access channel is changed to %s",
				NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	} else {
		command_print(CMD, "%s: memory access channel: %s",
				target_name(target),
				NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_memory_mode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (nds32->edm.access_control == false) {
			command_print(CMD, "%s does not support ACC_CTL. "
					"Set memory mode to MEMORY", target_name(target));
			nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_MEM;
		} else if (nds32->edm.direct_access_local_memory == false) {
			command_print(CMD, "%s does not support direct access "
					"local memory. Set memory mode to MEMORY",
					target_name(target));
			nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_MEM;

		} else {
			if (strcmp(CMD_ARGV[0], "auto") == 0) {
				nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_AUTO;
			} else if (strcmp(CMD_ARGV[0], "mem") == 0) {
				nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_MEM;
			} else if (strcmp(CMD_ARGV[0], "ilm") == 0) {
				if (nds32->memory.ilm_base == 0)
					command_print(CMD, "%s does not support ILM",
							target_name(target));
				else
					nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_ILM;
			} else if (strcmp(CMD_ARGV[0], "dlm") == 0) {
				if (nds32->memory.dlm_base == 0)
					command_print(CMD, "%s does not support DLM",
							target_name(target));
				else
					nds32->memory.select_acc_mode = NDS_MEMORY_SELECT_DLM;
			}
		}
	}

	command_print(CMD, "%s: memory mode: %s",
			target_name(target),
			NDS_MEMORY_SELECT_NAME[nds32->memory.select_acc_mode]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_cache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cache *icache = &(nds32->memory.icache);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	int result;

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_cache_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (icache->line_size == 0 && dcache->line_size == 0) {
			command_print(CMD, "%s: No instruction cache and data cache",
					target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			uint32_t value;
			aice_read_reg(target, MR8, &value);
			if ((dcache->line_size != 0) && (((value&0x2)>>1) == 1 )) {
				/* D$ write back */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_WBALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Write back data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Write back data cache...done",
						target_name(target));

				/* D$ invalidate */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate data cache...done",
						target_name(target));
			} else {
				if (dcache->line_size == 0)
					command_print(CMD, "%s: No data cache",
							target_name(target));
				else
					command_print(CMD, "%s: Data cache disabled",
							target_name(target));
			}

			if ((icache->line_size != 0) && ((value&0x1) == 1 )) {
				/* I$ invalidate */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1I_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate instruction cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate instruction cache...done",
						target_name(target));
			} else {
				if (icache->line_size == 0)
					command_print(CMD, "%s: No instruction cache",
							target_name(target));
				else
					command_print(CMD, "%s: Instruction cache disabled",
							target_name(target));
			}
			/* ISYNC (Instruction Data Coherence Synchronization), the address is don't care */
			result = aice_cache_ctl(target, AICE_CACHE_CTL_LOOPCACHE_ISYNC, 0);
			if (result != ERROR_OK)
				return result;
		} else
			command_print(CMD, "No valid parameter");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_icache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cache *icache = &(nds32->memory.icache);
	int result;

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_icache_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (icache->line_size == 0) {
			command_print(CMD, "%s: No instruction cache",
					target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			uint32_t value;
			aice_read_reg(target, MR8, &value);
			if( (value&0x1) == 1 ) {
				/* I$ invalidate */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1I_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate instruction cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate instruction cache...done",
						target_name(target));
			} else {
				command_print(CMD, "%s: Instruction cache disabled",
						target_name(target));
			}
		} else if (strcmp(CMD_ARGV[0], "enable") == 0) {
			uint32_t value;
			LOG_DEBUG("icache enable");
			aice_read_reg(target, MR8, &value);
			aice_write_reg(target, MR8, value | 0x1);

			// Check 
			aice_read_reg(target, MR8, &value);
			if( (value&0x1) != 1 )
				LOG_ERROR( "Unable enable icache" );
			else
				icache->enable = true;

			nds32->core_cache->reg_list[MR8].valid = false;
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			uint32_t value;
			LOG_DEBUG("icache disable");
			aice_read_reg(target, MR8, &value);
			aice_write_reg(target, MR8, value & ~0x1);

			// Check
			aice_read_reg(target, MR8, &value);
			if( (value&0x1) != 0 )
				LOG_ERROR( "Unable disable icache" );
			else
				icache->enable = false;

			nds32->core_cache->reg_list[MR8].valid = false;
		} else if (strcmp(CMD_ARGV[0], "dump") == 0) {
			if( strcmp(CMD_ARGV[1], "all") == 0 ) {
				//int debug_level_bak = debug_level;

				if( CMD_ARGC < 2 ) {
					command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
					return ERROR_FAIL;
				}

				if( CMD_ARGC == 3 ) { 
					LOG_DEBUG("dump all icache to file: %s", CMD_ARGV[2]);
					//debug_level = LOG_LVL_ERROR;
					result = aice_dump_cache(target, ICACHE, CMD_ARGV[2]);
				}
				else {
					LOG_DEBUG("dump all icache to file: %s", "icache.dump");
					//debug_level = LOG_LVL_ERROR;
					result = aice_dump_cache(target, ICACHE, "icache.dump");
				}

				//debug_level = debug_level_bak;
				return result;
			}
			else if( strcmp(CMD_ARGV[1], "va") == 0 ) {
				uint32_t va;
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], va);
				int retval = aice_dump_cache_va(target, ICACHE, va);
				return retval;
			}
			else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
				return ERROR_FAIL;
			}
		} else {
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_dcache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	int result;

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_dcache_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (dcache->line_size == 0) {
			command_print(CMD, "%s: No data cache", target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			uint32_t value;
			aice_read_reg(target, MR8, &value);
			if( ((value&0x2)>>1) == 1 ) {
				/* D$ write back */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_WBALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Write back data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Write back data cache...done",
						target_name(target));

				/* D$ invalidate */
				result = aice_cache_ctl(target, AICE_CACHE_CTL_L1D_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate data cache...done",
						target_name(target));
			} else {
				command_print(CMD, "%s: Data cache disabled",
						target_name(target));
			}
		} else if (strcmp(CMD_ARGV[0], "enable") == 0) {
			uint32_t value;
			LOG_DEBUG("dcache enable");
			aice_read_reg(target, MR8, &value);
			aice_write_reg(target, MR8, value | 0x2);

			// Check
			aice_read_reg(target, MR8, &value);
			if( ((value&0x2)>>1) != 1 )
				LOG_ERROR( "Unable enable dcache" );
			else
				dcache->enable = true;

			nds32->core_cache->reg_list[MR8].valid = false;
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			uint32_t value;
			LOG_DEBUG("dcache disable");
			aice_read_reg(target, MR8, &value);
			aice_write_reg(target, MR8, value & ~0x2);

			// Check
			aice_read_reg(target, MR8, &value);
			if( ((value&0x2)>>1) != 0 )
				LOG_ERROR( "Unable disable dcache" );
			else
				dcache->enable = false;

			nds32->core_cache->reg_list[MR8].valid = false;
		} else if (strcmp(CMD_ARGV[0], "dump") == 0) {
			if( strcmp(CMD_ARGV[1], "all") == 0 ) {
				//int debug_level_bak = debug_level;

				if( CMD_ARGC < 2 ) {
					command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
					return ERROR_FAIL;
				}

				if( CMD_ARGC == 3 ) {
					LOG_DEBUG("dump all dcache to file: %s", CMD_ARGV[2]);
					//debug_level = LOG_LVL_ERROR;
					result = aice_dump_cache( target, DCACHE, CMD_ARGV[2] );
				}
				else {
					LOG_DEBUG("dump all dcache to file: %s", "dcache.dump");
					//debug_level = LOG_LVL_ERROR;
					result = aice_dump_cache( target, DCACHE, "dcache.dump" );
				}

				//debug_level = debug_level_bak;
				return result;
			}
			else if( strcmp(CMD_ARGV[1], "va") == 0 ) {
				uint32_t va;
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], va);
				int retval = aice_dump_cache_va(target, DCACHE, va);
				return retval;
			}
			else {
				command_print(CMD, "%s: No valid parameter", target_name(target));
				command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
				return ERROR_FAIL;
			}

		} else {
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_tlb_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
    struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	int result;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (mmu_config->memory_protection == 0) {
			command_print(CMD, "%s: No memory management", target_name(target));
			return ERROR_OK;
		}

        if (strcmp(CMD_ARGV[0], "dump") == 0) {
            /* TODO: dump cache content */
            if( strcmp(CMD_ARGV[1], "all") == 0 ) {
                if( CMD_ARGC < 2 ) {
                    command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
                    return ERROR_FAIL;
                }

                char *filename = NULL;
                if( CMD_ARGC == 3 )
                    filename = strdup(CMD_ARGV[2]); 
                else 
                    filename = strdup("tlb.dump"); 

                LOG_DEBUG("dump all dcache to file: %s", filename);
                result = aice_dump_tlb( target, filename );
                free(filename); 
                return result;
            }
            else if( strcmp(CMD_ARGV[1], "va") == 0 ) {
                uint32_t va;
                COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], va);
                int retval = aice_dump_tlb_va(target, va);
                return retval;
            }
            else {
                command_print(CMD, "%s: No valid parameter", target_name(target));
                command_print(CMD, "Usage: dump all <filename>(optimal) / dump va <address>");
                return ERROR_FAIL;
            }

        } else {
            command_print(CMD, "%s: No valid parameter", target_name(target));
        }
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_auto_break_command)
{
	bool auto_convert_hw_bp = false;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			auto_convert_hw_bp = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			auto_convert_hw_bp = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			if (is_ndsv5(target)) {
				ndsv5cmd_set_auto_convert_hw_bp(target, auto_convert_hw_bp);
			}
		} else {
			nds32->auto_convert_hw_bp = auto_convert_hw_bp;
		}
		if (auto_convert_hw_bp)
			command_print(CMD, "%s: convert sw break to hw break on ROM: on",
					target_name(target));
		else
			command_print(CMD, "%s: convert sw break to hw break on ROM: off",
					target_name(target));
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_set_hw_break_command)
{
	uint32_t addr;
	uint32_t length;
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC == 0) {
		struct breakpoint *breakpoint = target->breakpoints;

		while (breakpoint) {
			command_print(CMD, "Breakpoint: 0x%08x, 0x%x",
							(unsigned int)breakpoint->address,
							(breakpoint->length & BP_WP_LENGTH_MASK));
			LOG_DEBUG("nds-breakpoint address: 0x%" TARGET_PRIxADDR, breakpoint->address);
			breakpoint = breakpoint->next;
		}
		return ERROR_OK;
	}

	switch (CMD_ARGC) {
		case 2:
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], length);
			return breakpoint_add(target, addr, length|BP_WP_NON_SIMPLE, BKPT_HARD);

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
}

COMMAND_HANDLER(handle_nds32_clr_hw_break_command)
{
	uint32_t addr;
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	switch (CMD_ARGC) {
		case 1:
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
			breakpoint_remove(target, addr);
			return ERROR_OK;

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
}

COMMAND_HANDLER(handle_nds32_set_watch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}
	uint32_t wp_value_support=0, value_compare_support=0, do_value_compare=0;
	char watch_type[3] = {'r','w','a'};

	if (CMD_ARGC == 0) {
		struct watchpoint *watchpoint = target->watchpoints;

		while (watchpoint) {
			if (watchpoint->length & BP_WP_DATA_COMPARE) {
				command_print(CMD, "watchpoint address: 0x%8.8" PRIx32
					", len: 0x%8.8" PRIx32
					", r/w/a: %c, value: 0x%8.8" PRIx32
					", mask: 0x%8.8" PRIx32,
					(unsigned int)watchpoint->address,
					(watchpoint->length & BP_WP_LENGTH_MASK),
					watch_type[watchpoint->rw],
					watchpoint->value,
					watchpoint->mask);
			}
			else {
				command_print(CMD, "watchpoint address: 0x%08x, len: 0x%08x, r/w/a: %c",
					(unsigned int)watchpoint->address, watchpoint->length, watch_type[watchpoint->rw]);
			}
			LOG_DEBUG("nds-watchpoint address: 0x%" TARGET_PRIxADDR ", length: 0x%08x", watchpoint->address, watchpoint->length);
			watchpoint = watchpoint->next;
		}
		return ERROR_OK;
	}

	enum watchpoint_rw type = WPT_ACCESS;
	uint32_t addr = 0;
	uint32_t length = 0;
	uint32_t data_value = 0x0;
	uint32_t data_mask = 0xffffffff;

	switch (CMD_ARGC) {
	case 5:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[4], data_mask);
		/* fall through */
	case 4:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], data_value);
		/* fall through */
		do_value_compare = 1;
	case 3:
		switch (CMD_ARGV[2][0]) {
		case 'r':
			type = WPT_READ;
			break;
		case 'w':
			type = WPT_WRITE;
			break;
		case 'a':
			type = WPT_ACCESS;
			break;
		default:
			LOG_ERROR("invalid watchpoint mode ('%c')", CMD_ARGV[2][0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (do_value_compare) {
			/* BPV_TYPE, EDM_CFG[7-6]
				0: Only stores are supported but loads are not supported.
				1: Both loads and stores are supported.
				2: Neither loads nor stores are supported.
				3: Only loads are supported but stores are not supported.  */
			wp_value_support = (nds32->edm.edm_cfg_reg & 0x03) >> 6;
			if ((wp_value_support == 1) ||
					((type == WPT_READ) && (wp_value_support == 3)) ||
					((type == WPT_WRITE) && (wp_value_support == 0)) ) {
				value_compare_support = 1;
			}
			if (value_compare_support == 0) {
				if (wp_value_support == 0)
					LOG_ERROR("BPV_TYPE=0, Only stores are supported");
				else if (wp_value_support == 2)
					LOG_ERROR("BPV_TYPE=2, Neither loads nor stores are supported");
				else if (wp_value_support == 3)
					LOG_ERROR("BPV_TYPE=3, Only loads are supported");
				return ERROR_OK;
			}
		}
		/* fall through */
	case 2:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], length);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
		break;

	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	/*
	uint32_t tmp_length = (length & BP_WP_LENGTH_MASK);
	data_mask = tmp_length - 1;
	if ((addr % tmp_length) != 0)
		data_mask = (data_mask << 1) + 1;
	*/
	if (do_value_compare)
		length |= BP_WP_DATA_COMPARE;

	int retval = watchpoint_add(target, addr, length|BP_WP_USER_WATCH,
								type, data_value, data_mask);
	if (ERROR_OK != retval)
		LOG_ERROR("Failure setting watchpoints");

	return retval;
}

COMMAND_HANDLER(handle_nds32_clr_watch_command)
{
	uint32_t addr;
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}
	switch (CMD_ARGC) {
		case 1:
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
			watchpoint_remove(target, addr);
			return ERROR_OK;

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
}

COMMAND_HANDLER(handle_nds32_set_ACR_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t reg_r2_value = 0;
	uint8_t val;
	const char *val_start = NULL, *p_val = NULL;
	char *acr_val = NULL, *p =NULL;
	struct reg *reg = NULL;
	uint32_t size_in_byte = 0;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		// support hexidecimal formal only, Now.
		if (CMD_ARGV[1][0] != '0') {
			goto unsupported;
		}
		if (CMD_ARGV[1][1] != 'x' && CMD_ARGV[1][1] != 'X') {
			goto unsupported;
		}

		LOG_DEBUG("%s: set ACR register %s, value: %s", target_name(target), CMD_ARGV[0], CMD_ARGV[1]);
		// lookup register list by the given name
		int reg_number = nds32_get_reg_number(CMD_ARGV[0]);
		if (reg_number != -1) {
			LOG_DEBUG("number of register is %d", reg_number);

			reg = &nds32->core_cache->reg_list[reg_number];
			p = acr_val = reg->value;
			size_in_byte = DIV_ROUND_UP(reg->size, 8);
			memset(reg->value, 0, size_in_byte);

			val_start = CMD_ARGV[1] + 2;
			p_val = val_start + strlen(val_start) - 2;
			while (p_val >= val_start) {
				val = ((p_val[0] - '0') << 4) | (p_val[1] - '0');
				*p++ = val;
				p_val -= 2;
			}
			if (&p_val[1] == val_start) {
				*p = p_val[1] - '0';
			}

			// backup R2 for ACR access
			aice_read_reg(target, R2, &reg_r2_value);

			aice_write_acr(target, reg_number, acr_val);

			// restore R2 for ACR access
			aice_write_reg(target, R2, reg_r2_value);

			// invalidate relative reg
			reg->valid = false;
		} else {
			LOG_ERROR("cannot find register %s", CMD_ARGV[0]);
		}

		return ERROR_OK;

unsupported:
		LOG_INFO("set ACR register with unknown value: %s", CMD_ARGV[1]);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_global_stop_command)
{
	bool global_stop =false;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			global_stop = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			global_stop = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (is_nds32(nds32)) {
			nds32->global_stop = global_stop;
			if (nds32->global_stop)
				LOG_INFO("%s: global stop: on", target_name(target));
			else
				LOG_INFO("%s: global stop: off", target_name(target));
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_va_to_pa_command)
{
	int va_to_pa_off = 0;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			va_to_pa_off = 0;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			va_to_pa_off = 1;
	}
	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			if (is_ndsv5(target)) {
				ndsv5cmd_set_va_to_pa_off(target, va_to_pa_off);
			}
		} else {
			nds32->memory.va_to_pa_off = va_to_pa_off;
			LOG_INFO("va_to_pa_off: %d", nds32->memory.va_to_pa_off);
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_dis_global_stop_warning_command)
{
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "1") == 0)
			nds32_dis_global_stop_warning = 1;
		else
			nds32_dis_global_stop_warning = 0;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_bytecode_parsing_command)
{
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "1") == 0)
			nds32_bytecode_parsing = 1;
		else
			nds32_bytecode_parsing = 0;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_soft_reset_halt_command)
{
	bool soft_reset_halt = false;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			soft_reset_halt = true;
		else //if (strcmp(CMD_ARGV[0], "off") == 0)
			soft_reset_halt = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			command_print(CMD, "current target isn't an Andes core");
		} else {
			nds32->soft_reset_halt = soft_reset_halt;
			if (nds32->soft_reset_halt)
				LOG_INFO("%s: soft-reset-halt: on", target_name(target));
			else
				LOG_INFO("%s: soft-reset-halt: off", target_name(target));
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_security_compat_display_command)
{
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32_security_compat_display = 1;
	}

	if (nds32_security_compat_display)
		LOG_DEBUG("security_compat_display: on");
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_boot_time_command)
{
	uint32_t boot_time = 0;
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], boot_time);
	}
	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			if (is_ndsv5(target)) {
				ndsv5cmd_set_boot_time(target, boot_time);
			}
		} else {
			nds32->boot_time = boot_time;
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_reset_time_command)
{
	uint32_t reset_time = 0;
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reset_time);
	}
	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			if (is_ndsv5(target)) {
				ndsv5cmd_set_reset_time(target, reset_time);
			}
		} else {
			nds32->reset_time = reset_time;
		}
	}
	return ERROR_OK;
}

extern uint32_t runtest_num_clocks;
extern uint32_t scan_field_max;
COMMAND_HANDLER(handle_nds32_runtest_num_clocks_command)
{
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], runtest_num_clocks);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_scan_field_max_command)
{
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], scan_field_max);
	}
	return ERROR_OK;
}

extern uint32_t nds_ftdi_jtag_opt;
COMMAND_HANDLER(handle_nds32_ftdi_jtag_opt_command)
{
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nds_ftdi_jtag_opt);
	}
	return ERROR_OK;
}

extern uint32_t nds_ftdi_log_detail;
COMMAND_HANDLER(handle_nds32_ftdi_log_detail_command)
{
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nds_ftdi_log_detail);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_log_file_size_command)
{
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], MaxLogFileSize);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_idlm_base_size_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	struct nds32_memory *memory = &(nds32->memory);
	unsigned int ilm_start, ilm_size, dlm_start, dlm_size;

	if (CMD_ARGC >= 4) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], ilm_start);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ilm_size);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], dlm_start);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], dlm_size);
		memory->ilm_start = ilm_start;
		memory->ilm_end = ilm_start + ilm_size;
		memory->ilm_enable = true;
		memory->dlm_start = dlm_start;
		memory->dlm_end = dlm_start + dlm_size;
		memory->dlm_enable = true;
		nds32_custom_def_idlm_base = 1;
		//LOG_DEBUG("nds32_idlm_base_size: %x, %x, %x, %x",
		//	memory->ilm_start, memory->ilm_end, memory->dlm_start, memory->dlm_end);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_login_edm_passcode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	nds32->edm_passcode = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}
#if 0
COMMAND_HANDLER(handle_nds32_login_edm_operation_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 1) {

		uint32_t misc_reg_no;
		uint32_t data;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], misc_reg_no);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], data);

		if (nds32_edm_ops_num >= NDS32_EDM_OPERATION_MAX_NUM)
			return ERROR_FAIL;

		/* Just save the operation. Execute it in nds32_login() */
		nds32_edm_ops[nds32_edm_ops_num].reg_no = misc_reg_no;
		nds32_edm_ops[nds32_edm_ops_num].value = data;
		nds32_edm_ops_num++;
	} else
		return ERROR_FAIL;

	return ERROR_OK;
}
#endif

COMMAND_HANDLER(handle_nds32_reset_halt_as_init_command)
{
	/* only do reset_halt() in the 1st target(core-0) */
	//struct target *target = get_current_target(CMD_CTX);
	bool reset_halt_as_examine = false;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			reset_halt_as_examine = true;
		else if (strcmp(CMD_ARGV[0], "off") == 0)
			reset_halt_as_examine = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (!is_nds32(nds32)) {
			if (is_ndsv5(target)) {
				ndsv5cmd_set_reset_halt_as_examine(target, reset_halt_as_examine);
			}
		} else {
			nds32->reset_halt_as_examine = reset_halt_as_examine;
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_keep_target_edm_ctl_command)
{
	bool keep_target_edm_ctl = 0;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			keep_target_edm_ctl = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			keep_target_edm_ctl = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (is_nds32(nds32)) {
			nds32->keep_target_edm_ctl = keep_target_edm_ctl;
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_decode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 1) {

		uint32_t addr;
		uint32_t insn_count;
		uint32_t read_addr;
		uint32_t i;
		struct nds32_instruction instruction;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], insn_count);

		read_addr = addr;
		i = 0;
		while (i < insn_count) {
			if (ERROR_OK != nds32_evaluate_opcode(nds32, read_addr, &instruction))
				return ERROR_FAIL;

			command_print(CMD, "%s", instruction.text);

			read_addr += instruction.instruction_size;
			i++;
		}
	} else if (CMD_ARGC == 1) {

		uint32_t addr;
		struct nds32_instruction instruction;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);

		if (ERROR_OK != nds32_evaluate_opcode(nds32, addr, &instruction))
			return ERROR_FAIL;

		command_print(CMD, "%s", instruction.text);
	} else
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_word_access_mem_command)
{
	bool word_access_mem = 0;
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			word_access_mem = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			word_access_mem = false;
	}

	struct target *target;
	struct nds32 *nds32;
	for (target = all_targets; target; target = target->next) {
		nds32 = target_to_nds32(target);
		if (is_nds32(nds32)) {
			nds32->word_access_mem = word_access_mem;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_write_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t addr;
	uint32_t count;
	uint8_t *data;
	uint32_t i;
	int result;

	if (CMD_ARGC < 3) {
		command_print(CMD, "usage: %s <address> <count> <data>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);

	data = malloc (count);

	for (i = 0; i < count; i++) {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2 + i], data[i]);
	}

	result = target_write_buffer(target, addr, count, data);

	free (data);

	return result;
}

COMMAND_HANDLER(handle_nds32_read_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t addr;
	uint32_t count;
	uint8_t *data;
	uint32_t i;
	int result;

	if (CMD_ARGC < 2) {
		command_print(CMD, "usage: %s <address> <count>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);

	data = malloc (count);

	result = target_read_buffer(target, addr, count, data);

	for (i = 0; i < count; i++) {
		if ((i & 0xF) == 0)
			command_print_sameline(CMD, "0x%08x: ", (int)(addr + i));

		command_print_sameline(CMD, "%02x ", data[i]);

		if (((i & 0xF) == 0xF) || (i == count - 1))
			command_print_sameline(CMD, "\n");
	}

	free (data);

	return result;
}

COMMAND_HANDLER(handle_nds32_memAccSize_command)
{
	uint32_t access_size;
	uint64_t lowAddr, highAddr;
	int result;

	if (CMD_ARGC < 3) {
		command_print(CMD, "usage: %s <lowAddr> <highAddr> <access_size>",
				CMD_NAME);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], lowAddr);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], highAddr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], access_size);
	highAddr = (uint64_t)(highAddr - 1);
	if ((highAddr < lowAddr) ||
			((access_size != 8) && (access_size != 16) &&
			 (access_size != 32) && (access_size != 64))) {
		command_print(CMD, "Invalid parameter");
		return ERROR_FAIL;
	}
	if (access_size == 64)
		access_size = 32;
	access_size = (access_size >> 3);
	result = nds32_set_buffer_access_size(lowAddr, highAddr, access_size);
	return result;
}

COMMAND_HANDLER(handle_nds32_reset_memAccSize_command)
{
	nds32_reset_buffer_access_size();
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_target_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_query_target_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	command_print(CMD, "OCD");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_endian_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_query_endian_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	uint32_t value_psw;
	nds32_get_mapped_reg(nds32, IR0, &value_psw);

	if (value_psw & 0x20)
		command_print(CMD, "%s: BE", target_name(target));
	else
		command_print(CMD, "%s: LE", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_cpuid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_query_cpuid_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	struct target *tmptarget;
	uint32_t number_of_core = 0;

	for (tmptarget = all_targets; tmptarget; tmptarget = tmptarget->next) {
		number_of_core ++;
	}
	if (number_of_core == 0) {
		command_print(CMD, "CPUID: ");
	} else if (number_of_core == 1) {
		command_print(CMD, "CPUID: v3_core");
	} else {
		//command_print(CMD, "CPUID: v3_core%d", target->coreid);
		command_print(CMD, "%s", target_name(target));
	}
	return ERROR_OK;
}

extern int nds32_tracer_check(struct target *target);
extern int nds32_pwr_check(struct target *target);
COMMAND_HANDLER(handle_nds32_query_capability_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t if_tracer = 0, if_profiling = nds32->profiling_support, disable_busmode = 0;
	uint32_t hit_exception = 1, if_targetburn = 1;
	uint32_t if_pwr_sample = 1;
	uint32_t q_access_mode = 0;

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_query_capability_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}
#if NDS32_TRACER_SUPPORT
	if (nds32_tracer_check(target) == ERROR_OK) {
		if_tracer = 1;
	}
#endif
	if (nds32_pwr_check(target) != ERROR_OK) {
		if_pwr_sample = 0;
	}
	command_print(CMD, "tracer:%d;profiling:%d;disbus:%d;exception:%d;targetburn:%d;pwr:%d;q_access_mode:%d",
		if_tracer, if_profiling, disable_busmode, hit_exception, if_targetburn, if_pwr_sample, q_access_mode);
	return ERROR_OK;
}

extern int32_t get_ace_file_name_for_gdb (const char *aceconf, const char *platform, char **name);

COMMAND_HANDLER(handle_nds32_ace_command)
{
#if NDS32_ACE_SUPPORT
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	char *ace_file_name = NULL;

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_ace_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int32_t ret = get_ace_file_name_for_gdb (nds32->aceconf, CMD_ARGV[0], &ace_file_name);

		if (ret != -1) {
			if (ace_file_name != NULL) {
				command_print(CMD, "%s", ace_file_name);
				free(ace_file_name);
			}
		}
	}
#endif
	return ERROR_OK;
}
#if 1
COMMAND_HANDLER(handle_nds32_cop0_command)
{
#if 0
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	char *ace_file_name = NULL;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int32_t ret = get_ace_file_name_for_gdb (nds32->aceconf, 0, CMD_ARGV[0], &ace_file_name);

		if (ret != -1) {
			if (ace_file_name != NULL) {
				command_print(CMD, "%s", ace_file_name);
				free(ace_file_name);
			}
		}
	}
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_cop1_command)
{
#if 0
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	char *ace_file_name = NULL;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int32_t ret = get_ace_file_name_for_gdb (nds32->aceconf, 1, CMD_ARGV[0], &ace_file_name);

		if (ret != -1) {
			if (ace_file_name != NULL) {
				command_print(CMD, "%s", ace_file_name);
				free(ace_file_name);
			}
		}
	}
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_cop2_command)
{
#if 0
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	char *ace_file_name = NULL;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int32_t ret = get_ace_file_name_for_gdb (nds32->aceconf, 2, CMD_ARGV[0], &ace_file_name);

		if (ret != -1) {
			if (ace_file_name != NULL) {
				command_print(CMD, "%s", ace_file_name);
				free(ace_file_name);
			}
		}
	}
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_cop3_command)
{
#if 0
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	char *ace_file_name = NULL;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int32_t ret = get_ace_file_name_for_gdb (nds32->aceconf, 3, CMD_ARGV[0], &ace_file_name);

		if (ret != -1) {
			if (ace_file_name != NULL) {
				command_print(CMD, "%s", ace_file_name);
				free(ace_file_name);
			}
		}
	}
#endif
	return ERROR_OK;
}
#endif

static const char* property_names[] = {
		[PROPERTY_RUN_MODE] = "run_mode",
		[PROPERTY_PROF_ADDR_MIN] = "prof_addr_min",
		[PROPERTY_PROF_ADDR_MAX] = "prof_addr_max",
		[PROPERTY_PROF_WITH_RANGE] = "prof_with_range",
		[PROPERTY_CAN_PROFILE] = "can_profile",
		[PROPERTY_PWR_SAMPLE_RATE] = "pwr_sample_rate",
		[PROPERTY_PWR_SAMPLE_MODE] = "pwr_sample_mode",
		[PROPERTY_PWR_DUMP] = "dump_pwr",
		[PROPERTY_PWR_DISABLE] = "disable_pwr",
		[PROPERTY_PROFILE_DISABLE] = "disable_profile",
		[PROPERTY_MAX] = "max",
};
#define SIZE_PROPERTY (sizeof(property_names)/sizeof(char*))

static const char* run_mode_names[] = {
		[RUN_MODE_DEBUG] = "debug",
		[RUN_MODE_PROFILE] = "profile",
		[RUN_MODE_PWR_MONITOR] = "pwr",
		[RUN_MODE_PROFILE_PWR] = "profile_pwr",
};
#define SIZE_RUN_MODE (sizeof(run_mode_names)/sizeof(char*))

unsigned long long string_to_ulong(const char *str) {
	errno = 0;
	unsigned long long value = strtoul(str, NULL, 0);
	if ((errno == ERANGE && (value == ULONG_MAX)) || (errno != 0 && value == 0)) {
		perror("strtoul");
	}
	return value;
}

COMMAND_HANDLER(handle_nds32_configure_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_configure_command(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

#if 0
	command_print(CMD, "ARGC = %d", CMD_ARGC);
	for (unsigned i = 0; i < CMD_ARGC; ++i)
		command_print(CMD, "ARGV[%d] = %s", i, CMD_ARGV[i]);
#endif

	// check if any property is given
	if (0 >= CMD_ARGC) {
		command_print(CMD, "configure: no property given!");
		return ERROR_FAIL;
	}

	// check if property is supported
	size_t property;
	for (property = 0; property < SIZE_PROPERTY; ++property) {
		if (0 == strncasecmp(CMD_ARGV[0], property_names[property], 32))
			break;
	}
	if (PROPERTY_MAX <= property) {
		if (strcmp(CMD_ARGV[0], "algorithm_bin") == 0) {
			if (user_algorithm_path)
				free(user_algorithm_path);
			user_algorithm_path = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s = %s", CMD_ARGV[0], user_algorithm_path);
			algorithm_bin_read = true;
			nds32_edm_config(nds32);
			return ERROR_OK;
		}
		command_print(CMD, "configure: property '%s' unknown!", CMD_ARGV[0]);
		NDS32_LOG("<-- configure: property '%s' unknown! -->", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	// handle property configuration
	int rz = ERROR_OK;
	if (PROPERTY_RUN_MODE == property) {
		if (1 < CMD_ARGC) {
			// set
			size_t mode = 0;
			for (; mode < SIZE_RUN_MODE; ++mode) {
				if (0 == strncasecmp(CMD_ARGV[1], run_mode_names[mode], 32)) {
					if (mode == 0)
						nds32->gdb_run_mode = 0;
					else
						nds32->gdb_run_mode |= mode;
					break;
				}
			}
			if (mode >= SIZE_RUN_MODE)
				rz = ERROR_FAIL;
		} else {
			// get
		}
		if (ERROR_OK == rz)
			command_print(CMD, "configure: %s = %s", CMD_ARGV[0], run_mode_names[nds32->gdb_run_mode]);
	} else 	if (PROPERTY_PROF_ADDR_MIN == property) {
		if (1 < CMD_ARGC) {
			//set
			nds32->prof_addr_min = (uint32_t)string_to_ulong(CMD_ARGV[1]);
		} else {
			//get
		}
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32->prof_addr_min);
	} else 	if (PROPERTY_PROF_ADDR_MAX == property) {
		if (1 < CMD_ARGC) {
			//set
			nds32->prof_addr_max = (uint32_t)string_to_ulong(CMD_ARGV[1]);
		} else {
			//get
		}
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32->prof_addr_max);
	} else 	if (PROPERTY_PROF_WITH_RANGE == property) {
		if (1 < CMD_ARGC) {
			//set
			nds32->prof_with_range = (uint32_t)string_to_ulong(CMD_ARGV[1]);
		} else {
			//get
		}
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32->prof_with_range);
	} else 	if (PROPERTY_CAN_PROFILE == property) { //can_profile no work but keep this command for ide
	} else 	if (PROPERTY_PWR_SAMPLE_RATE == property) {
		if (1 < CMD_ARGC) {
			nds32_pwr_sample_rate = (uint32_t)string_to_ulong(CMD_ARGV[1]);
		}
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32_pwr_sample_rate);
	} else 	if (PROPERTY_PWR_SAMPLE_MODE == property) {
		if (1 < CMD_ARGC) {
			nds32_pwr_sample_mode = (uint32_t)string_to_ulong(CMD_ARGV[1]);
		}
		command_print(CMD, "configure: %s = 0x%08x", CMD_ARGV[0], nds32_pwr_sample_mode);
	} else 	if (PROPERTY_PWR_DUMP == property) {
		if (1 < CMD_ARGC) {
			if (dump_pwr_path)
				free (dump_pwr_path);
			dump_pwr_path = strdup(CMD_ARGV[1]);
			command_print(CMD, "configure: %s = %s", CMD_ARGV[0], dump_pwr_path);
			nds32_pwr_write_file(nds32);
		}
	} else 	if (property == PROPERTY_PWR_DISABLE) {
		nds32->gdb_run_mode &= ~RUN_MODE_PWR_MONITOR;
	} else 	if (property == PROPERTY_PROFILE_DISABLE) {
		nds32->gdb_run_mode &= ~RUN_MODE_PROFILE;
	} else {
		command_print(CMD, "configure: property '%s' is not found!", CMD_ARGV[0]);
		rz = ERROR_FAIL;
	}

	return rz;
}

static const char* bench_item_names[] = {
		[BENCH_ITEM_PROFILE_RATE] = "profile_rate",
};
#define SIZE_BENCH_ITEM_NAME (sizeof(bench_item_names)/sizeof(char*))

COMMAND_HANDLER(handle_nds32_bench_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

#if 0
	command_print(CMD, "ARGC = %d", CMD_ARGC);
	for (unsigned i = 0; i < CMD_ARGC; ++i)
		command_print(CMD, "ARGV[%d] = %s", i, CMD_ARGV[i]);
#endif

	// check if any item is given
	if (0 >= CMD_ARGC) {
		command_print(CMD, "bench: no item given!");
		return ERROR_FAIL;
	}

	// check if item is supported
	size_t item;
	for (item = 0; item < SIZE_BENCH_ITEM_NAME; ++item) {
		if (0 == strncasecmp(CMD_ARGV[0], bench_item_names[item], 32))
			break;
	}
	if (BENCH_ITEM_MAX <= item) {
		command_print(CMD, "bench: item '%s' unknown!", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	// handle property configuration
	int rz = ERROR_OK;
	if (BENCH_ITEM_PROFILE_RATE == item) {
		rz = aice_profile_bench(target, cmd);
	} else 	if (BENCH_ITEM_MAX < item) {
		command_print(CMD, "bench: item '%s' is not supported!", CMD_ARGV[0]);
	} else {
		command_print(CMD, "bench: item '%s' is not defined!", CMD_ARGV[0]);
		rz = ERROR_FAIL;
	}

	return rz;
}

static int jim_nds32_bulk_write(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 3) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <address> <count> <data>", cmd_name);
		return JIM_ERR;
	}

	int e;
	jim_wide address;
	e = Jim_GetOpt_Wide(&goi, &address);
	if (e != JIM_OK)
		return e;

	jim_wide count;
	e = Jim_GetOpt_Wide(&goi, &count);
	if (e != JIM_OK)
		return e;

	uint32_t *data = malloc(count * sizeof(uint32_t));
	jim_wide i;
	for (i = 0; i < count; i++) {
		jim_wide tmp;
		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK)
			return e;
		data[i] = (uint32_t)tmp;
	}

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	struct target *target = Jim_CmdPrivData(goi.interp);
	int result;

	result = target_write_buffer(target, address, count * 4, (const uint8_t *)data);

	free(data);

	return result;
}

static int jim_nds32_multi_write(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 4) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s w|h|b # of pairs [<address> <data>]+", cmd_name);
		return JIM_ERR;
	}

	const char *data_type;
	int e;
	e = Jim_GetOpt_String(&goi, &data_type, NULL);
	if (e != JIM_OK)
		return e;

	jim_wide num_of_pairs;
	e = Jim_GetOpt_Wide(&goi, &num_of_pairs);
	if (e != JIM_OK)
		return e;

	struct target *target = Jim_CmdPrivData(goi.interp);
	int result;
	uint32_t address;
	uint32_t data;
	jim_wide i;

	aice_set_command_mode(AICE_COMMAND_MODE_PACK);
	for (i = 0; i < num_of_pairs; i++) {
		jim_wide tmp;
		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK)
			break;
		address = (uint32_t)tmp;

		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK)
			break;
		data = (uint32_t)tmp;

		switch (data_type[0]) {
			case 'w':
				result = target_write_buffer(target, address, 4, (const uint8_t *)&data);
				break;
			case 'h':
				result = target_write_buffer(target, address, 2, (const uint8_t *)&data);
				break;
			case 'b':
				result = target_write_buffer(target, address, 1, (const uint8_t *)&data);
				break;
			default:
				Jim_SetResultFormatted(goi.interp,
						"usage: %s w|h|b # of pairs [<address> <data>]+", cmd_name);
				return JIM_ERR;
		}
		if (result != ERROR_OK)
			break;
	}
	aice_set_command_mode(AICE_COMMAND_MODE_NORMAL);

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	return ERROR_OK;
}

static int jim_nds32_bulk_read(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 2) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <address> <count>", cmd_name);
		return JIM_ERR;
	}

	int e;
	jim_wide address;
	e = Jim_GetOpt_Wide(&goi, &address);
	if (e != JIM_OK)
		return e;

	jim_wide count;
	e = Jim_GetOpt_Wide(&goi, &count);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	struct target *target = Jim_CmdPrivData(goi.interp);
	uint32_t *data = malloc(count * sizeof(uint32_t));
	int result;
	result = target_read_buffer(target, address, count * 4, (uint8_t *)data);
	char data_str[12];

	jim_wide i;
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	for (i = 0; i < count; i++) {
		sprintf(data_str, "0x%08x ", data[i]);
		Jim_AppendStrings(interp, Jim_GetResult(interp), data_str, NULL);
	}

	free(data);

	return result;
}

static int jim_nds32_read_edm_sr(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 1) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <edm_sr_name>", cmd_name);
		return JIM_ERR;
	}

	int e;
	const char *edm_sr_name;
	int edm_sr_name_len;
	e = Jim_GetOpt_String(&goi, &edm_sr_name, &edm_sr_name_len);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	uint32_t edm_sr_number;
	uint32_t edm_sr_value;
	if (strncmp(edm_sr_name, "edm_dtr", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDM_DTR;
	else if (strncmp(edm_sr_name, "edmsw", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDMSW;
	else
		return ERROR_FAIL;

	struct target *target = Jim_CmdPrivData(goi.interp);
	char data_str[12];

	aice_read_debug_reg(target, edm_sr_number, &edm_sr_value);

	sprintf(data_str, "0x%08x", edm_sr_value);
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	Jim_AppendStrings(interp, Jim_GetResult(interp), data_str, NULL);

	return ERROR_OK;
}

static int jim_nds32_write_edm_sr(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 2) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <edm_sr_name> <value>", cmd_name);
		return JIM_ERR;
	}

	int e;
	const char *edm_sr_name;
	int edm_sr_name_len;
	e = Jim_GetOpt_String(&goi, &edm_sr_name, &edm_sr_name_len);
	if (e != JIM_OK)
		return e;

	jim_wide value;
	e = Jim_GetOpt_Wide(&goi, &value);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	uint32_t edm_sr_number;
	if (strncmp(edm_sr_name, "edm_dtr", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDM_DTR;
	else
		return ERROR_FAIL;

	struct target *target = Jim_CmdPrivData(goi.interp);

	aice_write_debug_reg(target, edm_sr_number, value);

	return ERROR_OK;
}

char **aice_custom_monitor_command;
int  *len;
COMMAND_HANDLER(handle_nds32_custom_monitor_command)
{	
    struct target *target = get_current_target(CMD_CTX);
    struct nds32 *nds32 = target_to_nds32(target);

    if (!is_nds32(nds32)) {
        command_print(CMD, "current target isn't an Andes core");
        return ERROR_FAIL;
    }
    unsigned int i;
    char *ret_data = NULL;  //NOUSE

    aice_custom_monitor_command = (char**)malloc(CMD_ARGC*sizeof(char*));
    len                         = (int*)malloc(CMD_ARGC*sizeof(int));
    for( i=0; i < CMD_ARGC; i++ ) {
        len[i] = strlen(CMD_ARGV[i]);
        aice_custom_monitor_command[i] = strdup(CMD_ARGV[i]);
    }

    LOG_DEBUG("handle_nds32_custom_monitor_command #: %d", CMD_ARGC);
    int retval = aice_monitor_command(target, CMD_ARGC, aice_custom_monitor_command, len, &ret_data);

    // Free
    for( i=0; i < CMD_ARGC; i++ ) {
        if( aice_custom_monitor_command[i] != NULL )
            free(aice_custom_monitor_command[i]);
    }
    free(aice_custom_monitor_command);
    free(len);
    free(ret_data);

    return retval;
}

COMMAND_HANDLER(handle_nds32_dump_alltlb_command)
{
    struct target *target = get_current_target(CMD_CTX);
    struct nds32 *nds32 = target_to_nds32(target);

    if (!is_nds32(nds32)) {
        command_print(CMD, "current target isn't an Andes core");
        return ERROR_FAIL;
    }
    char *filename = NULL;

    if( CMD_ARGC != 1 ) {
        LOG_ERROR( "Usage: dump all_tlb filename");
    }

    filename = strdup(CMD_ARGV[0]); 
    LOG_DEBUG("dump all tlb to file: %s", filename);

    int retval = aice_dump_tlb( target, filename );

	return retval;
}

COMMAND_HANDLER(handle_nds32_reset_and_hold)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		if (is_ndsv5(target)) {
			return handle_ndsv5_reset_and_hold(cmd);
		}
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	target->reset_halt = true;
	int retval = nds32_assert_reset(target);
	return retval;
}
COMMAND_HANDLER(handle_nds32_custom_srst_script_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		aice_usb_set_custom_srst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(handle_nds32_custom_trst_script_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		aice_usb_set_custom_trst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(handle_nds32_custom_restart_script_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		aice_usb_set_custom_restart_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

extern char *custom_initial_script;
COMMAND_HANDLER(handle_nds32_custom_initial_script_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		custom_initial_script = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

extern uint32_t aice_no_crst_detect;
COMMAND_HANDLER(handle_nds32_no_crst_detect_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], aice_no_crst_detect);
	else
		LOG_ERROR("expected exactly one argument to aice no_crst_detect");
	nds_no_crst_detect = aice_no_crst_detect;
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_dis_deh_sel_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC == 1) {
		uint32_t if_disable = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], if_disable);
		aice_set_dis_DEH_SEL(if_disable);
	}
	else
		LOG_ERROR("expected exactly one argument to aice no_crst_detect");

	return ERROR_OK;
}

extern unsigned int diagnosis_memory;
extern unsigned int diagnosis_address;
extern unsigned int aice_do_diagnosis;

COMMAND_HANDLER(handle_nds32_diagnosis_command)
{
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], diagnosis_memory);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], diagnosis_address);
	LOG_DEBUG("aice_handle_aice_diagnosis_command %x, %x", diagnosis_memory, diagnosis_address);
	aice_do_diagnosis = 1;
	return ERROR_OK;
}

extern const char *burner_port;
COMMAND_HANDLER(handle_nds32_burn_port_command)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		burner_port = strdup(CMD_ARGV[0]);
		LOG_DEBUG("nds burn_port %s", burner_port);
	}
	else
		LOG_ERROR("expected more argument to nds burn_port <burner_port>");

	return ERROR_OK;
}

extern uint32_t DIMBR_D4;
COMMAND_HANDLER(handle_nds32_edm_dimb)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], DIMBR_D4);
		LOG_DEBUG("change DIMBR to 0x%x", DIMBR_D4);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

extern unsigned long nds32_chk_tracer_support;
extern int burner_debug_mode;
extern void nds32_reg_set_cop_reg_nums(uint32_t cop_id, uint32_t cop_reg_nums, uint32_t if_ena);
extern unsigned int log_usb_packets;
extern uint32_t aice_usb_pack_command;
COMMAND_HANDLER(handle_nds32_misc_config)
{
	uint32_t cop_reg_nums = 0, if_ena = 0, reg_id = 0;
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "tracer_disable") == 0) {
			LOG_DEBUG("tracer_disable");
#if NDS32_TRACER_SUPPORT
			nds32_chk_tracer_support = 0xFF;
#endif
		} else if (strcmp(CMD_ARGV[0], "usb_pack_disable") == 0) {
			LOG_DEBUG("usb_pack_disable");
			aice_usb_pack_command = 0;
		} else if (strcmp(CMD_ARGV[0], "usb_pack_level") == 0) {
			uint32_t usb_pack_level = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], usb_pack_level);
			LOG_DEBUG("usb_pack_level %d", usb_pack_level);
			if (aice_usb_pack_command != 0) {
				if (usb_pack_level == 2)
					aice_usb_pack_command = 2;
				else if (usb_pack_level == 1)
					aice_usb_pack_command = 1;
				else
					aice_usb_pack_command = 0;
			}
		} else if (strcmp(CMD_ARGV[0], "usb_log_enable") == 0) {
			LOG_DEBUG("usb_log_enable");
			uint32_t log_usb_ena = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], log_usb_ena);
			if (log_usb_ena == 1)
				log_usb_packets = 1;
			else
				log_usb_packets = 0;
		} else if (strcmp(CMD_ARGV[0], "burner_debug_enable") == 0) {
			LOG_DEBUG("burner_debug_enable");
#if NDS32_BURNER_SERVER_SUPPORT
			burner_debug_mode = 1;
#endif
		} else if (strcmp(CMD_ARGV[0], "cop") == 0) {
			if (CMD_ARGC < 4)
				return ERROR_FAIL;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_id);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], cop_reg_nums);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], if_ena);
			LOG_DEBUG("cop%d, cop_reg_nums=%d, if_ena=%d", reg_id, cop_reg_nums, if_ena);
			nds32_reg_set_cop_reg_nums(reg_id, cop_reg_nums, if_ena);
		} else if (strcmp(CMD_ARGV[0], "usb_timeout") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], aice_set_usb_timeout);
		}
	}
	return ERROR_OK;
}

extern uint32_t l2c_pa_base;
COMMAND_HANDLER(handle_nds32_l2c_base)
{
	LOG_DEBUG("%s", __func__);
	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], l2c_pa_base);
		LOG_DEBUG("change L2C Base to 0x%x", l2c_pa_base);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

extern uint32_t aice_default_use_sdm, aice_current_use_sdm;
extern uint32_t aice_sdm_support_ena;
extern int aice_sdm_direct_select(uint32_t coreid);
extern int aice_sdm_write_misc(uint32_t address, uint32_t val);
extern int aice_sdm_scan_chain(uint32_t *p_idcode, uint8_t *num_of_idcode);
extern int aice_sdm_daisy_chain(void);

COMMAND_HANDLER(aice_handle_sdm)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if ((!is_nds32(nds32)) && (!is_ndsv5(target)) ) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "enable") == 0) {
			aice_sdm_support_ena = 1;
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			aice_sdm_support_ena = 0;
		} else if (strcmp(CMD_ARGV[0], "use_sdm") == 0) {
			aice_default_use_sdm = 1;
			aice_sdm_support_ena = 1;
		} else if (strcmp(CMD_ARGV[0], "type_daisy_chain") == 0) {
			aice_sdm_daisy_chain();
		} else if (strcmp(CMD_ARGV[0], "type_direct") == 0) {
			aice_current_use_sdm = 1;
		} else if (strcmp(CMD_ARGV[0], "scan_chain") == 0) {
			uint32_t i, idcode[16];
			uint8_t num_of_idcode = 0;
			for (i=0; i<16; i++)
				idcode[i] = 0;
			aice_sdm_scan_chain(&idcode[0], &num_of_idcode);
		} else if (strcmp(CMD_ARGV[0], "direct_select") == 0) {
			aice_current_use_sdm = 1;
			uint32_t select_core = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], select_core);
			aice_sdm_direct_select(select_core);
		} else if (strcmp(CMD_ARGV[0], "write_misc") == 0) {
			uint32_t write_misc_addr = 0, write_misc_data = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], write_misc_addr);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], write_misc_data);
			aice_sdm_write_misc(write_misc_addr, write_misc_data);
		}
	}
	return ERROR_OK;
}


static int parseLine(char* line)
{
	// This assumes that a digit will be found and the line ends in " Kb".
	int i = strlen(line);
	const char* p = line;
	while (*p <'0' || *p > '9') p++;
	line[i-3] = '\0';
	i = atoi(p);
	return i;
}

COMMAND_HANDLER(handle_openocd_info)
{
	FILE* file = NULL;
	file = fopen("/proc/self/status", "r");
	if( !file ) {
		printf("Unable to open /proc/self/status\n");
		return ERROR_OK;
	}

	int result_VmSize = -1;
	int result_VmRSS = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL){
		if (strncmp(line, "VmSize:", 7) == 0) {		// Virtual Memory currently used by current process
			//printf("%s\n", line);
			result_VmSize = parseLine(line);
		} else if (strncmp(line, "VmRSS:", 6) == 0)  {	// Physical Memory currently used by current process
			//printf("%s\n", line);
			result_VmRSS = parseLine(line);
		}
	}
	fclose(file);

	printf("Current OpenOCD VmSize: %d Kb\n", result_VmSize);
	printf("Current OpenOCD VmRSS : %d Kb\n", result_VmRSS);
	printf("Current OpenOCD FD_SETSIZE: %d\n", FD_SETSIZE);

	return ERROR_OK;
}

static const struct command_registration nds32_query_command_handlers[] = {
	{
		.name = "target",
		.handler = handle_nds32_query_target_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "reply 'OCD' for gdb to identify server-side is OpenOCD",
	},
	{
		.name = "endian",
		.handler = handle_nds32_query_endian_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query target endian",
	},
	{
		.name = "cpuid",
		.handler = handle_nds32_query_cpuid_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query CPU ID",
	},
	{
		.name = "capability",
		.handler = handle_nds32_query_capability_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query target capability",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nds32_dump_command_handlers[] = {
	{
		.name = "all_tlb",
		.handler = handle_nds32_dump_alltlb_command,
		.mode = COMMAND_EXEC,
		.usage = "dump all_tlb filename",
		.help = "Dump all TLB  entries to file",
    },

    COMMAND_REGISTRATION_DONE
};

const struct command_registration nds32_exec_command_handlers[] = {
	{
		.name = "dssim",
		.handler = handle_nds32_dssim_command,
		.mode = COMMAND_EXEC,
		.usage = "['on'|'off']",
		.help = "display/change $INT_MASK.DSSIM status",
	},
	{
		.name = "mem_access",
		.handler = handle_nds32_memory_access_command,
		.mode = COMMAND_EXEC,
		.usage = "['bus'|'cpu']",
		.help = "display/change memory access channel",
	},
	{
		.name = "mem_mode",
		.handler = handle_nds32_memory_mode_command,
		.mode = COMMAND_EXEC,
		.usage = "['auto'|'mem'|'ilm'|'dlm']",
		.help = "display/change memory mode",
	},
	{
		.name = "cache",
		.handler = handle_nds32_cache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate']",
		.help = "cache control",
	},
	{
		.name = "icache",
		.handler = handle_nds32_icache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate'|'enable'|'disable'|'dump']",
		.help = "icache control",
	},
	{
		.name = "dcache",
		.handler = handle_nds32_dcache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate'|'enable'|'disable'|'dump']",
		.help = "dcache control",
	},
	{
		.name = "tlb",
		.handler = handle_nds32_tlb_command,
		.mode = COMMAND_EXEC,
		.usage = "['dump']",
		.help = "tlb control",
	},
	{
		.name = "auto_break",
		.handler = handle_nds32_auto_break_command,
		.mode = COMMAND_EXEC,
		.usage = "['on'|'off']",
		.help = "convert software breakpoints to hardware breakpoints if needed",
	},
	{
		.name = "bp",
		.handler = handle_nds32_set_hw_break_command,
		.mode = COMMAND_EXEC,
		.help = "list or set hardware breakpoint",
		.usage = "<address> <length>",
	},
	{
		.name = "rbp",
		.handler = handle_nds32_clr_hw_break_command,
		.mode = COMMAND_EXEC,
		.help = "remove breakpoint",
		.usage = "address",
	},
	{
		.name = "wp",
		.handler = handle_nds32_set_watch_command,
		.mode = COMMAND_EXEC,
		.help = "list (no params) or create watchpoints",
		.usage = "[address length [('r'|'w'|'a') value [mask]]]",
	},
	{
		.name = "rwp",
		.handler = handle_nds32_clr_watch_command,
		.mode = COMMAND_EXEC,
		.help = "remove watchpoint",
		.usage = "address",
	},

	{
		.name = "global_stop",
		.handler = handle_nds32_global_stop_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "turn on/off global stop. After turning on, every load/store" \
			 "instructions will be stopped to check memory access.",
	},
	{
		.name = "set_ACR",
		.handler = handle_nds32_set_ACR_command,
		.mode = COMMAND_ANY,
		.usage = "[register] [value] , eg: r200b_1 0x112233445566778899",
		.help = "set ACR value",
	},
	{
		.name = "va",
		.handler = handle_nds32_va_to_pa_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "turn on/off va_to_pa.",
	},
	{
		.name = "security_compat_display",
		.handler = handle_nds32_security_compat_display_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "display IPC and IPSW.",
	},
	{
		.name = "disable_global_stop_warning",
		.handler = handle_nds32_dis_global_stop_warning_command,
		.mode = COMMAND_ANY,
		.usage = "['1'|'0']",
		.help = "disable global stop warning message.",
	},
	{
		.name = "bytecode_parsing",
		.handler = handle_nds32_bytecode_parsing_command,
		.mode = COMMAND_ANY,
		.usage = "['1'|'0']",
		.help = "enable bytecode_parsing.",
	},
	{
		.name = "soft_reset_halt",
		.handler = handle_nds32_soft_reset_halt_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "as issuing rest-halt, to use soft-reset-halt or not." \
			 "the feature is for backward-compatible.",
	},
	{
		.name = "boot_time",
		.handler = handle_nds32_boot_time_command,
		.mode = COMMAND_ANY,
		.usage = "milliseconds",
		.help = "set the period to wait after srst.",
	},
	{
		.name = "reset_time",
		.handler = handle_nds32_reset_time_command,
		.mode = COMMAND_ANY,
		.usage = "milliseconds",
		.help = "set the period to wait after restart.",
	},
	{
		.name = "runtest_num_clocks",
		.handler = handle_nds32_runtest_num_clocks_command,
		.mode = COMMAND_ANY,
		.usage = "clocks",
		.help = "set the runtest_num_clocks of nds-ftdi.",
	},
	{
		.name = "scan_field_max",
		.handler = handle_nds32_scan_field_max_command,
		.mode = COMMAND_ANY,
		.usage = "scans",
		.help = "set the max_scans of nds-ftdi memory access.",
	},
	{
		.name = "ftdi_jtag_opt",
		.handler = handle_nds32_ftdi_jtag_opt_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "",
	},
	{
		.name = "ftdi_log_detail",
		.handler = handle_nds32_ftdi_log_detail_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "",
	},
	{
		.name = "log_file_size",
		.handler = handle_nds32_log_file_size_command,
		.mode = COMMAND_ANY,
		.usage = "bytes",
		.help = "set the log file size.",
	},
	{
		.name = "idlm_base_size",
		.handler = handle_nds32_idlm_base_size_command,
		.mode = COMMAND_ANY,
		.usage = "ilm_base ilm_size dlm_base dlm_size",
		.help = "set the ILM/DLM base & size.",
	},
	{
		.name = "login_edm_passcode",
		.handler = handle_nds32_login_edm_passcode_command,
		.mode = COMMAND_ANY,
		.usage = "passcode",
		.help = "set EDM passcode for secure MCU debugging.",
	},
#if 0
	{
		.name = "login_edm_operation",
		.handler = handle_nds32_login_edm_operation_command,
		.mode = COMMAND_ANY,
		.usage = "login_edm_operation misc_reg_no value",
		.help = "add EDM operations for secure MCU debugging.",
	},
#endif
	{
		.name = "reset_halt_as_init",
		.handler = handle_nds32_reset_halt_as_init_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "reset halt as openocd init.",
	},
	{
		.name = "keep_target_edm_ctl",
		.handler = handle_nds32_keep_target_edm_ctl_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "Backup/Restore target EDM_CTL register.",
	},
	{
		.name = "decode",
		.handler = handle_nds32_decode_command,
		.mode = COMMAND_EXEC,
		.usage = "address icount",
		.help = "decode instruction.",
	},
	{
		.name = "word_access_mem",
		.handler = handle_nds32_word_access_mem_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "Always use word-aligned address to access memory.",
	},
	{
		.name = "memAccSize",
		.handler = handle_nds32_memAccSize_command,
		.mode = COMMAND_ANY,
		.usage = "<lowaddr> <highaddr> <access_size>",
		.help = "Write memory access attribute.",
	},
	{
		.name = "reset_memAccSize",
		.handler = handle_nds32_reset_memAccSize_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "reset memory access attribute.",
	},
	{
		.name = "write_buffer",
		.handler = handle_nds32_write_buffer_command,
		.mode = COMMAND_ANY,
		.usage = "<address> <count> <data>",
		.help = "Write data to target buffer.",
	},
	{
		.name = "read_buffer",
		.handler = handle_nds32_read_buffer_command,
		.mode = COMMAND_ANY,
		.usage = "<address> <count>",
		.help = "Read data from target buffer.",
	},
	{
		.name = "bulk_write",
		.jim_handler = jim_nds32_bulk_write,
		.mode = COMMAND_EXEC,
		.help = "Write multiple 32-bit words to target memory",
		.usage = "address count data",
	},
	{
		.name = "multi_write",
		.jim_handler = jim_nds32_multi_write,
		.mode = COMMAND_EXEC,
		.help = "Write multiple addresses/word|half word|byte to target memory",
		.usage = "data_type num_of_pairs [address data]+",
	},
	{
		.name = "bulk_read",
		.jim_handler = jim_nds32_bulk_read,
		.mode = COMMAND_EXEC,
		.help = "Read multiple 32-bit words from target memory",
		.usage = "address count",
	},
	{
		.name = "read_edmsr",
		.jim_handler = jim_nds32_read_edm_sr,
		.mode = COMMAND_EXEC,
		.help = "Read EDM system register",
		.usage = "['edmsw'|'edm_dtr']",
	},
	{
		.name = "write_edmsr",
		.jim_handler = jim_nds32_write_edm_sr,
		.mode = COMMAND_EXEC,
		.help = "Write EDM system register",
		.usage = "['edm_dtr'] value",
	},
	{
		.name = "bench",
		.handler = handle_nds32_bench_command,
		.mode = COMMAND_ANY,
		.help = "bench performance",
		.usage = "item",
	},
	{
		.name = "configure",
		.handler = handle_nds32_configure_command,
		.mode = COMMAND_ANY,
		.help = "Configure/Query property",
		.usage = "property [value]",
	},
	{
		.name = "query",
		.mode = COMMAND_EXEC,
		.help = "Andes query command group",
		.usage = "",
		.chain = nds32_query_command_handlers,
	},
	{
		.name = "ace",
		.handler = handle_nds32_ace_command,
		.mode = COMMAND_ANY,
		.usage = "sysname",
		.help = "Get ACE share library file name for platform sysname",
	},
#if 1
	{
		.name = "cop0",
		.handler = handle_nds32_cop0_command,
		.mode = COMMAND_EXEC,
		.usage = "sysname",
		.help = "Get coprocessor 0 share library file name for platform sysname",
	},
	{
		.name = "cop1",
		.handler = handle_nds32_cop1_command,
		.mode = COMMAND_EXEC,
		.usage = "sysname",
		.help = "Get coprocessor 1 share library file name for platform sysname",
	},
	{
		.name = "cop2",
		.handler = handle_nds32_cop2_command,
		.mode = COMMAND_EXEC,
		.usage = "sysname",
		.help = "Get coprocessor 2 share library file name for platform sysname",
	},
	{
		.name = "cop3",
		.handler = handle_nds32_cop3_command,
		.mode = COMMAND_EXEC,
		.usage = "sysname",
		.help = "Get coprocessor 3 share library file name for platform sysname",
	},
#endif
#if 0
	{
		.name = "test",
		.mode = COMMAND_EXEC,
		.help = "Andes test command group",
		.usage = "",
		.chain = nds32_test_command_handlers,
	},
#endif
    {
        .name = "custom_monitor_cmd",
        .handler = handle_nds32_custom_monitor_command,
        .mode = COMMAND_ANY,
        .usage = "Data1 Data2 ...",
        .help = "Send custom command to AICE while using pipe mode",
    },
    {
        .name = "dump",
        .mode = COMMAND_EXEC,
        .usage = "",
        .help = "Andes dump command group",
        .chain = nds32_dump_command_handlers,
    },
#if NDS32_TRACER_SUPPORT
	{
		.name = "trace",
		.handler = handle_nds32_tracer_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "tracer setting",
	},
#endif

	{
		.name = "reset_and_hold",
		.handler = handle_nds32_reset_and_hold,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "reset_and_hold",
	},
	{
		.name = "no_crst_detect",
		.handler = handle_nds32_no_crst_detect_command,
		.mode = COMMAND_ANY,
		.help = "No CRST detection in debug session",
		.usage = "no_crst_detect 0",
	},
	{
		.name = "dis_deh_sel",
		.handler = handle_nds32_dis_deh_sel_command,
		.mode = COMMAND_ANY,
		.help = "disable DEH_SEL when exiting debug session",
		.usage = "dis_deh_sel 1",
	},
	{
		.name = "custom_srst_script",
		.handler = handle_nds32_custom_srst_script_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "custom_srst_script",
	},
	{
		.name = "custom_trst_script",
		.handler = handle_nds32_custom_trst_script_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "custom_trst_script",
	},
	{
		.name = "custom_restart_script",
		.handler = handle_nds32_custom_restart_script_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "custom_restart_script",
	},
	{
		.name = "custom_initial_script",
		.handler = handle_nds32_custom_initial_script_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "custom_initial_script",
	},
	{
		.name = "diagnosis",
		.handler = handle_nds32_diagnosis_command,
		.mode = COMMAND_ANY,
		.usage = "diagnosis memory",
		.help = "diagnosis",
	},
	{
		.name = "burn_port",
		.handler = handle_nds32_burn_port_command,
		.mode = COMMAND_ANY,
		.usage = "burn_port [burner_port]",
		.help = "set burner port",
	},
	{
		.name = "edm_dimb",
		.handler = handle_nds32_edm_dimb,
		.mode = COMMAND_ANY,
		.usage = "edm_dimb <Debug Instruction Memory Base>",
		.help = "set Debug Instruction Memory Base register value.",
	},
	{
		.name = "misc_config",
		.handler = handle_nds32_misc_config,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "others config",
	},
	{
		.name = "l2c_base",
		.handler = handle_nds32_l2c_base,
		.mode = COMMAND_ANY,
		.usage = "l2c_base <L2C Base>",
		.help = "set L2C Base addess.",
	},
	{
		.name = "sdm",
		.handler = &aice_handle_sdm,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "",
	},
	{
		.name = "openocd_info",
		.handler = handle_openocd_info,
		.mode = COMMAND_ANY,
		.usage = "openocd_mem",
		.help = "Print current OpenOCD info.",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration nds32_command_handlers[] = {
	{
		.name = "nds",
		.mode = COMMAND_ANY,
		.help = "Andes command group",
		.usage = "",
		.chain = nds32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

