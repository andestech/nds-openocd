/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_BREAKPOINTS_H
#define OPENOCD_TARGET_BREAKPOINTS_H

#include <stdint.h>

struct target;

enum breakpoint_type {
	BKPT_HARD,
	BKPT_SOFT,
};

enum watchpoint_rw {
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

struct breakpoint {
	target_addr_t address;
	uint32_t asid;
	int length;
	enum breakpoint_type type;
	int set;
	uint8_t *orig_instr;
	struct breakpoint *next;
	uint32_t unique_id;
	int linked_BRP;
#if _NDS32_ONLY_
	uint8_t *bytecode;
#endif
};

struct watchpoint {
	target_addr_t address;
	uint32_t length;
	uint32_t mask;
	uint32_t value;
	enum watchpoint_rw rw;
	int set;
	struct watchpoint *next;
	int unique_id;
#if _NDS32_ONLY_
	uint8_t *bytecode;
#endif
};

void breakpoint_clear_target(struct target *target);
int breakpoint_add(struct target *target,
		target_addr_t address, uint32_t length, enum breakpoint_type type);
int context_breakpoint_add(struct target *target,
		uint32_t asid, uint32_t length, enum breakpoint_type type);
int hybrid_breakpoint_add(struct target *target,
		target_addr_t address, uint32_t asid, uint32_t length, enum breakpoint_type type);
void breakpoint_remove(struct target *target, target_addr_t address);

struct breakpoint *breakpoint_find(struct target *target, target_addr_t address);

void watchpoint_clear_target(struct target *target);
int watchpoint_add(struct target *target,
		target_addr_t address, uint32_t length,
		enum watchpoint_rw rw, uint32_t value, uint32_t mask);
void watchpoint_remove(struct target *target, target_addr_t address);

/* report type and address of just hit watchpoint */
int watchpoint_hit(struct target *target, enum watchpoint_rw *rw,
		target_addr_t *address);

#if _NDS32_ONLY_
#define BP_WP_LENGTH_MASK    (~0xFF000000)
#define BP_WP_NON_SIMPLE     0x80000000
#define BP_WP_TRIGGER_ON     0x40000000
#define BP_WP_FORCE_VA_ON    0x20000000

#define BP_WP_DATA_COMPARE   0x08000000
#define BP_WP_USER_WATCH     0x04000000
#define BP_WP_CONDITIONAL    0x02000000

int nds32_backup_tmp_bytecode_data(char *p_bytecode);
#endif /* _NDS32_ONLY_ */

#endif /* OPENOCD_TARGET_BREAKPOINTS_H */
