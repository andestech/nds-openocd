/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifndef __NDSV5_013_H_
#define __NDSV5_013_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/breakpoints.h"
#include "target/target_type.h"
#include "riscv.h"
#include "encoding.h"

/* All this functions are implement in riscv-013.c */

int ndsv5_set_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t reg_value);
int ndsv5_get_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t *preg_value);
int ndsv5_haltonreset(struct target *target, int enable);
int riscv013_poll_wo_announce(struct target *target);
int ndsv5_reset_halt_as_examine(struct target *target);
int ndsv5_access_memory_pack_batch_run(struct target *target, uint32_t free_after_run);
int ndsv5_read_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
int ndsv5_write_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);


int ndsv5_vector_restore_vtype_vl(struct target *target, uint64_t reg_vtype);
int ndsv5_get_vector_VLMAX(struct target *target);
int ndsv5_get_vector_register(struct target *target, enum gdb_regno r, char *pRegValue);
int ndsv5_set_vector_register(struct target *target, enum gdb_regno r, char *pRegValue);

#endif
