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
void ndsv5_decode_csr(char *text, unsigned address, unsigned data);


int ndsv5_vector_restore_vtype_vl(struct target *target, uint64_t reg_vtype);
int ndsv5_get_vector_VLMAX(struct target *target);
int ndsv5_get_vector_register(struct target *target, enum gdb_regno r, char *pRegValue);
int ndsv5_set_vector_register(struct target *target, enum gdb_regno r, char *pRegValue);



#include "target/ndsv5_ace.h"
bool isAceCsrEnable;
struct riscv_batch *busmode_batch;
uint32_t nds_sys_bus_supported;
extern uint32_t nds_dmi_access_mem;
extern uint32_t nds_halt_on_reset;
extern uint32_t nds_dmi_busy_retry_times;
uint32_t read_abstract_reg_number, write_abstract_reg_number;
#define MAX_RETRY  3

#if _NDS_DMI_CHECK_TIMEOUT_
struct timeval begin_time;
struct timeval end_time;
extern uint32_t v5_count_to_check_dm;
double timeval_diff_ms(struct timeval *a_timeval_begin, struct timeval *a_timeval_end);
int dmi_check_timeout(struct target *target);
#endif /* _NDS_DMI_CHECK_TIMEOUT_ */

#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
struct riscv_batch *write_debug_buffer_batch;
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF */

#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
uint64_t backup_debug_buffer[RISCV_MAX_HARTS][16];
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF */

#if _NDS_MEM_Q_ACCESS_
int ndsv5_read_memory_quick_access(struct target *target, target_addr_t address,
				uint32_t size, uint32_t count, uint8_t *buffer);
#endif /* _NDS_MEM_Q_ACCESS_ */

/*
const char * const dmi_reg_string[] = {
	"",                  // 0x00
	"",                  // 0x01
	"",                  // 0x02
	"",                  // 0x03
	"DMI_DATA0",         // 0x04
	"DMI_DATA1",         // 0x05
	"DMI_DATA2",         // 0x06
	"DMI_DATA3",         // 0x07
	"DMI_DATA4",         // 0x08
	"DMI_DATA5",         // 0x09
	"DMI_DATA6",         // 0x0a
	"DMI_DATA7",         // 0x0b
	"DMI_DATA8",         // 0x0c
	"DMI_DATA9",         // 0x0d
	"DMI_DATA10",        // 0x0e
	"DMI_DATA11",        // 0x0f
	"DMI_DMCONTROL",     // 0x10
	"DMI_DMSTATUS",      // 0x11
	"DMI_HARTINFO",      // 0x12
	"DMI_HALTSUM",       // 0x13
	"DMI_HAWINDOWSEL",   // 0x14
	"DMI_HAWINDOW",      // 0x15
	"DMI_ABSTRACTCS",    // 0x16
	"DMI_COMMAND",       // 0x17
	"DMI_ABSTRACTAUTO",  // 0x18
	"DMI_DEVTREEADDR0",  // 0x19
	"DMI_DEVTREEADDR1",  // 0x1a
	"DMI_DEVTREEADDR2",  // 0x1b
	"DMI_DEVTREEADDR3",  // 0x1c
	"",                  // 0x1d
	"",                  // 0x1e
	"",                  // 0x1f
	"DMI_PROGBUF0",      // 0x20
	"DMI_PROGBUF1",      // 0x21
	"DMI_PROGBUF2",      // 0x22
	"DMI_PROGBUF3",      // 0x23
	"DMI_PROGBUF4",      // 0x24
	"DMI_PROGBUF5",      // 0x25
	"DMI_PROGBUF6",      // 0x26
	"DMI_PROGBUF7",      // 0x27
	"DMI_PROGBUF8",      // 0x28
	"DMI_PROGBUF9",      // 0x29
	"DMI_PROGBUF10",     // 0x2a
	"DMI_PROGBUF11",     // 0x2b
	"DMI_PROGBUF12",     // 0x2c
	"DMI_PROGBUF13",     // 0x2d
	"DMI_PROGBUF14",     // 0x2e
	"DMI_PROGBUF15",     // 0x2f
	"DMI_AUTHDATA",      // 0x30
	"",                  // 0x31
	"",                  // 0x32
	"",                  // 0x33
	"",                  // 0x34
	"",                  // 0x35
	"",                  // 0x36
	"",                  // 0x37
	"DMI_SBCS",          // 0x38
	"DMI_SBADDRESS0",    // 0x39
	"DMI_SBADDRESS1",    // 0x3a
	"DMI_SBADDRESS2",    // 0x3b
	"DMI_SBDATA0",       // 0x3c
	"DMI_SBDATA1",       // 0x3d
	"DMI_SBDATA2",       // 0x3e
	"DMI_SBDATA3",       // 0x3f
};
*/
extern uint32_t nds_jtag_scans_optimize;
extern uint32_t nds_jtag_max_scans;
unsigned acr_reg_count_v5;
unsigned acr_type_count_v5;
extern uint32_t v5_dmi_busy_delay_count;
bool reset_halt;
extern bool rv32e;
int read_memory_bus_v1_opt(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
int write_memory_bus_v1_opt(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);
extern uint32_t nds_no_crst_detect;



#endif /* __NDSV5_013_H_ */
