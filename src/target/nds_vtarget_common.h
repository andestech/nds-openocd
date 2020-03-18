#ifndef NDS_VTARGET_COMMON_H
#define NDS_VTARGET_COMMON_H

#include <stdint.h>
#include "riscv/opcodes.h"
#include "riscv/gdb_regs.h"
/** for target->rtos->hart_unavailable, riscv_update_threads, etc. **/
#include "rtos/riscv_debug.h"
#include "riscv/debug_defines.h"
#include "riscv/encoding.h"

/**
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 *	jtag_add_[id]r_scan
 *	jtag_execute_query
 *	jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 *	dtmcontrol_scan
 *	idcode_scan
 *	dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 *	scans_new
 *	scans_delete
 *	scans_execute
 *	scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 *	cache_get...
 *	cache_set...
 *	cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

/*** JTAG registers. ***/

#define DTMCONTROL					0x10
#define DTMCONTROL_DBUS_RESET		(1<<16)
#define DTMCONTROL_IDLE				(7<<10)
#define DTMCONTROL_ADDRBITS			(0xf<<4)
#define DTMCONTROL_VERSION			(0xf)

#define DBUS						0x11
#define DBUS_OP_START				0
#define DBUS_OP_SIZE				2
typedef enum {
	DBUS_OP_NOP = 0,
	DBUS_OP_READ = 1,
	DBUS_OP_WRITE = 2
} dbus_op_t;

typedef enum {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
} dmi_op_t;
typedef enum {
#if _NDS_DMI_CHECK_TIMEOUT_
	DMI_STATUS_CHECK_TIMEOUT = 0xFF,
#endif
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
} dmi_status_t;

#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

/*** Debug Bus registers. ***/
#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(((uint64_t)1)<<33)
#define DMCONTROL_HALTNOT		(((uint64_t)1)<<32)
#define DMCONTROL_BUSERROR		(7<<19)
#define DMCONTROL_SERIAL		(3<<16)
#define DMCONTROL_AUTOINCREMENT	(1<<15)
#define DMCONTROL_ACCESS		(7<<12)
#define DMCONTROL_HARTID		(0x3ff<<2)
#define DMCONTROL_NDRESET		(1<<1)
#define DMCONTROL_FULLRESET		1

/*** Info about the core being debugged. ***/
#define DBUS_ADDRESS_UNKNOWN	0xffff
#define WALL_CLOCK_TIMEOUT		2

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16
#define MAX_RETRY  3

/* The register cache is staticly allocated. */
#define RISCV_MAX_HARTS 1
#define RISCV_MAX_REGISTERS 5000
#define RISCV_MAX_TRIGGERS 32
#define RISCV_MAX_HWBPS 16

#define DEFAULT_COMMAND_TIMEOUT_SEC		2
#define DEFAULT_RESET_TIMEOUT_SEC		15

/*
 * Definitions shared by code supporting all RISC-V versions.
 */
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;

typedef struct {
	unsigned dtm_version;

	riscv_reg_t misa;

	struct command_context *cmd_ctx;

	/* The hart that the RTOS thinks is currently being debugged. */
	int rtos_hartid;

	/* The hart that is currently being debugged.  Note that this is
	 * different than the hartid that the RTOS is expected to use.  This
	 * one will change all the time, it's more of a global argument to
	 * every function than an actual */
	int current_hartid;

	/* The register cache points into here. */
	uint64_t reg_cache_values[RISCV_MAX_REGISTERS];

	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;

	/* It's possible that each core has a different supported ISA set. */
	int xlen;

	/* This avoids invalidating the register cache too often. */
	bool registers_initialized;

	/** 013 version specific */
	/* Number of address bits in the dbus register. */
	unsigned abits;
	/* Number of abstract command data registers. */
	unsigned datacount;
	/* Number of words in the Program Buffer. */
	unsigned progbufsize;

	/* We cache the read-only bits of sbcs here. */
	uint32_t sbcs;

	/* Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned int dtmcontrol_idle;

	/* This value is incremented every time a dbus access comes back as "busy".
	 * It's used to determine how many run-test/idle cycles to feed the target
	 * in between accesses. */
	unsigned int dmi_busy_delay;

	/* Number of run-test/idle cycles to add between consecutive bus master
	 * reads/writes respectively. */
	unsigned int bus_master_write_delay, bus_master_read_delay;

	/* This value is increased every time we tried to execute two commands
	 * consecutively, and the second one failed because the previous hadn't
	 * completed yet.  It's used to add extra run-test/idle cycles after
	 * starting a command, so we don't have to waste time checking for busy to
	 * go low. */
	unsigned int ac_busy_delay;

	bool abstract_read_csr_supported;
	bool abstract_write_csr_supported;
	bool abstract_read_fpr_supported;
	bool abstract_write_fpr_supported;
} riscv_info_t;

/* Everything needs the RISC-V specific info structure, so here's a nice macro
 * that provides that. */
static inline riscv_info_t *riscv_info(const struct target *target) __attribute__((unused));
static inline riscv_info_t *riscv_info(const struct target *target)
{ return target->arch_info; }
#define RISCV_INFO(R) riscv_info_t *R = riscv_info(target);

/* gdb's register list is defined in ianchi_riscv_gdb_reg_names gdb/ianchi_riscv-tdep.c in
 * its source tree. We must interpret the numbers the same here. */

enum {
	REG_XPR0 = 0,
	REG_XPR31 = 31,
	REG_PC = 32,
	REG_FPR0 = 33,
	REG_FPR31 = 64,
	REG_CSR0 = 65,
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,
	REG_CSR4095 = 4160,
	REG_PRIV = 4161,
	REG_V0,
	REG_V31 = REG_V0 + 31,
	REG_ALL_COUNT
};

/** extern variable **/
/** in nds_vtarget.c **/
/** for dmi **/
extern uint8_t vtarget_ir_dtmcontrol[1];
extern struct scan_field vtarget_select_dtmcontrol;
extern uint8_t vtarget_ir_dbus[1];
extern struct scan_field vtarget_select_dbus;
extern uint8_t vtarget_ir_idcode[1];
extern struct scan_field vtarget_select_idcode;
extern uint32_t vtarget_jtag_scans_optimize;

/** in nds_vtarget_cmd.c **/
extern uint32_t vtarget_dmi_busy_delay_count;
extern uint32_t vtarget_xlen;


/** extern functions **/
extern struct reg *vtarget_get_reg_by_CSR(struct target *target, uint32_t csr_id);

#endif /* NDS_VTARGET_COMMON_H */
