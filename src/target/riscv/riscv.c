#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include "helper/log.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "gdb_regs.h"
#include "rtos/rtos.h"

#if _NDS_V5_ONLY_
#include "target/nds32_v5.h"
#include "jtag/interfaces.h"
#include "target/nds32_v5_ace.h"
int nds_triggered_hart = 0;
uint32_t nds_skip_dmi = 0;
extern unsigned* global_acr_reg_count_v5;
extern uint32_t nds_dmi_quick_access;
extern int nds_targetburn_corenum;
extern uint32_t ndsv5_system_bus_access;

#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
extern uint32_t nds_without_announce;
#endif

static int ndsv5_handle_triggered(struct target *target);
int ndsv5_use_mprv_mode = 0;
int byte_access_from_burn = 0;
uint64_t ndsv5_backup_mstatus = 0;
uint32_t ndsv5_dis_cache_busmode = 1;
extern int ndsv5_get_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t *preg_value);
extern int ndsv5_set_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t reg_value);
#endif

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

/**
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 *		jtag_add_[id]r_scan
 *		jtag_execute_query
 *		jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 *		dtmcontrol_scan
 *		idcode_scan
 *		dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 *		scans_new
 *		scans_delete
 *		scans_execute
 *		scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 *		cache_get...
 *		cache_set...
 *		cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

/* Constants for legacy SiFive hardware breakpoints. */
#define CSR_BPCONTROL_X			(1<<0)
#define CSR_BPCONTROL_W			(1<<1)
#define CSR_BPCONTROL_R			(1<<2)
#define CSR_BPCONTROL_U			(1<<3)
#define CSR_BPCONTROL_S			(1<<4)
#define CSR_BPCONTROL_H			(1<<5)
#define CSR_BPCONTROL_M			(1<<6)
#define CSR_BPCONTROL_BPMATCH	(0xf<<7)
#define CSR_BPCONTROL_BPACTION	(0xff<<11)

#define DEBUG_ROM_START         0x800
#define DEBUG_ROM_RESUME	(DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION	(DEBUG_ROM_START + 8)
#define DEBUG_RAM_START         0x400

#define SETHALTNOT				0x10c

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
	DBUS_STATUS_SUCCESS = 0,
	DBUS_STATUS_FAILED = 2,
	DBUS_STATUS_BUSY = 3
} dbus_status_t;
#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

typedef enum {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
} riscv_error_t;

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

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

#define DMINFO					0x11
#define DMINFO_ABUSSIZE			(0x7fU<<25)
#define DMINFO_SERIALCOUNT		(0xf<<21)
#define DMINFO_ACCESS128		(1<<20)
#define DMINFO_ACCESS64			(1<<19)
#define DMINFO_ACCESS32			(1<<18)
#define DMINFO_ACCESS16			(1<<17)
#define DMINFO_ACCESS8			(1<<16)
#define DMINFO_DRAMSIZE			(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY			(1<<4)
#define DMINFO_AUTHTYPE			(3<<2)
#define DMINFO_VERSION			3

/*** Info about the core being debugged. ***/

#define DBUS_ADDRESS_UNKNOWN	0xffff

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16

uint8_t ir_dtmcontrol[1] = {DTMCONTROL};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};
uint8_t ir_dbus[1] = {DBUS};
struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
uint8_t ir_idcode[1] = {0x1};
struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

#if _NDS_V5_ONLY_
static int modify_trigger_address_mbit_match(struct target *target, struct trigger *trigger);
#endif

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

/* Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

bool riscv_use_scratch_ram;
uint64_t riscv_scratch_ram_address;


/* In addition to the ones in the standard spec, we'll also expose additional
 * CSRs in this list.
 * The list is either NULL, or a series of ranges (inclusive), terminated with
 * 1,0. */
struct {
	uint16_t low, high;
} *expose_csr;

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4];

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCONTROL: 0x%x -> 0x%x", out, in);

	return in;
}

#if _NDS_V5_ONLY_
struct target_type *get_target_type(struct target *target)
#else
static struct target_type *get_target_type(struct target *target)
#endif
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	switch (info->dtm_version) {
		case 0:
			return &riscv011_target;
		case 1:
			return &riscv013_target;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return NULL;
	}
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("riscv_init_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info)
		return ERROR_FAIL;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	riscv_info_init(target, info);
	info->cmd_ctx = cmd_ctx;

	select_dtmcontrol.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_idcode.num_bits = target->tap->ir_length;

	return ERROR_OK;
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	struct target_type *tt = get_target_type(target);
	tt->deinit_target(target);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info);
	target->arch_info = NULL;
}

static int oldriscv_halt(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->halt(target);
}

static void trigger_from_breakpoint(struct trigger *trigger,
		const struct breakpoint *breakpoint)
{
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~0LL;
	trigger->read = false;
	trigger->write = false;
	trigger->execute = true;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = breakpoint->unique_id;
}

static int maybe_add_trigger_t1(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	const uint32_t bpcontrol_x = 1<<0;
	const uint32_t bpcontrol_w = 1<<1;
	const uint32_t bpcontrol_r = 1<<2;
	const uint32_t bpcontrol_u = 1<<3;
	const uint32_t bpcontrol_s = 1<<4;
	const uint32_t bpcontrol_h = 1<<5;
	const uint32_t bpcontrol_m = 1<<6;
	const uint32_t bpcontrol_bpmatch = 0xf << 7;
	const uint32_t bpcontrol_bpaction = 0xff << 11;

	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u, !!(r->misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s, !!(r->misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h, !!(r->misa & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb;
	if (riscv_get_register_on_hart(target, &tdata1_rb, hartid,
				GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int maybe_add_trigger_t2(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	if (trigger->length == 0)
		return ERROR_FAIL;

	/* tselect is already set */
	if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION,
			MCONTROL_ACTION_DEBUG_MODE);
#if _NDS_V5_ONLY_
	tdata1 = set_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)), 2);
	if (trigger->execute) {
		//trigger from breakpoint:ignor address alignment and length issue
		tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	} else {
		if (trigger->length == 1) {
			tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
		} else {
			tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_NAPOT);
		}
	}
#else
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
#endif
	tdata1 |= MCONTROL_M;
	if (r->misa & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;
	if (r->misa & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;
	if (r->misa & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;
	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;
	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	uint64_t tdata1_rb;
	int result = riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);
	if (result != ERROR_OK)
		return result;
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	/* In RTOS mode, we need to set the same trigger in the same slot on every
	 * hart, to keep up the illusion that each hart is a thread running on the
	 * same core. */

	/* Otherwise, we just set the trigger on the one hart this target deals
	 * with. */

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0)
			first_hart = hartid;
		int result = riscv_get_register_on_hart(target, &tselect[hartid],
				hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] != -1)
			continue;

		riscv_set_register_on_hart(target, first_hart, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		int result = riscv_get_register_on_hart(target, &tdata1, first_hart,
				GDB_REGNO_TDATA1);
		if (result != ERROR_OK)
			return result;
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		result = ERROR_OK;
		if (trigger->execute) {
				//trigger from breakpoint:ignor address alignment and length issue
		} else {
				//trigger from watchpoint:address alignment and extend length
				LOG_DEBUG("ori_trigger_address:0x%lx, ori_trigger_length:0x%x", (long unsigned int)trigger->address, trigger->length);
				uint64_t end_address;
				end_address = trigger->address + trigger->length;
				//8bytes alignment:contain rv32(4bytes or rv32dc:8bytes) and rv64(8bytes)
				trigger->address &= ~((0x1 << 3) - 1);
				trigger->length = end_address - trigger->address;
				// redefine: trigger->address
				modify_trigger_address_mbit_match(target, trigger);
		}

		for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
			if (!riscv_hart_enabled(target, hartid))
				continue;
			if (hartid > first_hart)
				riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
			switch (type) {
				case 1:
					result = maybe_add_trigger_t1(target, hartid, trigger, tdata1);
					break;
				case 2:
					result = maybe_add_trigger_t2(target, hartid, trigger, tdata1);
					break;
				default:
					LOG_DEBUG("trigger %d has unknown type %d", i, type);
					result = ERROR_FAIL;
					continue;
			}

			if (result != ERROR_OK)
				continue;
		}

		if (result != ERROR_OK)
			continue;

		LOG_DEBUG("Using trigger %d (type %d) for bp %d", i, type,
				trigger->unique_id);
		r->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT,
				tselect[hartid]);
	}

	if (i >= r->trigger_count[first_hart]) {
#if _NDS_IDE_MESSAGE_
		NDS32_LOG(NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT, riscv_count_triggers(target));
#endif
		LOG_ERROR("Couldn't find an available hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_read_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to read original instruction at 0x%" TARGET_PRIxADDR,
					breakpoint->address);
			return ERROR_FAIL;
		}

		int retval;
		if (breakpoint->length == 4)
			retval = target_write_u32(target, breakpoint->address, ebreak());
		else
			retval = target_write_u16(target, breakpoint->address, ebreak_c());

#if _NDS_V5_ONLY_
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		//struct nds32_v5_memory *memory = &(nds32->memory);
		uint8_t *instr = malloc(4);
		memset(instr, 0, 4);
		target_read_memory(target, breakpoint->address, breakpoint->length, 1,
					instr);
		if(memcmp(breakpoint->orig_instr, instr, breakpoint->length) == 0 )
			retval = ERROR_FAIL;

		free(instr);
#endif

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
#if _NDS_V5_ONLY_
			/* auto convert to hardware breakpoint if failed */
			//struct nds32_v5 *nds32 = target_to_nds32_v5(target);
			if (nds32->auto_convert_hw_bp) {
				/* convert to hardware breakpoint */
				LOG_DEBUG("auto_convert to hw bp at 0x%" TARGET_PRIxADDR, breakpoint->address);
				breakpoint->type = BKPT_HARD;
				return riscv_add_breakpoint(target, breakpoint);
			}
#endif
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = add_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;

	return ERROR_OK;
}

static int remove_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0) {
			first_hart = hartid;
			break;
		}
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] == trigger->unique_id)
			break;
	}
	if (i >= r->trigger_count[first_hart]) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stop using resource %d for bp %d", i, trigger->unique_id);
	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
#if _NDS_V5_ONLY_
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, set_field(0, MCONTROL_TYPE(riscv_xlen(target)), 2));
#else
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
#endif
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);
	}
	r->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_write_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = remove_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

static void trigger_from_watchpoint(struct trigger *trigger,
		const struct watchpoint *watchpoint)
{
	trigger->address = watchpoint->address;
	trigger->length = watchpoint->length;
	trigger->mask = watchpoint->mask;
	trigger->value = watchpoint->value;
	trigger->read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->execute = false;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = watchpoint->unique_id;
}

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
	watchpoint->set = true;

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
	watchpoint->set = false;

	return ERROR_OK;
}

static int oldriscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct target_type *tt = get_target_type(target);
	return tt->step(target, current, address, handle_breakpoints);
}

static int old_or_new_riscv_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints
){
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else {
		return riscv_openocd_step(target, current, address, handle_breakpoints);
	}
}

static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		LOG_DEBUG("Target was already examined.");
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("  version=0x%x", info->dtm_version);

	struct target_type *tt = get_target_type(target);
	if (tt == NULL)
		return ERROR_FAIL;

	int result = tt->init_target(info->cmd_ctx, target);
	if (result != ERROR_OK)
		return result;

	return tt->examine(target);
}

static int oldriscv_poll(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->poll(target);
}

static int old_or_new_riscv_poll(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_poll(target);
	else
		return riscv_openocd_poll(target);
}

static int old_or_new_riscv_halt(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_halt(target);
	else
		return riscv_openocd_halt(target);
}

static int riscv_assert_reset(struct target *target)
{
	struct target_type *tt = get_target_type(target);

#if _NDS_V5_ONLY_
	LOG_DEBUG("Assert reset on [%s] hart %d", target->tap->dotted_name, target->coreid);
	riscv_select_current_hart(target);
	if (target_was_examined(target))
		riscv_invalidate_register_cache(target);
#endif
	return tt->assert_reset(target);
}

static int riscv_deassert_reset(struct target *target)
{
	LOG_DEBUG("RISCV DEASSERT RESET");

#if _NDS_V5_ONLY_
	LOG_DEBUG("Deassert reset on [%s] hart %d", target->tap->dotted_name, target->coreid);
	riscv_select_current_hart(target);
#endif
	struct target_type *tt = get_target_type(target);
	return tt->deassert_reset(target);
}


static int oldriscv_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	struct target_type *tt = get_target_type(target);
	return tt->resume(target, current, address, handle_breakpoints,
			debug_execution);
}

static int old_or_new_riscv_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution
){
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_resume(target, current, address, handle_breakpoints, debug_execution);
	else
		return riscv_openocd_resume(target, current, address, handle_breakpoints, debug_execution);
}

#if _NDS_V5_ONLY_
void riscv_select_current_hart(struct target *target)
#else
static void riscv_select_current_hart(struct target *target)
#endif
{
	RISCV_INFO(r);
	if (r->rtos_hartid != -1 && riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, r->rtos_hartid);
	else
		riscv_set_current_hartid(target, target->coreid);
}

uint64_t nds_ilm_bpa = 0, nds_ilm_lmsz = 0;
uint64_t nds_dlm_bpa = 0, nds_dlm_lmsz = 0;
uint32_t nds_local_memory_slave_port = 0;
uint32_t nds_check_idlm_capability_before = 0;
uint32_t nds_ilm_ena = 0, nds_dlm_ena = 0;

int nds_idlm_status_update(struct target *target)
{
	uint64_t value_micm_cfg=0, value_mdcm_cfg=0, reg_mmsc_cfg_value=0;
	uint64_t value_milmb=0, value_mdlmb=0;

	if (nds_check_idlm_capability_before == 0) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			// use quick mode to read CSR while target_not_halted
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MICM_CFG + GDB_REGNO_CSR0, &value_micm_cfg);
			ndsv5_get_csr_reg_quick_access(target, CSR_MDCM_CFG + GDB_REGNO_CSR0, &value_mdcm_cfg);
			ndsv5_get_csr_reg_quick_access(target, CSR_MMSC_CFG + GDB_REGNO_CSR0, &reg_mmsc_cfg_value);
		} else {
			struct reg *reg_micm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
			if (reg_micm_cfg != NULL)
				value_micm_cfg = ndsv5_get_register_value(reg_micm_cfg);
			struct reg *reg_mdcm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
			if (reg_mdcm_cfg != NULL)
				value_mdcm_cfg = ndsv5_get_register_value(reg_mdcm_cfg);
			struct reg *reg_mmsc_cfg = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
			if (reg_mmsc_cfg != NULL)
				reg_mmsc_cfg_value = ndsv5_get_register_value(reg_mmsc_cfg);
		}
		if ((value_micm_cfg & 0x7000) == 0)
			target->reg_cache->reg_list[REG_CSR0 + CSR_MILMB].exist = false;
		else
			target->reg_cache->reg_list[REG_CSR0 + CSR_MILMB].exist = true;
		if ((value_mdcm_cfg & 0x7000) == 0)
			target->reg_cache->reg_list[REG_CSR0 + CSR_MDLMB].exist = false;
		else
			target->reg_cache->reg_list[REG_CSR0 + CSR_MDLMB].exist = true;

		NDS_INFO("value_micm_cfg: 0x%" PRIx64 " value_mdcm_cfg: 0x%" PRIx64, value_micm_cfg, value_mdcm_cfg);
		NDS_INFO("reg_mmsc_cfg_value: 0x%" PRIx64, reg_mmsc_cfg_value);
		if ((reg_mmsc_cfg_value & 0x4000) == 0) {
			NDS_INFO("local memory slave port is not supported");
			nds_local_memory_slave_port = 0;
		} else {
			nds_local_memory_slave_port = 1;
		}
		nds_check_idlm_capability_before = 1;
	}
	// checking idlm enable while target_is_halted
	if (target->reg_cache->reg_list[REG_CSR0 + CSR_MILMB].exist == true) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MILMB + GDB_REGNO_CSR0, &value_milmb);
		} else {
			struct reg *reg_milmb = ndsv5_get_reg_by_CSR(target, CSR_MILMB);
			if (reg_milmb != NULL)
				value_milmb = ndsv5_get_register_value(reg_milmb);
		}
		if (value_milmb & 0x1) {
			nds_ilm_bpa = value_milmb & ~0x3ff;
			nds_ilm_ena = 1;
		} else {
			nds_ilm_ena = 0;
		}
	}
	if (target->reg_cache->reg_list[REG_CSR0 + CSR_MDLMB].exist == true) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			if (riscv_debug_buffer_size(target) < 6)
				return ERROR_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MDLMB + GDB_REGNO_CSR0, &value_mdlmb);
		} else {
			struct reg *reg_mdlmb = ndsv5_get_reg_by_CSR(target, CSR_MDLMB);
			if (reg_mdlmb != NULL)
				value_mdlmb = ndsv5_get_register_value(reg_mdlmb);
		}
		if (value_mdlmb & 0x1) {
			nds_dlm_bpa = value_mdlmb & ~0x3ff;
			nds_dlm_ena = 1;
		} else {
			nds_dlm_ena = 0;
		}
	}
	return ERROR_OK;
}

int lm_slvp_support(struct target *target, target_addr_t address, uint32_t csr_id_lmb)
{
	uint64_t checking_bpa=0, checking_lmsz=0;

	LOG_DEBUG("nds_check_idlm_capability_before: %d", nds_check_idlm_capability_before);
	if (nds_check_idlm_capability_before) {
		if (nds_local_memory_slave_port == 0) {
			//LOG_ERROR("<-- local memory slave port is not supported -->");
			return ERROR_FAIL;
		}
		if ((csr_id_lmb == CSR_MILMB) && (nds_ilm_lmsz == 0))
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		if ((csr_id_lmb == CSR_MDLMB) && (nds_dlm_lmsz == 0))
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	nds_idlm_status_update(target);
	if (csr_id_lmb == CSR_MILMB) {
		if (nds_ilm_ena == 0)
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		checking_bpa = nds_ilm_bpa;
		checking_lmsz = nds_ilm_lmsz;
	} else { // csr_id_lmb == CSR_MDLMB
		if (nds_dlm_ena == 0)
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		checking_bpa = nds_dlm_bpa;
		checking_lmsz = nds_dlm_lmsz;
	}
	if ((address >= checking_bpa) && (address < (checking_bpa + checking_lmsz))) {
		return ERROR_OK;
	}
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
/*
	struct reg *reg_name = NULL;
	uint64_t value = 0, bpa = 0;
	int lmsz = 0;
	if (target->reg_cache->reg_list[REG_CSR0 + csr_id_lmb].exist == true) {
		reg_name = ndsv5_get_reg_by_CSR(target, csr_id_lmb);
		if (reg_name != NULL) {
			value = ndsv5_get_register_value(reg_name);
			if (value & 0x1) {
				bpa = value & ~0x3ff;
				reg_name = ndsv5_get_reg_by_CSR(target, csr_id_cm_cfg);
				if (reg_name != NULL) {
					value = ndsv5_get_register_value(reg_name);
					lmsz = (value & 0xf8000) >> 15;
					if (lmsz > 0) {
						if (address >= bpa && address < (bpa + ((1 << (lmsz - 1)) *1024))) {
							if (target->reg_cache->reg_list[REG_CSR0 + CSR_MMSC_CFG].exist == true) {
								reg_name = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
								if (reg_name != NULL) {
									if (ndsv5_get_register_value(reg_name) & 0x4000) {
										return ERROR_OK;
									} else {
										LOG_ERROR("<-- local memory slave port is not supported -->");
										return ERROR_FAIL;
									}
								} else {
									LOG_ERROR("get MMSC_CFG reg failed");
								}
							}
						}
					}
				} else {
					LOG_ERROR("get MI/DCM_CFG reg failed");
				}
			}
		} else {
			LOG_ERROR("get MI/DLMB reg failed");
		}
	}
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
*/
}

uint64_t LM_BASE = 0xa0000000;
static int riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("%s", __func__);

	riscv_select_current_hart(target);
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	int retval = ERROR_OK;
	uint64_t reg_mcache_ctl_value = 0;

	target_addr_t physical_address = address;
	if( ndsv5_virtual_to_physical(target, address, &physical_address) != ERROR_OK )
		return ERROR_FAIL;

	// ilm and dlm exist and mmsc_cfg.LMSLVP == 1, when bus mode access address need add LM_BASE
	if (memory->access_channel == NDS_MEMORY_ACC_BUS) {
		if (!nds32->execute_register_init && (ndsv5_dis_cache_busmode == 1)) {
			struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
			uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
			NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
			reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
			uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
			NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);

			// micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6), CSR_MCACHE_CTL exist
			if (((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) {
				NDS_INFO("CSR_MCACHE_CTL not exist");
				ndsv5_dis_cache_busmode = 0;
			}
		}
		if (ndsv5_dis_cache_busmode == 1) {
			//if dis cache bus mode, saved and disable cache
			bus_mode_on(target, &reg_mcache_ctl_value);
		} else {
			//according to eticket 16199, default no support system bus access, so ndsv5_system_bus_access default value is 0
			if (ndsv5_system_bus_access == 1) {
				retval = lm_slvp_support(target, physical_address, CSR_MILMB);
				if (retval == ERROR_OK)
					physical_address = physical_address + LM_BASE;
				else if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
					retval = lm_slvp_support(target, physical_address, CSR_MDLMB);
					if (retval == ERROR_OK)
						physical_address = physical_address + LM_BASE;
				}
			}
		}
	}

	if (byte_access_from_burn == 1)
		retval = ndsv5_readwrite_byte(target, physical_address, size, count, buffer, 0);
	else
		retval = ndsv5_read_memory(target, physical_address, size, count, buffer);

	if( ndsv5_use_mprv_mode == 1 ) {
		uint64_t dcsr;
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			// Resotre mstatus
			ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, ndsv5_backup_mstatus);
			// Restore dcsr
			ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
			dcsr &= ~(0x10);
			ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_DCSR, dcsr);
		} else {
			// Resotre mstatus
			riscv_set_register(target, GDB_REGNO_MSTATUS, ndsv5_backup_mstatus);

			// Restore dcsr
			riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
			dcsr &= ~(0x10);
			riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
		}
	}
	if (memory->access_channel == NDS_MEMORY_ACC_BUS) {
		if (ndsv5_dis_cache_busmode == 1) {
			//if bus mode, restore cache
			bus_mode_off(target, reg_mcache_ctl_value);
		}
	}

	return retval;
#endif
	struct target_type *tt = get_target_type(target);
	return tt->read_memory(target, address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("%s", __func__);
	riscv_select_current_hart(target);
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	int retval = ERROR_OK;
	uint64_t reg_mcache_ctl_value = 0;

	target_addr_t physical_address = address;
	if( ndsv5_virtual_to_physical(target, address, &physical_address) != ERROR_OK )
		return ERROR_FAIL;

	// ilm and dlm exist and mmsc_cfg.LMSLVP == 1, when bus mode access address need add LM_BASE
	if (memory->access_channel == NDS_MEMORY_ACC_BUS) {
		if (!nds32->execute_register_init && (ndsv5_dis_cache_busmode == 1)) {
			struct reg *reg_name = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
			uint64_t micm_cfg_value = ndsv5_get_register_value(reg_name);
			NDS_INFO("micm_cfg = 0x%" PRIx64, micm_cfg_value);
			reg_name = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
			uint64_t mdcm_cfg_value = ndsv5_get_register_value(reg_name);
			NDS_INFO("mdcm_cfg = 0x%" PRIx64, mdcm_cfg_value);

			// micm_cfg.ISZ != 0 | mdcm_cfg.DSZ != 0 (bit 8-6), CSR_MCACHE_CTL exist
			if (((micm_cfg_value & 0x1c0) == 0) && ((mdcm_cfg_value & 0x1c0) == 0)) {
				NDS_INFO("CSR_MCACHE_CTL not exist");
				ndsv5_dis_cache_busmode = 0;
			}
		}
		if (ndsv5_dis_cache_busmode == 1) {
			//if dis cache bus mode, saved and disable cache
			bus_mode_on(target, &reg_mcache_ctl_value);
		} else {
			//according to eticket 16199, default no support system bus access, so ndsv5_system_bus_access default value is 0
			if (ndsv5_system_bus_access == 1) {
				retval = lm_slvp_support(target, physical_address, CSR_MILMB);
				if (retval == ERROR_OK)
					physical_address = physical_address + LM_BASE;
				else if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
					retval = lm_slvp_support(target, physical_address, CSR_MDLMB);
					if (retval == ERROR_OK)
						physical_address = physical_address + LM_BASE;
				}
			}
		}
	}

	if (byte_access_from_burn == 1)
		retval = ndsv5_readwrite_byte(target, physical_address, size, count, buffer, 1);
	else
		retval = ndsv5_write_memory(target, physical_address, size, count, buffer);

	if( ndsv5_use_mprv_mode == 1 ) {
		uint64_t dcsr;
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			// Resotre mstatus
			ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, ndsv5_backup_mstatus);
			// Restore dcsr
			ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
			dcsr &= ~(0x10);
			ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_DCSR, dcsr);
		} else {
			// Resotre mstatus
			riscv_set_register(target, GDB_REGNO_MSTATUS, ndsv5_backup_mstatus);

			// Restore dcsr
			riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
			dcsr &= ~(0x10);
			riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
		}
	}
	if (memory->access_channel == NDS_MEMORY_ACC_BUS) {
		if (ndsv5_dis_cache_busmode == 1) {
			//if bus mode, restore cache
			bus_mode_off(target, reg_mcache_ctl_value);
		}
	}
	return retval;
#endif
	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, address, size, count, buffer);
}

#if _NDS_V5_ONLY_
int ndsv5_examine(struct target *target);
#endif
static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	RISCV_INFO(r);
	LOG_DEBUG("reg_class=%d", reg_class);
	LOG_DEBUG("rtos_hartid=%d current_hartid=%d", r->rtos_hartid, r->current_hartid);

#if _NDS_V5_ONLY_
	if (!target_was_examined(target)) {
		NDS_INFO("target was NOT examined");
		ndsv5_examine(target);
	}
#endif

	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}

	riscv_select_current_hart(target);

#if _NDS_V5_ONLY_
	unsigned acr_reg_count_v5 = 0;
	if (global_acr_reg_count_v5 != 0) {
		acr_reg_count_v5 = *global_acr_reg_count_v5;
	}
#endif

	if (r->rtos_hartid != -1 && riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, r->rtos_hartid);
	else
		riscv_set_current_hartid(target, target->coreid);

	switch (reg_class) {
		case REG_CLASS_GENERAL:
#if _NDS_V5_ONLY_
			if (riscv_supports_extension(target, 'E'))
				*reg_list_size = 16;
			else
				*reg_list_size = 32;
#else
			*reg_list_size = 32;
#endif
			break;
		case REG_CLASS_ALL:
#if _NDS_V5_ONLY_
			*reg_list_size = GDB_REGNO_COUNT + acr_reg_count_v5;
#else
			*reg_list_size = GDB_REGNO_COUNT;
#endif
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (int i = 0; i < *reg_list_size; i++) {
		assert(!target->reg_cache->reg_list[i].valid ||
				target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
	}

	return ERROR_OK;
}

static int riscv_arch_state(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->arch_state(target);
}

/* Algorithm must end with a software breakpoint instruction. */
static int riscv_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (num_mem_params > 0) {
		LOG_ERROR("Memory parameters are not supported for RISC-V algorithms.");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK)
			return ERROR_FAIL;
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);
		if (r->type->set(r, reg_params[i].value) != ERROR_OK)
			return ERROR_FAIL;
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	uint64_t current_mstatus;
	uint8_t mstatus_bytes[8];

	LOG_DEBUG("Disabling Interrupts");
	char *mstatus_name = ndsv5_get_CSR_name(target, CSR_MSTATUS);
	if (mstatus_name == NULL) {
		LOG_DEBUG("get mstatus_name ERROR");
		return ERROR_FAIL;
	}
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			mstatus_name, 1);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
#if _NDS_V5_ONLY_
	buf_set_u64(mstatus_bytes, 0, info->xlen[nds_targetburn_corenum], set_field(current_mstatus,
				ie_mask, 0));
#else
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], set_field(current_mstatus,
				ie_mask, 0));
#endif
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
#if _NDS_V5_ONLY_
	if (ndsv5_openocd_resume_one_hart(target, 0, entry_point, 0, 0, nds_targetburn_corenum) != ERROR_OK)
#else
	if (oldriscv_resume(target, 0, entry_point, 0, 0) != ERROR_OK)
#endif
		return ERROR_FAIL;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %d ms.", timeout_ms);
			LOG_ERROR("  now   = 0x%08x", (uint32_t) now);
			LOG_ERROR("  start = 0x%08x", (uint32_t) start);
#if _NDS_V5_ONLY_
			ndsv5_openocd_halt_one_hart(target, nds_targetburn_corenum);
			ndsv5_openocd_poll_one_hart(target, nds_targetburn_corenum);
#else
			oldriscv_halt(target);
			old_or_new_riscv_poll(target);
#endif
			return ERROR_TARGET_TIMEOUT;
		}

		int result;
#if _NDS_V5_ONLY_
		result = ndsv5_openocd_poll_one_hart(target, nds_targetburn_corenum);
#else
		result = old_or_new_riscv_poll(target);
#endif
		if (result != ERROR_OK)
			return result;
	}

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Restore Interrupts */
	LOG_DEBUG("Restoring Interrupts");
#if _NDS_V5_ONLY_
	buf_set_u64(mstatus_bytes, 0, info->xlen[nds_targetburn_corenum], current_mstatus);
#else
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], current_mstatus);
#endif
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Restore registers */
	uint8_t buf[8];
#if _NDS_V5_ONLY_
	buf_set_u64(buf, 0, info->xlen[nds_targetburn_corenum], saved_pc);
#else
	buf_set_u64(buf, 0, info->xlen[0], saved_pc);
#endif
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
#if _NDS_V5_ONLY_
		buf_set_u64(buf, 0, info->xlen[nds_targetburn_corenum], saved_regs[r->number]);
#else
		buf_set_u64(buf, 0, info->xlen[0], saved_regs[r->number]);
#endif
		if (r->type->set(r, buf) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Should run code on the target to perform CRC of
memory. Not yet implemented.
*/

static int riscv_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count,
		uint32_t *checksum)
{
	*checksum = 0xFFFFFFFF;
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/* Should run code on the target to check whether a memory
block holds all-ones (because this is generally called on
NOR flash which is 1 when "blank")
Not yet implemented.
*/
int riscv_blank_check_memory(struct target *target,
				target_addr_t address,
				uint32_t count,
				uint32_t *blank,
				uint8_t erased_value)
{
	*blank = 0;

	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/*** OpenOCD Helper Functions ***/

/* 0 means nothing happened, 1 means the hart's state changed (and thus the
 * poll should terminate), and -1 means there was an error. */
static int riscv_poll_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_set_current_hartid(target, hartid);

#if _NDS_V5_ONLY_
	LOG_DEBUG("polling [%s] hart %d, target->state=%d (TARGET_HALTED=%d)", target->tap->dotted_name, hartid, target->state, TARGET_HALTED);
#else
	LOG_DEBUG("polling hart %d, target->state=%d (TARGET_HALTED=%d)", hartid, target->state, TARGET_HALTED);
#endif


	/* If OpenOCD this we're running but this hart is halted then it's time
	 * to raise an event. */
	if (target->state != TARGET_HALTED && riscv_is_halted(target)) {
		LOG_DEBUG("  triggered a halt");
		r->on_halt(target);
		return 1;
	}

	return 0;
}

#if _NDS_V5_ONLY_
int ndsv5_openocd_poll_one_hart(struct target *target, int hartid)
{
	LOG_DEBUG("polling hart : %d", hartid);
	int triggered_hart = hartid;
	if (riscv_poll_hart(target, triggered_hart) == 0)
		return ERROR_OK;

	LOG_DEBUG("  hart %d halted", triggered_hart);

	target->state = TARGET_HALTED;
	switch (riscv_halt_reason(target, triggered_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;
	case RISCV_HALT_UNKNOWN:
		target->debug_reason = DBG_REASON_UNDEFINED;
		break;
	}

	target->state = TARGET_HALTED;
#if _NDS_V5_ONLY_
	nds_triggered_hart = triggered_hart;
#endif

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return ERROR_OK;
}

int ndsv5_openocd_halt_one_hart(struct target *target, int hartid)
{
	LOG_DEBUG("halting hart : %d", hartid);

	int out = riscv_halt_one_hart(target, hartid);
	if (out != ERROR_OK) {
		LOG_ERROR("Unable to halt hart : %d", hartid);
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return out;
}

int ndsv5_openocd_resume_one_hart(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		int hartid
) {
	LOG_DEBUG("resuming hart : %d", hartid);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	int out = riscv_resume_one_hart(target, hartid);
	riscv_invalidate_register_cache(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to resume hart : %d", hartid);
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return out;
}
#endif

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_DEBUG("polling all harts");
	int triggered_hart = -1;
	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			int out = riscv_poll_hart(target, i);
			switch (out) {
			case 0:
				continue;
			case 1:
				triggered_hart = i;
				break;
			case -1:
				return ERROR_FAIL;
			}
		}
		if (triggered_hart == -1) {
			LOG_DEBUG("  no harts just halted, target->state=%d", target->state);
			return ERROR_OK;
		}
		LOG_DEBUG("  hart %d halted", triggered_hart);

		/* If we're here then at least one hart triggered.  That means
		 * we want to go and halt _every_ hart in the system, as that's
		 * the invariant we hold here.	Some harts might have already
		 * halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those
		 * harts. */
#if _NDS_V5_ONLY_
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			if (!riscv_hart_enabled(target, i))
				continue;
			riscv_halt_one_hart(target, i);
		}
#else
		for (int i = 0; i < riscv_count_harts(target); ++i)
			riscv_halt_one_hart(target, i);
#endif
	} else {
		if (riscv_poll_hart(target, riscv_current_hartid(target)) == 0)
			return ERROR_OK;

		triggered_hart = riscv_current_hartid(target);
		LOG_DEBUG("  hart %d halted", triggered_hart);
	}

	target->state = TARGET_HALTED;
	switch (riscv_halt_reason(target, triggered_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;
	case RISCV_HALT_UNKNOWN:
		target->debug_reason = DBG_REASON_UNDEFINED;
		break;
	}

	if (riscv_rtos_enabled(target)) {
#if _NDS_V5_ONLY_
		if (!target->after_reset_run) {
			/* call riscv_set_rtos_hartid function update r->rtos_hartid and target->rtos->current_threadid/thread */
			LOG_DEBUG("riscv_set_rtos_hartid %d", triggered_hart);
			riscv_set_rtos_hartid(target, triggered_hart);
		} else {
			/* reset_and_run command no need change r->rtos_hartid and target->rtos->current_threadid/thread */
			LOG_DEBUG("reset run command, no change rtos hart id and current_threadid");
			target->after_reset_run = false;
		}
#else
		target->rtos->current_threadid = triggered_hart + 1;
		target->rtos->current_thread = triggered_hart + 1;
#endif
	}

	target->state = TARGET_HALTED;
#if _NDS_V5_ONLY_
	nds_triggered_hart = triggered_hart;
#endif

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return ERROR_OK;
}

int riscv_openocd_halt(struct target *target)
{
	RISCV_INFO(r);

	LOG_DEBUG("halting all harts");

	int out = riscv_halt_all_harts(target);
	if (out != ERROR_OK) {
		LOG_ERROR("Unable to halt all harts");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	if (riscv_rtos_enabled(target)) {
		target->rtos->current_threadid = r->rtos_hartid + 1;
		target->rtos->current_thread = r->rtos_hartid + 1;
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;

#if (!_NDS_SUPPORT_WITHOUT_ANNOUNCING_)
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
#endif
	return out;
}

int riscv_openocd_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution
) {
	LOG_DEBUG("resuming all harts");

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	int out = riscv_resume_all_harts(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to resume all harts");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return out;
}

int riscv_openocd_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints
) {
	LOG_DEBUG("stepping rtos hart");

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	int out = riscv_step_rtos_hart(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to step rtos hart");
		return out;
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	target->state = TARGET_HALTED;
#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
	if (nds_without_announce) {
		nds_without_announce = 0;
		return out;
	}
#endif
	target->debug_reason = DBG_REASON_SINGLESTEP;
#if _NDS_V5_ONLY_
	LOG_DEBUG("checking reason");
	if (ndsv5_handle_triggered(target) != ERROR_OK) {
		LOG_DEBUG("ndsv5_handle_triggered ERROR");
		return ERROR_FAIL;
	}
#endif
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	LOG_DEBUG("target->debug_reason = 0x%x", target->debug_reason);
	return out;
}

/* Command Handlers */
COMMAND_HANDLER(riscv_set_command_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_command_timeout_sec = timeout;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_reset_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_reset_timeout_sec = timeout;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_scratch_ram)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (!strcmp(CMD_ARGV[0], "none")) {
		riscv_use_scratch_ram = false;
		return ERROR_OK;
	}

	long long unsigned int address;
	int result = sscanf(CMD_ARGV[0], "%llx", &address);
	if (result != (int) strlen(CMD_ARGV[0])) {
		LOG_ERROR("%s is not a valid address for command.", CMD_ARGV[0]);
		riscv_use_scratch_ram = false;
		return ERROR_FAIL;
	}

	riscv_scratch_ram_address = address;
	riscv_use_scratch_ram = true;
	return ERROR_OK;
}

void parse_error(const char *string, char c, unsigned position)
{
	char buf[position+2];
	for (unsigned i = 0; i < position; i++)
		buf[i] = ' ';
	buf[position] = '^';
	buf[position + 1] = 0;

	LOG_ERROR("Parse error at character %c in:", c);
	LOG_ERROR("%s", string);
	LOG_ERROR("%s", buf);
}

COMMAND_HANDLER(riscv_set_expose_csrs)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (unsigned pass = 0; pass < 2; pass++) {
		unsigned range = 0;
		unsigned low = 0;
		bool parse_low = true;
		unsigned high = 0;
		for (unsigned i = 0; i == 0 || CMD_ARGV[0][i-1]; i++) {
			char c = CMD_ARGV[0][i];
			if (isspace(c)) {
				/* Ignore whitespace. */
				continue;
			}

			if (parse_low) {
				if (isdigit(c)) {
					low *= 10;
					low += c - '0';
				} else if (c == '-') {
					parse_low = false;
				} else if (c == ',' || c == 0) {
					if (pass == 1) {
						expose_csr[range].low = low;
						expose_csr[range].high = low;
					}
					low = 0;
					range++;
				} else {
					parse_error(CMD_ARGV[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			} else {
				if (isdigit(c)) {
					high *= 10;
					high += c - '0';
				} else if (c == ',' || c == 0) {
					parse_low = true;
					if (pass == 1) {
						expose_csr[range].low = low;
						expose_csr[range].high = high;
					}
					low = 0;
					high = 0;
					range++;
				} else {
					parse_error(CMD_ARGV[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}
		}

		if (pass == 0) {
			if (expose_csr)
				free(expose_csr);
			expose_csr = calloc(range + 2, sizeof(*expose_csr));
		} else {
			expose_csr[range].low = 1;
			expose_csr[range].high = 0;
		}
	}
	return ERROR_OK;
}

#if _NDS_V5_ONLY_
const struct command_registration riscv_exec_command_handlers[] = {
#else
static const struct command_registration riscv_exec_command_handlers[] = {
#endif
	{
		.name = "set_command_timeout_sec",
		.handler = riscv_set_command_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_command_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) for individual commands"
	},
	{
		.name = "set_reset_timeout_sec",
		.handler = riscv_set_reset_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_reset_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) after reset is deasserted"
	},
	{
		.name = "set_scratch_ram",
		.handler = riscv_set_scratch_ram,
		.mode = COMMAND_ANY,
		.usage = "riscv set_scratch_ram none|[address]",
		.help = "Set address of 16 bytes of scratch RAM the debugger can use, or 'none'."
	},
	{
		.name = "expose_csrs",
		.handler = riscv_set_expose_csrs,
		.mode = COMMAND_ANY,
		.usage = "riscv expose_csrs n0[-m0][,n0[-m0]]...",
		.help = "Configure a list of inclusive ranges for CSRs to expose in "
				"addition to the standard ones. This must be executed before "
				"`init`."
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration riscv_command_handlers[] = {
	{
		.name = "riscv",
		.mode = COMMAND_ANY,
		.help = "RISC-V Command Group",
		.usage = "",
		.chain = riscv_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type riscv_target = {
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = old_or_new_riscv_halt,
	.resume = old_or_new_riscv_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,

	.blank_check_memory = riscv_blank_check_memory,
	.checksum_memory = riscv_checksum_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = riscv_command_handlers
};

/*** RISC-V Interface ***/

void riscv_info_init(struct target *target, riscv_info_t *r)
{
	memset(r, 0, sizeof(*r));
	r->dtm_version = 1;
	r->registers_initialized = false;
	r->current_hartid = target->coreid;

	memset(r->trigger_unique_id, 0xff, sizeof(r->trigger_unique_id));

	for (size_t h = 0; h < RISCV_MAX_HARTS; ++h) {
		r->xlen[h] = -1;

		for (size_t e = 0; e < RISCV_MAX_REGISTERS; ++e)
			r->valid_saved_registers[h][e] = false;
	}
}

int riscv_halt_all_harts(struct target *target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_halt_one_hart(target, i);
	}

	return ERROR_OK;
}

int riscv_halt_one_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);

#if _NDS_V5_ONLY_
	LOG_DEBUG("halting [%s] hart %d", target->tap->dotted_name, hartid);
#else
	LOG_DEBUG("halting hart %d", hartid);
#endif
	riscv_set_current_hartid(target, hartid);
	if (riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested halt, but was already halted", hartid);
		return ERROR_OK;
	}

	return r->halt_current_hart(target);
}

int riscv_resume_all_harts(struct target *target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_resume_one_hart(target, i);
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_resume_one_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	LOG_DEBUG("resuming hart %d", hartid);
	riscv_set_current_hartid(target, hartid);
	if (!riscv_is_halted(target)) {
		LOG_DEBUG("  hart %d requested resume, but was already resumed", hartid);
		return ERROR_OK;
	}

	r->on_resume(target);
	return r->resume_current_hart(target);
}

int riscv_step_rtos_hart(struct target *target)
{
	RISCV_INFO(r);
	int hartid = r->current_hartid;
	if (riscv_rtos_enabled(target)) {
		hartid = r->rtos_hartid;
		if (hartid == -1) {
			LOG_USER("GDB has asked me to step \"any\" thread, so I'm stepping hart 0.");
			hartid = 0;
		}
	}
	riscv_set_current_hartid(target, hartid);
	LOG_DEBUG("stepping hart %d", hartid);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart isn't halted before single step!");
		return ERROR_FAIL;
	}
	riscv_invalidate_register_cache(target);
	r->on_step(target);
	if (r->step_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	riscv_invalidate_register_cache(target);
	r->on_halt(target);
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart was not halted after single step!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

bool riscv_supports_extension(struct target *target, char letter)
{
	RISCV_INFO(r);
	unsigned num;
	if (letter >= 'a' && letter <= 'z')
		num = letter - 'a';
	else if (letter >= 'A' && letter <= 'Z')
		num = letter - 'A';
	else
		return false;
	return r->misa & (1 << num);
}

int riscv_xlen(const struct target *target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

int riscv_xlen_of_hart(const struct target *target, int hartid)
{
	RISCV_INFO(r);
	assert(r->xlen[hartid] != -1);
	return r->xlen[hartid];
}

bool riscv_rtos_enabled(const struct target *target)
{
	return target->rtos != NULL;
}

void riscv_set_current_hartid(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (!r->select_current_hart)
		return;

	int previous_hartid = riscv_current_hartid(target);
	r->current_hartid = hartid;
#if _NDS_V5_ONLY_
	if (!riscv_rtos_enabled(target))
		assert(riscv_hart_enabled(target, hartid));
#else
	assert(riscv_hart_enabled(target, hartid));
#endif

#if _NDS_V5_ONLY_
	LOG_DEBUG("setting [%s] hartid to %d, was %d", target->tap->dotted_name, hartid, previous_hartid);
#else
	LOG_DEBUG("setting hartid to %d, was %d", hartid, previous_hartid);
#endif
	r->select_current_hart(target);

	/* This might get called during init, in which case we shouldn't be
	 * setting up the register cache. */
	if (!target_was_examined(target))
		return;

	/* Avoid invalidating the register cache all the time. */
	if (r->registers_initialized
			&& (!riscv_rtos_enabled(target) || (previous_hartid == hartid))
			&& target->reg_cache->reg_list[GDB_REGNO_ZERO].size == (unsigned)riscv_xlen(target)
			&& (!riscv_rtos_enabled(target) || (r->rtos_hartid != -1))) {
		return;
	} else
		LOG_DEBUG("Initializing registers: xlen=%d", riscv_xlen(target));

	riscv_invalidate_register_cache(target);
}

void riscv_invalidate_register_cache(struct target *target)
{
#if _NDS_V5_ONLY_
	unsigned acr_reg_count_v5 = 0;
	if (global_acr_reg_count_v5 != 0) {
		acr_reg_count_v5 = *global_acr_reg_count_v5;
	}
#endif

	RISCV_INFO(r);

	register_cache_invalidate(target->reg_cache);


#if _NDS_V5_ONLY_
	for (size_t i = GDB_REGNO_COUNT; i < GDB_REGNO_COUNT + acr_reg_count_v5; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];

		reg->value = &r->reg_cache_values[i];
		reg->valid = false;
	}
#else
	for (size_t i = 0; i < GDB_REGNO_COUNT; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		reg->valid = false;
	}
#endif

	r->registers_initialized = true;
}

int riscv_current_hartid(const struct target *target)
{
	RISCV_INFO(r);
	return r->current_hartid;
}

void riscv_set_all_rtos_harts(struct target *target)
{
	RISCV_INFO(r);
	r->rtos_hartid = -1;
}

void riscv_set_rtos_hartid(struct target *target, int hartid)
{
	LOG_DEBUG("setting RTOS hartid %d", hartid);
	RISCV_INFO(r);
	r->rtos_hartid = hartid;
#if _NDS_V5_ONLY_
	target->rtos->current_threadid = hartid + 1;
	target->rtos->current_thread = hartid + 1;
#endif
}

int riscv_count_harts(struct target *target)
{
	if (target == NULL)
		return 1;
	RISCV_INFO(r);
	if (r == NULL)
		return 1;
	return r->hart_count;
}

bool riscv_has_register(struct target *target, int hartid, int regid)
{
	return 1;
}

int riscv_set_register(struct target *target, enum gdb_regno r, riscv_reg_t v)
{
	LOG_DEBUG("%s, r=0x%x, v=0x%lx", __func__, r, (long unsigned int)v);
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), r, v);
}

int riscv_set_register_on_hart(struct target *target, int hartid,
		enum gdb_regno regid, uint64_t value)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%d] %s <- 0x%" PRIx64, hartid, gdb_regno_name(regid), value);
	assert(r->set_register);
	return r->set_register(target, hartid, regid, value);
}

int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r)
{
	return riscv_get_register_on_hart(target, value,
			riscv_current_hartid(target), r);
}

int riscv_get_register_on_hart(struct target *target, riscv_reg_t *value,
		int hartid, enum gdb_regno regid)
{
	RISCV_INFO(r);
	int result = r->get_register(target, value, hartid, regid);
	LOG_DEBUG("[%d] %s: 0x%" PRIx64, hartid, gdb_regno_name(regid), *value);
	return result;
}

bool riscv_is_halted(struct target *target)
{
	RISCV_INFO(r);
#if _NDS_DISABLE_ABORT_
#else
	assert(r->is_halted);
#endif
	return r->is_halted(target);
}

enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_set_current_hartid(target, hartid);
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart is not halted!");
		return RISCV_HALT_UNKNOWN;
	}
	return r->halt_reason(target);
}

int riscv_count_triggers(struct target *target)
{
	return riscv_count_triggers_of_hart(target, riscv_current_hartid(target));
}

int riscv_count_triggers_of_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	assert(hartid < riscv_count_harts(target));
	return r->trigger_count[hartid];
}

size_t riscv_debug_buffer_size(struct target *target)
{
	RISCV_INFO(r);
	return r->debug_buffer_size[riscv_current_hartid(target)];
}

int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn)
{
	LOG_DEBUG("%s, index=0x%x, insn=0x%x", __func__, index, insn);
	RISCV_INFO(r);
	r->write_debug_buffer(target, index, insn);
	return ERROR_OK;
}

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index)
{
	LOG_DEBUG("%s, index=0x%x", __func__, index);
	RISCV_INFO(r);
	return r->read_debug_buffer(target, index);
}

int riscv_execute_debug_buffer(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	RISCV_INFO(r);
	return r->execute_debug_buffer(target);
}

void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV_INFO(r);
	r->fill_dmi_write_u64(target, buf, a, d);
}

void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV_INFO(r);
	r->fill_dmi_read_u64(target, buf, a);
}

void riscv_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV_INFO(r);
	r->fill_dmi_nop_u64(target, buf);
}

int riscv_dmi_write_u64_bits(struct target *target)
{
	RISCV_INFO(r);
	return r->dmi_write_u64_bits(target);
}

bool riscv_hart_enabled(struct target *target, int hartid)
{
	/* FIXME: Add a hart mask to the RTOS. */
#if _NDS_V5_ONLY_
	if (riscv_rtos_enabled(target)) {
		if (hartid < riscv_count_harts(target)) {
			if (target->rtos->hart_unavailable[hartid] == 0xFF) {
				LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
				return false;
			}
			else
				return true;
		}
		return false;
	}
#else
	if (riscv_rtos_enabled(target))
		return hartid < riscv_count_harts(target);
#endif

	return hartid == target->coreid;
}

/**
 * Count triggers, and initialize trigger_count for each hart.
 * trigger_count is initialized even if this function fails to discover
 * something.
 * Disable any hardware triggers that have dmode set. We can't have set them
 * ourselves. Maybe they're left over from some killed debug session.
 * */
int riscv_enumerate_triggers(struct target *target)
{
	RISCV_INFO(r);

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid,
				GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			r->trigger_count[hartid] = t;

			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t);
			uint64_t tselect_rb;
			result = riscv_get_register_on_hart(target, &tselect_rb, hartid,
					GDB_REGNO_TSELECT);
			if (result != ERROR_OK)
				return result;
			/* Mask off the top bit, which is used as tdrmode in old
			 * implementations. */
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));
			if (tselect_rb != t)
				break;
			uint64_t tdata1;
			result = riscv_get_register_on_hart(target, &tdata1, hartid,
					GDB_REGNO_TDATA1);
			if (result != ERROR_OK)
				return result;

			int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));
			switch (type) {
				case 1:
					/* On these older cores we don't support software using
					 * triggers. */
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
				case 2:
					if (tdata1 & MCONTROL_DMODE(riscv_xlen(target)))
						riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
			}
		}

		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);

		LOG_INFO("[%d] Found %d triggers", hartid, r->trigger_count[hartid]);
	}

	return ERROR_OK;
}

const char *gdb_regno_name(enum gdb_regno regno)
{
	static char buf[32];

	switch (regno) {
		case GDB_REGNO_ZERO:
			return "zero";
		case GDB_REGNO_S0:
			return "s0";
		case GDB_REGNO_S1:
			return "s1";
		case GDB_REGNO_PC:
			return "pc";
		case GDB_REGNO_FPR0:
			return "fpr0";
		case GDB_REGNO_FPR31:
			return "fpr31";
		case GDB_REGNO_CSR0:
			return "csr0";
		case GDB_REGNO_TSELECT:
			return "tselect";
		case GDB_REGNO_TDATA1:
			return "tdata1";
		case GDB_REGNO_TDATA2:
			return "tdata2";
		case GDB_REGNO_MISA:
			return "misa";
		case GDB_REGNO_DPC:
			return "dpc";
		case GDB_REGNO_DCSR:
			return "dcsr";
		case GDB_REGNO_DSCRATCH:
			return "dscratch";
		case GDB_REGNO_MSTATUS:
			return "mstatus";
		case GDB_REGNO_PRIV:
			return "priv";
		default:
			if (regno <= GDB_REGNO_XPR31)
				sprintf(buf, "x%d", regno - GDB_REGNO_ZERO);
			else if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095)
				sprintf(buf, "csr%d", regno - GDB_REGNO_CSR0);
			else if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
				sprintf(buf, "fpr%d", regno - GDB_REGNO_FPR0);
			else if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31)
				sprintf(buf, "v%d", regno - GDB_REGNO_V0);
			else
				sprintf(buf, "gdb_regno_%d", regno);
			return buf;
	}
}

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	uint64_t value;
	int result = riscv_get_register(target, &value, reg->number);
	if (result != ERROR_OK)
		return result;
	buf_set_u64(reg->value, 0, reg->size, value);
	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;

	uint64_t value = buf_get_u64(buf, 0, reg->size);

	LOG_DEBUG("write 0x%" PRIx64 " to %s", value, reg->name);
	struct reg *r = &target->reg_cache->reg_list[reg->number];

#if _NDS_V5_ONLY_
	r->valid = false;
#else
	r->valid = true;
#endif
	memcpy(r->value, buf, (r->size + 7) / 8);

	riscv_set_register(target, reg->number, value);
#if _NDS_V5_ONLY_
	// Read CSR back after write
	if( reg->number >= GDB_REGNO_CSR0 ) {
		uint64_t actual_value;
		riscv_get_register(target, &actual_value, reg->number);
		if(value != actual_value) {
			LOG_ERROR("Written reg %s (0x%" PRIx64 ") does not match read back "
					   "value (0x%" PRIx64 ")", reg->name, value, actual_value);
			memcpy(r->value, (uint8_t*)&actual_value, (r->size + 7) / 8);
		}
	}
#endif
	return ERROR_OK;
}

#if _NDS_V5_ONLY_
struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};
#else
static struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};
#endif

struct csr_info {
	unsigned number;
	const char *name;
};

static int cmp_csr_info(const void *p1, const void *p2)
{
	return (int) (((struct csr_info *)p1)->number) - (int) (((struct csr_info *)p2)->number);
}

int riscv_init_registers(struct target *target)
{
	RISCV_INFO(info);

	if (target->reg_cache) {
		if (target->reg_cache->reg_list)
			free(target->reg_cache->reg_list);
		free(target->reg_cache);
	}

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	target->reg_cache->name = "RISC-V Registers";
#if _NDS_V5_ONLY_
	extern unsigned acr_reg_count_v5;
	extern unsigned acr_type_count_v5;
	target->reg_cache->num_regs = GDB_REGNO_COUNT + acr_reg_count_v5;
	target->reg_cache->reg_list = calloc(GDB_REGNO_COUNT + acr_reg_count_v5, sizeof(struct reg));
#else
	target->reg_cache->num_regs = GDB_REGNO_COUNT;
	target->reg_cache->reg_list = calloc(GDB_REGNO_COUNT, sizeof(struct reg));
#endif

	const unsigned int max_reg_name_len = 12;
	if (info->reg_names)
		free(info->reg_names);

#if _NDS_V5_ONLY_
	info->reg_names = calloc(1, (GDB_REGNO_COUNT + acr_reg_count_v5) * max_reg_name_len);
#else
	info->reg_names = calloc(1, GDB_REGNO_COUNT * max_reg_name_len);
#endif

	char *reg_name = info->reg_names;

	static struct reg_feature feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};
	static struct reg_feature feature_fpu = {
//for gdb8.x , feature name :cpu, fpu, csr,virtual
/*#if _NDS_V5_ONLY_ // tmp-for-gdb-tdesc
		.name = "org.gnu.gdb.riscv.cpu"
#else*/
		.name = "org.gnu.gdb.riscv.fpu"
//#endif
	};
	static struct reg_feature feature_csr = {
/*#if _NDS_V5_ONLY_ // tmp-for-gdb-tdesc
		.name = "org.gnu.gdb.riscv.cpu"
#else*/
		.name = "org.gnu.gdb.riscv.csr"
//#endif
	};
	static struct reg_feature feature_virtual = {
/*#if _NDS_V5_ONLY_ // tmp-for-gdb-tdesc
		.name = "org.gnu.gdb.riscv.cpu"
#else*/
		.name = "org.gnu.gdb.riscv.virtual"
//#endif
	};
	static struct reg_feature feature_vector = {
		.name = "org.gnu.gdb.riscv.vector"
	};

	static struct reg_data_type type_data_ptr = {
		.type = REG_TYPE_DATA_PTR,
		.id = "data_ptr"
	};
	static struct reg_data_type type_code_ptr = {
		.type = REG_TYPE_CODE_PTR,
		.id = "code_ptr"
	};
	static struct reg_data_type type_ieee_single = {
		.type = REG_TYPE_IEEE_SINGLE,
		.id = "ieee_single"
	};
	static struct reg_data_type type_ieee_double = {
		.type = REG_TYPE_IEEE_DOUBLE,
		.id = "ieee_double"
	};
#if _NDS_V5_ONLY_
	static struct reg_data_type type_vec128 = {
		.type = REG_TYPE_ARCH_DEFINED,
		.type_class = REG_TYPE_CLASS_VENDOR_DEF,
		.id = "vec128"
	};
	static struct reg_data_type type_vec256 = {
		.type = REG_TYPE_ARCH_DEFINED,
		.type_class = REG_TYPE_CLASS_VENDOR_DEF,
		.id = "vec256"
	};
	static struct reg_data_type type_vec512 = {
		.type = REG_TYPE_ARCH_DEFINED,
		.type_class = REG_TYPE_CLASS_VENDOR_DEF,
		.id = "vec512"
	};
#endif

	struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
	};
	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, DIM(csr_info), sizeof(*csr_info), cmp_csr_info);
	unsigned csr_info_index = 0;

	/* When gdb request register N, gdb_get_register_packet() assumes that this
	 * is register at index N in reg_list. So if there are certain registers
	 * that don't exist, we need to leave holes in the list (or renumber, but
	 * it would be nice not to have yet another set of numbers to translate
	 * between). */
	for (uint32_t number = 0; number < GDB_REGNO_COUNT; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];
		r->caller_save = true;
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = target;
		r->number = number;
		r->size = riscv_xlen(target);

		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
#if _NDS_V5_ONLY_
			//RV32E only use first 15 gprs(ex: x0~x15)
			if (riscv_supports_extension(target, 'E')) {
				if (number > GDB_REGNO_A5)
					r->exist = false;
			}
			switch (number) {
				case GDB_REGNO_RA:
					r->reg_data_type = &type_code_ptr;
					break;
				case GDB_REGNO_SP:
				case GDB_REGNO_GP:
				case GDB_REGNO_TP:
				case GDB_REGNO_S0:
					r->reg_data_type = &type_data_ptr;
					break;
			}
			r->name = gpr_and_fpu_name[number];
#else
			switch (number) {
				case GDB_REGNO_ZERO:
					r->name = "zero";
					break;
				case GDB_REGNO_RA:
					r->name = "ra";
					break;
				case GDB_REGNO_SP:
					r->name = "sp";
					break;
				case GDB_REGNO_GP:
					r->name = "gp";
					break;
				case GDB_REGNO_TP:
					r->name = "tp";
					break;
				case GDB_REGNO_T0:
					r->name = "t0";
					break;
				case GDB_REGNO_T1:
					r->name = "t1";
					break;
				case GDB_REGNO_T2:
					r->name = "t2";
					break;
				//case GDB_REGNO_FP:
				//	r->name = REG_FP;
				//	break;
				case GDB_REGNO_S0:
					r->name = "s0";
					break;
				case GDB_REGNO_S1:
					r->name = "s1";
					break;
				case GDB_REGNO_A0:
					r->name = "a0";
					break;
				case GDB_REGNO_A1:
					r->name = "a1";
					break;
				case GDB_REGNO_A2:
					r->name = "a2";
					break;
				case GDB_REGNO_A3:
					r->name = "a3";
					break;
				case GDB_REGNO_A4:
					r->name = "a4";
					break;
				case GDB_REGNO_A5:
					r->name = "a5";
					break;
				case GDB_REGNO_A6:
					r->name = "a6";
					break;
				case GDB_REGNO_A7:
					r->name = "a7";
					break;
				case GDB_REGNO_S2:
					r->name = "s2";
					break;
				case GDB_REGNO_S3:
					r->name = "s3";
					break;
				case GDB_REGNO_S4:
					r->name = "s4";
					break;
				case GDB_REGNO_S5:
					r->name = "s5";
					break;
				case GDB_REGNO_S6:
					r->name = "s6";
					break;
				case GDB_REGNO_S7:
					r->name = "s7";
					break;
				case GDB_REGNO_S8:
					r->name = "s8";
					break;
				case GDB_REGNO_S9:
					r->name = "s9";
					break;
				case GDB_REGNO_S10:
					r->name = "s10";
					break;
				case GDB_REGNO_S11:
					r->name = "s11";
					break;
				case GDB_REGNO_T3:
					r->name = "t3";
					break;
				case GDB_REGNO_T4:
					r->name = "t4";
					break;
				case GDB_REGNO_T5:
					r->name = "t5";
					break;
				case GDB_REGNO_T6:
					r->name = "t6";
					break;
			}
#endif
			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number == GDB_REGNO_PC) {
			sprintf(reg_name, "pc");
			r->group = "general";
			r->feature = &feature_cpu;
			r->reg_data_type = &type_code_ptr;
		} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D')) {
				r->reg_data_type = &type_ieee_double;
				r->size = 64;
			} else if (riscv_supports_extension(target, 'F')) {
				r->reg_data_type = &type_ieee_single;
				r->size = 32;
			} else {
				r->exist = false;
			}
#if _NDS_V5_ONLY_
			r->name = gpr_and_fpu_name[number];
#else
			switch (number) {
				case GDB_REGNO_FT0:
					r->name = "ft0";
					break;
				case GDB_REGNO_FT1:
					r->name = "ft1";
					break;
				case GDB_REGNO_FT2:
					r->name = "ft2";
					break;
				case GDB_REGNO_FT3:
					r->name = "ft3";
					break;
				case GDB_REGNO_FT4:
					r->name = "ft4";
					break;
				case GDB_REGNO_FT5:
					r->name = "ft5";
					break;
				case GDB_REGNO_FT6:
					r->name = "ft6";
					break;
				case GDB_REGNO_FT7:
					r->name = "ft7";
					break;
				case GDB_REGNO_FS0:
					r->name = "fs0";
					break;
				case GDB_REGNO_FS1:
					r->name = "fs1";
					break;
				case GDB_REGNO_FA0:
					r->name = "fa0";
					break;
				case GDB_REGNO_FA1:
					r->name = "fa1";
					break;
				case GDB_REGNO_FA2:
					r->name = "fa2";
					break;
				case GDB_REGNO_FA3:
					r->name = "fa3";
					break;
				case GDB_REGNO_FA4:
					r->name = "fa4";
					break;
				case GDB_REGNO_FA5:
					r->name = "fa5";
					break;
				case GDB_REGNO_FA6:
					r->name = "fa6";
					break;
				case GDB_REGNO_FA7:
					r->name = "fa7";
					break;
				case GDB_REGNO_FS2:
					r->name = "fs2";
					break;
				case GDB_REGNO_FS3:
					r->name = "fs3";
					break;
				case GDB_REGNO_FS4:
					r->name = "fs4";
					break;
				case GDB_REGNO_FS5:
					r->name = "fs5";
					break;
				case GDB_REGNO_FS6:
					r->name = "fs6";
					break;
				case GDB_REGNO_FS7:
					r->name = "fs7";
					break;
				case GDB_REGNO_FS8:
					r->name = "fs8";
					break;
				case GDB_REGNO_FS9:
					r->name = "fs9";
					break;
				case GDB_REGNO_FS10:
					r->name = "fs10";
					break;
				case GDB_REGNO_FS11:
					r->name = "fs11";
					break;
				case GDB_REGNO_FT8:
					r->name = "ft8";
					break;
				case GDB_REGNO_FT9:
					r->name = "ft9";
					break;
				case GDB_REGNO_FT10:
					r->name = "ft10";
					break;
				case GDB_REGNO_FT11:
					r->name = "ft11";
					break;
			}
#endif
			r->group = "float";
			r->feature = &feature_fpu;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			r->group = "csr";
			r->feature = &feature_csr;
			unsigned csr_number = number - GDB_REGNO_CSR0;

			while (csr_info[csr_info_index].number < csr_number &&
					csr_info_index < DIM(csr_info) - 1) {
				csr_info_index++;
			}
#if _NDS_V5_ONLY_ // tmp-for-gdb-tdesc
#else
			if (csr_info[csr_info_index].number == csr_number) {
				r->name = csr_info[csr_info_index].name;
			} else
#endif
			{
				sprintf(reg_name, "csr%d", csr_number);
				/* Assume unnamed registers don't exist, unless we have some
				 * configuration that tells us otherwise. That's important
				 * because eg. Eclipse crashes if a target has too many
				 * registers, and apparently has no way of only showing a
				 * subset of registers in any case. */
				r->exist = false;
			}

			switch (csr_number) {
				case CSR_FFLAGS:
				case CSR_FRM:
				case CSR_FCSR:
					r->exist = riscv_supports_extension(target, 'F');
					r->group = "float";
					r->feature = &feature_fpu;
					break;
				case CSR_SSTATUS:
				case CSR_STVEC:
				case CSR_SIP:
				case CSR_SIE:
				case CSR_SCOUNTEREN:
				case CSR_SSCRATCH:
				case CSR_SEPC:
				case CSR_SCAUSE:
				case CSR_STVAL:
				case CSR_SATP:
					r->exist = riscv_supports_extension(target, 'S');
					break;
			}

			if (riscv_xlen(target) == 64) {
				switch (csr_number) {
					case CSR_SCOUNTEREN:
					case CSR_MCOUNTEREN:
					case CSR_DCSR:
					case CSR_MCOUNTERWEN:
					case CSR_MCOUNTERINTEN:
					case CSR_MCOUNTERMASK_M:
					case CSR_MCOUNTERMASK_S:
					case CSR_MCOUNTERMASK_U:
					case CSR_MCOUNTEROVF:
					case CSR_SCOUNTERINTEN:
					case CSR_SCOUNTERMASK_S:
					case CSR_SCOUNTERMASK_U:
					case CSR_SCOUNTEROVF:
					case CSR_MCOUNTINHIBIT:
					case CSR_SCOUNTINHIBIT:
					case CSR_SCOUNTERMASK_M:
						r->size = 32;
				}
			}

			if (!r->exist && expose_csr) {
				for (unsigned i = 0; expose_csr[i].low <= expose_csr[i].high; i++) {
					if (csr_number >= expose_csr[i].low && csr_number <= expose_csr[i].high) {
						LOG_INFO("Exposing additional CSR %d", csr_number);
						r->exist = true;
						break;
					}
				}
			}

		} else if (number == GDB_REGNO_PRIV) {
			sprintf(reg_name, "priv");
			r->group = "general";
			r->feature = &feature_virtual;
			r->size = 8;
#if _NDS_V5_ONLY_
		} else if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31) {
			sprintf(reg_name, "v%d", number - GDB_REGNO_V0);
			r->group = "general";
			r->feature = &feature_vector;

			struct nds32_v5 *nds32 = target_to_nds32_v5(target);
			//LOG_DEBUG("nds32->nds_vector_length: %d", nds32->nds_vector_length);
			if (nds32->nds_vector_length == 128) {
				r->reg_data_type = &type_vec128;
				r->size = 128;
			} else if (nds32->nds_vector_length == 256) {
				r->reg_data_type = &type_vec256;
				r->size = 256;
			} else { //if (nds32->nds_vector_length == 512) {
				r->reg_data_type = &type_vec512;
				r->size = 512;
			}

#endif
		}
		if (reg_name[0])
			r->name = reg_name;
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + GDB_REGNO_COUNT * max_reg_name_len);
		r->value = &info->reg_cache_values[number];
	}

#if _NDS_V5_ONLY_
	// Handle ACR
	//    ACR_info acr_list[] = {
	//      {"r32b", 32, 2, {"rd32_r77b", "rd64_r77b"}, {"wr32_r77b", "wr64_r77b"} },
	//      {"r64b", 64, 5, {"rd32_r12b", ""}, {"wr32_r12b", ""} },
	//      {"sram_matrix", 64, 256, {"", ""}, {"", ""} },
	//    };
	extern ACR_INFO_T_V5 *acr_info_list_v5;
	extern struct reg_arch_type nds_ace_reg_access_type;
	LOG_DEBUG("ACR ID starts from %d", GDB_REGNO_COUNT);
	unsigned int reg_list_idx = GDB_REGNO_COUNT;
	for (unsigned int i = 0; i < acr_type_count_v5; i++) {
		unsigned int acr_number = acr_info_list_v5->num;
		unsigned int acr_width = acr_info_list_v5->width;
		char* acr_name = acr_info_list_v5->name;
		LOG_DEBUG("ACR info -> Name : %s, Num : %d, Width : %d", acr_name, acr_number, acr_width);

		for (unsigned int idx = 0; idx < acr_number; idx++, reg_list_idx++) {      
			struct reg *r = &target->reg_cache->reg_list[reg_list_idx];
			r->number = reg_list_idx;
			r->caller_save = true;
			r->dirty = false;
			r->valid = false;
			r->exist = true;
			r->type = &nds_ace_reg_access_type;
			r->arch_info = target;      
			r->size = acr_width;
			r->group = "ace";
			// ByteSize = ceil(reg->size / 8)
			unsigned int ByteSize = (!r->size%8) ? r->size/8 : (r->size/8) + 1;
			r->value = (char*) calloc(ByteSize, sizeof(char));

			r->feature = calloc(sizeof(struct reg_feature), 1);      
			r->feature->name = "org.gnu.gdb.riscv.ace";

			r->reg_data_type = calloc(sizeof(struct reg_data_type), 1);
			r->reg_data_type->type = REG_TYPE_UINT8;
			r->reg_data_type->id = acr_name;

			char* acr_reg_name = calloc(strlen(acr_name) + 10, 1);
			sprintf(acr_reg_name, "%s_%d", acr_name, idx);
			if (acr_reg_name[0]) {
				r->name = acr_reg_name;
			}
			LOG_DEBUG("\t\tRegister ACR : %s, size = %d", r->name, r->size);
		}
		acr_info_list_v5++;
	}
#endif

#if _NDS_V5_ONLY_
	/* redirect all CSRs (r->name/r->exist) to NDS define */
	extern int ndsv5_redefine_CSR_name(struct target *target);
	extern int ndsv5_redefine_GPR_FPU_name(struct target *target);
	ndsv5_redefine_CSR_name(target);
	ndsv5_redefine_GPR_FPU_name(target);
#endif

	return ERROR_OK;
}

#if _NDS_V5_ONLY_
int ndsv5_step(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints)
{
	if (ndsv5_step_check(target) != ERROR_OK) {
		LOG_ERROR("ndsv5_step_check failed");
		return ERROR_OK;
	}

	if (old_or_new_riscv_step(target, current, address, handle_breakpoints) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_step failed");
		return ERROR_FAIL;
	}

#if 0//_NDS_SUPPORT_WITHOUT_ANNOUNCING_
	if (target->state == TARGET_HALTED) {
		if (nds_without_announce) {
			nds_without_announce = 0;
			LOG_DEBUG("nds_without_announce");
		} else {
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}
#endif
	return ERROR_OK;
}

static int strict_step(struct target *target, bool announce)
{
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("strict_step (before): 0x%lx", (long unsigned int)reg_pc_value);

	struct breakpoint *breakpoint = target->breakpoints;
	while (breakpoint) {
		riscv_remove_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	struct watchpoint *watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_remove_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	if (announce == false)
		nds_without_announce = 1;
	else
		nds_without_announce = 0;

	int result = riscv_openocd_step(target, 1, 0, 0);
	if (result != ERROR_OK) {
		LOG_ERROR("riscv_openocd_step failed");
		return ERROR_FAIL;
	}

	breakpoint = target->breakpoints;
	while (breakpoint) {
		riscv_add_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_add_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	reg_pc->type->get(reg_pc);
	reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("strict_step (after): 0x%lx", (long unsigned int)reg_pc_value);
	return ERROR_OK;
}

static int ndsv5_handle_triggered(struct target *target)
{
	int single_step_cmd = 0;

	uint64_t dcsr;
	riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);	
	int cause = get_field(dcsr, DCSR_CAUSE);

	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("halt at 0x%lx", (long unsigned int)reg_pc_value);

	switch (cause) {
		case DCSR_CAUSE_SWBP:
			ndsv5_virtual_hosting_check(target);
			break;
		case DCSR_CAUSE_HWBP:
			// step and watchpoint, record
			if (target->debug_reason == DBG_REASON_SINGLESTEP)
				single_step_cmd = 1;

			target->debug_reason = DBG_REASON_WPTANDBKPT;
			// If we halted because of a data trigger, gdb doesn't know to do
			// the disable-breakpoints-step-enable-breakpoints dance.
			if (ndsv5_hit_watchpoint_check(target) == ERROR_OK) {
				if (ndsv5_get_watched_address(target) == ERROR_OK) {
					LOG_DEBUG("match watchpoint");
					//strict_step(target, false);
					target->debug_reason = DBG_REASON_WATCHPOINT;
				} else {
					/* false match watchpoint, resume target */
					LOG_DEBUG("false match watchpoint");
					/* current pc, addr = 0, do not handle breakpoints, not debugging */
					strict_step(target, false);

					// step and watchpoint false match , return ok
					if (single_step_cmd == 1) {
						target->debug_reason = DBG_REASON_SINGLESTEP;
						LOG_DEBUG("single step and false match watchpoint");
						return ERROR_OK;
					}

					riscv_openocd_resume(target, 1, 0, 0, 0);

					return ERROR_FAIL;
				}
			}
		break;
	}
	return ERROR_OK;
}

static int ndsv5_poll(struct target *target)
{
	if (nds_skip_dmi == 1)
		return ERROR_OK;

	nds_triggered_hart = -1;
	if (old_or_new_riscv_poll(target) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_poll failed");
		return ERROR_FAIL;
	}
	if (nds_triggered_hart != -1) {
		if (ndsv5_handle_triggered(target) != ERROR_OK) {
			/* resume target */
		} else {
#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
			if (nds_without_announce) {
				nds_without_announce = 0;
				LOG_DEBUG("nds_without_announce");
			} else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
#endif
		}
	}
	return ndsv5_handle_poll(target);
}

static int ndsv5_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	if (old_or_new_riscv_halt(target) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_halt failed");
		return ERROR_FAIL;
	}
#if _NDS_SUPPORT_WITHOUT_ANNOUNCING_
	if (nds_without_announce) {
		nds_without_announce = 0;
		LOG_DEBUG("nds_without_announce");
	} else {
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}
#endif
	/* to update if target->state == TARGET_HALTED */
	ndsv5_poll(target);

	return ndsv5_handle_halt(target);
}

int ndsv5_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	if (ndsv5_resume_check(target) != ERROR_OK) {
		LOG_ERROR("ndsv5_resume_check failed");
		return ERROR_OK;
	}

	if (old_or_new_riscv_resume(target, current, address, handle_breakpoints,
			debug_execution) != ERROR_OK) {
		LOG_ERROR("old_or_new_riscv_resume failed");
		return ERROR_FAIL;
	}
	return ndsv5_handle_resume(target);
}

extern struct jtag_interface *jtag_interface;
int ndsv5_examine(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	if (nds_script_custom_initial != NULL) {
		LOG_DEBUG("doing custom_initial_script...");
		if (ndsv5_script_do_custom_reset(target, nds_script_custom_initial) != ERROR_OK)
			return ERROR_FAIL;
		nds_script_custom_initial = NULL;
		LOG_DEBUG("custom_initial_script finish");
	}
	if (target_was_examined(target)) {
		return ERROR_OK;
	}
	// Don't need to select dbus, since the first thing we do is read dtmcontrol.
	uint32_t retry_cnt = 0;
	uint32_t dtmcontrol = 0;
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt ++) {
		dtmcontrol = dtmcontrol_scan(target, 0);
		LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
		if ((dtmcontrol == 0x0) || (dtmcontrol == 0xFFFFFFFF)) {
			/* do jtag_interface->init() again when JTAG examine chain failed (SW workaround) */
			jtag_interface->quit();
			alive_sleep(1000);
			jtag_interface->init();
			jtag_init_inner(NULL);
		} else {
			break;
		}
	}

	riscv_examine(target);
	ndsv5_handle_examine(target);
	return ERROR_OK;
}

static int modify_trigger_address_mbit_match(struct target *target, struct trigger *trigger)
{
	uint64_t new_address = trigger->address;
	uint32_t i, new_length = trigger->length;
	uint64_t mbit_mask, mbit_value;

	while (1) { //(((new_address & mbit_mask) + new_length) < (trigger->address + trigger->length))
		for (i=0; i<32; i++) {
			if ((uint32_t)(0x01 << i) >= new_length)
				break;
		}
		new_length = (0x01 << i);
		if (new_address % new_length) {
			new_length <<= 1;
			i++;
		}
#if 0	
	for (i=0; i<32; i++) {
		if ((uint32_t)(0x01 << i) == new_length)
			break;
	}
#endif	
		if (i == 0) {
			LOG_DEBUG("ERROR length, new_address:0x%lx, new_length:0x%x", (long unsigned int)new_address, new_length);
			return ERROR_OK;
		}
		mbit_mask = ~((0x01 << i) - 1);
		mbit_value = (0x01 << (i - 1)) - 1;
		new_address &= mbit_mask;
		new_address |= mbit_value;

		LOG_DEBUG("new_address:0x%lx, new_length:0x%x", (long unsigned int)new_address, new_length);
		if (((new_address & mbit_mask) + new_length) < (trigger->address + trigger->length)) 
			new_address = trigger->address;
		else
			break;
	}
	LOG_DEBUG("real new_address:0x%lx, new_length:0x%x", (long unsigned int)new_address, new_length);
	// redefine: trigger->address
	trigger->address = new_address;
	return ERROR_OK;
}

static int ndsv5_writebuffer(struct target *target, target_addr_t address,
		uint32_t writesize, const uint8_t *buffer)
{
	//target_addr_t physical_address;
	riscv_select_current_hart(target);

	// write_memory will doing va2pa!!!
	//physical_address = address;
	//if( ndsv5_virtual_to_physical(target, address, &physical_address) != ERROR_OK )
	//	return ERROR_FAIL;
	//return ndsv5_write_buffer(target, physical_address, writesize, buffer);
	return ndsv5_write_buffer(target, address, writesize, buffer);
}

/* Page table PPN shift amount */
//#define PTE_PPN_SHIFT       10
//#define RISCV_PGSHIFT       12
//#define RISCV_PGSIZE (1 << RISCV_PGSHIFT)
/* Leaf page shift amount */
#define PGSHIFT             12

extern uint64_t ndsv5_reg_misa_value;
int ndsv5_get_physical_address(struct target *target, target_addr_t addr, target_addr_t *physical)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
	nds32->nds_va_to_pa_off = 1;
	uint64_t physical_addr = addr;
	uint64_t value_misa = 0;

	if (ndsv5_reg_misa_value == 0) {
		if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
			// use quick mode to read CSR while target_not_halted
			if (riscv_debug_buffer_size(target) < 6)
				goto ndsv5_get_physical_address_OK;
			ndsv5_get_csr_reg_quick_access(target, CSR_MISA + GDB_REGNO_CSR0, &value_misa);
		} else {
			struct reg *reg_misa = ndsv5_get_reg_by_CSR(target, CSR_MISA);
			if (reg_misa != NULL)
				value_misa = ndsv5_get_register_value(reg_misa);
		}
		if ((value_misa & 0x40000) == 0) {
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist = false;
		} else {
			target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist = true;
		}
	}
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist == false) {
		goto ndsv5_get_physical_address_OK;
	}

	uint64_t base, satp;
	uint32_t levels, ptidxbits, ptesize, vm;

	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_CSR0 + CSR_SATP, &satp);
	} else {
		riscv_get_register(target, &satp, GDB_REGNO_CSR0 + CSR_SATP);
	}
	if (riscv_xlen(target) == 64) {
		base = ((satp & SATP64_PPN) << PGSHIFT);
		vm = ((satp & SATP64_MODE) >> 60);
	} else {
		base = ((satp & SATP32_PPN) << PGSHIFT);
		vm = ((satp & SATP32_MODE) >> 31);
	}
	switch (vm) {
		case SATP_MODE_SV32:
			levels = 2; ptidxbits = 10; ptesize = 4; break;
		case SATP_MODE_SV39:
			levels = 3; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_SV48:
			levels = 4; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_SV57:
			levels = 5; ptidxbits = 9; ptesize = 8; break;
		case SATP_MODE_OFF:
			goto ndsv5_get_physical_address_OK;
		default:
			goto ndsv5_get_physical_address_ERR;
	}

	//uint32_t va_bits = PGSHIFT + levels * ptidxbits;
	//uint64_t mask = (1L << (TARGET_LONG_BITS - (va_bits - 1))) - 1;
	//uint64_t masked_msbs = (addr >> (va_bits - 1)) & mask;
	//if (masked_msbs != 0 && masked_msbs != mask) {
	//	LOG_ERROR("ERROR");
	//	return ERROR_FAIL;
	//}

	uint32_t ptshift = (levels - 1) * ptidxbits;
	LOG_DEBUG("ptshift: 0x%x, levels: 0x%x, ptesize: 0x%x", (int)ptshift, (int)levels, (int)ptesize);
	uint32_t i;
	for (i = 0; i < levels; i++, ptshift -= ptidxbits) {
		uint64_t idx = (addr >> (PGSHIFT + ptshift)) &
			((1 << ptidxbits) - 1);
		idx &= ((1 << ptidxbits) - 1);
		/* check that physical address of PTE is legal */
		uint64_t pte_addr = base + idx * ptesize;
		uint64_t pte = 0; //ldl_phys(cs->as, pte_addr);
		target_read_memory(target, pte_addr, ptesize, 1, (uint8_t *)&pte);

		uint64_t ppn = pte >> PTE_PPN_SHIFT;
		LOG_DEBUG("i: 0x%x, pte: 0x%" PRIx64 " ppn: 0x%" PRIx64, i, pte, ppn);
		LOG_DEBUG("pte_addr: 0x%" PRIx64 " base: 0x%" PRIx64, pte_addr, base);
		if (!(pte & PTE_V)) {
			/* Invalid PTE */
			LOG_ERROR("Invalid PTE");
			goto ndsv5_get_physical_address_ERR;
		} else if (!(pte & (PTE_R | PTE_W | PTE_X))) {
			/* Inner PTE, continue walking */
			base = ppn << PGSHIFT;
		} else if ((pte & (PTE_R | PTE_W | PTE_X)) == PTE_W) {
			/* Reserved leaf PTE flags: PTE_W */
			LOG_ERROR("Reserved leaf PTE flags: PTE_W");
			goto ndsv5_get_physical_address_ERR;
		} else if ((pte & (PTE_R | PTE_W | PTE_X)) == (PTE_W | PTE_X)) {
			/* Reserved leaf PTE flags: PTE_W + PTE_X */
			LOG_ERROR("Reserved leaf PTE flags: PTE_W + PTE_X");
			goto ndsv5_get_physical_address_ERR;
			//} else if ((pte & PTE_U) && ((mode != PRV_U) &&
			//           (!sum || access_type == MMU_INST_FETCH))) {
			//    /* User PTE flags when not U mode and mstatus.SUM is not set,
			//       or the access type is an instruction fetch */
			//    goto ndsv5_get_physical_address_ERR;
			//} else if (!(pte & PTE_U) && (mode != PRV_S)) {
			//    /* Supervisor PTE flags when not S mode */
			//    goto ndsv5_get_physical_address_ERR;
	} else if (ppn & ((1ULL << ptshift) - 1)) {
		/* Misaligned PPN */
		LOG_ERROR("Misaligned PPN");
		goto ndsv5_get_physical_address_ERR;
		//} else if (access_type == MMU_DATA_LOAD && !((pte & PTE_R) ||
		//           ((pte & PTE_X) && mxr))) {
		//    /* Read access check failed */
		//		LOG_ERROR("Read access check failed");
		//    goto ndsv5_get_physical_address_ERR;
		//} else if (access_type == MMU_DATA_STORE && !(pte & PTE_W)) {
		//    /* Write access check failed */
		//		LOG_ERROR("Write access check failed");
		//    goto ndsv5_get_physical_address_ERR;
		//} else if (access_type == MMU_INST_FETCH && !(pte & PTE_X)) {
		//    /* Fetch access check failed */
		//		LOG_ERROR("Fetch access check failed");
		//    goto ndsv5_get_physical_address_ERR;
	} else {
		/* for superpage mappings, make a fake leaf PTE for the TLB's
		   benefit. */
		uint64_t vpn = addr >> PGSHIFT;
		physical_addr = (ppn | (vpn & ((1L << ptshift) - 1))) << PGSHIFT;
		physical_addr |= (addr & ((1L << PGSHIFT) - 1));
		goto ndsv5_get_physical_address_OK;
	}
	}  // for (i = 0; i < levels; ...

ndsv5_get_physical_address_ERR:
	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	LOG_DEBUG("ERROR: VP: 0x%" PRIx64 " PA: 0x%" PRIx64, addr, physical_addr);
	*physical = physical_addr;
	return ERROR_FAIL;

ndsv5_get_physical_address_OK:
	nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
	LOG_DEBUG("OK: VP: 0x%" PRIx64 " PA: 0x%" PRIx64, addr, physical_addr);
	*physical = physical_addr;
	return ERROR_OK;
}

int ndsv5_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	//struct target_type *tt = get_target_type(target);
	uint64_t mstatus = 0;
	uint64_t dcsr = 0;

	LOG_DEBUG("va: 0x%" TARGET_PRIxADDR ", va_mode: %s", address, (nds32->nds_va_to_pa_off==1)? "OFF":"ON");
	if(nds32->nds_va_to_pa_off == 1) {  // va: off
		*physical = address;
		return ERROR_OK;
	}
	ndsv5_use_mprv_mode = 0;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6) {
			*physical = address;
			return ERROR_OK;
		}
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
		NDS_INFO("qmode get dcsr: 0x%" PRIx64, dcsr);
	} else {
		riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
	}
	// Check prv
	if((dcsr & 0x3) == 0x3)  {		// M-mode => VA=PA
		*physical = address;
		return ERROR_OK;
	}

	if (memory->access_channel == NDS_MEMORY_ACC_BUS) {
		return ndsv5_get_physical_address(target, address, physical);
	}
	// else if (memory->access_channel == NDS_MEMORY_ACC_CPU)
	// Check mprven
	dcsr |= 0x10; 			// Enable mprven
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, &mstatus);
		ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_DCSR, dcsr);
		ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DCSR, &dcsr);
	} else {
		riscv_get_register(target, &mstatus, GDB_REGNO_MSTATUS);
		riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
		riscv_get_register(target, &dcsr, GDB_REGNO_DCSR);
	}
	if( (dcsr&0x10) == 0x0 ) {
		LOG_DEBUG("This core doesn't support use mprv mode!!");
		*physical = address;
		return ERROR_OK;
	}

	ndsv5_use_mprv_mode  = 1;
	ndsv5_backup_mstatus = mstatus;
	mstatus |= MSTATUS_MPRV; 		// Enable mprv
	mstatus &= ~(MSTATUS_MPP);
	mstatus |= (1) << 11;		// Set mstatus.MPP to supervisor
	mstatus |= MSTATUS_SUM;		// Set SUM = 1
	mstatus |= MSTATUS_MXR;		// Set MXR = 1

	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		ndsv5_set_csr_reg_quick_access(target, GDB_REGNO_MSTATUS, mstatus);
	} else {
		riscv_set_register(target, GDB_REGNO_MSTATUS, mstatus);
	}
	*physical = address;
	return ERROR_OK;
}

void bus_mode_on(struct target *target, uint64_t *reg_value_backup)
{
	uint64_t old_mcache_ctl = 0;
	uint64_t new_mcache_ctl = 0;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6)
				return;
		ndsv5_get_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, &old_mcache_ctl);
		*reg_value_backup = old_mcache_ctl;
		new_mcache_ctl = (old_mcache_ctl & ~(0x3));
		ndsv5_set_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, new_mcache_ctl);
		return;
	}
	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG("No mcache_ctl, No support dis_cache_busmode");
		return;
	}
	old_mcache_ctl = ndsv5_get_register_value(reg_mcache_ctl);
	*reg_value_backup = old_mcache_ctl;
	new_mcache_ctl = (old_mcache_ctl & ~(0x3));
	ndsv5_set_register_value(reg_mcache_ctl, new_mcache_ctl);
	LOG_DEBUG("old_mcache_ctl: 0x%x, new_mcache_ctl: 0x%x", (int)old_mcache_ctl, (int)new_mcache_ctl);
}

void bus_mode_off(struct target *target, uint64_t reg_value)
{
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) < 6)
				return;
		ndsv5_set_csr_reg_quick_access(target, CSR_MCACHE_CTL + GDB_REGNO_CSR0, reg_value);
		return;
	}
	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG("No mcache_ctl, No support dis_cache_busmode");
		return;
	}
	ndsv5_set_register_value(reg_mcache_ctl, reg_value);
}

struct reg *ndsv5_get_reg_by_CSR(struct target *target, uint32_t csr_id)
{
    char *reg_name;
    struct reg *reg;
    if ((reg_name = ndsv5_get_CSR_name(target, csr_id)) == NULL)
    {
        LOG_DEBUG("get reg_name ERROR");
        return NULL;
    }
    if ((reg = register_get_by_name(target->reg_cache, reg_name, 1)) == NULL)
    {
        LOG_DEBUG("get reg ERROR");
        return NULL;
    }
    return reg;
}

uint64_t ndsv5_get_register_value(struct reg *reg)
{
    uint64_t reg_value;
    reg->type->get(reg);
    reg_value = buf_get_u64(reg->value, 0, reg->size);
    return reg_value;
}

void ndsv5_set_register_value(struct reg *reg, uint64_t reg_value)
{
    uint8_t reg_bytes[8];
    buf_set_u64(reg_bytes, 0, reg->size, reg_value);
    reg->type->set(reg, reg_bytes);
}

struct target_type nds_v5_target =
{
	.name = "nds_v5",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = ndsv5_examine,

	/* poll current target status */
	.poll = ndsv5_poll,

	.halt = ndsv5_halt,
	.resume = ndsv5_resume,
	.step = ndsv5_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,
	.write_buffer = ndsv5_writebuffer,

	.blank_check_memory = riscv_blank_check_memory,
	.checksum_memory = riscv_checksum_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = ndsv5_command_handlers,
	.target_create = ndsv5_target_create,
	.get_gdb_fileio_info = ndsv5_get_gdb_fileio_info,
	.gdb_fileio_end = ndsv5_gdb_fileio_end,
	.hit_watchpoint = ndsv5_hit_watchpoint,
};
#endif

