/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "helper/list.h"
#include "riscv.h"
#include "rtos/riscv_debug.h"
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

#if _NDS_V5_ONLY_
#include "ndsv5.h"
#include "ndsv5-013.h"
#include "target/nds32_log.h"
#endif


#define DMI_DATA1 (DMI_DATA0 + 1)
#define DMI_PROGBUF1 (DMI_PROGBUF0 + 1)

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step, bool use_hasel);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int hid, int rid);
static int riscv013_set_register(struct target *target, int hartid, int regid, uint64_t value);
static int riscv013_select_current_hart(struct target *target);
static int riscv013_halt_prep(struct target *target);
static int riscv013_halt_go(struct target *target);
static int riscv013_resume_go(struct target *target);
static int riscv013_step_current_hart(struct target *target);
static int riscv013_on_halt(struct target *target);
static int riscv013_on_step(struct target *target);
static int riscv013_resume_prep(struct target *target);
static bool riscv013_is_halted(struct target *target);
static enum riscv_halt_reason riscv013_halt_reason(struct target *target);
static int riscv013_write_debug_buffer(struct target *target, unsigned index,
		riscv_insn_t d);
static riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned
		index);
static int riscv013_execute_debug_buffer(struct target *target);
static void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
static void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a);
static int riscv013_dmi_write_u64_bits(struct target *target);
static void riscv013_fill_dmi_nop_u64(struct target *target, char *buf);
static int register_read(struct target *target, uint64_t *value, uint32_t number);
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int riscv013_test_sba_config_reg(struct target *target, target_addr_t legal_address,
		uint32_t num_words, target_addr_t illegal_address, bool run_sbbusyerror_test);
void write_memory_sba_simple(struct target *target, target_addr_t addr, uint32_t* write_data,
		uint32_t write_size, uint32_t sbcs);
void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs);
static int	riscv013_test_compliance(struct target *target);

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

#define CSR_DCSR_CAUSE_SWBP		1
#define CSR_DCSR_CAUSE_TRIGGER	2
#define CSR_DCSR_CAUSE_DEBUGINT	3
#define CSR_DCSR_CAUSE_STEP		4
#define CSR_DCSR_CAUSE_HALT		5

#define RISCV013_INFO(r) riscv013_info_t *r = get_info(target)

/*** JTAG registers. ***/

typedef enum {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
} dmi_op_t;
typedef enum {
#if _NDS_DMI_CHECK_TIMEOUT_
	DMI_STATUS_CHECK_TIMEOUT = 0xFF,
#endif /* _NDS_DMI_CHECK_TIMEOUT_ */
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
} dmi_status_t;

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

#define CMDERR_NONE				0
#define CMDERR_BUSY				1
#define CMDERR_NOT_SUPPORTED	2
#define CMDERR_EXCEPTION		3
#define CMDERR_HALT_RESUME		4
#define CMDERR_OTHER			7

/*** Info about the core being debugged. ***/

#if _NDS_V5_ONLY_
/* Move this struct declaration to riscv.h */
#else /* _NDS_V5_ONLY_ */
struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};
#endif /* _NDS_V5_ONLY_ */

typedef enum {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
} yes_no_maybe_t;

typedef struct {
	struct list_head list;
	int abs_chain_position;

	/* The number of harts connected to this DM. */
	int hart_count;
	/* Indicates we already reset this DM, so don't need to do it again. */
	bool was_reset;
	/* Targets that are connected to this DM. */
	struct list_head target_list;
	/* The currently selected hartid on this DM. */
	int current_hartid;
	bool hasel_supported;

	/* The program buffer stores executable code. 0 is an illegal instruction,
	 * so we use 0 to mean the cached value is invalid. */
	uint32_t progbuf_cache[16];
} dm013_info_t;

typedef struct {
	struct list_head list;
	struct target *target;
} target_list_t;

typedef struct {
	/* The indexed used to address this hart in its DM. */
	unsigned index;
	/* Number of address bits in the dbus register. */
	unsigned abits;
	/* Number of abstract command data registers. */
	unsigned datacount;
	/* Number of words in the Program Buffer. */
	unsigned progbufsize;

	/* We cache the read-only bits of sbcs here. */
	uint32_t sbcs;

	yes_no_maybe_t progbuf_writable;
	/* We only need the address so that we know the alignment of the buffer. */
	riscv_addr_t progbuf_address;

	/* Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned int dtmcs_idle;

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

	/* When a function returns some error due to a failure indicated by the
	 * target in cmderr, the caller can look here to see what that error was.
	 * (Compare with errno.) */
	uint8_t cmderr;

	/* Some fields from hartinfo. */
	uint8_t datasize;
	uint8_t dataaccess;
	int16_t dataaddr;

	/* The width of the hartsel field. */
	unsigned hartsellen;

	/* DM that provides access to this target. */
	dm013_info_t *dm;
} riscv013_info_t;

LIST_HEAD(dm_list);

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv013_info_t *) info->version_specific;
}

/**
 * Return the DM structure for this target. If there isn't one, find it in the
 * global list of DMs. If it's not in there, then create one and initialize it
 * to 0.
 */
dm013_info_t *get_dm(struct target *target)
{
	RISCV013_INFO(info);
	if (info->dm)
		return info->dm;

	int abs_chain_position = target->tap->abs_chain_position;

	dm013_info_t *entry;
	dm013_info_t *dm = NULL;
	list_for_each_entry(entry, &dm_list, list) {
		if (entry->abs_chain_position == abs_chain_position) {
			dm = entry;
			break;
		}
	}

	if (!dm) {
		LOG_DEBUG("[%d] Allocating new DM", target->coreid);
		dm = calloc(1, sizeof(dm013_info_t));
		dm->abs_chain_position = abs_chain_position;
		dm->current_hartid = -1;
		dm->hart_count = -1;
		INIT_LIST_HEAD(&dm->target_list);
		list_add(&dm->list, &dm_list);
	}

	info->dm = dm;
	target_list_t *target_entry;
	list_for_each_entry(target_entry, &dm->target_list, list) {
		if (target_entry->target == target)
			return dm;
	}
	target_entry = calloc(1, sizeof(*target_entry));
	target_entry->target = target;
	list_add(&target_entry->list, &dm->target_list);

	return dm;
}

static uint32_t set_hartsel(uint32_t initial, uint32_t index)
{
	initial &= ~DMI_DMCONTROL_HARTSELLO;
	initial &= ~DMI_DMCONTROL_HARTSELHI;

	uint32_t index_lo = index & ((1 << DMI_DMCONTROL_HARTSELLO_LENGTH) - 1);
	initial |= index_lo << DMI_DMCONTROL_HARTSELLO_OFFSET;
	uint32_t index_hi = index >> DMI_DMCONTROL_HARTSELLO_LENGTH;
	assert(index_hi < 1 << DMI_DMCONTROL_HARTSELHI_LENGTH);
	initial |= index_hi << DMI_DMCONTROL_HARTSELHI_OFFSET;

	return initial;
}

static void decode_dmi(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ, "haltreq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_RESUMEREQ, "resumereq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HARTRESET, "hartreset" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HASEL, "hasel" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HARTSELHI, "hartselhi" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HARTSELLO, "hartsello" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_NDMRESET, "ndmreset" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE, "dmactive" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_ACKHAVERESET, "ackhavereset" },

		{ DMI_DMSTATUS, DMI_DMSTATUS_IMPEBREAK, "impebreak" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLHAVERESET, "allhavereset" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYHAVERESET, "anyhavereset" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLRESUMEACK, "allresumeack" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYRESUMEACK, "anyresumeack" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLNONEXISTENT, "allnonexistent" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYNONEXISTENT, "anynonexistent" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLUNAVAIL, "allunavail" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYUNAVAIL, "anyunavail" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLRUNNING, "allrunning" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYRUNNING, "anyrunning" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ALLHALTED, "allhalted" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_ANYHALTED, "anyhalted" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_AUTHENTICATED, "authenticated" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_AUTHBUSY, "authbusy" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_HASRESETHALTREQ, "hasresethaltreq" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_CONFSTRPTRVALID, "confstrptrvalid" },
		{ DMI_DMSTATUS, DMI_DMSTATUS_VERSION, "version" },

		{ DMI_ABSTRACTCS, DMI_ABSTRACTCS_PROGBUFSIZE, "progbufsize" },
		{ DMI_ABSTRACTCS, DMI_ABSTRACTCS_BUSY, "busy" },
		{ DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR, "cmderr" },
		{ DMI_ABSTRACTCS, DMI_ABSTRACTCS_DATACOUNT, "datacount" },

		{ DMI_COMMAND, DMI_COMMAND_CMDTYPE, "cmdtype" },

		{ DMI_SBCS, DMI_SBCS_SBVERSION, "sbversion" },
		{ DMI_SBCS, DMI_SBCS_SBBUSYERROR, "sbbusyerror" },
		{ DMI_SBCS, DMI_SBCS_SBBUSY, "sbbusy" },
		{ DMI_SBCS, DMI_SBCS_SBREADONADDR, "sbreadonaddr" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS, "sbaccess" },
		{ DMI_SBCS, DMI_SBCS_SBAUTOINCREMENT, "sbautoincrement" },
		{ DMI_SBCS, DMI_SBCS_SBREADONDATA, "sbreadondata" },
		{ DMI_SBCS, DMI_SBCS_SBERROR, "sberror" },
		{ DMI_SBCS, DMI_SBCS_SBASIZE, "sbasize" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS128, "sbaccess128" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS64, "sbaccess64" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS32, "sbaccess32" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS16, "sbaccess16" },
		{ DMI_SBCS, DMI_SBCS_SBACCESS8, "sbaccess8" },
#if _NDS_V5_ONLY_
		/*{ AC_ACCESS_REGISTER, DMI_COMMAND_AC_QUICK_ACCESS, "quick" },*/
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_POSTEXEC, "postexec" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_AARSIZE, "size" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_TRANSFER, "transfer" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_WRITE, "write" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_REGNO, "regno" },
		{ DMI_DMCS2, DMI_DMCS2_HGSELECT, "hgselect" },
		{ DMI_DMCS2, DMI_DMCS2_HGWRITE, "hgwrite" },
		{ DMI_DMCS2, DMI_DMCS2_HALTGROUP, "haltgroup" },
		{ DMI_DMCS2, DMI_DMCS2_EXTTRIGGER, "exttrigger" },
#endif /* _NDS_V5_ONLY_ */
	};

	text[0] = 0;
	for (unsigned i = 0; i < DIM(description); i++) {
		if (description[i].address == address) {
			uint64_t mask = description[i].mask;
			unsigned value = get_field(data, mask);
			if (value) {
				if (i > 0)
					*(text++) = ' ';
				if (mask & (mask >> 1)) {
					/* If the field is more than 1 bit wide. */
					sprintf(text, "%s=%d", description[i].name, value);
				} else {
					strcpy(text, description[i].name);
				}
				text += strlen(text);
			}
		}
	}
}

static void dump_field(int idle, const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

#if _NDS_V5_ONLY_
	if (debug_level < LOG_LVL_INFO)
		return;
#else /* _NDS_V5_ONLY_ */
	if (debug_level < LOG_LVL_DEBUG)
		return;
#endif /* _NDS_V5_ONLY_ */

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %08x @%02x -> %s %08x @%02x; %di",
			field->num_bits, op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address, idle);

	char out_text[500];
	char in_text[500];
	decode_dmi(out_text, out_address, out_data);
	decode_dmi(in_text, in_address, in_data);
	if (in_text[0] || out_text[0]) {
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "%s -> %s",
				out_text, in_text);
	}
}

/*** Utility functions. ***/

static void select_dmi(struct target *target)
{
	if (bscan_tunnel_ir_width != 0) {
		select_dmi_via_bscan(target);
		return;
	}
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
}

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4];

	if (bscan_tunnel_ir_width != 0)
		return dtmcontrol_scan_via_bscan(target, out);

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dmi. */
	select_dmi(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
}

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

/**
 * exec: If this is set, assume the scan results in an execution, so more
 * run-test/idle cycles may be required.
 */
static dmi_status_t dmi_scan(struct target *target, uint32_t *address_in,
		uint32_t *data_in, dmi_op_t op, uint32_t address_out, uint32_t data_out,
		bool exec)
{
	riscv013_info_t *info = get_info(target);
	RISCV_INFO(r);
	unsigned num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH;
	size_t num_bytes = (num_bits + 7) / 8;
	//uint8_t in[num_bytes];
	//uint8_t out[num_bytes];
	uint8_t in[8] = {0};
	uint8_t out[8];
	struct scan_field field = {
		.num_bits = num_bits,
		.out_value = out,
		.in_value = in
	};
	riscv_bscan_tunneled_scan_context_t bscan_ctxt;

	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait--;
		if (r->reset_delays_wait < 0) {
#if _NDS_V5_ONLY_
			info->dmi_busy_delay = v5_dmi_busy_delay_count;
#else
			info->dmi_busy_delay = 0;
#endif
			info->ac_busy_delay = 0;
		}
	}

	//memset(in, 0, num_bytes);

	assert(info->abits != 0);

	//buf_set_u32(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	//buf_set_u32(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	///buf_set_u32(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);
	buf_set_u64(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u64(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u64(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	/* I wanted to place this code in a different function, but the way JTAG command
	   queueing works in the jtag handling functions, the scan fields either have to be
	   heap allocated, global/static, or else they need to stay on the stack until
	   the jtag_execute_queue() call.  Heap or static fields in this case doesn't seem
	   the best fit.  Declaring stack based field values in a subsidiary function call wouldn't
	   work. */
	if (bscan_tunnel_ir_width != 0) {
		riscv_add_bscan_tunneled_scan(target, &field, &bscan_ctxt);
	} else {
		/* Assume dbus is already selected. */
		jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	}

	int idle_count = info->dmi_busy_delay;

#if _NDS_V5_ONLY_
	if ( idle_count < info->dtmcs_idle )
		idle_count = info->dtmcs_idle;
#endif /* _NDS_V5_ONLY_ */

	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		return DMI_STATUS_FAILED;
	}

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		buffer_shr(in, num_bytes, 1);
	}

	if (data_in)
		*data_in = buf_get_u32(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);

#if _NDS_V5_ONLY_
	/* Check if target hang or disconnect */
	uint32_t in_value = buf_get_u32(in, 0, 32);
	if (in_value == 0xFFFFFFFF) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
	}
#endif /* _NDS_V5_ONLY_ */

	dump_field(idle_count, &field);

	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

/**
 * @param data_in  The data we received from the target.
 * @param dmi_op   The operation to perform (read/write/nop).
 * @param dmi_busy_encountered
 *                 If non-NULL, will be updated to reflect whether DMI busy was
 *                 encountered while executing this operation or not.
 * @param address  The address argument to that operation.
 * @param data_out The data to send to the target.
 * @param exec     When true, this scan will execute something, so extra RTI
 *                 cycles may be added.
 * @param ensure_success
 *                 Scan a nop after the requested operation, ensuring the
 *                 DMI operation succeeded.
 */
static int dmi_op_timeout(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, int timeout_sec, bool exec, bool ensure_success)
{
	select_dmi(target);

	dmi_status_t status;
	uint32_t address_in;

	if (dmi_busy_encountered)
		*dmi_busy_encountered = false;

	const char *op_name;
	switch (dmi_op) {
		case DMI_OP_NOP:
			op_name = "nop";
			break;
		case DMI_OP_READ:
			op_name = "read";
			break;
		case DMI_OP_WRITE:
			op_name = "write";
			break;
		default:
			LOG_ERROR("Invalid DMI operation: %d", dmi_op);
			return ERROR_FAIL;
	}

	time_t start = time(NULL);
	/* This first loop performs the request.  Note that if for some reason this
	 * stays busy, it is actually due to the previous access. */
	while (1) {
		status = dmi_scan(target, NULL, NULL, dmi_op, address, data_out,
				exec);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
			if (dmi_busy_encountered)
				*dmi_busy_encountered = true;
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed %s at 0x%x, status=%d", op_name, address, status);
			return ERROR_FAIL;
		}
		if (time(NULL) - start > timeout_sec)
			return ERROR_TIMEOUT_REACHED;
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed %s at 0x%x; status=%d", op_name, address, status);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Failed %s at 0x%x; status=%d", op_name, address, status);
#endif
		return ERROR_FAIL;
	}

	if (ensure_success) {
		/* This second loop ensures the request succeeded, and gets back data.
		 * Note that NOP can result in a 'busy' result as well, but that would be
		 * noticed on the next DMI access we do. */
		while (1) {
			status = dmi_scan(target, &address_in, data_in, DMI_OP_NOP, address, 0,
					false);
			if (status == DMI_STATUS_BUSY) {
				increase_dmi_busy_delay(target);
				if (dmi_busy_encountered)
					*dmi_busy_encountered = true;
			} else if (status == DMI_STATUS_SUCCESS) {
				break;
			} else {
				if (data_in) {
					LOG_ERROR("Failed %s (NOP) at 0x%x; value=0x%x, status=%d",
							op_name, address, *data_in, status);
				} else {
					LOG_ERROR("Failed %s (NOP) at 0x%x; status=%d", op_name, address,
							status);
				}
				return ERROR_FAIL;
			}
			if (time(NULL) - start > timeout_sec)
				return ERROR_TIMEOUT_REACHED;
		}
	}

	return ERROR_OK;
}

static int dmi_op(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, bool exec, bool ensure_success)
{
	int result = dmi_op_timeout(target, data_in, dmi_busy_encountered, dmi_op,
			address, data_out, riscv_command_timeout_sec, exec, ensure_success);
	if (result == ERROR_TIMEOUT_REACHED) {
#if _NDS_V5_ONLY_
		LOG_ERROR("DMI operation didn't complete in %d seconds. The target was "
				"either too slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				riscv_command_timeout_sec);
#else
		LOG_ERROR("DMI operation didn't complete in %d seconds. The target is "
				"either really slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				riscv_command_timeout_sec);
#endif /* _NDS_V5_ONLY_ */
		return ERROR_FAIL;
	}
	return result;
}

static int dmi_read(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, false, true);
}

static int dmi_read_exec(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, true, true);
}

static int dmi_write(struct target *target, uint32_t address, uint32_t value)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, false, true);
}

static int dmi_write_exec(struct target *target, uint32_t address,
		uint32_t value, bool ensure_success)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, true, ensure_success);
}

int dmstatus_read_timeout(struct target *target, uint32_t *dmstatus,
		bool authenticated, unsigned timeout_sec)
{
	int result = dmi_op_timeout(target, dmstatus, NULL, DMI_OP_READ,
			DMI_DMSTATUS, 0, timeout_sec, false, true);
	if (result != ERROR_OK)
		return result;
	int dmstatus_version = get_field(*dmstatus, DMI_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2 (0.13) and 3 (0.14), not "
				"%d (dmstatus=0x%x). This error might be caused by a JTAG "
				"signal issue. Try reducing the JTAG clock speed.",
				get_field(*dmstatus, DMI_DMSTATUS_VERSION), *dmstatus);
#if _NDS_V5_ONLY_
		assert(0);
#endif /* _NDS_V5_ONLY_ */
	} else if (authenticated && !get_field(*dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", *dmstatus);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int dmstatus_read(struct target *target, uint32_t *dmstatus,
		bool authenticated)
{
	return dmstatus_read_timeout(target, dmstatus, authenticated,
			riscv_command_timeout_sec);
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);
}

uint32_t abstract_register_size(unsigned width)
{
	switch (width) {
		case 32:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 3);
			break;
		case 128:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 4);
			break;
		default:
			LOG_ERROR("Unsupported register width: %d", width);
			return 0;
	}
}

static int wait_for_idle(struct target *target, uint32_t *abstractcs)
{
	RISCV013_INFO(info);
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, abstractcs, DMI_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(*abstractcs, DMI_ABSTRACTCS_BUSY) == 0)
			return ERROR_OK;

		if (time(NULL) - start > riscv_command_timeout_sec) {
			info->cmderr = get_field(*abstractcs, DMI_ABSTRACTCS_CMDERR);
			if (info->cmderr != CMDERR_NONE) {
				const char *errors[8] = {
					"none",
					"busy",
					"not supported",
					"exception",
					"halt/resume",
					"reserved",
					"reserved",
					"other" };

				LOG_ERROR("Abstract command ended in error '%s' (abstractcs=0x%x)",
						errors[info->cmderr], *abstractcs);
			}

#if _NDS_V5_ONLY_
			LOG_ERROR("Timed out after %ds waiting for \"busy\" to go low (abstractcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					*abstractcs);
#else
			LOG_ERROR("Timed out after %ds waiting for busy to go low (abstractcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					*abstractcs);
#endif /* _NDS_V5_ONLY_ */
			return ERROR_FAIL;
		}
	}
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	RISCV013_INFO(info);
	if (debug_level >= LOG_LVL_DEBUG) {
		switch (get_field(command, DMI_COMMAND_CMDTYPE)) {
			case 0:
				LOG_DEBUG("command=0x%x; access register, size=%d, postexec=%d, "
						"transfer=%d, write=%d, regno=0x%x",
						command,
						8 << get_field(command, AC_ACCESS_REGISTER_AARSIZE),
						get_field(command, AC_ACCESS_REGISTER_POSTEXEC),
						get_field(command, AC_ACCESS_REGISTER_TRANSFER),
						get_field(command, AC_ACCESS_REGISTER_WRITE),
						get_field(command, AC_ACCESS_REGISTER_REGNO));
				break;
			default:
				LOG_DEBUG("command=0x%x", command);
				break;
		}
	}

#if _NDS_JTAG_SCANS_OPTIMIZE_
	uint32_t cs = 0;
	if (nds_jtag_scans_optimize > 0) {
		if (write_debug_buffer_batch == NULL) {
			write_debug_buffer_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		}
		riscv_batch_add_dmi_write(write_debug_buffer_batch, DMI_COMMAND, command);
		size_t index_ABSTRACTCS = riscv_batch_add_dmi_read(write_debug_buffer_batch, DMI_ABSTRACTCS);

#if _NDS_MEM_Q_ACCESS_
		if (nds_dmi_quick_access_ena)
			index_ABSTRACTCS = riscv_batch_add_dmi_read(write_debug_buffer_batch, DMI_ABSTRACTCS);
#endif /* _NDS_MEM_Q_ACCESS_ */

		uint32_t retry_cnt = 0;
		while (retry_cnt < ndsv5_dmi_busy_retry_times) {
			retry_cnt ++;
			if (riscv_batch_run(write_debug_buffer_batch) != ERROR_OK) {
				LOG_DEBUG("riscv_batch_run_FAIL");
				increase_dmi_busy_delay(target);
				riscv013_clear_abstract_error(target);
				write_debug_buffer_batch->idle_count = info->dmi_busy_delay + info->ac_busy_delay;
				if (retry_cnt == ndsv5_dmi_busy_retry_times) {
					LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", ndsv5_dmi_busy_retry_times);
					return ERROR_FAIL;
				}
			} else {
				break;
			}
		}
		uint64_t dmi_out = riscv_batch_get_dmi_read(write_debug_buffer_batch, index_ABSTRACTCS);
		riscv_batch_free(write_debug_buffer_batch);
		write_debug_buffer_batch = NULL;

		cs = (uint32_t)buf_get_u64((uint8_t *)&dmi_out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
		if (get_field(cs, DMI_ABSTRACTCS_BUSY) != 0) {
			wait_for_idle(target, (uint32_t *)&cs);
		}
	} else {
		dmi_write(target, DMI_COMMAND, command);
		uint32_t abstractcs = 0;
		wait_for_idle(target, &abstractcs);
		dmi_read(target, &cs, DMI_ABSTRACTCS);
	}

	info->cmderr = get_field(cs, DMI_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, cs);
		/* Clear the error. */
		dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
		return ERROR_FAIL;
	}
#else /* _NDS_JTAG_SCANS_OPTIMIZE_ */

	if (dmi_write_exec(target, DMI_COMMAND, command, false) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t abstractcs = 0;
	int result = wait_for_idle(target, &abstractcs);

	info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0 || result != ERROR_OK) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, abstractcs);
		/* Clear the error. */
		dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
		return ERROR_FAIL;
	}
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_ */

	return ERROR_OK;
}

static riscv_reg_t read_abstract_arg(struct target *target, unsigned index,
		unsigned size_bits)
{
#if _NDS_V5_ONLY_
	if ((index >= GDB_REGNO_FPR0) && (index <= GDB_REGNO_FPR31))
		size_bits = 64;
#endif

	riscv_reg_t value = 0;
	uint32_t v;
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ~0;
		case 64:
			dmi_read(target, &v, DMI_DATA0 + offset + 1);
			value |= ((uint64_t) v) << 32;
			/* falls through */
		case 32:
			dmi_read(target, &v, DMI_DATA0 + offset);
			value |= v;
	}
	return value;
}

static int write_abstract_arg(struct target *target, unsigned index,
		riscv_reg_t value, unsigned size_bits)
{
#if _NDS_V5_ONLY_
	if ((index >= GDB_REGNO_FPR0) && (index <= GDB_REGNO_FPR31))
		size_bits = 64;
#endif

	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ERROR_FAIL;
		case 64:
			dmi_write(target, DMI_DATA0 + offset + 1, value >> 32);
			/* falls through */
		case 32:
			dmi_write(target, DMI_DATA0 + offset, value);
	}
	return ERROR_OK;
}

/**
 * @par size in bits
 */
static uint32_t access_register_command(struct target *target, uint32_t number,
		unsigned size, uint32_t flags)
{
	uint32_t command = set_field(0, DMI_COMMAND_CMDTYPE, 0);
	switch (size) {
		case 32:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 2);
			break;
		case 64:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 3);
			break;
		default:
			LOG_ERROR("%d-bit register %s not supported.", size,
					gdb_regno_name(number));
			assert(0);
	}

	if (number <= GDB_REGNO_XPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1000 + number - GDB_REGNO_ZERO);
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1020 + number - GDB_REGNO_FPR0);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				number - GDB_REGNO_CSR0);
	} else if (number >= GDB_REGNO_COUNT) {
		/* Custom register. */
		assert(target->reg_cache->reg_list[number].arch_info);
		riscv_reg_info_t *reg_info = target->reg_cache->reg_list[number].arch_info;
		assert(reg_info);
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0xc000 + reg_info->custom_number);
	} else {
		assert(0);
	}

	command |= flags;

	return command;
}

static int register_read_abstract(struct target *target, uint64_t *value,
		uint32_t number, unsigned size)
{
	RISCV013_INFO(info);

#if _NDS_IDE_MESSAGE_
	if (number > GDB_REGNO_CSR4095)
		NDS32_LOG("Unsupported register (enum gdb_regno)(%d)", number);
#endif

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_read_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_read_csr_supported)
		return ERROR_FAIL;
	/* The spec doesn't define abstract register numbers for vector registers. */
	if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31)
		return ERROR_FAIL;

#if _NDS_V5_ONLY_
	if ((number >= GDB_REGNO_FPR0) && (number <= GDB_REGNO_FPR31))
		size = 64;
	//read_abstract_reg_number = number;
#endif

	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER);

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_read_fpr_supported = false;
				LOG_INFO("Disabling abstract command reads from FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_read_csr_supported = false;
				LOG_INFO("Disabling abstract command reads from CSRs.");
			}
		}
		return result;
	}

	if (value)
		*value = read_abstract_arg(target, 0, size);

	return ERROR_OK;
}

static int register_write_abstract(struct target *target, uint32_t number,
		uint64_t value, unsigned size)
{
	RISCV013_INFO(info);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_write_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_write_csr_supported)
		return ERROR_FAIL;

#if _NDS_IDE_MESSAGE_
	if (number > GDB_REGNO_CSR4095)
		NDS32_LOG("Unsupported register (enum gdb_regno)(%d)", number);

	if ((number >= GDB_REGNO_FPR0) && (number <= GDB_REGNO_FPR31))
		size = 64;
#endif


	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);

	if (write_abstract_arg(target, 0, value, size) != ERROR_OK)
		return ERROR_FAIL;

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_write_fpr_supported = false;
				LOG_INFO("Disabling abstract command writes to FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_write_csr_supported = false;
				LOG_INFO("Disabling abstract command writes to CSRs.");
			}
		}
		return result;
	}

	return ERROR_OK;
}

/*
 * Sets the AAMSIZE field of a memory access abstract command based on
 * the width (bits).
 */
static uint32_t abstract_memory_size(unsigned width)
{
	switch (width) {
		case 8:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 0);
		case 16:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 1);
		case 32:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 3);
		case 128:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 4);
		default:
			LOG_ERROR("Unsupported memory width: %d", width);
			return 0;
	}
}

/*
 * Creates a memory access abstract command.
 */
static uint32_t access_memory_command(struct target *target, bool virtual,
		unsigned width, bool postincrement, bool write)
{
	uint32_t command = set_field(0, AC_ACCESS_MEMORY_CMDTYPE, 2);
	command = set_field(command, AC_ACCESS_MEMORY_AAMVIRTUAL, virtual);
	command |= abstract_memory_size(width);
	command = set_field(command, AC_ACCESS_MEMORY_AAMPOSTINCREMENT,
						postincrement);
	command = set_field(command, AC_ACCESS_MEMORY_WRITE, write);

	return command;
}

static int examine_progbuf(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->progbuf_writable != YNM_MAYBE)
		return ERROR_OK;

	/* Figure out if progbuf is writable. */

	if (info->progbufsize < 1) {
		info->progbuf_writable = YNM_NO;
		LOG_INFO("No program buffer present.");
		return ERROR_OK;
	}

	uint64_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, auipc(S0));
	if (riscv_program_exec(&program, target) != ERROR_OK)
		return ERROR_FAIL;

	if (register_read_direct(target, &info->progbuf_address, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	riscv_program_init(&program, target);
	riscv_program_insert(&program, sw(S0, S0, 0));
	int result = riscv_program_exec(&program, target);

	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	if (result != ERROR_OK) {
		/* This program might have failed if the program buffer is not
		 * writable. */
		info->progbuf_writable = YNM_NO;
		return ERROR_OK;
	}

	uint32_t written;
	if (dmi_read(target, &written, DMI_PROGBUF0) != ERROR_OK)
		return ERROR_FAIL;
	if (written == (uint32_t) info->progbuf_address) {
		LOG_INFO("progbuf is writable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_YES;

	} else {
		LOG_INFO("progbuf is not writeable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_NO;
	}

	return ERROR_OK;
}

static int is_fpu_reg(uint32_t gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_FPR0 && gdb_regno <= GDB_REGNO_FPR31) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FFLAGS) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FRM) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FCSR);
}

static int is_vector_reg(uint32_t gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_V0 && gdb_regno <= GDB_REGNO_V31) ||
		gdb_regno == GDB_REGNO_VSTART ||
		gdb_regno == GDB_REGNO_VXSAT ||
		gdb_regno == GDB_REGNO_VXRM ||
		gdb_regno == GDB_REGNO_VL ||
		gdb_regno == GDB_REGNO_VTYPE ||
		gdb_regno == GDB_REGNO_VLENB;
}

static int prep_for_register_access(struct target *target, uint64_t *mstatus,
		int regno)
{
	if (is_fpu_reg(regno) || is_vector_reg(regno)) {
		if (register_read(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if (is_fpu_reg(regno) && (*mstatus & MSTATUS_FS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (is_vector_reg(regno) && (*mstatus & MSTATUS_VS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_VS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int cleanup_after_register_access(struct target *target,
		uint64_t mstatus, int regno)
{
	if ((is_fpu_reg(regno) && (mstatus & MSTATUS_FS) == 0) ||
			(is_vector_reg(regno) && (mstatus & MSTATUS_VS) == 0))
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;
	return ERROR_OK;
}

typedef enum {
	SPACE_DMI_DATA,
	SPACE_DMI_PROGBUF,
	SPACE_DMI_RAM
} memory_space_t;

typedef struct {
	/* How can the debugger access this memory? */
	memory_space_t memory_space;
	/* Memory address to access the scratch memory from the hart. */
	riscv_addr_t hart_address;
	/* Memory address to access the scratch memory from the debugger. */
	riscv_addr_t debug_address;
	struct working_area *area;
} scratch_mem_t;

/**
 * Find some scratch memory to be used with the given program.
 */
static int scratch_reserve(struct target *target,
		scratch_mem_t *scratch,
		struct riscv_program *program,
		unsigned size_bytes)
{
	riscv_addr_t alignment = 1;
	while (alignment < size_bytes)
		alignment *= 2;

	scratch->area = NULL;

	riscv013_info_t *info = get_info(target);

	/* Option 1: See if data# registers can be used as the scratch memory */
	if (info->dataaccess == 1) {
		/* Sign extend dataaddr. */
		scratch->hart_address = info->dataaddr;
		if (info->dataaddr & (1<<11))
			scratch->hart_address |= 0xfffffffffffff000ULL;
		/* Align. */
		scratch->hart_address = (scratch->hart_address + alignment - 1) & ~(alignment - 1);

		if ((size_bytes + scratch->hart_address - info->dataaddr + 3) / 4 >=
				info->datasize) {
			scratch->memory_space = SPACE_DMI_DATA;
			scratch->debug_address = (scratch->hart_address - info->dataaddr) / 4;
			return ERROR_OK;
		}
	}

	/* Option 2: See if progbuf can be used as the scratch memory */
	if (examine_progbuf(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Allow for ebreak at the end of the program. */
	unsigned program_size = (program->instruction_count + 1) * 4;
	scratch->hart_address = (info->progbuf_address + program_size + alignment - 1) &
		~(alignment - 1);
	if ((info->progbuf_writable == YNM_YES) &&
			((size_bytes + scratch->hart_address - info->progbuf_address + 3) / 4 >=
			info->progbufsize)) {
		scratch->memory_space = SPACE_DMI_PROGBUF;
		scratch->debug_address = (scratch->hart_address - info->progbuf_address) / 4;
		return ERROR_OK;
	}

	/* Option 3: User-configured memory area as scratch RAM */
	if (target_alloc_working_area(target, size_bytes + alignment - 1,
				&scratch->area) == ERROR_OK) {
		scratch->hart_address = (scratch->area->address + alignment - 1) &
			~(alignment - 1);
		scratch->memory_space = SPACE_DMI_RAM;
		scratch->debug_address = scratch->hart_address;
		return ERROR_OK;
	}

	LOG_ERROR("Couldn't find %d bytes of scratch RAM to use. Please configure "
			"a work area with 'configure -work-area-phys'.", size_bytes);
	return ERROR_FAIL;
}

static int scratch_release(struct target *target,
		scratch_mem_t *scratch)
{
	if (scratch->area)
		return target_free_working_area(target, scratch->area);

	return ERROR_OK;
}

static int scratch_read64(struct target *target, scratch_mem_t *scratch,
		uint64_t *value)
{
	uint32_t v;
	switch (scratch->memory_space) {
		case SPACE_DMI_DATA:
			if (dmi_read(target, &v, DMI_DATA0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DMI_DATA1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			if (dmi_read(target, &v, DMI_PROGBUF0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DMI_PROGBUF1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8];
				if (read_memory(target, scratch->debug_address, 4, 2, buffer) != ERROR_OK)
					return ERROR_FAIL;
				*value = buffer[0] |
					(((uint64_t) buffer[1]) << 8) |
					(((uint64_t) buffer[2]) << 16) |
					(((uint64_t) buffer[3]) << 24) |
					(((uint64_t) buffer[4]) << 32) |
					(((uint64_t) buffer[5]) << 40) |
					(((uint64_t) buffer[6]) << 48) |
					(((uint64_t) buffer[7]) << 56);
			}
			break;
	}
	return ERROR_OK;
}

static int scratch_write64(struct target *target, scratch_mem_t *scratch,
		uint64_t value)
{
	switch (scratch->memory_space) {
		case SPACE_DMI_DATA:
			dmi_write(target, DMI_DATA0 + scratch->debug_address, value);
			dmi_write(target, DMI_DATA1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_PROGBUF:
			dmi_write(target, DMI_PROGBUF0 + scratch->debug_address, value);
			dmi_write(target, DMI_PROGBUF1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {
					value,
					value >> 8,
					value >> 16,
					value >> 24,
					value >> 32,
					value >> 40,
					value >> 48,
					value >> 56
				};
				if (write_memory(target, scratch->debug_address, 4, 2, buffer) != ERROR_OK)
					return ERROR_FAIL;
			}
			break;
	}
	return ERROR_OK;
}

/** Return register size in bits. */
static unsigned register_size(struct target *target, unsigned number)
{
	/* If reg_cache hasn't been initialized yet, make a guess. We need this for
	 * when this function is called during examine(). */
	if (target->reg_cache)
		return target->reg_cache->reg_list[number].size;
	else
		return riscv_xlen(target);
}

/**
 * Immediately write the new value to the requested register. This mechanism
 * bypasses any caches.
 */
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

#if _NDS_V5_ONLY_
	char in_text[500];
	in_text[0] = 0x0;
	ndsv5_decode_csr(in_text, number, value);
	NDS_INFO("[%s] hart[%d] reg[%s] <- 0x%" PRIx64 " %s", target->tap->dotted_name, riscv_current_hartid(target),
			gdb_regno_name(number), value, in_text);
#else /* _NDS_V5_ONLY_ */
	LOG_DEBUG("{%d} %s <- 0x%" PRIx64, riscv_current_hartid(target),
			gdb_regno_name(number), value);
#endif /* _NDS_V5_ONLY_ */

#if _NDS_USE_SCRIPT_
	if (ndsv5_script_reg_write(number, value) == ERROR_OK)
		return ERROR_OK;
#endif /* _NDS_USE_SCRIPT_ */

	int result = register_write_abstract(target, number, value,
			register_size(target, number));
	if (result == ERROR_OK || info->progbufsize + r->impebreak < 2 ||
			!riscv_is_halted(target))
		return result;

	struct riscv_program program;
	riscv_program_init(&program, target);

	uint64_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	scratch_mem_t scratch;
	bool use_scratch = false;
	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			riscv_supports_extension(target, riscv_current_hartid(target), 'D') &&
			riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register, so
		 * we need to use some scratch RAM. */
		use_scratch = true;
		riscv_program_insert(&program, fld(number - GDB_REGNO_FPR0, S0, 0));

		if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
			return ERROR_FAIL;

		if (register_write_direct(target, GDB_REGNO_S0, scratch.hart_address)
				!= ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

		if (scratch_write64(target, &scratch, value) != ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

	} else if (number == GDB_REGNO_VTYPE) {
		riscv_program_insert(&program, csrr(S0, CSR_VL));
		riscv_program_insert(&program, vsetvli(ZERO, S0, value));

	} else {
		if (register_write_direct(target, GDB_REGNO_S0, value) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D'))
				riscv_program_insert(&program, fmv_d_x(number - GDB_REGNO_FPR0, S0));
			else
				riscv_program_insert(&program, fmv_w_x(number - GDB_REGNO_FPR0, S0));
		} else if (number == GDB_REGNO_VL) {
			/* "The XLEN-bit-wide read-only vl CSR can only be updated by the
			 * vsetvli and vsetvl instructions, and the fault-only-rst vector
			 * load instruction variants." */
			riscv_reg_t vtype;
			if (register_read(target, &vtype, GDB_REGNO_VTYPE) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvli(ZERO, S0, vtype)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			riscv_program_csrw(&program, S0, number);
		} else {
			LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
#if _NDS_DISABLE_ABORT_
			NDS32_LOG("Unsupported register (enum gdb_regno)(%d)", number);
#endif
			return ERROR_FAIL;
		}
	}

	int exec_out = riscv_program_exec(&program, target);
	/* Don't message on error. Probably the register doesn't exist. */
	if (exec_out == ERROR_OK && target->reg_cache) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, value);
	}

	if (use_scratch)
		scratch_release(target, &scratch);

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore S0. */
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return exec_out;
}

/** Return the cached value, or read from the target if necessary. */
static int register_read(struct target *target, uint64_t *value, uint32_t number)
{
	if (number == GDB_REGNO_ZERO) {
		*value = 0;
		return ERROR_OK;
	}
	int result = register_read_direct(target, value, number);
	if (result != ERROR_OK)
		return ERROR_FAIL;
	if (target->reg_cache) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, *value);
	}
	return ERROR_OK;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

#if _NDS_USE_SCRIPT_
	if (ndsv5_script_reg_read(value, number) == ERROR_OK)
		return ERROR_OK;
#endif

	int result = register_read_abstract(target, value, number,
			register_size(target, number));

	if (result != ERROR_OK &&
			info->progbufsize + r->impebreak >= 2 &&
			number > GDB_REGNO_XPR31) {
		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		riscv_reg_t s0;
		if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;

		/* Write program to move data into s0. */

		uint64_t mstatus;
		if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')
					&& riscv_xlen(target) < 64) {
				/* There are no instructions to move all the bits from a
				 * register, so we need to use some scratch RAM. */
				riscv_program_insert(&program, fsd(number - GDB_REGNO_FPR0, S0,
							0));

				if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
					return ERROR_FAIL;
				use_scratch = true;

				if (register_write_direct(target, GDB_REGNO_S0,
							scratch.hart_address) != ERROR_OK) {
					scratch_release(target, &scratch);
					return ERROR_FAIL;
				}
			} else if (riscv_supports_extension(target,
						riscv_current_hartid(target), 'D')) {
				riscv_program_insert(&program, fmv_x_d(S0, number - GDB_REGNO_FPR0));
			} else {
				riscv_program_insert(&program, fmv_x_w(S0, number - GDB_REGNO_FPR0));
			}
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
#if _NDS_V5_ONLY_
			if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
				return ERROR_FAIL;
			if ((mstatus & MSTATUS_VS) == 0) {
				if (register_write_direct(target, GDB_REGNO_MSTATUS,
					set_field(mstatus, MSTATUS_VS, 1)) != ERROR_OK)
					return ERROR_FAIL;
			}
#endif
			riscv_program_csrr(&program, S0, number);
		} else {
			LOG_ERROR("Unsupported register: %s", gdb_regno_name(number));
			return ERROR_FAIL;
		}

		/* Execute program. */
		result = riscv_program_exec(&program, target);
		/* Don't message on error. Probably the register doesn't exist. */

		if (use_scratch) {
			result = scratch_read64(target, &scratch, value);
			scratch_release(target, &scratch);
			if (result != ERROR_OK)
				return result;
		} else {
			/* Read S0 */
			if (register_read_direct(target, value, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
			return ERROR_FAIL;

		/* Restore S0. */
		if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (result == ERROR_OK) {
#if _NDS_V5_ONLY_
		char in_text[500];
		in_text[0] = 0x0;
		ndsv5_decode_csr(in_text, number, *value);
		NDS_INFO("[%s] hart[%d] reg[%s] = 0x%" PRIx64 " %s", target->tap->dotted_name, riscv_current_hartid(target),
				gdb_regno_name(number), *value, in_text);
#else /* _NDS_V5_ONLY_ */
		LOG_DEBUG("{%d} %s = 0x%" PRIx64, riscv_current_hartid(target),
				gdb_regno_name(number), *value);
#endif /* _NDS_V5_ONLY_ */
	}

	return result;
}

#if _NDS_V5_ONLY_
int nds_register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	return register_read_direct(target, value, number);
}

int nds_register_write_direct(struct target *target, unsigned number, uint64_t value)
{
	return register_write_direct(target, number, value);
}
#endif

int wait_for_authbusy(struct target *target, uint32_t *dmstatus)
{
	time_t start = time(NULL);
	while (1) {
		uint32_t value;
		if (dmstatus_read(target, &value, false) != ERROR_OK)
			return ERROR_FAIL;
		if (dmstatus)
			*dmstatus = value;
		if (!get_field(value, DMI_DMSTATUS_AUTHBUSY))
			break;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for authbusy to go low (dmstatus=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					value);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/*** OpenOCD target functions. ***/

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (target_was_examined(target)) {
		if (nds32->attached) {
			LOG_DEBUG("deinit_target(): gdb_detach process, resume dcsr");
			if (target->state != TARGET_HALTED)
				target_halt(target);

			/* clear all breakpoints & watchpoints */
			breakpoint_clear_target(nds32->target);
			watchpoint_clear_target(nds32->target);

			nds32->gdb_run_mode = RUN_MODE_DEBUG;

			/* Set attached to false before resume */
			nds32->attached = false;
		}

		/* free run in debug mode */
		LOG_DEBUG("Free run all target");
		target_resume(target, 1, 0, 0, 0);
	} else
		LOG_DEBUG("target was not examined, skip resume");
#endif


	free(info->version_specific);
	/* TODO: free register arch_info */
	info->version_specific = NULL;
}

static int set_haltgroup(struct target *target, bool *supported)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	int user_def_hart_count = (int)target->corenums;
	if (user_def_hart_count == 0)
		user_def_hart_count = RISCV_MAX_HARTS;
	LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
	uint32_t dmstatus, dmcontrol;
	int i;
	uint32_t control = 0;

	/* reset the Debug Module */
	dmi_write(target, DMI_DMCONTROL, 0);
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	dmi_read(target, &dmcontrol, DMI_DMCONTROL);

	/* check existence harts */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		dmi_read(target, &dmstatus, DMI_DMSTATUS);
		if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
			user_def_hart_count = i;
			LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
			break;
		}
	}

	/* Assert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = set_field(0, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		/* SETRESETHALTREQ can halt on 0x80000000 for AMP/SMP all harts */
		control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
		control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);

		/* Assert ndmreset */
		control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);

		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &dmcontrol, DMI_DMCONTROL);
	}
	alive_sleep(nds32->reset_time);
	dmi_read(target, &dmstatus, DMI_DMSTATUS);

	/* Deassert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		time_t start = time(NULL);
		control = 0;

		/* Clear the reset, but make sure haltreq is still set */
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
		control = set_field(control, DMI_DMCONTROL_CLRRESETHALTREQ, 1);
		control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		do {
			dmi_read(target, &dmstatus, DMI_DMSTATUS);
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart didn't halt coming out of reset in %ds; "
				"dmstatus=0x%x; "
				"Increase the timeout with riscv set_reset_timeout_sec.",
				riscv_reset_timeout_sec, dmstatus);

				return ERROR_FAIL;
			}
		} while (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0);

		control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &dmcontrol, DMI_DMCONTROL);
		dmi_read(target, &dmstatus, DMI_DMSTATUS);
	}
	reset_halt = true;
	return ERROR_OK;
}

static int discover_vlenb(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_reg_t vlenb;

	if (register_read(target, &vlenb, GDB_REGNO_VLENB) != ERROR_OK)
		return ERROR_FAIL;
	r->vlenb[hartid] = vlenb;

	LOG_INFO("hart %d: Vector support with vlenb=%d", hartid, r->vlenb[hartid]);

	return ERROR_OK;
}

static int examine(struct target *target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG("  dmireset=%d", get_field(dtmcontrol, DTM_DTMCS_DMIRESET));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTM_DTMCS_IDLE));
	LOG_DEBUG("  dmistat=%d", get_field(dtmcontrol, DTM_DTMCS_DMISTAT));
	LOG_DEBUG("  abits=%d", get_field(dtmcontrol, DTM_DTMCS_ABITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTM_DTMCS_VERSION));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTM_DTMCS_VERSION) != 1) {
		LOG_ERROR("Unsupported DTM version %d. (dtmcontrol=0x%x)",
				get_field(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	/* TODO: This won't be true if there are multiple DMs. */
	info->index = target->coreid;
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcs_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	/* Reset the Debug Module. */
	dm013_info_t *dm = get_dm(target);
	if (!dm->was_reset) {
		dmi_write(target, DMI_DMCONTROL, 0);
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
		dm->was_reset = true;
	}

#if _NDS_V5_ONLY_
	/* -H: reset as init */
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (!target_was_examined(target)) {
		if (nds32->reset_halt_as_examine) {
			if (ndsv5_script_custom_reset_halt)
				ndsv5_script_do_custom_reset(target, ndsv5_script_custom_reset_halt);
			else if (!reset_halt)
				ndsv5_reset_halt_as_examine(target);
			NDS32_LOG(NDS32_MSG_HW_RESET_HOLD);
		}
	}

	/* Forced reset dmi_busy_dealy to v5_dmi_busy_dealy_count */
	info->dmi_busy_delay = v5_dmi_busy_delay_count;
	LOG_DEBUG("info->dmi_busy_delay: 0x%08x", info->dmi_busy_delay);
#endif /* _NDS_V5_ONLY_ */


	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_HARTSELLO |
			DMI_DMCONTROL_HARTSELHI | DMI_DMCONTROL_DMACTIVE |
			DMI_DMCONTROL_HASEL);
	uint32_t dmcontrol;
	if (dmi_read(target, &dmcontrol, DMI_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	dm->hasel_supported = get_field(dmcontrol, DMI_DMCONTROL_HASEL);

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);
	int dmstatus_version = get_field(dmstatus, DMI_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		/* Error was already printed out in dmstatus_read(). */
		return ERROR_FAIL;
	}

	uint32_t hartsel =
		(get_field(dmcontrol, DMI_DMCONTROL_HARTSELHI) <<
		 DMI_DMCONTROL_HARTSELLO_LENGTH) |
		get_field(dmcontrol, DMI_DMCONTROL_HARTSELLO);
	info->hartsellen = 0;
	while (hartsel & 1) {
		info->hartsellen++;
		hartsel >>= 1;
	}
	LOG_DEBUG("hartsellen=%d", info->hartsellen);

	uint32_t hartinfo;
	if (dmi_read(target, &hartinfo, DMI_HARTINFO) != ERROR_OK)
		return ERROR_FAIL;

	info->datasize = get_field(hartinfo, DMI_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DMI_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DMI_HARTINFO_DATAADDR);

	if (!get_field(dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", dmstatus);
		/* If we return ERROR_FAIL here, then in a multicore setup the next
		 * core won't be examined, which means we won't set up the
		 * authentication commands for them, which means the config script
		 * needs to be a lot more complex. */
		return ERROR_OK;
	}

	if (dmi_read(target, &info->sbcs, DMI_SBCS) != ERROR_OK)
		return ERROR_FAIL;

#if _NDS_V5_ONLY_
	ndsv5_dis_cache_busmode = 1;
	if (info->sbcs & 0x1F) {
		int sb_version = get_field(info->sbcs, DMI_SBCS_SBVERSION);
		if ((sb_version == 0) || (sb_version == 1)) {
			/* According to e-16199, default no support system bus access,
			   so ndsv5_system_bus_access default value is 0 */
			if (ndsv5_system_bus_access == 1) {
				nds_sys_bus_supported = 1;
				ndsv5_dis_cache_busmode = 0;
			}
		}
	}
	LOG_DEBUG("info->sbcs = 0x%x, nds_sys_bus_supported = 0x%x", (int)info->sbcs, nds_sys_bus_supported);
	LOG_DEBUG("nds_jtag_max_scans: %d", nds_jtag_max_scans);
#endif

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs;
	if (dmi_read(target, &abstractcs, DMI_ABSTRACTCS) != ERROR_OK)
		return ERROR_FAIL;
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE);

	LOG_INFO("datacount=%d progbufsize=%d", info->datacount, info->progbufsize);

	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DMI_DMSTATUS_IMPEBREAK);

	if (info->progbufsize + r->impebreak < 2) {
		LOG_WARNING("We won't be able to execute fence instructions on this "
				"target. Memory may not always appear consistent. "
				"(progbufsize=%d, impebreak=%d)", info->progbufsize,
				r->impebreak);
	}

	if (info->progbufsize < 4 && riscv_enable_virtual) {
		LOG_ERROR("set_enable_virtual is not available on this target. It "
				"requires a program buffer size of at least 4. (progbufsize=%d) "
				"Use `riscv set_enable_virtual off` to continue."
					, info->progbufsize);
	}

	/* Before doing anything else we must first enumerate the harts. */
	if (dm->hart_count < 0) {
		for (int i = 0; i < MIN(RISCV_MAX_HARTS, 1 << info->hartsellen); ++i) {
			r->current_hartid = i;
			if (riscv013_select_current_hart(target) != ERROR_OK)
				return ERROR_FAIL;

			uint32_t s;
			if (dmstatus_read(target, &s, true) != ERROR_OK)
				return ERROR_FAIL;
			if (get_field(s, DMI_DMSTATUS_ANYNONEXISTENT))
				break;
			dm->hart_count = i + 1;

			if (get_field(s, DMI_DMSTATUS_ANYHAVERESET))
				dmi_write(target, DMI_DMCONTROL,
						set_hartsel(DMI_DMCONTROL_DMACTIVE | DMI_DMCONTROL_ACKHAVERESET, i));
		}

		LOG_DEBUG("Detected %d harts.", dm->hart_count);
	}

	if (dm->hart_count == 0) {
		LOG_ERROR("No harts found!");
		return ERROR_FAIL;
	}

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */
	for (int i = 0; i < dm->hart_count; ++i) {
		if (!riscv_rtos_enabled(target) && i != target->coreid)
			continue;

		r->current_hartid = i;
		if (riscv013_select_current_hart(target) != ERROR_OK)
			return ERROR_FAIL;

#if _NDS_V5_ONLY_
		uint32_t s;
		dmi_read(target, &s, DMI_DMSTATUS);
		if(get_field(s, DMI_DMSTATUS_ANYHAVERESET)) {
			uint32_t control;
			dmi_read(target, &control, DMI_DMCONTROL);
			control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
			dmi_write(target, DMI_DMCONTROL, control);
		}

		if (!riscv_is_halted(target)) {
			if (riscv013_halt_go(target) != ERROR_OK) {
				if (riscv_rtos_enabled(target)) {
					if (i != 0) {
						LOG_DEBUG("riscv_rtos_enabled, riscv013_halt_current_hart %d FAIL, but return OK", i);
						return ERROR_OK;
					}
				}
				r->debug_buffer_size[i] = info->progbufsize;
				r->xlen[i] = 0;
				LOG_DEBUG("riscv013_halt_current_hart FAIL return");
				return ERROR_FAIL;
			}
		}

		/* Enable halt-on-reset */
		if ( nds_halt_on_reset == 1 ) {
			if(get_field(s, DMI_DMSTATUS_HASRESETHALTREQ)) {
				uint32_t control;
				dmi_read(target, &control, DMI_DMCONTROL);
				control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
				dmi_write(target, DMI_DMCONTROL, control);
				LOG_DEBUG("hart [%s] %d: halt-on-reset is on!", target->tap->dotted_name, i);
			} else {
				LOG_ERROR("The Debug Module doesn't supports halt-on-reset functionality!!");
			}
		}
#else /* _NDS_V5_ONLY_ */
		bool halted = riscv_is_halted(target);
		if (!halted) {
			if (riscv013_halt_go(target) != ERROR_OK) {
				LOG_ERROR("Fatal: Hart %d failed to halt during examine()", i);
				return ERROR_FAIL;
			}
		}
#endif /* _NDS_V5_ONLY_ */

		/* Without knowing anything else we can at least mess with the
		 * program buffer. */
		r->debug_buffer_size[i] = info->progbufsize;

		int result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
		if (result == ERROR_OK)
			r->xlen[i] = 64;
		else {
#if _NDS_V5_ONLY_
			riscv013_clear_abstract_error(target);
#endif /* _NDS_V5_ONLY_ */
			r->xlen[i] = 32;
		}

		if (register_read(target, &r->misa[i], GDB_REGNO_MISA)) {
			LOG_ERROR("Fatal: Failed to read MISA from hart %d.", i);
			return ERROR_FAIL;
		}

#if _NDS_V5_ONLY_
#else /* _NDS_V5_ONLY_ */
		if (riscv_supports_extension(target, i, 'V')) {
			if (discover_vlenb(target, i) != ERROR_OK)
				return ERROR_FAIL;
		}
#endif /* _NDS_V5_ONLY_ */

		/* Now init registers based on what we discovered. */
		if (riscv_init_registers(target) != ERROR_OK)
			return ERROR_FAIL;

		/* Display this as early as possible to help people who are using
		 * really slow simulators. */
#if _NDS_V5_ONLY_
		LOG_DEBUG(" [%s] hart %d: XLEN=%d, misa=0x%" PRIx64, target->tap->dotted_name, i, r->xlen[i],
				r->misa[i]);
#else /* _NDS_V5_ONLY_ */
		LOG_DEBUG(" hart %d: XLEN=%d, misa=0x%" PRIx64, i, r->xlen[i],
				r->misa[i]);
#endif /* _NDS_V5_ONLY_ */

#if _NDS_V5_ONLY_
		/* Init V */
		if (riscv_supports_extension(target, i, 'V'))
			ndsv5_get_vector_VLMAX(target);

		/* Check if rv32e for target burn*/
		rv32e = false;
		if (riscv_supports_extension(target, i, 'E')) {
			rv32e = true;
			LOG_DEBUG("target is rv32e");
		}

		if(nds32->reset_halt_as_examine) {
			LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
			target->debug_reason = DBG_REASON_DBGRQ;
			target->state = TARGET_HALTED;
			LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
		} else {
			riscv013_step_or_resume_current_hart(target, false, false);
			target->state = TARGET_RUNNING;
		}
#else /* _NDS_V5_ONLY_ */
		if (!halted)
			riscv013_step_or_resume_current_hart(target, false, false);
#endif /* _NDS_V5_ONLY_ */
	}

	target_set_examined(target);

	if (target->smp) {
		bool haltgroup_supported;
		if (set_haltgroup(target, &haltgroup_supported) != ERROR_OK)
			return ERROR_FAIL;
		if (haltgroup_supported)
			LOG_INFO("Core %d made part of halt group %d.", target->coreid,
					target->smp);
		else
			LOG_INFO("Core %d could not be made part of halt group %d.",
					target->coreid, target->smp);
	}

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_INFO("Examined RISC-V core; found %d harts",
			riscv_count_harts(target));
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (riscv_hart_enabled(target, i)) {
			LOG_INFO(" hart %d: XLEN=%d, misa=0x%" PRIx64, i, r->xlen[i],
					r->misa[i]);
		} else {
			LOG_INFO(" hart %d: currently disabled", i);
		}
	}
	return ERROR_OK;
}

int riscv013_authdata_read(struct target *target, uint32_t *value)
{
	if (wait_for_authbusy(target, NULL) != ERROR_OK)
		return ERROR_FAIL;

	return dmi_read(target, value, DMI_AUTHDATA);
}

int riscv013_authdata_write(struct target *target, uint32_t value)
{
	uint32_t before, after;
	if (wait_for_authbusy(target, &before) != ERROR_OK)
		return ERROR_FAIL;

	dmi_write(target, DMI_AUTHDATA, value);

	if (wait_for_authbusy(target, &after) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(before, DMI_DMSTATUS_AUTHENTICATED) &&
			get_field(after, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_INFO("authdata_write resulted in successful authentication");
		int result = ERROR_OK;
		dm013_info_t *dm = get_dm(target);
		target_list_t *entry;
		list_for_each_entry(entry, &dm->target_list, list) {
			if (examine(entry->target) != ERROR_OK)
				result = ERROR_FAIL;
		}
		return result;
	}

	return ERROR_OK;
}

static int riscv013_hart_count(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	return dm->hart_count;
}

static unsigned riscv013_data_bits(struct target *target)
{
	RISCV013_INFO(info);
	/* TODO: Once there is a spec for discovering abstract commands, we can
	 * take those into account as well.  For now we assume abstract commands
	 * support XLEN-wide accesses. */
	if (info->progbufsize >= 2 && !riscv_prefer_sba)
		return riscv_xlen(target);

	if (get_field(info->sbcs, DMI_SBCS_SBACCESS128))
		return 128;
	if (get_field(info->sbcs, DMI_SBCS_SBACCESS64))
		return 64;
	if (get_field(info->sbcs, DMI_SBCS_SBACCESS32))
		return 32;
	if (get_field(info->sbcs, DMI_SBCS_SBACCESS16))
		return 16;
	if (get_field(info->sbcs, DMI_SBCS_SBACCESS8))
		return 8;

	return riscv_xlen(target);
}

static int prep_for_vector_access(struct target *target, uint64_t *vtype,
		uint64_t *vl, unsigned *debug_vl)
{
	RISCV_INFO(r);
	/* TODO: this continuous save/restore is terrible for performance. */
	/* Write vtype and vl. */
	unsigned encoded_vsew;
	switch (riscv_xlen(target)) {
		case 32:
			encoded_vsew = 2;
			break;
		case 64:
			encoded_vsew = 3;
			break;
		default:
			LOG_ERROR("Unsupported xlen: %d", riscv_xlen(target));
			return ERROR_FAIL;
	}

	/* Save vtype and vl. */
	if (register_read(target, vtype, GDB_REGNO_VTYPE) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, vl, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;

	if (register_write_direct(target, GDB_REGNO_VTYPE, encoded_vsew << 2) != ERROR_OK)
		return ERROR_FAIL;
	*debug_vl = DIV_ROUND_UP(r->vlenb[r->current_hartid] * 8,
			riscv_xlen(target));
	if (register_write_direct(target, GDB_REGNO_VL, *debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cleanup_after_vector_access(struct target *target, uint64_t vtype,
		uint64_t vl)
{
	/* Restore vtype and vl. */
	if (register_write_direct(target, GDB_REGNO_VTYPE, vtype) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_VL, vl) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int riscv013_get_register_buf(struct target *target,
		uint8_t *value, int regno)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	riscv_reg_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vmv_x_s(S0, vnum));
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));

	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		/* Executing the program might result in an exception if there is some
		 * issue with the vector implementation/instructions we're using. If that
		 * happens, attempt to restore as usual. We may have clobbered the
		 * vector register we tried to read already.
		 * For other failures, we just return error because things are probably
		 * so messed up that attempting to restore isn't going to help. */
		result = riscv_program_exec(&program, target);
		if (result == ERROR_OK) {
			uint64_t v;
			if (register_read_direct(target, &v, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u64(value, xlen * i, xlen, v);
		} else {
			break;
		}
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int riscv013_set_register_buf(struct target *target,
		int regno, const uint8_t *value)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	riscv_reg_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));
	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		if (register_write_direct(target, GDB_REGNO_S0,
					buf_get_u64(value, xlen * i, xlen)) != ERROR_OK)
			return ERROR_FAIL;
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			break;
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;

#if _NDS_V5_ONLY_
	LOG_DEBUG("ACE init");
	if (global_acr_reg_count_v5 != 0) {
		acr_reg_count_v5 = *global_acr_reg_count_v5;
		LOG_DEBUG("*global_acr_reg_count_v5 = %d", *global_acr_reg_count_v5);
	}
	if (global_acr_type_count_v5 != 0) {
		acr_type_count_v5 = *global_acr_type_count_v5;
		LOG_DEBUG("*global_acr_type_count_v5 = %d", *global_acr_type_count_v5);
	}
	LOG_DEBUG("acr_reg_count_v5 = %d", acr_reg_count_v5);
	LOG_DEBUG("acr_type_count_v5 = %d", acr_type_count_v5);
#endif

	generic_info->get_register = &riscv013_get_register;
	generic_info->set_register = &riscv013_set_register;
	generic_info->get_register_buf = &riscv013_get_register_buf;
	generic_info->set_register_buf = &riscv013_set_register_buf;
	generic_info->select_current_hart = &riscv013_select_current_hart;
	generic_info->is_halted = &riscv013_is_halted;
	generic_info->resume_go = &riscv013_resume_go;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->on_halt = &riscv013_on_halt;
	generic_info->resume_prep = &riscv013_resume_prep;
	generic_info->halt_prep = &riscv013_halt_prep;
	generic_info->halt_go = &riscv013_halt_go;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->read_debug_buffer = &riscv013_read_debug_buffer;
	generic_info->write_debug_buffer = &riscv013_write_debug_buffer;
	generic_info->execute_debug_buffer = &riscv013_execute_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->authdata_read = &riscv013_authdata_read;
	generic_info->authdata_write = &riscv013_authdata_write;
	generic_info->dmi_read = &dmi_read;
	generic_info->dmi_write = &dmi_write;
	generic_info->test_sba_config_reg = &riscv013_test_sba_config_reg;
	generic_info->test_compliance = &riscv013_test_compliance;
	generic_info->hart_count = &riscv013_hart_count;
	generic_info->data_bits = &riscv013_data_bits;
	generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;
	riscv013_info_t *info = get_info(target);

	info->progbufsize = -1;

#if _NDS_V5_ONLY_
	info->dmi_busy_delay = v5_dmi_busy_delay_count;
#else /* _NDS_V5_ONLY_ */
	info->dmi_busy_delay = 0;
#endif /* _NDS_V5_ONLY_ */
	info->bus_master_read_delay = 0;
	info->bus_master_write_delay = 0;
	info->ac_busy_delay = 0;

	/* Assume all these abstract commands are supported until we learn
	 * otherwise.
	 * TODO: The spec allows eg. one CSR to be able to be accessed abstractly
	 * while another one isn't. We don't track that this closely here, but in
	 * the future we probably should. */
	info->abstract_read_csr_supported = true;
	info->abstract_write_csr_supported = true;
	info->abstract_read_fpr_supported = true;
	info->abstract_write_fpr_supported = true;

	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
#if _NDS_V5_ONLY_
	/* Disable halt-on-reset when reset-run */
	if ((nds_halt_on_reset == 1) && (target->reset_halt == 0))
		ndsv5_haltonreset(target, 0);

	if ((ndsv5_script_custom_reset_halt) && (target->reset_halt == 1)) {
		ndsv5_script_do_custom_reset(target, ndsv5_script_custom_reset_halt);
		return ERROR_OK;
	} else if ((ndsv5_script_custom_reset) && (target->reset_halt == 0)) {
		ndsv5_script_do_custom_reset(target, ndsv5_script_custom_reset);
		return ERROR_OK;
	}
#endif /* _NDS_V5_ONLY_ */

	RISCV_INFO(r);

	select_dmi(target);

	uint32_t control_base = set_field(0, DMI_DMCONTROL_DMACTIVE, 1);

#if _NDS_V5_ONLY_
	if (target->reset_halt) {
		/* single core no execute haltonreset: bitmap built before 2019/5 */
		ndsv5_haltonreset(target, 1);
	} else
		ndsv5_haltonreset(target, 0);
#endif /* _NDS_V5_ONLY_ */

	if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		 * We must reset them all. */

		/* TODO: Try to use hasel in dmcontrol */

		/* Set haltreq for each hart. */
		uint32_t control = control_base;
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			if (!riscv_hart_enabled(target, i))
				continue;

			control = set_hartsel(control_base, i);
			control = set_field(control, DMI_DMCONTROL_HALTREQ,
					target->reset_halt ? 1 : 0);
			dmi_write(target, DMI_DMCONTROL, control);
		}
		/* Assert ndmreset */
		control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DMI_DMCONTROL, control);

	} else {
		/* Reset just this hart. */
		uint32_t control = set_hartsel(control_base, r->current_hartid);
		control = set_field(control, DMI_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DMI_DMCONTROL, control);
	}

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	alive_sleep(nds32->reset_time);
#endif
	target->state = TARGET_RESET;

	dm013_info_t *dm = get_dm(target);

	/* The DM might have gotten reset if OpenOCD called us in some reset that
	 * involves SRST being toggled. So clear our cache which may be out of
	 * date. */
	memset(dm->progbuf_cache, 0, sizeof(dm->progbuf_cache));

	return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
	RISCV_INFO(r);
	RISCV013_INFO(info);
	select_dmi(target);

#if _NDS_V5_ONLY_
	isAceCsrEnable = false;
#endif


	/* Clear the reset, but make sure haltreq is still set */
	uint32_t control = 0;
	control = set_field(control, DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
	control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
#if _NDS_V5_ONLY_
	control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
#endif
	dmi_write(target, DMI_DMCONTROL,
			set_hartsel(control, r->current_hartid));

	uint32_t dmstatus;
	int dmi_busy_delay = info->dmi_busy_delay;
#if _NDS_V5_ONLY_
#else
	time_t start = time(NULL);
#endif

	for (int i = 0; i < riscv_count_harts(target); ++i) {
#if _NDS_V5_ONLY_
		time_t start = time(NULL);
#endif
		int index = i;
		if (target->rtos) {
			if (!riscv_hart_enabled(target, index))
				continue;
			dmi_write(target, DMI_DMCONTROL,
					set_hartsel(control, index));
		} else {
			index = r->current_hartid;
		}

		char *operation;
		uint32_t expected_field;
		if (target->reset_halt) {
			operation = "halt";
			expected_field = DMI_DMSTATUS_ALLHALTED;
		} else {
			operation = "run";
			expected_field = DMI_DMSTATUS_ALLRUNNING;
		}
		LOG_DEBUG("Waiting for hart %d to %s out of reset.", index, operation);
		while (1) {
			int result = dmstatus_read_timeout(target, &dmstatus, true,
					riscv_reset_timeout_sec);
			if (result == ERROR_TIMEOUT_REACHED)
				LOG_ERROR("Hart %d didn't complete a DMI read coming out of "
						"reset in %ds; Increase the timeout with riscv "
						"set_reset_timeout_sec.",
						index, riscv_reset_timeout_sec);
			if (result != ERROR_OK)
				return result;
			if (get_field(dmstatus, expected_field))
				break;
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart %d didn't %s coming out of reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						index, operation, riscv_reset_timeout_sec, dmstatus);

#if _NDS_V5_ONLY_
				/* Release haltreq when failed to halt */
				control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
				dmi_write(target, DMI_DMCONTROL, control);
#endif

				return ERROR_FAIL;
			}
		}

#if _NDS_V5_ONLY_
		if (target->reset_halt) {
			target->state = TARGET_HALTED;
			control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
			dmi_write(target, DMI_DMCONTROL,
					set_hartsel(control, index));
			NDS32_LOG(NDS32_MSG_HW_RESET_HOLD_ID, target->tap->dotted_name, index);
		} else {
			/* Halt again */
			struct nds32_v5 *nds32 = target_to_nds32_v5(target);
			alive_sleep(nds32->boot_time);
			riscv013_halt_go(target);
		}
#else /* _NDS_V5_ONLY_ */
		target->state = TARGET_HALTED;
#endif /* _NDS_V5_ONLY_ */

		if (get_field(dmstatus, DMI_DMSTATUS_ALLHAVERESET)) {
			/* Ack reset. */
			dmi_write(target, DMI_DMCONTROL,
					set_hartsel(control, index) |
					DMI_DMCONTROL_ACKHAVERESET);
		}

		if (!target->rtos)
			break;
	}

#if _NDS_V5_ONLY_
	/* Restore halt-on-reset */
	if(nds_halt_on_reset == 1 && target->rtos) {
		/* single core no execute haltonreset: bitmap built before 2019/5 */
		ndsv5_haltonreset(target, 1);
	} else
		ndsv5_haltonreset(target, 0);

	uint64_t dpc;
	riscv_get_register(target, &dpc, GDB_REGNO_PC);
	NDS_INFO("[%s] hart[%d] halt at 0x%" PRIx64, target->tap->dotted_name,
			riscv_current_hartid(target), dpc);
#endif /* _NDS_V5_ONLY_ */

	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
}

/**
 * @par size in bytes
 */
static uint64_t read_from_buf(const uint8_t *buffer, unsigned size)
{
	switch (size) {
		case 1:
			return buffer[0];
		case 2:
			return buffer[0]
				| ((uint64_t) buffer[1] << 8);
		case 4:
			return buffer[0]
				| ((uint64_t) buffer[1] << 8)
				| ((uint64_t) buffer[2] << 16)
				| ((uint64_t) buffer[3] << 24);
		case 8:
			return buffer[0]
				| ((uint64_t) buffer[1] << 8)
				| ((uint64_t) buffer[2] << 16)
				| ((uint64_t) buffer[3] << 24)
				| ((uint64_t) buffer[4] << 32)
				| ((uint64_t) buffer[5] << 40)
				| ((uint64_t) buffer[6] << 48)
				| ((uint64_t) buffer[7] << 56);
		default:
			assert(false);
	}
	return -1;
}

/**
 * @par size in bytes
 */
static void write_to_buf(uint8_t *buffer, uint64_t value, unsigned size)
{
	switch (size) {
		case 8:
			buffer[7] = value >> 56;
			buffer[6] = value >> 48;
			buffer[5] = value >> 40;
			buffer[4] = value >> 32;
			/* falls through */
		case 4:
			buffer[3] = value >> 24;
			buffer[2] = value >> 16;
			/* falls through */
		case 2:
			buffer[1] = value >> 8;
			/* falls through */
		case 1:
			buffer[0] = value;
			break;
		default:
			assert(false);
	}
}

#if _NDS_V5_ONLY_
static int execute_fence_i(struct target *target)
{
	LOG_DEBUG("EXECUTE FENCE.I on [%s]", target->tap->dotted_name);
	int old_hartid = riscv_current_hartid(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->nds_do_fencei == false) {
		LOG_DEBUG("Skip to execute pre-fence.i on hart[%d]", old_hartid);
		return ERROR_OK;
	}



	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK)
		LOG_ERROR("Unable to execute pre-fence.i on hart[%d]", old_hartid);
	LOG_DEBUG("Execute fence.i on hart[%d]", old_hartid);


	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		if (i == old_hartid)
			/* Fence already executed for this hart */
			continue;

		riscv_set_current_hartid(target, i);
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		if (riscv_program_exec(&program, target) != ERROR_OK)
			LOG_ERROR("Unable to execute fence.i on hart[%d]", i);

		LOG_DEBUG("Execute fence.i on hart[%d]", i);
	}

	riscv_set_current_hartid(target, old_hartid);
	LOG_DEBUG("EXECUTE FENCE.I(DONE)");
	nds32->nds_do_fencei = false;

	return ERROR_OK;
}
#endif /* _NDS_V5_ONLY_ */


static int execute_fence(struct target *target)
{
#if _NDS_V5_ONLY_
	LOG_DEBUG("EXECUTE FENCE!");
#else
	LOG_ERROR("EXECUTE FENCE!");
#endif /* _NDS_V5_ONLY_ */

	int old_hartid = riscv_current_hartid(target);

	/* FIXME: For non-coherent systems we need to flush the caches right
	 * here, but there's no ISA-defined way of doing that. */
	{
		struct riscv_program program;
		riscv_program_init(&program, target);
#if _NDS_V5_ONLY_
#else
		riscv_program_fence_i(&program);
#endif /* _NDS_V5_ONLY_ */
		riscv_program_fence(&program);
		int result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			LOG_DEBUG("Unable to execute pre-fence");
	}

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		if (i == old_hartid)
			/* Fence already executed for this hart */
			continue;

		riscv_set_current_hartid(target, i);

		struct riscv_program program;
		riscv_program_init(&program, target);
#if _NDS_V5_ONLY_
#else
		riscv_program_fence_i(&program);
#endif /* _NDS_V5_ONLY_ */
		riscv_program_fence(&program);
		int result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			LOG_DEBUG("Unable to execute fence on hart %d", i);
	}

	riscv_set_current_hartid(target, old_hartid);

#if _NDS_V5_ONLY_
	LOG_DEBUG("EXECUTE FENCE(DONE)!");
#else
	LOG_ERROR("EXECUTE FENCE(DONE)!");
#endif /* _NDS_V5_ONLY_ */

	return ERROR_OK;
}

static void log_memory_access(target_addr_t address, uint64_t value,
		unsigned size_bytes, bool read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%0%d" PRIx64,
			address, read ? "read" : "write", size_bytes * 2);
	switch (size_bytes) {
		case 1:
			value &= 0xff;
			break;
		case 2:
			value &= 0xffff;
			break;
		case 4:
			value &= 0xffffffff;
			break;
		case 8:
			break;
		default:
			assert(false);
	}
	LOG_DEBUG(fmt, value);
}

/* Read the relevant sbdata regs depending on size, and put the results into
 * buffer. */
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint32_t value;
	int result;
	static int sbdata[4] = { DMI_SBDATA0, DMI_SBDATA1, DMI_SBDATA2, DMI_SBDATA3 };
	assert(size <= 16);
	for (int i = (size-1) / 4; i >= 0; i--) {
		result = dmi_op(target, &value, NULL, DMI_OP_READ, sbdata[i], 0, false, true);
		if (result != ERROR_OK)
			return result;
		write_to_buf(buffer + i * 4, value, MIN(size, 4));
		log_memory_access(address + i * 4, value, MIN(size, 4), true);
	}
	return ERROR_OK;
}

static uint32_t sb_sbaccess(unsigned size_bytes)
{
	switch (size_bytes) {
		case 1:
			return set_field(0, DMI_SBCS_SBACCESS, 0);
		case 2:
			return set_field(0, DMI_SBCS_SBACCESS, 1);
		case 4:
			return set_field(0, DMI_SBCS_SBACCESS, 2);
		case 8:
			return set_field(0, DMI_SBCS_SBACCESS, 3);
		case 16:
			return set_field(0, DMI_SBCS_SBACCESS, 4);
	}
	assert(0);
	return 0;	/* Make mingw happy. */
}

static target_addr_t sb_read_address(struct target *target)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DMI_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
#if BUILD_TARGET64
		dmi_read(target, &v, DMI_SBADDRESS1);
		address |= v;
		address <<= 32;
#endif
	}
	dmi_read(target, &v, DMI_SBADDRESS0);
	address |= v;
	return address;
}

static int sb_write_address(struct target *target, target_addr_t address)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DMI_SBCS_SBASIZE);
	/* There currently is no support for >64-bit addresses in OpenOCD. */
	if (sbasize > 96)
		dmi_write(target, DMI_SBADDRESS3, 0);
	if (sbasize > 64)
		dmi_write(target, DMI_SBADDRESS2, 0);
	if (sbasize > 32)
#if BUILD_TARGET64
		dmi_write(target, DMI_SBADDRESS1, address >> 32);
#else
		dmi_write(target, DMI_SBADDRESS1, 0);
#endif
	return dmi_write(target, DMI_SBADDRESS0, address);
}

static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs)
{
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, sbcs, DMI_SBCS) != ERROR_OK)
			return ERROR_FAIL;
		if (!get_field(*sbcs, DMI_SBCS_SBBUSY))
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
#if _NDS_V5_ONLY_
			LOG_ERROR("Timed out after %ds waiting for \"sbbusy\" to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec, *sbcs);
#else
			LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec, *sbcs);
#endif /* _NDS_V5_ONLY_  */
			return ERROR_FAIL;
		}
	}
}

static int modify_privilege(struct target *target, uint64_t *mstatus, uint64_t *mstatus_old)
{
	RISCV013_INFO(info);

	if (riscv_enable_virtual && info->progbufsize >= 4) {
		/* Read DCSR */
		uint64_t dcsr;
		if (register_read(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;

		/* Read and save MSTATUS */
		if (register_read(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		*mstatus_old = *mstatus;

		/* If we come from m-mode with mprv set, we want to keep mpp */
		if (get_field(dcsr, DCSR_PRV) < 3) {
			/* MPP = PRIV */
			*mstatus = set_field(*mstatus, MSTATUS_MPP, get_field(dcsr, DCSR_PRV));

			/* MPRV = 1 */
			*mstatus = set_field(*mstatus, MSTATUS_MPRV, 1);

			/* Write MSTATUS */
			if (*mstatus != *mstatus_old)
				if (register_write_direct(target, GDB_REGNO_MSTATUS, *mstatus) != ERROR_OK)
					return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int read_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	uint8_t *t_buffer = buffer;
	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	uint32_t access = 0;

	const int DMI_SBCS_SBSINGLEREAD_OFFSET = 20;
	const uint32_t DMI_SBCS_SBSINGLEREAD = (0x1U << DMI_SBCS_SBSINGLEREAD_OFFSET);

	const int DMI_SBCS_SBAUTOREAD_OFFSET = 15;
	const uint32_t DMI_SBCS_SBAUTOREAD = (0x1U << DMI_SBCS_SBAUTOREAD_OFFSET);

	/* ww favorise one off reading if there is an issue */
	if (count == 1) {
		for (uint32_t i = 0; i < count; i++) {
			if (dmi_read(target, &access, DMI_SBCS) != ERROR_OK)
				return ERROR_FAIL;
			dmi_write(target, DMI_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DMI_SBCS_SBACCESS, size/2);
			access = set_field(access, DMI_SBCS_SBSINGLEREAD, 1);
			LOG_DEBUG("\r\nread_memory: sab: access:  0x%08x", access);
			dmi_write(target, DMI_SBCS, access);
			/* 3) read */
			uint32_t value;
			if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("\r\nread_memory: sab: value:  0x%08x", value);
			write_to_buf(t_buffer, value, size);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_DEBUG("reading block until final address 0x%" PRIx64, fin_addr);
	if (dmi_read(target, &access, DMI_SBCS) != ERROR_OK)
		return ERROR_FAIL;
	/* set current address */
	dmi_write(target, DMI_SBADDRESS0, cur_addr);
	/* 2) write sbaccess=2, sbsingleread,sbautoread,sbautoincrement
	 * size/2 matching the bit access of the spec 0.13 */
	access = set_field(access, DMI_SBCS_SBACCESS, size/2);
	access = set_field(access, DMI_SBCS_SBAUTOREAD, 1);
	access = set_field(access, DMI_SBCS_SBSINGLEREAD, 1);
	access = set_field(access, DMI_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08x", access);
	dmi_write(target, DMI_SBCS, access);

	while (cur_addr < fin_addr) {
		LOG_DEBUG("\r\nsab:autoincrement: \r\n size: %d\tcount:%d\taddress: 0x%08"
				PRIx64, size, count, cur_addr);
		/* read */
		uint32_t value;
		if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
			return ERROR_FAIL;
		write_to_buf(t_buffer, value, size);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dmi_write(target, DMI_SBCS, 0);
			if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			write_to_buf(t_buffer, value, size);
		}
	}

	return ERROR_OK;
}

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs_write = set_field(0, DMI_SBCS_SBREADONADDR, 1);
		sbcs_write |= sb_sbaccess(size);
		sbcs_write = set_field(sbcs_write, DMI_SBCS_SBAUTOINCREMENT, 1);
		if (count > 1)
			sbcs_write = set_field(sbcs_write, DMI_SBCS_SBREADONDATA, count > 1);
		if (dmi_write(target, DMI_SBCS, sbcs_write) != ERROR_OK)
			return ERROR_FAIL;

		/* This address write will trigger the first read. */
		if (sb_write_address(target, next_address) != ERROR_OK)
			return ERROR_FAIL;

		if (info->bus_master_read_delay) {
			jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("Failed to scan idle sequence");
				return ERROR_FAIL;
			}
		}

		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			if (read_memory_bus_word(target, address + i * size, size,
					buffer + i * size) != ERROR_OK)
				return ERROR_FAIL;
		}

		uint32_t sbcs_read = 0;
		if (count > 1) {
			/* "Writes to sbcs while sbbusy is high result in undefined behavior.
			 * A debugger must not write to sbcs until it reads sbbusy as 0." */
			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;

			sbcs_write = set_field(sbcs_write, DMI_SBCS_SBREADONDATA, 0);
			if (dmi_write(target, DMI_SBCS, sbcs_write) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Read the last word, after we disabled sbreadondata if necessary. */
		if (!get_field(sbcs_read, DMI_SBCS_SBERROR) &&
				!get_field(sbcs_read, DMI_SBCS_SBBUSYERROR)) {
			if (read_memory_bus_word(target, address + (count - 1) * size, size,
						buffer + (count - 1) * size) != ERROR_OK)
				return ERROR_FAIL;

			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs_read, DMI_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			if (dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR) != ERROR_OK)
				return ERROR_FAIL;
			next_address = sb_read_address(target);
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs_read, DMI_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			if (dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int batch_run(const struct target *target, struct riscv_batch *batch)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);
	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait -= batch->used_scans;
		if (r->reset_delays_wait <= 0) {
			batch->idle_count = 0;
			info->dmi_busy_delay = 0;
			info->ac_busy_delay = 0;
		}
	}
	return riscv_batch_run(batch);
}

/*
 * Performs a memory read using memory access abstract commands. The read sizes
 * supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16 byte
 * aamsize fields in the memory access abstract command.
 */
static int read_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	memset(buffer, 0, count*size);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;
	if (width > 64) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_ERROR("Unsupported size: %d bits", size);
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	uint32_t command;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	//const addr mode set aampostincrement=0
	if (nds32->nds_const_addr_mode == 0)
		command = access_memory_command(target, false, width, true, false);
	else
		command = access_memory_command(target, false, width, false, false);
#else
	/* Create the command (physical address, postincrement, read) */
	uint32_t command = access_memory_command(target, false, width, true, false);
#endif

	/* Execute the reads */
	uint8_t *p = buffer;
	bool updateaddr = true;
	unsigned width32 = (width + 31) / 32 * 32;
	for (uint32_t c = 0; c < count; c++) {
		/* Only update the addres initially and let postincrement update it */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during read_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to execute command read_memory_abstract().");
			return result;
		}

		/* Copy arg0 to buffer (rounded width up to nearest 32) */
		riscv_reg_t value = read_abstract_arg(target, 0, width32);
		write_to_buf(p, value, size);

		updateaddr = false;
		p += size;
	}

	return result;
}

/*
 * Performs a memory write using memory access abstract commands. The write
 * sizes supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16
 * byte aamsize fields in the memory access abstract command.
 */
static int write_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int result = ERROR_OK;

	LOG_DEBUG("writing %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;
	if (width > 64) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_ERROR("Unsupported size: %d bits", width);
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	uint32_t command;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	//const addr mode set aampostincrement=0
	if (nds32->nds_const_addr_mode == 0)
		command = access_memory_command(target, false, width, true, true);
	else
		command = access_memory_command(target, false, width, false, true);
#else
	/* Create the command (physical address, postincrement, write) */
	uint32_t command = access_memory_command(target, false, width, true, true);
#endif

	/* Execute the writes */
	const uint8_t *p = buffer;
	bool updateaddr = true;
	for (uint32_t c = 0; c < count; c++) {
		/* Move data to arg0 */
		riscv_reg_t value = read_from_buf(p, size);
		result = write_abstract_arg(target, 0, value, riscv_xlen(target));
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to write arg0 during write_memory_abstract().");
			return result;
		  }

		/* Only update the addres initially and let postincrement update it */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during write_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to execute command write_memory_abstract().");
			return result;
		}

		updateaddr = false;
		p += size;
	}

	return result;
}

/**
 * Read the requested memory, taking care to execute every read exactly once,
 * even if cmderr=busy is encountered.
 */
static int read_memory_progbuf_inner(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;

	/* Write address to S0. */
	result = register_write_direct(target, GDB_REGNO_S0, address);
	if (result != ERROR_OK)
		return result;
	uint32_t command = access_register_command(target, GDB_REGNO_S1,
			riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;

	/* First read has just triggered. Result is in s1. */
	if (count == 1) {
		uint64_t value;
		if (register_read_direct(target, &value, GDB_REGNO_S1) != ERROR_OK)
			return ERROR_FAIL;
		write_to_buf(buffer, value, size);
		log_memory_access(address, value, size, true);
		return ERROR_OK;
	}

	if (dmi_write(target, DMI_ABSTRACTAUTO,
			1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET) != ERROR_OK)
		goto error;
	/* Read garbage from dmi_data0, which triggers another execution of the
	 * program. Now dmi_data0 contains the first good result, and s1 the next
	 * memory value. */
	if (dmi_read_exec(target, NULL, DMI_DATA0) != ERROR_OK)
		goto error;

	/* read_addr is the next address that the hart will read from, which is the
	 * value in s0. */
	riscv_addr_t read_addr = address + 2 * size;
	riscv_addr_t fin_addr = address + (count * size);
	while (read_addr < fin_addr) {
		LOG_DEBUG("read_addr=0x%" PRIx64 ", fin_addr=0x%" PRIx64, read_addr,
				fin_addr);
		/* The pipeline looks like this:
		 * memory -> s1 -> dm_data0 -> debugger
		 * Right now:
		 * s0 contains read_addr
		 * s1 contains mem[read_addr-size]
		 * dm_data0 contains[read_addr-size*2]
		 */

		LOG_DEBUG("creating burst to read from 0x%" PRIx64
				" up to 0x%" PRIx64, read_addr, fin_addr);
		assert(read_addr >= address && read_addr < fin_addr);
#if _NDS_JTAG_SCANS_OPTIMIZE_
		struct riscv_batch *batch = riscv_batch_alloc(target, nds_jtag_max_scans,
				info->dmi_busy_delay + info->ac_busy_delay);
#else
		struct riscv_batch *batch = riscv_batch_alloc(target, 32,
				info->dmi_busy_delay + info->ac_busy_delay);
#endif

		size_t reads = 0;
		for (riscv_addr_t addr = read_addr; addr < fin_addr; addr += size) {
			if (size > 4)
				riscv_batch_add_dmi_read(batch, DMI_DATA1);
			riscv_batch_add_dmi_read(batch, DMI_DATA0);

			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		batch_run(target, batch);

		/* Wait for the target to finish performing the last abstract command,
		 * and update our copy of cmderr. If we see that DMI is busy here,
		 * dmi_busy_delay will be incremented. */
		uint32_t abstractcs;
		if (dmi_read(target, &abstractcs, DMI_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DMI_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);

		riscv_addr_t next_read_addr;
		unsigned ignore_last = 0;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_read_addr = read_addr + reads * size;
				break;
			case CMDERR_BUSY:
				LOG_DEBUG("memory read resulted in busy response");

				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);

				dmi_write(target, DMI_ABSTRACTAUTO, 0);

				uint32_t dmi_data0, dmi_data1 = 0;
				/* This is definitely a good version of the value that we
				 * attempted to read when we discovered that the target was
				 * busy. */
				if (dmi_read(target, &dmi_data0, DMI_DATA0) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}
				if (size > 4 && dmi_read(target, &dmi_data1, DMI_DATA1) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* See how far we got, clobbering dmi_data0. */
				result = register_read_direct(target, &next_read_addr,
						GDB_REGNO_S0);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}
				uint64_t value64 = (((uint64_t) dmi_data1) << 32) | dmi_data0;
				write_to_buf(buffer + next_read_addr - 2 * size - address, value64, size);
				log_memory_access(next_read_addr - 2 * size, value64, size, true);

				/* Restore the command, and execute it.
				 * Now DMI_DATA0 contains the next value just as it would if no
				 * error had occurred. */
				dmi_write_exec(target, DMI_COMMAND, command, true);
				next_read_addr += size;

				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				ignore_last = 1;

				break;
			default:
				LOG_DEBUG("error when reading memory, abstractcs=0x%08lx", (long)abstractcs);
				riscv013_clear_abstract_error(target);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
		}

		/* Now read whatever we got out of the batch. */
		dmi_status_t status = DMI_STATUS_SUCCESS;
		riscv_addr_t receive_addr = read_addr - size * 2;
		unsigned read = 0;
		for (size_t i = 0; i < reads; i++) {
			assert(receive_addr < address + size * count);
			if (receive_addr < address)
				continue;
			if (receive_addr > next_read_addr - (3 + ignore_last) * size)
				break;

			uint64_t dmi_out = riscv_batch_get_dmi_read(batch, read++);
			status = get_field(dmi_out, DTM_DMI_OP);
			if (status != DMI_STATUS_SUCCESS) {
				/* If we're here because of busy count, dmi_busy_delay will
				 * already have been increased and busy state will have been
				 * cleared in dmi_read(). */
				/* In at least some implementations, we issue a read, and then
				 * can get busy back when we try to scan out the read result,
				 * and the actual read value is lost forever. Since this is
				 * rare in any case, we return error here and rely on our
				 * caller to reread the entire block. */
				LOG_WARNING("Batch memory read encountered DMI error %d. "
						"Falling back on slower reads.", status);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
			}
			uint64_t value = get_field(dmi_out, DTM_DMI_DATA);
			if (size > 4) {
				dmi_out = riscv_batch_get_dmi_read(batch, read++);
				status = get_field(dmi_out, DTM_DMI_OP);
				if (status != DMI_STATUS_SUCCESS) {
					LOG_WARNING("Batch memory read encountered DMI error %d. "
							"Falling back on slower reads.", status);
					riscv_batch_free(batch);
					result = ERROR_FAIL;
					goto error;
				}
				value <<= 32;
				value |= get_field(dmi_out, DTM_DMI_DATA);
			}
			riscv_addr_t offset = receive_addr - address;
			write_to_buf(buffer + offset, value, size);
			log_memory_access(receive_addr, value, size, true);

			receive_addr += size;
		}

		read_addr = next_read_addr;

		riscv_batch_free(batch);
	}

	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (count > 1) {
		/* Read the penultimate word. */
		uint32_t dmi_data0, dmi_data1 = 0;
		if (dmi_read(target, &dmi_data0, DMI_DATA0) != ERROR_OK)
			return ERROR_FAIL;
		if (size > 4 && dmi_read(target, &dmi_data1, DMI_DATA1) != ERROR_OK)
			return ERROR_FAIL;
		uint64_t value64 = (((uint64_t) dmi_data1) << 32) | dmi_data0;
		write_to_buf(buffer + size * (count-2), value64, size);
		log_memory_access(address + size * (count-2), value64, size, true);
	}

	/* Read the last word. */
	uint64_t value;
	result = register_read_direct(target, &value, GDB_REGNO_S1);
	if (result != ERROR_OK)
		goto error;
	write_to_buf(buffer + size * (count-1), value, size);
	log_memory_access(address + size * (count-1), value, size, true);

	return ERROR_OK;

error:
	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	return result;
}

/* Only need to save/restore one GPR to read a single word, and the progbuf
 * program doesn't need to increment. */
static int read_memory_progbuf_one(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	RISCV013_INFO(info);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t s0;

	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	/* Write address to S0, and execute buffer. */
	if (write_abstract_arg(target, 0, address, riscv_xlen(target)) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t command = access_register_command(target, GDB_REGNO_S0,
			riscv_xlen(target), AC_ACCESS_REGISTER_WRITE |
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t value;
	if (register_read(target, &value, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	write_to_buf(buffer, value, size);
	log_memory_access(address, value, size, true);

	if (riscv_set_register(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Read the requested memory, silently handling memory access errors.
 */
static int read_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
#if _NDS_MEM_Q_ACCESS_
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) >= 7) {
			return ndsv5_read_memory_quick_access(target, address, size, count, buffer);
		}
	}
#endif

	RISCV013_INFO(info);

	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory read.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			size, address);

	select_dmi(target);

	memset(buffer, 0, count*size);

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	if (count == 1)
		return read_memory_progbuf_one(target, address, size, buffer);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */
	uint64_t s0, s1;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}

	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->nds_const_addr_mode == 0)
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#else /* _NDS_V5_ONLY_ */
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#endif /* _NDS_V5_ONLY_ */

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	result = read_memory_progbuf_inner(target, address, size, count, buffer);

	if (result != ERROR_OK) {
		/* The full read did not succeed, so we will try to read each word individually. */
		/* This will not be fast, but reading outside actual memory is a special case anyway. */
		/* It will make the toolchain happier, especially Eclipse Memory View as it reads ahead. */
		target_addr_t address_i = address;
		uint32_t size_i = size;
		uint32_t count_i = 1;
		uint8_t *buffer_i = buffer;

		for (uint32_t i = 0; i < count; i++, address_i += size_i, buffer_i += size_i) {
			/* TODO: This is much slower than it needs to be because we end up
			 * writing the address to read for every word we read. */
			result = read_memory_progbuf_inner(target, address_i, size_i, count_i, buffer_i);

			/* The read of a single word failed, so we will just return 0 for that instead */
			if (result != ERROR_OK) {
				LOG_DEBUG("error reading single word of %d bytes from 0x%" TARGET_PRIxADDR,
						size_i, address_i);

				uint64_t value_i = 0;
				write_to_buf(buffer_i, value_i, size_i);
			}
		}
		result = ERROR_OK;
	}

	riscv_set_register(target, GDB_REGNO_S0, s0);
	riscv_set_register(target, GDB_REGNO_S1, s1);

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return result;
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);

	int result = ERROR_FAIL;
	if (info->progbufsize >= 2 && (memory->access_channel == NDS_MEMORY_ACC_CPU)) {
		result = read_memory_progbuf(target, address, size, count, buffer);
		goto read_memory_finish;
	} else if( (memory->access_channel == NDS_MEMORY_ACC_CPU) && nds_dmi_access_mem ) {
		result = read_memory_abstract(target, address, size, count, buffer);
		if(result == ERROR_OK)
			goto read_memory_finish;
	}
#else /* _NDS_V5_ONLY_ */
	if (info->progbufsize >= 2 && !riscv_prefer_sba)
		return read_memory_progbuf(target, address, size, count, buffer);
#endif /* _NDS_V5_ONLY_ */

	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 0)
			return read_memory_bus_v0(target, address, size, count, buffer);
		else if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 1)
#if _NDS_V5_ONLY_
			if (nds_jtag_scans_optimize > 0) {
				result = read_memory_bus_v1_opt(target, address, size, count, buffer);
				goto read_memory_finish;
			} else {
				result = read_memory_bus_v1(target, address, size, count, buffer);
				goto read_memory_finish;
			}
#else /* _NDS_V5_ONLY_ */
			return read_memory_bus_v1(target, address, size, count, buffer);
#endif /* _NDS_V5_ONLY_ */
	}

#if _NDS_V5_ONLY_
	if (info->progbufsize >= 2) {
		result = read_memory_progbuf(target, address, size, count, buffer);
		goto read_memory_finish;
	}
#else /* _NDS_V5_ONLY_ */
	if (info->progbufsize >= 2)
		return read_memory_progbuf(target, address, size, count, buffer);
#endif /* _NDS_V5_ONLY_ */

	return read_memory_abstract(target, address, size, count, buffer);

#if _NDS_V5_ONLY_
read_memory_finish:
	if (result == ERROR_OK) {
		uint32_t *p_word_data = (uint32_t *)buffer;
		NDS_INFO("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR " = 0x%08x", count,
				size, address, *p_word_data);
	}
	return result;
#endif /* _NDS_V5_ONLY_ */
}

static int write_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/*1) write sbaddress: for singlewrite and autoincrement, we need to write the address once*/
	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	dmi_write(target, DMI_SBADDRESS0, address);
	int64_t value = 0;
	int64_t access = 0;
	riscv_addr_t offset = 0;
	riscv_addr_t t_addr = 0;
	const uint8_t *t_buffer = buffer + offset;

	/* B.8 Writing Memory, single write check if we write in one go */
	if (count == 1) { /* count is in bytes here */
		/* TODO: Test with read_from_buf(&value, t_buffer, size) */
		/* check the size */
		switch (size) {
			case 1:
				value = t_buffer[0];
				break;
			case 2:
				value = t_buffer[0]
					| ((uint32_t) t_buffer[1] << 8);
				break;
			case 4:
				value = t_buffer[0]
					| ((uint32_t) t_buffer[1] << 8)
					| ((uint32_t) t_buffer[2] << 16)
					| ((uint32_t) t_buffer[3] << 24);
				break;
			default:
				LOG_ERROR("unsupported access size: %d", size);
				return ERROR_FAIL;
		}

		access = 0;
		access = set_field(access, DMI_SBCS_SBACCESS, size/2);
		dmi_write(target, DMI_SBCS, access);
		LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
		LOG_DEBUG("\r\nwrite_memory:SAB: ONE OFF: value 0x%08" PRIx64, value);
		dmi_write(target, DMI_SBDATA0, value);
		return ERROR_OK;
	}

	/*B.8 Writing Memory, using autoincrement*/

	access = 0;
	access = set_field(access, DMI_SBCS_SBACCESS, size/2);
	access = set_field(access, DMI_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
	dmi_write(target, DMI_SBCS, access);

	/*2)set the value according to the size required and write*/
	for (riscv_addr_t i = 0; i < count; ++i) {
		offset = size*i;
		/* for monitoring only */
		t_addr = address + offset;
		t_buffer = buffer + offset;

		/* TODO: Test with read_from_buf(&value, t_buffer, size) */
		switch (size) {
			case 1:
				value = t_buffer[0];
				break;
			case 2:
				value = t_buffer[0]
					| ((uint32_t) t_buffer[1] << 8);
				break;
			case 4:
				value = t_buffer[0]
					| ((uint32_t) t_buffer[1] << 8)
					| ((uint32_t) t_buffer[2] << 16)
					| ((uint32_t) t_buffer[3] << 24);
				break;
			default:
				LOG_ERROR("unsupported access size: %d", size);
				return ERROR_FAIL;
		}
		LOG_DEBUG("SAB:autoincrement: expected address: 0x%08x value: 0x%08x"
				PRIx64, (uint32_t)t_addr, (uint32_t)value);
		dmi_write(target, DMI_SBDATA0, value);
	}
	/*reset the autoincrement when finished (something weird is happening if this is not done at the end*/
	access = set_field(access, DMI_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DMI_SBCS, access);

	return ERROR_OK;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DMI_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DMI_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	int result;

	sb_write_address(target, next_address);
	while (next_address < end_address) {
		LOG_DEBUG("transferring burst starting at address 0x%" TARGET_PRIxADDR,
				next_address);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				32,
				info->dmi_busy_delay + info->bus_master_write_delay);

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (riscv_batch_available_scans(batch) < (size + 3) / 4)
				break;

			if (size > 12)
				riscv_batch_add_dmi_write(batch, DMI_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));

			if (size > 8)
				riscv_batch_add_dmi_write(batch, DMI_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				riscv_batch_add_dmi_write(batch, DMI_SBDATA1,
						((uint32_t) p[4]) |
						(((uint32_t) p[5]) << 8) |
						(((uint32_t) p[6]) << 16) |
						(((uint32_t) p[7]) << 24));
			uint32_t value = p[0];
			if (size > 2) {
				value |= ((uint32_t) p[2]) << 16;
				value |= ((uint32_t) p[3]) << 24;
			}
			if (size > 1)
				value |= ((uint32_t) p[1]) << 8;
			riscv_batch_add_dmi_write(batch, DMI_SBDATA0, value);

			log_memory_access(address + i * size, value, size, false);
			next_address += size;
		}

		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			return result;

		bool dmi_busy_encountered;
		if (dmi_op(target, &sbcs, &dmi_busy_encountered, DMI_OP_READ,
				DMI_SBCS, 0, false, false) != ERROR_OK)
			return ERROR_FAIL;

		time_t start = time(NULL);
		bool dmi_busy = dmi_busy_encountered;
		while (get_field(sbcs, DMI_SBCS_SBBUSY) || dmi_busy) {
			if (time(NULL) - start > riscv_command_timeout_sec) {
#if _NDS_V5_ONLY_
				LOG_ERROR("Timed out after %ds waiting for \"sbbusy\" to go low (sbcs=0x%x). "
					  "Increase the timeout with riscv set_command_timeout_sec.",
					  riscv_command_timeout_sec, sbcs);
#else
				LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					  "Increase the timeout with riscv set_command_timeout_sec.",
					  riscv_command_timeout_sec, sbcs);
#endif /* _NDS_V5_ONLY_ */
				return ERROR_FAIL;
			}

			if (dmi_op(target, &sbcs, &dmi_busy, DMI_OP_READ,
						DMI_SBCS, 0, false, true) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
		}

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR) || dmi_busy_encountered) {
			next_address = sb_read_address(target);
			if (next_address < address) {
				/* This should never happen, probably buggy hardware. */
				LOG_DEBUG("unexpected system bus address 0x%" TARGET_PRIxADDR,
					  next_address);
				return ERROR_FAIL;
			}

			continue;
		}

		unsigned error = get_field(sbcs, DMI_SBCS_SBERROR);
		if (error != 0) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory write.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	LOG_DEBUG("writing %d words of %d bytes to 0x%08lx", count, size, (long)address);

	select_dmi(target);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */

	int result = ERROR_OK;
	uint64_t s0, s1;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (store, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_sdr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("write_memory_progbuf(): Unsupported size: %d", size);
			result = ERROR_FAIL;
			goto error;
	}

	if (riscv_enable_virtual && info->progbufsize >= 4 && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->nds_const_addr_mode == 0)
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#else /* _NDS_V5_ONLY_ */
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#endif /* _NDS_V5_ONLY_ */

	result = riscv_program_ebreak(&program);
	if (result != ERROR_OK)
		goto error;
	riscv_program_write(&program);

	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	bool setup_needed = true;
	LOG_DEBUG("writing until final address 0x%016" PRIx64, fin_addr);
	while (cur_addr < fin_addr) {
		LOG_DEBUG("transferring burst starting at address 0x%016" PRIx64,
				cur_addr);

#if _NDS_JTAG_SCANS_OPTIMIZE_
		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				nds_jtag_max_scans,
				info->dmi_busy_delay + info->ac_busy_delay);
#else /* _NDS_JTAG_SCANS_OPTIMIZE_ */
		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				32,
				info->dmi_busy_delay + info->ac_busy_delay);
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_ */

		/* To write another word, we put it in S1 and execute the program. */
		unsigned start = (cur_addr - address) / size;
		for (unsigned i = start; i < count; ++i) {
			unsigned offset = size*i;
			const uint8_t *t_buffer = buffer + offset;

			uint64_t value = read_from_buf(t_buffer, size);

			log_memory_access(address + offset, value, size, false);
			cur_addr += size;

			if (setup_needed) {
				result = register_write_direct(target, GDB_REGNO_S0,
						address + offset);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Write value. */
				if (size > 4)
					dmi_write(target, DMI_DATA1, value >> 32);
				dmi_write(target, DMI_DATA0, value);

				/* Write and execute command that moves value into S1 and
				 * executes program buffer. */
				uint32_t command = access_register_command(target,
						GDB_REGNO_S1, riscv_xlen(target),
						AC_ACCESS_REGISTER_POSTEXEC |
						AC_ACCESS_REGISTER_TRANSFER |
						AC_ACCESS_REGISTER_WRITE);
				result = execute_abstract_command(target, command);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Turn on autoexec */
				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				if (size > 4)
					riscv_batch_add_dmi_write(batch, DMI_DATA1, value >> 32);
				riscv_batch_add_dmi_write(batch, DMI_DATA0, value);
				if (riscv_batch_full(batch))
					break;
			}
		}

		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			goto error;

		/* Note that if the scan resulted in a Busy DMI response, it
		 * is this read to abstractcs that will cause the dmi_busy_delay
		 * to be incremented if necessary. */

		uint32_t abstractcs;
		bool dmi_busy_encountered;
		result = dmi_op(target, &abstractcs, &dmi_busy_encountered,
				DMI_OP_READ, DMI_ABSTRACTCS, 0, false, true);
		if (result != ERROR_OK)
			goto error;
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DMI_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		if (info->cmderr == CMDERR_NONE && !dmi_busy_encountered) {
			LOG_DEBUG("successful (partial?) memory write");
		} else if (info->cmderr == CMDERR_BUSY || dmi_busy_encountered) {
			if (info->cmderr == CMDERR_BUSY)
				LOG_DEBUG("Memory write resulted in abstract command busy response.");
			else if (dmi_busy_encountered)
				LOG_DEBUG("Memory write resulted in DMI busy response.");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);

			dmi_write(target, DMI_ABSTRACTAUTO, 0);
			result = register_read_direct(target, &cur_addr, GDB_REGNO_S0);
			if (result != ERROR_OK)
				goto error;
			setup_needed = true;
		} else {
			LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
			riscv013_clear_abstract_error(target);
			result = ERROR_FAIL;
			goto error;
		}
	}

error:
	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (register_write_direct(target, GDB_REGNO_S1, s1) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

#if _NDS_V5_ONLY_
	/* Do fence.i when resume */
	nds32->nds_do_fencei = true;
#endif

	return result;
}

static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);

	uint32_t *p_word_data = (uint32_t *)buffer;
	NDS_INFO("writing %d words of %d bytes to 0x%08lx = 0x%08x", count, size, (long)address, *p_word_data);

	if (info->progbufsize >= 2 && (memory->access_channel == NDS_MEMORY_ACC_CPU))
		return write_memory_progbuf(target, address, size, count, buffer);
	else if( (memory->access_channel == NDS_MEMORY_ACC_CPU) && nds_dmi_access_mem ) {
		if(write_memory_abstract(target, address, size, count, buffer) == ERROR_OK)
			return ERROR_OK;
	}
#endif

	if (info->progbufsize >= 2 && !riscv_prefer_sba)
		return write_memory_progbuf(target, address, size, count, buffer);

	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 0)
			return write_memory_bus_v0(target, address, size, count, buffer);
		else if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 1)
#if _NDS_V5_ONLY_
			if (nds_jtag_scans_optimize > 0) {
				return write_memory_bus_v1_opt(target, address, size, count, buffer);
			}
#endif /* _NDS_V5_ONLY_ */
			return write_memory_bus_v1(target, address, size, count, buffer);
	}

	if (info->progbufsize >= 2)
		return write_memory_progbuf(target, address, size, count, buffer);

	return write_memory_abstract(target, address, size, count, buffer);
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

struct target_type riscv013_target = {
	.name = "riscv",

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	.poll = &riscv_openocd_poll,
	.halt = &riscv_halt,
	.resume = &riscv_resume,
	.step = &riscv_openocd_step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.read_memory = read_memory,
	.write_memory = write_memory,

	.arch_state = arch_state,
};

/*** 0.13-specific implementations of various RISC-V helper functions. ***/
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int hid, int rid)
{
#if _NDS_V5_ONLY_
	LOG_DEBUG("[%d] reading register %s on [%s] hart %d", target->coreid, gdb_regno_name(rid), 
			target->tap->dotted_name, hid);
#else /* _NDS_V5_ONLY_ */
	LOG_DEBUG("[%d] reading register %s on hart %d", target->coreid,
			gdb_regno_name(rid), hid);
#endif /* _NDS_V5_ONLY_ */

	riscv_set_current_hartid(target, hid);
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		*value = -1;
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif /* _NDS_V5_ONLY_ */

	int result = ERROR_OK;
	if (rid == GDB_REGNO_PC) {
		/* TODO: move this into riscv.c. */
		result = register_read(target, value, GDB_REGNO_DPC);
		LOG_DEBUG("[%d] read PC from DPC: 0x%" PRIx64, target->coreid, *value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		/* TODO: move this into riscv.c. */
		result = register_read(target, &dcsr, GDB_REGNO_DCSR);
		*value = get_field(dcsr, CSR_DCSR_PRV);
	} else {
		result = register_read(target, value, rid);
		if (result != ERROR_OK)
			*value = -1;
	}

	return result;
}

static int riscv013_set_register(struct target *target, int hid, int rid, uint64_t value)
{
#if _NDS_V5_ONLY_
	LOG_DEBUG("[%d] writing 0x%" PRIx64 " to register %s on [%s] hart %d", target->coreid, value,
			gdb_regno_name(rid), target->tap->dotted_name, hid);
#else /* _NDS_V5_ONLY_ */
	LOG_DEBUG("[%d] writing 0x%" PRIx64 " to register %s on hart %d",
			target->coreid, value, gdb_regno_name(rid), hid);
#endif /* _NDS_V5_ONLY_ */

	riscv_set_current_hartid(target, hid);
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif /* _NDS_V5_ONLY_ */

	if (rid <= GDB_REGNO_XPR31) {
		return register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_DEBUG("[%d] writing PC to DPC: 0x%" PRIx64, target->coreid, value);
		register_write_direct(target, GDB_REGNO_DPC, value);
		uint64_t actual_value;
		register_read_direct(target, &actual_value, GDB_REGNO_DPC);
		LOG_DEBUG("[%d]   actual DPC written: 0x%016" PRIx64, target->coreid, actual_value);
		if (value != actual_value) {
			LOG_ERROR("Written PC (0x%" PRIx64 ") does not match read back "
					"value (0x%" PRIx64 ")", value, actual_value);
			return ERROR_FAIL;
		}
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, value);
		return register_write_direct(target, GDB_REGNO_DCSR, dcsr);
	} else {
		return register_write_direct(target, rid, value);
	}

	return ERROR_OK;
}

static int riscv013_select_current_hart(struct target *target)
{
	RISCV_INFO(r);

	dm013_info_t *dm = get_dm(target);
#if _NDS_V5_ONLY_
	LOG_DEBUG("[%s] r->current_hartid: %d", target->tap->dotted_name, r->current_hartid);
	LOG_DEBUG("[%s] dm->current_hartid: %d", target->tap->dotted_name, dm->current_hartid);

	static struct jtag_tap *current_tap;
	if (current_tap == target->tap && nds_mixed_mode_checking != 0x3) {
		if (r->current_hartid == dm->current_hartid)
			return ERROR_OK;
	} else
		current_tap = target->tap;
#else
	if (r->current_hartid == dm->current_hartid)
		return ERROR_OK;
#endif

	uint32_t dmcontrol;
	/* TODO: can't we just "dmcontrol = DMI_DMACTIVE"? */
	if (dmi_read(target, &dmcontrol, DMI_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;
#if _NDS_V5_ONLY_
	if( !get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE) ) {
		LOG_INFO("TARGET WARNING! Debug Module was not become active (dmcontrol=0x%x), forced turn on!",
				dmcontrol);
		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_DMACTIVE, 1);
	}
#endif /* _NDS_V5_ONLY_ */
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	int result = dmi_write(target, DMI_DMCONTROL, dmcontrol);
	dm->current_hartid = r->current_hartid;
	return result;
}

/* Select all harts that were prepped and that are selectable, clearing the
 * prepped flag on the harts that actually were selected. */
static int select_prepped_harts(struct target *target, bool *use_hasel)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm->hasel_supported) {
		RISCV_INFO(r);
		r->prepped = false;
		*use_hasel = false;
		return ERROR_OK;
	}

	assert(dm->hart_count);
	unsigned hawindow_count = (dm->hart_count + 31) / 32;
	uint32_t hawindow[hawindow_count];

	memset(hawindow, 0, sizeof(uint32_t) * hawindow_count);

	target_list_t *entry;
	unsigned total_selected = 0;
	list_for_each_entry(entry, &dm->target_list, list) {
		struct target *t = entry->target;
		riscv_info_t *r = riscv_info(t);
		riscv013_info_t *info = get_info(t);
		unsigned index = info->index;
		LOG_DEBUG("index=%d, coreid=%d, prepped=%d", index, t->coreid, r->prepped);
		r->selected = r->prepped;
		if (r->prepped) {
			hawindow[index / 32] |= 1 << (index % 32);
			r->prepped = false;
			total_selected++;
		}
		index++;

#if _NDS_V5_ONLY_
		/* For AMP, we only need selected 1 core */
		if(!target->smp && total_selected == 1)
			break;
#endif /* _NDS_V5_ONLY_ */
	}

	/* Don't use hasel if we only need to talk to one hart. */
	if (total_selected <= 1) {
		*use_hasel = false;
		return ERROR_OK;
	}

	for (unsigned i = 0; i < hawindow_count; i++) {
		if (dmi_write(target, DMI_HAWINDOWSEL, i) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_write(target, DMI_HAWINDOW, hawindow[i]) != ERROR_OK)
			return ERROR_FAIL;
	}

	*use_hasel = true;
	return ERROR_OK;
}

static int riscv013_halt_prep(struct target *target)
{
	return ERROR_OK;
}

static int riscv013_halt_go(struct target *target)
{
	bool use_hasel = false;
	if (!riscv_rtos_enabled(target)) {
		if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
			return ERROR_FAIL;
	}

	RISCV_INFO(r);
#if _NDS_V5_ONLY_
	LOG_DEBUG("halting [%s] hart %d", target->tap->dotted_name, r->current_hartid);
#else
	LOG_DEBUG("halting hart %d", r->current_hartid);
#endif /* _NDS_V5_ONLY_ */

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DMI_DMCONTROL_DMACTIVE | DMI_DMCONTROL_HALTREQ;
	if (use_hasel)
		dmcontrol |= DMI_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (riscv_is_halted(target))
			break;

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus;
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_read(target, &dmcontrol, DMI_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

#if _NDS_V5_ONLY_
		NDS32_LOG("<-- Unable to halt [%s] hart %d -->", target->tap->dotted_name, r->current_hartid);
		NDS32_LOG("  dmcontrol=0x%08x", dmcontrol);
		NDS32_LOG("  dmstatus =0x%08x", dmstatus);
		if (riscv_rtos_enabled(target)) {
			target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
		}

		/* Release haltreq when failed to halt */
		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DMI_DMCONTROL, dmcontrol);

#else /* _NDS_V5_ONLY_ */
		LOG_ERROR("unable to halt hart %d", r->current_hartid);
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
#endif /* _NDS_V5_ONLY_ */
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HALTREQ, 0);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);

#if _NDS_V5_ONLY_
	if (riscv_rtos_enabled(target)) {
		target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0x0;
		LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
	}
#endif

	if (use_hasel) {
		target_list_t *entry;
		dm013_info_t *dm = get_dm(target);
		list_for_each_entry(entry, &dm->target_list, list) {
			struct target *t = entry->target;
			t->state = TARGET_HALTED;
			if (t->debug_reason == DBG_REASON_NOTHALTED)
				t->debug_reason = DBG_REASON_DBGRQ;
		}
	}
	/* The "else" case is handled in halt_go(). */

	return ERROR_OK;
}

static int riscv013_resume_go(struct target *target)
{
	bool use_hasel = false;
	if (!riscv_rtos_enabled(target)) {
		if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
			return ERROR_FAIL;
	}

	return riscv013_step_or_resume_current_hart(target, false, use_hasel);
}

static int riscv013_step_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, true, false);
}

static int riscv013_resume_prep(struct target *target)
{
	return riscv013_on_step_or_resume(target, false);
}

static int riscv013_on_step(struct target *target)
{
	return riscv013_on_step_or_resume(target, true);
}

static int riscv013_on_halt(struct target *target)
{
	return ERROR_OK;
}

static bool riscv013_is_halted(struct target *target)
{
#if _NDS_V5_ONLY_
	if (riscv_rtos_enabled(target)) {
		target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0x0;
	}
#endif /* _NDS_V5_ONLY_ */

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return false;
#if _NDS_V5_ONLY_
	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {
		LOG_ERROR("[%s] hart %d is unavailiable", target->tap->dotted_name, riscv_current_hartid(target));
		if (riscv_rtos_enabled(target)) {
			target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
		}
		return get_field(dmstatus, DMI_DMSTATUS_ALLHALTED);
	}
	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
		LOG_ERROR("[%s] hart %d doesn't exist", target->tap->dotted_name, riscv_current_hartid(target));
		if (riscv_rtos_enabled(target)) {
			target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
		}
		return get_field(dmstatus, DMI_DMSTATUS_ALLHALTED);
	}

	if ((target->state != TARGET_HALTED) &&
	    (nds_no_crst_detect == 0) &&
	    (get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET) == 1) ) {
		LOG_DEBUG(NDS32_ERRMSG_TARGET_RESET);

		uint32_t dmcontrol;
		dmi_read(target, &dmcontrol, DMI_DMCONTROL);
		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_ACKHAVERESET, 1);

		int retry = 0;
		do {
			dmi_write(target, DMI_DMCONTROL, dmcontrol);
			dmi_read(target, &dmstatus, DMI_DMSTATUS);
			alive_sleep(10);
		} while( get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET) && (retry++ < MAX_RETRY) );

		if(retry >= MAX_RETRY) {
			NDS32_LOG_ERROR("<-- TARGET WARNING! Unable to clear havereset on [%s] hart %d. -->", target->tap->dotted_name, riscv_current_hartid(target));
		}
	}
#else /* _NDS_V5_ONLY_ */
	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("Hart %d is unavailable.", riscv_current_hartid(target));
	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("Hart %d doesn't exist.", riscv_current_hartid(target));
	if (get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET)) {
		int hartid = riscv_current_hartid(target);
		LOG_INFO("Hart %d unexpectedly reset!", hartid);
		/* TODO: Can we make this more obvious to eg. a gdb user? */
		uint32_t dmcontrol = DMI_DMCONTROL_DMACTIVE |
			DMI_DMCONTROL_ACKHAVERESET;
		dmcontrol = set_hartsel(dmcontrol, hartid);
		/* If we had been halted when we reset, request another halt. If we
		 * ended up running out of reset, then the user will (hopefully) get a
		 * message that a reset happened, that the target is running, and then
		 * that it is halted again once the request goes through.
		 */
		if (target->state == TARGET_HALTED)
			dmcontrol |= DMI_DMCONTROL_HALTREQ;
		dmi_write(target, DMI_DMCONTROL, dmcontrol);
	}
#endif /* _NDS_V5_ONLY_ */

	return get_field(dmstatus, DMI_DMSTATUS_ALLHALTED);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	riscv_reg_t dcsr;
	int result = register_read(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return RISCV_HALT_UNKNOWN;

	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_SWBP:
		return RISCV_HALT_BREAKPOINT;
	case CSR_DCSR_CAUSE_TRIGGER:
		/* We could get here before triggers are enumerated if a trigger was
		 * already set when we connected. Force enumeration now, which has the
		 * side effect of clearing any triggers we did not set. */
		riscv_enumerate_triggers(target);
		LOG_DEBUG("{%d} halted because of trigger", target->coreid);
		return RISCV_HALT_TRIGGER;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_DEBUGINT:
	case CSR_DCSR_CAUSE_HALT:
		return RISCV_HALT_INTERRUPT;
	}

#if _NDS_DISABLE_ABORT_
	NDS32_LOG("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	NDS32_LOG("  dcsr=0x%016lx", (long)dcsr);
#endif
	LOG_ERROR("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%016lx", (long)dcsr);
	return RISCV_HALT_UNKNOWN;
}

int riscv013_write_debug_buffer(struct target *target, unsigned index, riscv_insn_t data)
{
#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
	if (nds_jtag_scans_optimize > 0) {
		RISCV013_INFO(info);
		if (write_debug_buffer_batch == NULL) {
			write_debug_buffer_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		}

		if (index >= info->progbufsize)
			riscv_batch_add_dmi_write(write_debug_buffer_batch, DMI_DATA0 + index - info->progbufsize, data);
		else
			riscv_batch_add_dmi_write(write_debug_buffer_batch, DMI_PROGBUF0 + index, data);
		return ERROR_OK;
	}
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF */

	dm013_info_t *dm = get_dm(target);
	if (dm->progbuf_cache[index] != data) {
		if (dmi_write(target, DMI_PROGBUF0 + index, data) != ERROR_OK)
			return ERROR_FAIL;
		dm->progbuf_cache[index] = data;
	} else {
		LOG_DEBUG("cache hit for 0x%x @%d", data, index);
	}
	return ERROR_OK;
}

riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned index)
{
#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
	RISCV013_INFO(info);
	if (index >= info->progbufsize) {
		LOG_DEBUG("error index=0x%x", index);
		return 0;
	}else {
		uint64_t return_val = backup_debug_buffer[riscv_current_hartid(target)][index];
		LOG_DEBUG("%s, return_val=0x%lx", __func__, (long unsigned int)return_val);
		return return_val;
	}
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF */

	uint32_t value;
	dmi_read(target, &value, DMI_PROGBUF0 + index);
	return value;
}

int riscv013_execute_debug_buffer(struct target *target)
{
	uint32_t run_program = 0;

#if _NDS_MEM_Q_ACCESS_
	nds_dmi_quick_access_ena = 0;
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		run_program = set_field(run_program, AC_ACCESS_REGISTER_CMDTYPE, 1);
		nds_dmi_quick_access_ena = 1;
		LOG_DEBUG("Quick Access command, nds_dmi_quick_access_ena=%d", nds_dmi_quick_access_ena);
	}
#endif

	run_program = set_field(run_program, AC_ACCESS_REGISTER_AARSIZE, 2);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_POSTEXEC, 1);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_TRANSFER, 0);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_REGNO, 0x1000);

	return execute_abstract_command(target, run_program);
}

void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

/* Helper function for riscv013_test_sba_config_reg */
static int get_max_sbaccess(struct target *target)
{
	RISCV013_INFO(info);

	uint32_t sbaccess128 = get_field(info->sbcs, DMI_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DMI_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DMI_SBCS_SBACCESS32);
	uint32_t sbaccess16 = get_field(info->sbcs, DMI_SBCS_SBACCESS16);
	uint32_t sbaccess8 = get_field(info->sbcs, DMI_SBCS_SBACCESS8);

	if (sbaccess128)
		return 4;
	else if (sbaccess64)
		return 3;
	else if (sbaccess32)
		return 2;
	else if (sbaccess16)
		return 1;
	else if (sbaccess8)
		return 0;
	else
		return -1;
}

static uint32_t get_num_sbdata_regs(struct target *target)
{
	RISCV013_INFO(info);

	uint32_t sbaccess128 = get_field(info->sbcs, DMI_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DMI_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DMI_SBCS_SBACCESS32);

	if (sbaccess128)
		return 4;
	else if (sbaccess64)
		return 2;
	else if (sbaccess32)
		return 1;
	else
		return 0;
}

static int riscv013_test_sba_config_reg(struct target *target,
		target_addr_t legal_address, uint32_t num_words,
		target_addr_t illegal_address, bool run_sbbusyerror_test)
{
	LOG_INFO("Testing System Bus Access as defined by RISC-V Debug Spec v0.13");

	uint32_t tests_failed = 0;

	uint32_t rd_val;
	uint32_t sbcs_orig;
	dmi_read(target, &sbcs_orig, DMI_SBCS);

	uint32_t sbcs = sbcs_orig;
	bool test_passed;

	int max_sbaccess = get_max_sbaccess(target);

	if (max_sbaccess == -1) {
		LOG_ERROR("System Bus Access not supported in this config.");
		return ERROR_FAIL;
	}

	if (get_field(sbcs, DMI_SBCS_SBVERSION) != 1) {
		LOG_ERROR("System Bus Access unsupported SBVERSION (%d). Only version 1 is supported.",
				get_field(sbcs, DMI_SBCS_SBVERSION));
		return ERROR_FAIL;
	}

	uint32_t num_sbdata_regs = get_num_sbdata_regs(target);

	uint32_t rd_buf[num_sbdata_regs];

	/* Test 1: Simple write/read test */
	test_passed = true;
	sbcs = set_field(sbcs_orig, DMI_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DMI_SBCS, sbcs);

	uint32_t test_patterns[4] = {0xdeadbeef, 0xfeedbabe, 0x12345678, 0x08675309};
	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DMI_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DMI_SBCS, sbcs);

		uint32_t compare_mask = (sbaccess == 0) ? 0xff : (sbaccess == 1) ? 0xffff : 0xffffffff;

		for (uint32_t i = 0; i < num_words; i++) {
			uint32_t addr = legal_address + (i << sbaccess);
			uint32_t wr_data[num_sbdata_regs];
			for (uint32_t j = 0; j < num_sbdata_regs; j++)
				wr_data[j] = test_patterns[j] + i;
			write_memory_sba_simple(target, addr, wr_data, num_sbdata_regs, sbcs);
		}

		for (uint32_t i = 0; i < num_words; i++) {
			uint32_t addr = legal_address + (i << sbaccess);
			read_memory_sba_simple(target, addr, rd_buf, num_sbdata_regs, sbcs);
			for (uint32_t j = 0; j < num_sbdata_regs; j++) {
				if (((test_patterns[j]+i)&compare_mask) != (rd_buf[j]&compare_mask)) {
					LOG_ERROR("System Bus Access Test 1: Error reading non-autoincremented address %x,"
							"expected val = %x, read val = %x", addr, test_patterns[j]+i, rd_buf[j]);
					test_passed = false;
					tests_failed++;
				}
			}
		}
	}
	if (test_passed)
		LOG_INFO("System Bus Access Test 1: Simple write/read test PASSED.");

	/* Test 2: Address autoincrement test */
	target_addr_t curr_addr;
	target_addr_t prev_addr;
	test_passed = true;
	sbcs = set_field(sbcs_orig, DMI_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DMI_SBCS, sbcs);

	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DMI_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DMI_SBCS, sbcs);

		dmi_write(target, DMI_SBADDRESS0, legal_address);
		read_sbcs_nonbusy(target, &sbcs);
		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; i++) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);
			if ((curr_addr - prev_addr != (uint32_t)(1 << sbaccess)) && (i != 0)) {
				LOG_ERROR("System Bus Access Test 2: Error with address auto-increment, sbaccess = %x.", sbaccess);
				test_passed = false;
				tests_failed++;
			}
			dmi_write(target, DMI_SBDATA0, i);
		}

		read_sbcs_nonbusy(target, &sbcs);

		dmi_write(target, DMI_SBADDRESS0, legal_address);

		uint32_t val;
		sbcs = set_field(sbcs, DMI_SBCS_SBREADONDATA, 1);
		dmi_write(target, DMI_SBCS, sbcs);
		dmi_read(target, &val, DMI_SBDATA0); /* Dummy read to trigger first system bus read */
		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; i++) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);
			if ((curr_addr - prev_addr != (uint32_t)(1 << sbaccess)) && (i != 0)) {
				LOG_ERROR("System Bus Access Test 2: Error with address auto-increment, sbaccess = %x", sbaccess);
				test_passed = false;
				tests_failed++;
			}
			dmi_read(target, &val, DMI_SBDATA0);
			read_sbcs_nonbusy(target, &sbcs);
			if (i != val) {
				LOG_ERROR("System Bus Access Test 2: Error reading auto-incremented address,"
						"expected val = %x, read val = %x.", i, val);
				test_passed = false;
				tests_failed++;
			}
		}
	}
	if (test_passed)
		LOG_INFO("System Bus Access Test 2: Address auto-increment test PASSED.");

	/* Test 3: Read from illegal address */
	read_memory_sba_simple(target, illegal_address, rd_buf, 1, sbcs_orig);

	dmi_read(target, &rd_val, DMI_SBCS);
	if (get_field(rd_val, DMI_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DMI_SBCS_SBERROR, 2);
		dmi_write(target, DMI_SBCS, sbcs);
		dmi_read(target, &rd_val, DMI_SBCS);
		if (get_field(rd_val, DMI_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 3: Illegal address read test PASSED.");
		else
			LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to clear to 0.");
	} else {
		LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to set error code.");
	}

	/* Test 4: Write to illegal address */
	write_memory_sba_simple(target, illegal_address, test_patterns, 1, sbcs_orig);

	dmi_read(target, &rd_val, DMI_SBCS);
	if (get_field(rd_val, DMI_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DMI_SBCS_SBERROR, 2);
		dmi_write(target, DMI_SBCS, sbcs);
		dmi_read(target, &rd_val, DMI_SBCS);
		if (get_field(rd_val, DMI_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 4: Illegal address write test PASSED.");
		else {
			LOG_ERROR("System Bus Access Test 4: Illegal address write test FAILED, unable to clear to 0.");
			tests_failed++;
		}
	} else {
		LOG_ERROR("System Bus Access Test 4: Illegal address write test FAILED, unable to set error code.");
		tests_failed++;
	}

	/* Test 5: Write with unsupported sbaccess size */
	uint32_t sbaccess128 = get_field(sbcs_orig, DMI_SBCS_SBACCESS128);

	if (sbaccess128) {
		LOG_INFO("System Bus Access Test 5: SBCS sbaccess error test PASSED, all sbaccess sizes supported.");
	} else {
		sbcs = set_field(sbcs_orig, DMI_SBCS_SBACCESS, 4);

		write_memory_sba_simple(target, legal_address, test_patterns, 1, sbcs);

		dmi_read(target, &rd_val, DMI_SBCS);
		if (get_field(rd_val, DMI_SBCS_SBERROR) == 4) {
			sbcs = set_field(sbcs_orig, DMI_SBCS_SBERROR, 4);
			dmi_write(target, DMI_SBCS, sbcs);
			dmi_read(target, &rd_val, DMI_SBCS);
			if (get_field(rd_val, DMI_SBCS_SBERROR) == 0)
				LOG_INFO("System Bus Access Test 5: SBCS sbaccess error test PASSED.");
			else {
				LOG_ERROR("System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to clear to 0.");
				tests_failed++;
			}
		} else {
			LOG_ERROR("System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to set error code.");
			tests_failed++;
		}
	}

	/* Test 6: Write to misaligned address */
	sbcs = set_field(sbcs_orig, DMI_SBCS_SBACCESS, 1);

	write_memory_sba_simple(target, legal_address+1, test_patterns, 1, sbcs);

	dmi_read(target, &rd_val, DMI_SBCS);
	if (get_field(rd_val, DMI_SBCS_SBERROR) == 3) {
		sbcs = set_field(sbcs_orig, DMI_SBCS_SBERROR, 3);
		dmi_write(target, DMI_SBCS, sbcs);
		dmi_read(target, &rd_val, DMI_SBCS);
		if (get_field(rd_val, DMI_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 6: SBCS address alignment error test PASSED");
		else {
			LOG_ERROR("System Bus Access Test 6: SBCS address alignment error test FAILED, unable to clear to 0.");
			tests_failed++;
		}
	} else {
		LOG_ERROR("System Bus Access Test 6: SBCS address alignment error test FAILED, unable to set error code.");
		tests_failed++;
	}

	/* Test 7: Set sbbusyerror, only run this case in simulation as it is likely
	 * impossible to hit otherwise */
	if (run_sbbusyerror_test) {
		sbcs = set_field(sbcs_orig, DMI_SBCS_SBREADONADDR, 1);
		dmi_write(target, DMI_SBCS, sbcs);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DMI_SBDATA0, 0xdeadbeef);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DMI_SBADDRESS0, legal_address);

		dmi_read(target, &rd_val, DMI_SBCS);
		if (get_field(rd_val, DMI_SBCS_SBBUSYERROR)) {
			sbcs = set_field(sbcs_orig, DMI_SBCS_SBBUSYERROR, 1);
			dmi_write(target, DMI_SBCS, sbcs);
			dmi_read(target, &rd_val, DMI_SBCS);
			if (get_field(rd_val, DMI_SBCS_SBBUSYERROR) == 0)
				LOG_INFO("System Bus Access Test 7: SBCS sbbusyerror test PASSED.");
			else {
				LOG_ERROR("System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to clear to 0.");
				tests_failed++;
			}
		} else {
			LOG_ERROR("System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to set error code.");
			tests_failed++;
		}
	}

	if (tests_failed == 0) {
		LOG_INFO("ALL TESTS PASSED");
		return ERROR_OK;
	} else {
		LOG_ERROR("%d TESTS FAILED", tests_failed);
		return ERROR_FAIL;
	}

}

void write_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *write_data, uint32_t write_size, uint32_t sbcs)
{
	RISCV013_INFO(info);

	uint32_t rd_sbcs;
	uint32_t masked_addr;

	uint32_t sba_size = get_field(info->sbcs, DMI_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_no_readonaddr = set_field(sbcs, DMI_SBCS_SBREADONADDR, 0);
	dmi_write(target, DMI_SBCS, sbcs_no_readonaddr);

	for (uint32_t i = 0; i < sba_size/32; i++) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DMI_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DMI_SBADDRESS3, masked_addr);
	}

	/* Write SBDATA registers starting with highest address, since write to
	 * SBDATA0 triggers write */
	for (int i = write_size-1; i >= 0; i--)
		dmi_write(target, DMI_SBDATA0+i, write_data[i]);
}

void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs)
{
	RISCV013_INFO(info);

	uint32_t rd_sbcs;
	uint32_t masked_addr;

	uint32_t sba_size = get_field(info->sbcs, DMI_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_readonaddr = set_field(sbcs, DMI_SBCS_SBREADONADDR, 1);
	dmi_write(target, DMI_SBCS, sbcs_readonaddr);

	/* Write addresses starting with highest address register */
	for (int i = sba_size/32-1; i >= 0; i--) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DMI_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DMI_SBADDRESS3, masked_addr);
	}

	read_sbcs_nonbusy(target, &rd_sbcs);

	for (uint32_t i = 0; i < read_size; i++)
		dmi_read(target, &(rd_buf[i]), DMI_SBDATA0+i);
}

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int maybe_execute_fence_i(struct target *target)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);
#if _NDS_V5_ONLY_
	if (info->progbufsize + r->impebreak >= 2)
		return execute_fence_i(target);
#else
	if (info->progbufsize + r->impebreak >= 3)
		return execute_fence(target);
#endif /* _NDS_V5_ONLY_ */
	return ERROR_OK;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	if (maybe_execute_fence_i(target) != ERROR_OK)
		return ERROR_FAIL;

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	int result = register_read(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, riscv_ebreakm);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, riscv_ebreaks);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, riscv_ebreaku);

#if _NDS_V5_ONLY_
	extern bool v5_stepie;
	if(v5_stepie == true)
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 1);
	else
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 0);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if((nds32->attached == false) && (nds32->target_burn_attached == false)) {
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 0);
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE , 0);
	}
#endif /* _NDS_V5_ONLY_ */

	return riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
}

static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step, bool use_hasel)
{
	RISCV_INFO(r);
#if _NDS_V5_ONLY_
	LOG_DEBUG("resuming [%s] hart %d (for step?=%d)", target->tap->dotted_name, r->current_hartid, step);
#else /* _NDS_V5_ONLY_ */
	LOG_DEBUG("resuming hart %d (for step?=%d)", r->current_hartid, step);
#endif /* _NDS_V5_ONLY_ */
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart %d is not halted!", r->current_hartid);
		return ERROR_FAIL;
	}

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DMI_DMCONTROL_DMACTIVE | DMI_DMCONTROL_RESUMEREQ;
	if (use_hasel)
		dmcontrol |= DMI_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HASEL, 0);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (get_field(dmstatus, DMI_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0)
			continue;

		dmi_write(target, DMI_DMCONTROL, dmcontrol);
#if _NDS_V5_ONLY_
		/* e-15336, when resuming, clear issued_halt */
		target->halt_issued = false;
#endif /* _NDS_V5_ONLY_ */
		return ERROR_OK;
	}

	dmi_write(target, DMI_DMCONTROL, dmcontrol);

	LOG_ERROR("unable to resume hart %d", r->current_hartid);
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return ERROR_FAIL;
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	if (step) {
		LOG_ERROR("  was stepping, halting");
		riscv_halt(target);
		return ERROR_OK;
	}

#if _NDS_DISABLE_ABORT_
	NDS32_LOG("resuming [%s] hart %d (for step?=%d) fail", target->tap->dotted_name, r->current_hartid, step);
#endif

	return ERROR_FAIL;
}

void riscv013_clear_abstract_error(struct target *target)
{
	/* Wait for busy to go away. */
	time_t start = time(NULL);
	uint32_t abstractcs;
	dmi_read(target, &abstractcs, DMI_ABSTRACTCS);
	while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY)) {
		dmi_read(target, &abstractcs, DMI_ABSTRACTCS);

		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("abstractcs.busy is not going low after %d seconds "
					"(abstractcs=0x%x). The target is either really slow or "
					"broken. You could increase the timeout with riscv "
					"set_command_timeout_sec.",
					riscv_command_timeout_sec, abstractcs);
			break;
		}
	}
	/* Clear the error status. */
	dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
}

#ifdef _WIN32
#define FILE_SEP '\\'
#else
#define FILE_SEP '/'
#endif
#define COMPLIANCE_TEST(b, message) \
{ \
	const char *last_sep = strrchr(__FILE__, FILE_SEP); \
	const char *fname = (last_sep == NULL ? __FILE__ : last_sep + 1); \
	LOG_INFO("Executing test %d (%s:%d): %s", total_tests, fname, __LINE__, message); \
	int pass = 0;		    \
	if (b) {		    \
		pass = 1;	    \
		passed_tests++;     \
	}			    \
	LOG_INFO("  %s", (pass) ? "PASSED" : "FAILED"); \
	assert(pass);		    \
	total_tests++;		    \
}

#define COMPLIANCE_MUST_PASS(b) COMPLIANCE_TEST(ERROR_OK == (b), "Regular calls must return ERROR_OK")

#define COMPLIANCE_READ(target, addr, value) COMPLIANCE_MUST_PASS(dmi_read(target, addr, value))
#define COMPLIANCE_WRITE(target, addr, value) COMPLIANCE_MUST_PASS(dmi_write(target, addr, value))

#define COMPLIANCE_CHECK_RO(target, addr)                               \
{                                                                       \
	uint32_t orig;                                                      \
	uint32_t inverse;                                                   \
	COMPLIANCE_READ(target, &orig, addr);                               \
	COMPLIANCE_WRITE(target, addr, ~orig);                              \
	COMPLIANCE_READ(target, &inverse, addr);                            \
	COMPLIANCE_TEST(orig == inverse, "Register must be read-only");     \
}

int riscv013_test_compliance(struct target *target)
{
	LOG_INFO("Basic compliance test against RISC-V Debug Spec v0.13");
	LOG_INFO("This test is not complete, and not well supported.");
	LOG_INFO("Your core might pass this test without being compliant.");
	LOG_INFO("Your core might fail this test while being compliant.");
	LOG_INFO("Use your judgment, and please contribute improvements.");

	if (!riscv_rtos_enabled(target)) {
		LOG_ERROR("Please run with -rtos riscv to run compliance test.");
		return ERROR_FAIL;
	}

	if (!target_was_examined(target)) {
		LOG_ERROR("Cannot run compliance test, because target has not yet "
			"been examined, or the examination failed.\n");
		return ERROR_FAIL;
	}

	int total_tests = 0;
	int passed_tests = 0;

	uint32_t dmcontrol_orig = DMI_DMCONTROL_DMACTIVE;
	uint32_t dmcontrol;
	uint32_t testvar;
	uint32_t testvar_read;
	riscv_reg_t value;
	RISCV013_INFO(info);

	/* All the bits of HARTSEL are covered by the examine sequence. */

	/* hartreset */
	/* This field is optional. Either we can read and write it to 1/0,
	or it is tied to 0. This check doesn't really do anything, but
	it does attempt to set the bit to 1 and then back to 0, which needs to
	work if its implemented. */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, set_field(dmcontrol_orig, DMI_DMCONTROL_HARTRESET, 1));
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, set_field(dmcontrol_orig, DMI_DMCONTROL_HARTRESET, 0));
	COMPLIANCE_READ(target, &dmcontrol, DMI_DMCONTROL);
	COMPLIANCE_TEST((get_field(dmcontrol, DMI_DMCONTROL_HARTRESET) == 0),
			"DMCONTROL.hartreset can be 0 or RW.");

	/* hasel */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, set_field(dmcontrol_orig, DMI_DMCONTROL_HASEL, 1));
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, set_field(dmcontrol_orig, DMI_DMCONTROL_HASEL, 0));
	COMPLIANCE_READ(target, &dmcontrol, DMI_DMCONTROL);
	COMPLIANCE_TEST((get_field(dmcontrol, DMI_DMCONTROL_HASEL) == 0),
			"DMCONTROL.hasel can be 0 or RW.");
	/* TODO: test that hamask registers exist if hasel does. */

	/* haltreq */
	COMPLIANCE_MUST_PASS(riscv_halt(target));
	/* This bit is not actually readable according to the spec, so nothing to check.*/

	/* DMSTATUS */
	COMPLIANCE_CHECK_RO(target, DMI_DMSTATUS);

	/* resumereq */
	/* This bit is not actually readable according to the spec, so nothing to check.*/
	COMPLIANCE_MUST_PASS(riscv_resume(target, true, 0, false, false));

	/* Halt all harts again so the test can continue.*/
	COMPLIANCE_MUST_PASS(riscv_halt(target));

	/* HARTINFO: Read-Only. This is per-hart, so need to adjust hartsel. */
	uint32_t hartinfo;
	COMPLIANCE_READ(target, &hartinfo, DMI_HARTINFO);
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		COMPLIANCE_CHECK_RO(target, DMI_HARTINFO);

		/* $dscratch CSRs */
		uint32_t nscratch = get_field(hartinfo, DMI_HARTINFO_NSCRATCH);
		for (unsigned int d = 0; d < nscratch; d++) {
			riscv_reg_t testval, testval_read;
			/* Because DSCRATCH is not guaranteed to last across PB executions, need to put
			this all into one PB execution. Which may not be possible on all implementations.*/
			if (info->progbufsize >= 5) {
				for (testval = 0x0011223300112233;
						 testval != 0xDEAD;
						 testval = testval == 0x0011223300112233 ? ~testval : 0xDEAD) {
					COMPLIANCE_TEST(register_write_direct(target, GDB_REGNO_S0, testval) == ERROR_OK,
							"Need to be able to write S0 in order to test DSCRATCH.");
					struct riscv_program program32;
					riscv_program_init(&program32, target);
					riscv_program_csrw(&program32, GDB_REGNO_S0, GDB_REGNO_DSCRATCH + d);
					riscv_program_csrr(&program32, GDB_REGNO_S1, GDB_REGNO_DSCRATCH + d);
					riscv_program_fence(&program32);
					riscv_program_ebreak(&program32);
					COMPLIANCE_TEST(riscv_program_exec(&program32, target) == ERROR_OK,
							"Accessing DSCRATCH with program buffer should succeed.");
					COMPLIANCE_TEST(register_read_direct(target, &testval_read, GDB_REGNO_S1) == ERROR_OK,
							"Need to be able to read S1 in order to test DSCRATCH.");
					if (riscv_xlen(target) > 32) {
						COMPLIANCE_TEST(testval == testval_read,
								"All DSCRATCH registers in HARTINFO must be R/W.");
					} else {
						COMPLIANCE_TEST(testval_read == (testval & 0xFFFFFFFF),
								"All DSCRATCH registers in HARTINFO must be R/W.");
					}
				}
			}
		}
		/* TODO: dataaccess */
		if (get_field(hartinfo, DMI_HARTINFO_DATAACCESS)) {
			/* TODO: Shadowed in memory map. */
			/* TODO: datasize */
			/* TODO: dataaddr */
		} else {
			/* TODO: Shadowed in CSRs. */
			/* TODO: datasize */
			/* TODO: dataaddr */
		}

	}

	/* HALTSUM -- TODO: More than 32 harts. Would need to loop over this to set hartsel */
	/* TODO: HALTSUM2, HALTSUM3 */
	/* HALTSUM0 */
	uint32_t expected_haltsum0 = 0;
	for (int i = 0; i < MIN(riscv_count_harts(target), 32); i++)
		expected_haltsum0 |= (1 << i);

	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0,
			"HALTSUM0 should report summary of up to 32 halted harts");

	COMPLIANCE_WRITE(target, DMI_HALTSUM0, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	COMPLIANCE_WRITE(target, DMI_HALTSUM0, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	/* HALTSUM1 */
	uint32_t expected_haltsum1 = 0;
	for (int i = 0; i < MIN(riscv_count_harts(target), 1024); i += 32)
		expected_haltsum1 |= (1 << (i/32));

	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1,
			"HALTSUM1 should report summary of up to 1024 halted harts");

	COMPLIANCE_WRITE(target, DMI_HALTSUM1, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	COMPLIANCE_WRITE(target, DMI_HALTSUM1, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	/* TODO: HAWINDOWSEL */

	/* TODO: HAWINDOW */

	/* ABSTRACTCS */

	uint32_t abstractcs;
	COMPLIANCE_READ(target, &abstractcs, DMI_ABSTRACTCS);

	/* Check that all reported Data Words are really R/W */
	for (int invert = 0; invert < 2; invert++) {
		for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_WRITE(target, DMI_DATA0 + i, testvar);
		}
		for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported DATA words must be R/W");
		}
	}

	/* Check that all reported ProgBuf words are really R/W */
	for (int invert = 0; invert < 2; invert++) {
		for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_WRITE(target, DMI_PROGBUF0 + i, testvar);
		}
		for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported PROGBUF words must be R/W");
		}
	}

	/* TODO: Cause and clear all error types */

	/* COMMAND
	According to the spec, this register is only W, so can't really check the read result.
	But at any rate, this is not legal and should cause an error. */
	COMPLIANCE_WRITE(target, DMI_COMMAND, 0xAAAAAAAA);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DMI_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED, \
			"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);

	COMPLIANCE_WRITE(target, DMI_COMMAND, 0x55555555);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DMI_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED, \
			"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);

	/* Basic Abstract Commands */
	for (unsigned int i = 1; i < 32; i = i << 1) {
		riscv_reg_t testval =	i | ((i + 1ULL) << 32);
		riscv_reg_t testval_read;
		COMPLIANCE_TEST(ERROR_OK == register_write_direct(target, GDB_REGNO_ZERO + i, testval),
				"GPR Writes should be supported.");
		COMPLIANCE_MUST_PASS(write_abstract_arg(target, 0, 0xDEADBEEFDEADBEEF, 64));
		COMPLIANCE_TEST(ERROR_OK == register_read_direct(target, &testval_read, GDB_REGNO_ZERO + i),
				"GPR Reads should be supported.");
		if (riscv_xlen(target) > 32) {
			/* Dummy comment to satisfy linter, since removing the brances here doesn't actually compile. */
			COMPLIANCE_TEST(testval == testval_read, "GPR Reads and writes should be supported.");
		} else {
			/* Dummy comment to satisfy linter, since removing the brances here doesn't actually compile. */
			COMPLIANCE_TEST((testval & 0xFFFFFFFF) == testval_read, "GPR Reads and writes should be supported.");
		}
	}

	/* ABSTRACTAUTO
	See which bits are actually writable */
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
	uint32_t abstractauto;
	uint32_t busy;
	COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0x0);
	if (abstractauto > 0) {
		/* This mechanism only works when you have a reasonable sized progbuf, which is not
		a true compliance requirement. */
		if (info->progbufsize >= 3) {

			testvar = 0;
			COMPLIANCE_TEST(ERROR_OK == register_write_direct(target, GDB_REGNO_S0, 0),
					"Need to be able to write S0 to test ABSTRACTAUTO");
			struct riscv_program program;
			COMPLIANCE_MUST_PASS(riscv_program_init(&program, target));
			/* This is also testing that WFI() is a NOP during debug mode. */
			COMPLIANCE_MUST_PASS(riscv_program_insert(&program, wfi()));
			COMPLIANCE_MUST_PASS(riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, 1));
			COMPLIANCE_MUST_PASS(riscv_program_ebreak(&program));
			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0x0);
			COMPLIANCE_MUST_PASS(riscv_program_exec(&program, target));
			testvar++;
			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
			COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);
			uint32_t autoexec_data = get_field(abstractauto, DMI_ABSTRACTAUTO_AUTOEXECDATA);
			uint32_t autoexec_progbuf = get_field(abstractauto, DMI_ABSTRACTAUTO_AUTOEXECPROGBUF);
			for (unsigned int i = 0; i < 12; i++) {
				COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
					busy = get_field(testvar_read, DMI_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_data & (1 << i)) {
					COMPLIANCE_TEST(i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT),
							"AUTOEXEC may be writable up to DATACOUNT bits.");
					testvar++;
				}
			}
			for (unsigned int i = 0; i < 16; i++) {
				COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
					busy = get_field(testvar_read, DMI_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_progbuf & (1 << i)) {
					COMPLIANCE_TEST(i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE),
							"AUTOEXEC may be writable up to PROGBUFSIZE bits.");
					testvar++;
				}
			}

			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0);
			COMPLIANCE_TEST(ERROR_OK == register_read_direct(target, &value, GDB_REGNO_S0),
					"Need to be able to read S0 to test ABSTRACTAUTO");

			COMPLIANCE_TEST(testvar == value,
					"ABSTRACTAUTO should cause COMMAND to run the expected number of times.");
		}
	}

	/* Single-Step each hart. */
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));
		COMPLIANCE_MUST_PASS(riscv013_on_step(target));
		COMPLIANCE_MUST_PASS(riscv013_step_current_hart(target));
		COMPLIANCE_TEST(riscv_halt_reason(target, hartsel) == RISCV_HALT_SINGLESTEP,
				"Single Step should result in SINGLESTEP");
	}

	/* Core Register Tests */
	uint64_t bogus_dpc = 0xdeadbeef;
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		/* DCSR Tests */
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DCSR, 0x0));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0,	"Not all bits in DCSR are writable by Debugger");
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DCSR, 0xFFFFFFFF));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0,	"At least some bits in DCSR must be 1");

		/* DPC. Note that DPC is sign-extended. */
		riscv_reg_t dpcmask = 0xFFFFFFFCUL;
		riscv_reg_t dpc;

		if (riscv_xlen(target) > 32)
			dpcmask |= (0xFFFFFFFFULL << 32);

		if (riscv_supports_extension(target, riscv_current_hartid(target), 'C'))
			dpcmask |= 0x2;

		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DPC, dpcmask));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpcmask == dpc,
				"DPC must be sign-extended to XLEN and writable to all-1s (except the least significant bits)");
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DPC, 0));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpc == 0, "DPC must be writable to 0.");
		if (hartsel == 0)
			bogus_dpc = dpc; /* For a later test step */
	}

	/* NDMRESET
	Asserting non-debug module reset should not reset Debug Module state.
	But it should reset Hart State, e.g. DPC should get a different value.
	Also make sure that DCSR reports cause of 'HALT' even though previously we single-stepped.
	*/

	/* Write some registers. They should not be impacted by ndmreset. */
	COMPLIANCE_WRITE(target, DMI_COMMAND, 0xFFFFFFFF);

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DMI_PROGBUF0 + i, testvar);
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DMI_DATA0 + i, testvar);
	}

	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
	COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);

	/* Pulse reset. */
	target->reset_halt = true;
	COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, 0));
	COMPLIANCE_TEST(ERROR_OK == assert_reset(target), "Must be able to assert NDMRESET");
	COMPLIANCE_TEST(ERROR_OK == deassert_reset(target), "Must be able to deassert NDMRESET");

	/* Verify that most stuff is not affected by ndmreset. */
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DMI_ABSTRACTCS_CMDERR)	== CMDERR_NOT_SUPPORTED,
			"NDMRESET should not affect DMI_ABSTRACTCS");
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == abstractauto, "NDMRESET should not affect DMI_ABSTRACTAUTO");

	/* Clean up to avoid future test failures */
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0);

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "PROGBUF words must not be affected by NDMRESET");
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "DATA words must not be affected by NDMRESET");
	}

	/* Verify that DPC *is* affected by ndmreset. Since we don't know what it *should* be,
	just verify that at least it's not the bogus value anymore. */

	COMPLIANCE_TEST(bogus_dpc != 0xdeadbeef, "BOGUS DPC should have been set somehow (bug in compliance test)");
	COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DPC));
	COMPLIANCE_TEST(bogus_dpc != value, "NDMRESET should move DPC to reset value.");

	COMPLIANCE_TEST(riscv_halt_reason(target, 0) == RISCV_HALT_INTERRUPT,
			"After NDMRESET halt, DCSR should report cause of halt");

	/* DMACTIVE -- deasserting DMACTIVE should reset all the above values. */

	/* Toggle dmactive */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, 0);
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DMI_ABSTRACTCS_CMDERR)	== 0, "ABSTRACTCS.cmderr should reset to 0");
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == 0, "ABSTRACTAUTO should reset to 0");

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); i++) {
		COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "PROGBUF words should reset to 0");
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT); i++) {
		COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "DATA words should reset to 0");
	}

	/*
	* TODO:
	* DCSR.cause priorities
	* DCSR.stoptime/stopcycle
	* DCSR.stepie
	* DCSR.ebreak
	* DCSR.prv
	*/

	/* Halt every hart for any follow-up tests*/
	COMPLIANCE_MUST_PASS(riscv_halt(target));

	uint32_t failed_tests = total_tests - passed_tests;
	if (total_tests == passed_tests) {
		LOG_INFO("ALL TESTS PASSED\n");
		return ERROR_OK;
	} else {
		LOG_INFO("%d TESTS FAILED\n", failed_tests);
		return ERROR_FAIL;
	}
}

/********************************************************************/
#if _NDS_V5_ONLY_
#include "ndsv5.h"



/********************************************************************/
/* ndsv5-013 local Var.                                             */
/********************************************************************/
static struct riscv_batch *ndsv5_access_memory_pack_batch;
static size_t index_access_mem_batch_ABSTRACTCS;
/********************************************************************/

int ndsv5_set_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t reg_value)
{
	uint32_t nds_dmi_quick_access_bak = nds_dmi_quick_access;
	nds_dmi_quick_access = 1;
	riscv_select_current_hart(target);

	struct riscv_program program;
	riscv_program_init(&program, target);

	dmi_write(target, DMI_DATA0, reg_value);
	if (riscv_xlen(target) == 64)
		dmi_write(target, DMI_DATA1, reg_value >> 32);

ndsv5_set_csr_reg_quick_access_retry:
	riscv_program_fence(&program);

	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
	}
	riscv_program_insert(&program, csrrw(GDB_REGNO_ZERO, GDB_REGNO_S0, reg_num - GDB_REGNO_CSR0));
	if (riscv_xlen(target) == 64)
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	else
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */

	int result = riscv_program_exec(&program, target);
	nds_dmi_quick_access = nds_dmi_quick_access_bak;
	if (result != ERROR_OK) {
		/* quick_access mode, if target state from freerun to halt */
		if (nds_dmi_quick_access_ena) {
			if (((ndsv5_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) ==
			    CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, set_csr_reg_quick_access_retry");
				goto ndsv5_set_csr_reg_quick_access_retry;
			}
		}
	}

	return ERROR_OK;
}

int ndsv5_get_csr_reg_quick_access(struct target *target, uint32_t reg_num, uint64_t *preg_value)
{
	uint32_t nds_dmi_quick_access_bak = nds_dmi_quick_access;
	nds_dmi_quick_access = 1;
	riscv_select_current_hart(target);

	struct riscv_program program;
	riscv_program_init(&program, target);

ndsv5_get_csr_reg_quick_access_retry:
	riscv_program_fence(&program);

	if (riscv_xlen(target) == 64)
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	else
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */

	riscv_program_insert(&program, csrrs(GDB_REGNO_S0, GDB_REGNO_ZERO, reg_num - GDB_REGNO_CSR0));
	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	}

	int result = riscv_program_exec(&program, target);
	nds_dmi_quick_access = nds_dmi_quick_access_bak;
	if (result != ERROR_OK) {
		/* quick_access mode, if target state from freerun to halt */
		if (nds_dmi_quick_access_ena) {
			if (((ndsv5_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) ==
			    CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, get_csr_reg_quick_access_retry");
				goto ndsv5_get_csr_reg_quick_access_retry;
			}
		}
	}

	uint32_t value = 0;
	dmi_read(target, &value, DMI_DATA0);
	LOG_DEBUG("value: 0x%x", value);
	uint32_t value_h = 0;
	if (riscv_xlen(target) == 64) {
		dmi_read(target, &value_h, DMI_DATA1);
		LOG_DEBUG("value_h: 0x%x", value_h);
	}
	uint64_t reg_value = value_h;
	reg_value = (reg_value << 32) | value;
	*preg_value = reg_value;
	return ERROR_OK;
}

int ndsv5_haltonreset(struct target *target, int enable)
{
	int i;

	if (enable == 1)
		LOG_DEBUG("Set setresethaltreq");
	else
		LOG_DEBUG("Set clrresethaltreq");

	RISCV_INFO(r);
	int hartid_bak = r->current_hartid;

	/* Correct current dm current_hartid */
	dm013_info_t *dm = get_dm(target);
	uint32_t tmp_dmcontrol;
	dmi_read(target, &tmp_dmcontrol, DMI_DMCONTROL);
	dm->current_hartid = get_field(tmp_dmcontrol, DMI_DMCONTROL_HARTSELLO);

	for (i = 0; i < r->hart_count(target); i++) {
		r->current_hartid = i;
		riscv013_select_current_hart(target);

		uint32_t s;
		dmi_read(target, &s, DMI_DMSTATUS);
		if (get_field(s, DMI_DMSTATUS_HASRESETHALTREQ)) {
			uint32_t control;
			dmi_read(target, &control, DMI_DMCONTROL);

			if (enable == 1)
				control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
			else
				control = set_field(control, DMI_DMCONTROL_CLRRESETHALTREQ, 1);
			dmi_write(target, DMI_DMCONTROL, control);

			if (enable == 1)
				LOG_DEBUG("hart %d: halt-on-reset is on!", i);
			else
				LOG_DEBUG("hart %d: halt-on-reset is off!", i);

		}

	}

	r->current_hartid = hartid_bak;
	riscv013_select_current_hart(target);
	return ERROR_OK;
}

int ndsv5_reexamine(struct target *target)
{
	LOG_DEBUG("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	/*
	if (riscv_init_registers(target) != ERROR_OK)
		return ERROR_FAIL;
	*/

	RISCV_INFO(r);
	r->current_hartid = target->coreid;
	riscv013_select_current_hart(target);

	return examine(target);
}

int riscv013_poll_wo_announce(struct target *target)
{
	ndsv5_without_announce = 1;
	return riscv_openocd_poll(target);
}

int ndsv5_reset_halt_as_examine(struct target *target)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	int user_def_hart_count = (int)target->corenums;
	if (user_def_hart_count == 0)
		user_def_hart_count = RISCV_MAX_HARTS;
	LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
	uint32_t dmstatus;
	int i;
	uint32_t control = 0;

	/* reset the Debug Module */
	dmi_write(target, DMI_DMCONTROL, 0);
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	dmi_read(target, &control, DMI_DMCONTROL);

	/* check existence harts */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &control, DMI_DMCONTROL);

		dmi_read(target, &dmstatus, DMI_DMSTATUS);
		if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
			user_def_hart_count = i;
			LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
			break;
		}
	}

	/* Assert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);

		/* SETRESETHALTREQ can halt on 0x80000000 for AMP/SMP all harts */
		control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
		control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &control, DMI_DMCONTROL);
	}

	/* Assert ndmreset */
	control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
	dmi_write(target, DMI_DMCONTROL, control);

	alive_sleep(nds32->reset_time);
	dmi_read(target, &dmstatus, DMI_DMSTATUS);

	/* Deassert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		time_t start = time(NULL);
		control = 0;

		/* Clear the reset, but make sure haltreq is still set */
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
		control = set_field(control, DMI_DMCONTROL_CLRRESETHALTREQ, 1);
		control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &control, DMI_DMCONTROL);

		do {
			dmi_read(target, &dmstatus, DMI_DMSTATUS);
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart didn't halt coming out of reset in %ds; "
				"dmstatus=0x%x; "
				"Increase the timeout with riscv set_reset_timeout_sec.",
				riscv_reset_timeout_sec, dmstatus);

				/* Release haltreq when failed to halt */
				control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
				dmi_write(target, DMI_DMCONTROL, control);

				return ERROR_FAIL;
			}
		} while (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0);

		control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, &control, DMI_DMCONTROL);
		dmi_read(target, &dmstatus, DMI_DMSTATUS);
	}
	reset_halt = true;
	return ERROR_OK;
}

int ndsv5_access_memory_pack_batch_run(struct target *target, uint32_t free_after_run)
{
	if (ndsv5_access_memory_pack_batch == NULL)
		return ERROR_OK;

	int result = ERROR_OK;

	LOG_DEBUG("riscv_batch_run: ndsv5_access_memory_pack_batch ");
	uint32_t ndsv5_retry_cnt = 0;

ndsv5_access_memory_pack_batch_run_retry:
	if (riscv_batch_run(ndsv5_access_memory_pack_batch) != ERROR_OK) {
		riscv013_clear_abstract_error(target);
		increase_dmi_busy_delay(target);
		if (ndsv5_retry_cnt < ndsv5_dmi_busy_retry_times) {
			ndsv5_retry_cnt++;
			LOG_DEBUG("riscv_batch_run(ndsv5_access_memory_pack_batch) retry !!");
			goto ndsv5_access_memory_pack_batch_run_retry;
		}
		LOG_ERROR("riscv_batch_run(ndsv5_access_memory_pack_batch) ERROR !!");
		result = ERROR_FAIL;
	}

	uint64_t dmi_out = riscv_batch_get_dmi_read(ndsv5_access_memory_pack_batch, index_access_mem_batch_ABSTRACTCS);
	uint64_t abstractcs = buf_get_u64((uint8_t *)&dmi_out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
	if (get_field(abstractcs, DMI_ABSTRACTCS_CMDERR) != 0) {
		riscv013_clear_abstract_error(target);
		uint32_t cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		LOG_ERROR("cmderr = 0x%x", cmderr);
		result = ERROR_FAIL;
	}

	if (free_after_run == 1) {
		riscv_batch_free(ndsv5_access_memory_pack_batch);
		ndsv5_access_memory_pack_batch = NULL;
	}
	return result;
}

static int ndsv5_write_abstract_arg_pack(struct target *target, unsigned index, riscv_reg_t value)
{
	if (ndsv5_access_memory_pack_batch == NULL) {
		LOG_ERROR("ndsv5_access_memory_pack_batch == NULL !!");
		return ERROR_FAIL;
	}

	unsigned xlen = riscv_xlen(target);
	unsigned offset = index * xlen / 32;
	switch (xlen) {
		default:
			LOG_ERROR("Unsupported xlen: %d", xlen);
			return ERROR_FAIL;
		case 64:
			riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_DATA0 + offset + 1, value >> 32);
		case 32:
			riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_DATA0 + offset, value);
	}
	return ERROR_OK;
}

static int ndsv5_access_s0s1_direct_pack(struct target *target, unsigned number, uint64_t value, uint32_t if_write)
{
	if (ndsv5_access_memory_pack_batch == NULL) {
		LOG_ERROR("ndsv5_access_memory_pack_batch == NULL !!");
		return ERROR_FAIL;
	}

	uint32_t command = access_register_command(target, number, riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER);
	if (if_write)
		command |= AC_ACCESS_REGISTER_WRITE;

	ndsv5_write_abstract_arg_pack(target, 0, value);
	riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_COMMAND, command);
	riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_ABSTRACTCS);
	return ERROR_OK;
}

static int ndsv5_write_debug_buffer_pack(struct target *target, unsigned index, riscv_insn_t data)
{
	RISCV013_INFO(info);
	if (ndsv5_access_memory_pack_batch == NULL) {
		LOG_ERROR("ndsv5_access_memory_pack_batch == NULL !!");
		return ERROR_FAIL;
	}

	if (index >= info->progbufsize)
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_DATA0 + index - info->progbufsize, data);
	else
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_PROGBUF0 + index, data);
	return ERROR_OK;
}

static int ndsv5_program_write_pack(struct riscv_program *program)
{
	for (unsigned i = 0; i < program->instruction_count; ++i) {
		/* LOG_DEBUG("%p: debug_buffer[%02x] = DASM(0x%08x)", program, i, program->debug_buffer[i]); */
		if (ndsv5_write_debug_buffer_pack(program->target, i,
					program->debug_buffer[i]) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

int ndsv5_read_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t i;
	uint32_t value;
	int result;

	for (i = 0; i < count; i++) {
		unsigned offset = size*i;

		if (ndsv5_access_memory_pack_batch == NULL) {
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target,
								nds_jtag_max_scans,
								info->dmi_busy_delay + info->ac_busy_delay);
		/* } else if (riscv_batch_full(ndsv5_access_memory_pack_batch)) { */
		} else if (ndsv5_access_memory_pack_batch->used_scans >
			   (ndsv5_access_memory_pack_batch->allocated_scans - 15)) {
			result = ndsv5_access_memory_pack_batch_run(target, 1);
			if (result != ERROR_OK) {
				LOG_ERROR("access_memory_pack_batch FAIL !!");
				return ERROR_FAIL;
			}
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target,
								nds_jtag_max_scans,
								info->dmi_busy_delay + info->ac_busy_delay);
		}
		select_dmi(target);
		struct riscv_program program;

		riscv_program_init(&program, target);
		switch (size) {
			case 1:
				riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			case 2:
				riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			case 4:
				riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			default:
				LOG_ERROR("Unsupported size: %d", size);
				return ERROR_FAIL;
		}

		riscv_program_ebreak(&program);
		ndsv5_program_write_pack(&program);

		ndsv5_access_s0s1_direct_pack(target, GDB_REGNO_S0, address + offset, 1);

		/* Write and execute command that moves value into S1 and
		 * executes program buffer. */
		uint32_t command = access_register_command(target, GDB_REGNO_S1, riscv_xlen(target),
				AC_ACCESS_REGISTER_POSTEXEC |
				AC_ACCESS_REGISTER_TRANSFER);
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_ABSTRACTCS);
		/* move S1 to DATA0 */
		ndsv5_access_s0s1_direct_pack(target, GDB_REGNO_S1, address + offset, 0);

		/* read value in DATA0 */
		size_t index_data0 = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_DATA0);

		result = ndsv5_access_memory_pack_batch_run(target, 0);

		uint64_t dmi_out = riscv_batch_get_dmi_read(ndsv5_access_memory_pack_batch, index_data0);
		value = get_field(dmi_out, DTM_DMI_DATA);
		write_to_buf(buffer + offset, value, size);

		LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08x", address + offset, value);
		riscv_batch_free(ndsv5_access_memory_pack_batch);
		ndsv5_access_memory_pack_batch = NULL;
		if (result != ERROR_OK) {
			LOG_ERROR("access_memory_pack_batch FAIL !!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

int ndsv5_write_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t i, value;
	int result;

	for (i = 0; i < count; i++) {
		unsigned offset = size*i;
		const uint8_t *t_buffer = buffer + offset;
		switch (size) {
				case 1:
					value = t_buffer[0];
					break;
				case 2:
					value = t_buffer[0]
						| ((uint32_t) t_buffer[1] << 8);
					break;
				case 4:
					value = t_buffer[0]
						| ((uint32_t) t_buffer[1] << 8)
						| ((uint32_t) t_buffer[2] << 16)
						| ((uint32_t) t_buffer[3] << 24);
					break;
				default:
					LOG_ERROR("unsupported access size: %d", size);
					return ERROR_FAIL;
		}
		LOG_DEBUG("M[0x%08" PRIx64 "] writes 0x%08x", address + offset, value);

		if (ndsv5_access_memory_pack_batch == NULL) {
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target,
								nds_jtag_max_scans,
								info->dmi_busy_delay + info->ac_busy_delay);
		/* } else if (riscv_batch_full(ndsv5_access_memory_pack_batch)) { */
		} else if (ndsv5_access_memory_pack_batch->used_scans >
			   (ndsv5_access_memory_pack_batch->allocated_scans - 12)) {
			result = ndsv5_access_memory_pack_batch_run(target, 1);
			if (result != ERROR_OK) {
				LOG_ERROR("access_memory_pack_batch FAIL !!");
				return ERROR_FAIL;
			}
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target,
								nds_jtag_max_scans,
								info->dmi_busy_delay + info->ac_busy_delay);
		}
		select_dmi(target);
		struct riscv_program program;

		riscv_program_init(&program, target);
		switch (size) {
			case 1:
				riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			case 2:
				riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			case 4:
				riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
				break;
			default:
				LOG_ERROR("Unsupported size: %d", size);
				return ERROR_FAIL;
		}
		riscv_program_ebreak(&program);
		ndsv5_program_write_pack(&program);

		ndsv5_access_s0s1_direct_pack(target, GDB_REGNO_S0, address + offset, 1);

		/* Write value. */
		ndsv5_write_abstract_arg_pack(target, 0, value);

		/* Write and execute command that moves value into S1 and
		 * executes program buffer. */
		uint32_t command = access_register_command(target, GDB_REGNO_S1, 32,
				AC_ACCESS_REGISTER_POSTEXEC |
				AC_ACCESS_REGISTER_TRANSFER |
				AC_ACCESS_REGISTER_WRITE);
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_ABSTRACTCS);
	}
	return ERROR_OK;
}

int ndsv5_vector_restore_vtype_vl(struct target *target, uint64_t reg_vtype)
{
	int result;

	register_write_direct(target, GDB_REGNO_S0, reg_vtype);
	LOG_DEBUG("reg_vtype: 0x%lx", (long unsigned int)reg_vtype);

	/* Restore vtype & vl */
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_vsetvl(&program, GDB_REGNO_S0, GDB_REGNO_S0);
	result = riscv_program_exec(&program, target);
	if (result != ERROR_OK) {
		LOG_ERROR("riscv_program_vsetvl ERROR !!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*
1. The debugger can first set the SEW to XLEN by using the vsetvli instruction.
2. Then the debugger can use CSRR to read the vtype CSR to check the vill bit.
3. If the vill bit is set, then the debugger set the SEW to (SEW lenth of last vsetvli)/2 again.
4. Continue to step 2 and step 3 until the vill bit is not set.
To get  VLMAX:
vsetvli s1, x0, vtypei
*/
int ndsv5_get_vector_VLMAX(struct target *target)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t vector_SEW = 64;
	uint64_t vector_vl = 0;
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vtype_tmp = 0, reg_vl_tmp = 0;
	int result;
	unsigned xlen = riscv_xlen(target);

	uint64_t mmsc_cfg;
	if(register_read_direct(target, &mmsc_cfg, GDB_REGNO_CSR0 + CSR_MMSC_CFG) != ERROR_OK) {
		LOG_ERROR("read mmsc_cfg error");
	}
	if ((mmsc_cfg & 0x1000000000)>>36) {
		MSTATUS_VS = 0x00000600;
	} else {
		MSTATUS_VS = 0x01800000;
	}

	uint64_t mstatus;
	if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
		LOG_ERROR("read mstatus error");

	if ((mstatus & MSTATUS_VS) == 0 &&
	   register_write_direct(target, GDB_REGNO_MSTATUS, set_field(mstatus, MSTATUS_VS, 1)) != ERROR_OK)
		LOG_ERROR("cannot write mstatus_vs");

	riscv_get_register(target, &reg_vtype, CSR_VTYPE + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vl, CSR_VL + GDB_REGNO_CSR0);
	LOG_DEBUG("start_reg_vtype: 0x%" PRIx64, reg_vtype);
	LOG_DEBUG("start_reg_vl: 0x%" PRIx64, reg_vl);

	struct riscv_program program;
	while (vector_SEW >= 8) {
		riscv_program_init(&program, target);
		riscv_program_vsetvli(&program, GDB_REGNO_S0, vector_SEW);
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK) {
			LOG_ERROR("riscv_program_vsetvli ERROR !!");
			return ERROR_FAIL;
		}
		/* Read S0 */
		if (register_read_direct(target, &vector_vl, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("vector_SEW: 0x%x", vector_SEW);
		LOG_DEBUG("vector_vl: 0x%" PRIx64, vector_vl);

		riscv_get_register(target, &reg_vtype_tmp, CSR_VTYPE + GDB_REGNO_CSR0);
		riscv_get_register(target, &reg_vl_tmp, CSR_VL + GDB_REGNO_CSR0);
		LOG_DEBUG("reg_vtype_tmp: 0x%" PRIx64, reg_vtype_tmp);
		LOG_DEBUG("reg_vl_tmp: 0x%" PRIx64, reg_vl_tmp);

		/* if (vector_vl != 0) {  // need check if vtype.vill == 1 ? */
		if ((reg_vtype_tmp & (0x01 << (xlen-1))) == 0) {
			/* check if vtype.vill == 1 */
			break;
		} else
			vector_SEW = (vector_SEW >> 1);
	}
	nds32->nds_vector_length = vector_SEW * vector_vl;
	nds32->nds_vector_SEW = vector_SEW;
	nds32->nds_vector_vl = (uint32_t)vector_vl;
	LOG_DEBUG("nds32->nds_vector_SEW: 0x%x, nds32->nds_vector_vl: 0x%x", nds32->nds_vector_SEW, nds32->nds_vector_vl);
	LOG_DEBUG("nds32->nds_vector_length: 0x%x", nds32->nds_vector_length);

	/* Restore vtype & vl */
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);

	riscv_get_register(target, &reg_vtype, CSR_VTYPE + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vl, CSR_VL + GDB_REGNO_CSR0);
	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
		return ERROR_FAIL;

	LOG_DEBUG("new_reg_vtype: 0x%lx, new_reg_vl: 0x%lx", (long unsigned int)reg_vtype, (long unsigned int)reg_vl);
	return ERROR_OK;
}

/*
A. Backup vtype CSRs
B. Use vsetvli to set vtype(vsew is set to XLEN).
C. Repeat vext.x.v VLEN/XLEN times to read the vector regieter.
   vext.x.v rd, vs2, rs1  # rd = vs2[rs1]
D. Use vsetvl to restore vtype
*/
int ndsv5_get_vector_register(struct target *target, enum gdb_regno r, char *pRegValue)
{
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vstart = 0;
	uint64_t vector_vl = 0, vector_value = 0;
	uint32_t i, j;
	int result;
	char *p_vector_value;
	uint64_t mstatus;
	if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
		LOG_ERROR("read mstatus error");

	if ((mstatus & MSTATUS_VS) == 0 && \
	    register_write_direct(target, GDB_REGNO_MSTATUS, set_field(mstatus, MSTATUS_VS, 1)) != ERROR_OK)
		LOG_ERROR("cannot write mstatus_vs");

	riscv_get_register(target, &reg_vtype, CSR_VTYPE + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vl, CSR_VL + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vstart, CSR_VSTART + GDB_REGNO_CSR0);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_vsetvli(&program, GDB_REGNO_S0, nds32->nds_vector_SEW);
	result = riscv_program_exec(&program, target);
	if (result != ERROR_OK) {
		LOG_ERROR("riscv_program_vsetvli ERROR !!");
		return ERROR_FAIL;
	}
	if (register_read_direct(target, &vector_vl, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;
	if (vector_vl != nds32->nds_vector_vl) {
		LOG_ERROR("vector_vl != nds32->nds_vector_vl !!");
		return ERROR_FAIL;
	}

	register_write_direct(target, CSR_VSTART + GDB_REGNO_CSR0, 0);  /* set vstart to 0 */
	for (i = 0; i < nds32->nds_vector_vl; i++) {
		riscv_program_init(&program, target);
		riscv_program_vmv_x_s(&program, GDB_REGNO_S0, r);
		riscv_program_vslide1down_vx(&program, r, r, GDB_REGNO_S0);
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK) {
			LOG_ERROR("riscv_program_vmv_x_s ERROR !!");
			return ERROR_FAIL;
		}
		/* Read S0 */
		if (register_read_direct(target, &vector_value, GDB_REGNO_S0) != ERROR_OK) {
			LOG_ERROR("Read S0 ERROR !!");
			return ERROR_FAIL;
		}
		p_vector_value = (char *)&vector_value;
		/* LOG_DEBUG("vector_value = 0x%lx", (long unsigned int)vector_value); */
		for (j = 0; j < (nds32->nds_vector_SEW/8); j++)
			*pRegValue++ = *p_vector_value++;
	}
	/* Restore vtype & vl */
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);
	riscv_set_register(target, CSR_VSTART + GDB_REGNO_CSR0, reg_vstart);
	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/*
A. Backup vstart, vl, vtype CSRs
B. Set vstart to 0.
C. Use vsetvli to set vl and vtype(vsew is set to XLEN, vl is set to VLEN/XLEN,  and vlmul is set to 0(no grouping)).
   vsetvli rd, rs1, vtypei # rd = new vl, rs1 = AVL, vtypei = new vtype setting
   Repeat vslide1down VLEN/XLEN times to write the vector register.
   vslide1down.vx vd, vs2, rs1, vm      # vd[i] = vs2[i+1], vd[vl-1]=x[rs1]
D. Use vsetvl to restore vl and vtype
   vsetvl  rd, rs1, rs2    # rd = new vl, rs1 = AVL, rs2 = new vtype value
E. Restore vstart CSR
*/
int ndsv5_set_vector_register(struct target *target, enum gdb_regno r, char *pRegValue)
{
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vstart = 0;
	uint64_t vector_vl = 0, vector_value = 0;
	uint32_t i, j;
	int result;
	char *p_vector_value;
	uint64_t mstatus;
	if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
		LOG_ERROR("read mstatus error");

	if ((mstatus & MSTATUS_VS) == 0 &&
	    register_write_direct(target, GDB_REGNO_MSTATUS, set_field(mstatus, MSTATUS_VS, 1)) != ERROR_OK)
		LOG_ERROR("cannot write mstatus_vs");

	riscv_get_register(target, &reg_vtype, CSR_VTYPE + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vl, CSR_VL + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vstart, CSR_VSTART + GDB_REGNO_CSR0);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_vsetvli(&program, GDB_REGNO_S0, nds32->nds_vector_SEW);
	result = riscv_program_exec(&program, target);
	if (result != ERROR_OK) {
		LOG_ERROR("riscv_program_vsetvli ERROR !!");
		return ERROR_FAIL;
	}

	if (register_read_direct(target, &vector_vl, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;
	if (vector_vl != nds32->nds_vector_vl) {
		LOG_ERROR("vector_vl != nds32->nds_vector_vl !!");
		return ERROR_FAIL;
	}

	register_write_direct(target, CSR_VSTART + GDB_REGNO_CSR0, 0);  /* set vstart to 0 */
	for (i = 0; i < nds32->nds_vector_vl; i++) {
		p_vector_value = (char *) &vector_value;
		for (j = 0; j < (nds32->nds_vector_SEW/8); j++)
			*p_vector_value++ = *pRegValue++;

		register_write_direct(target, GDB_REGNO_S0, vector_value);

		riscv_program_init(&program, target);
		riscv_program_vslide1down_vx(&program, r, r, GDB_REGNO_S0);
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK) {
			LOG_ERROR("riscv_program_vslide1down_vx ERROR !!");
			return ERROR_FAIL;
		}
	}
	/* Restore vtype & vl & vstart */
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);
	riscv_set_register(target, CSR_VSTART + GDB_REGNO_CSR0, reg_vstart);
	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

void ndsv5_decode_csr(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ GDB_REGNO_MSTATUS, MSTATUS_UIE, "uie" },
		{ GDB_REGNO_DCSR, DCSR_XDEBUGVER, "xdebugver" },
		{ GDB_REGNO_DCSR, DCSR_NDRESET, "ndreset" },
		{ GDB_REGNO_DCSR, DCSR_FULLRESET, "fullreset" },
		{ GDB_REGNO_DCSR, DCSR_EBREAKM, "ebreakm" },
		{ GDB_REGNO_DCSR, DCSR_EBREAKH, "ebreakh" },
		{ GDB_REGNO_DCSR, DCSR_EBREAKS, "ebreaks" },
		{ GDB_REGNO_DCSR, DCSR_EBREAKU, "ebreaku" },
		{ GDB_REGNO_DCSR, CSR_DCSR_STEPIE, "stepie" },
		{ GDB_REGNO_DCSR, DCSR_STOPCYCLE, "stopcycle" },
		{ GDB_REGNO_DCSR, DCSR_STOPTIME, "stoptime" },
		{ GDB_REGNO_DCSR, DCSR_CAUSE, "cause" },
		{ GDB_REGNO_DCSR, DCSR_DEBUGINT, "debugint" },
		{ GDB_REGNO_DCSR, DCSR_HALT, "halt" },
		{ GDB_REGNO_DCSR, DCSR_STEP, "step" },
		{ GDB_REGNO_DCSR, DCSR_PRV, "prv" },
	};

	text[0] = 0;
	for (unsigned i = 0; i < DIM(description); i++) {
		if (description[i].address == address) {
			uint64_t mask = description[i].mask;
			unsigned value = get_field(data, mask);
			if (value) {
				if (i > 0)
					*(text++) = ' ';
				if (mask & (mask >> 1)) {
					/* If the field is more than 1 bit wide. */
					sprintf(text, "%s=0x%x", description[i].name, value);
				} else {
					strcpy(text, description[i].name);
				}
				text += strlen(text);
			}
		}
	}
}

int read_memory_bus_v1_opt(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	target_addr_t next_address, end_address, cur_address;
	uint32_t value, if_last, read_cnt, i, batch_index;
	int64_t dmi_out;
	size_t *pindex_read = malloc(sizeof(size_t) * nds_jtag_max_scans);
	RISCV013_INFO(info);

read_memory_bus_v1_opt_retry:
	LOG_DEBUG("info->dmi_busy_delay=0x%x, info->ac_busy_delay=0x%x", info->dmi_busy_delay, info->ac_busy_delay);
	next_address = address;
	end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs = set_field(0, DMI_SBCS_SBREADONADDR, 1);
		sbcs |= sb_sbaccess(size);
		sbcs = set_field(sbcs, DMI_SBCS_SBAUTOINCREMENT, 1);
		sbcs = set_field(sbcs, DMI_SBCS_SBREADONDATA, count > 1);
		dmi_write(target, DMI_SBCS, sbcs);

		/* This address write will trigger the first read. */
		sb_write_address(target, next_address);

		if (info->bus_master_read_delay) {
			jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("Failed to scan idle sequence");
				return ERROR_FAIL;
			}
		}
		if (busmode_batch != NULL) {
			riscv_batch_free(busmode_batch);
			busmode_batch = NULL;
		}
		busmode_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		//LOG_DEBUG("nds_jtag_max_scans=0x%x", nds_jtag_max_scans);

		if_last = 0;
		read_cnt = 0;
		batch_index = 0;
		cur_address = next_address;
		for (i = (next_address - address) / size; i < count; i++) {
			if (busmode_batch->used_scans > (busmode_batch->allocated_scans - 8)) {
				if_last = 1;
			} else if (i == (count-1)) {
				if_last = 1;
			}

			if (if_last) {
				sbcs = set_field(sbcs, DMI_SBCS_SBREADONDATA, 0);
				//dmi_write(target, DMI_SBCS, sbcs);
				riscv_batch_add_dmi_write(busmode_batch, DMI_SBCS, sbcs);
			}
			if (size > 12) {
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DMI_SBDATA3);
			}
			if (size > 8) {
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DMI_SBDATA2);
			}
			if (size > 4) {
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DMI_SBDATA1);
			}
			pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DMI_SBDATA0);
			read_cnt ++;
			next_address += size;
			if (if_last) {
				break;
			}
		}

		if (riscv_batch_run(busmode_batch) != ERROR_OK) {
			LOG_DEBUG("riscv_batch_run FAIL, busmode_batch->idle_count=0x%x", (unsigned int)busmode_batch->idle_count);
			increase_dmi_busy_delay(target);
			goto read_memory_bus_v1_opt_retry;
		} else {
			LOG_DEBUG("riscv_batch_run OK");
		}

		batch_index = 0;
		for (i=0; i<read_cnt; i++) {
			riscv_addr_t offset = cur_address - address;
			if (size > 12) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + offset + i * size + 12, value, 4);
				log_memory_access(cur_address + i * size + 12, value, 4, true);
			}
			if (size > 8) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + offset + i * size + 8, value, 4);
				log_memory_access(cur_address + i * size + 8, value, 4, true);
			}
			if (size > 4) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + offset + i * size + 4, value, 4);
				log_memory_access(cur_address + i * size + 4, value, 4, true);
			}
			dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
			value = get_field(dmi_out, DTM_DMI_DATA);
			write_to_buf(buffer + offset + i * size, value, MIN(size, 4));
			log_memory_access(cur_address + i * size, value, 4, true);
		}

		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			LOG_DEBUG("DMI_SBCS_SBBUSYERROR");
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);
			increase_dmi_busy_delay(target);
			goto read_memory_bus_v1_opt_retry;
		}
		if (get_field(sbcs, DMI_SBCS_SBERROR)) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			unsigned sb_error = get_field(sbcs, DMI_SBCS_SBERROR);
			NDS32_LOG("<-- DMI_SBCS_SBERROR = 0x%x, address = 0x%lx -->", sb_error, (long unsigned int)address);
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			return ERROR_FAIL;
			//increase_dmi_busy_delay(target);
			//goto read_memory_bus_v1_opt_retry;
		}
	}
	return ERROR_OK;
}

#if _NDS_MEM_Q_ACCESS_
int ndsv5_read_memory_quick_access(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	riscv_select_current_hart(target);

	target_addr_t read_addr = address;
	LOG_DEBUG("reading start: %d words of %d bytes from 0x%" TARGET_PRIxADDR, count, size, address);
	struct riscv_program program;
ndsv5_read_memory_quick_access_retry:
	riscv_program_init(&program, target);

	riscv_program_fence(&program);
	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
	}

	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	}
	dmi_write(target, DMI_DATA0, read_addr);
	dmi_write(target, DMI_DATA1, read_addr >> 32);

	if (riscv_program_exec(&program, target) != ERROR_OK) {
		/* quick_access mode, if target state from freerun to halt */
		if (nds_dmi_quick_access_ena) {
			if ( ((nds_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) == CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, read_memory_retry");
				goto ndsv5_read_memory_quick_access_retry;
			}
		}
	}
	uint32_t value;
	dmi_read(target, &value, DMI_DATA0);
	LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08lx", address, (long)value);
	uint32_t value_h = 0;
	if (size == 8) {
		dmi_read(target, &value_h, DMI_DATA1);
		LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08lx", address+4, (long)value_h);
	}
	switch (size) {
	case 1:
		buffer[0] = value;
		break;
	case 2:
		buffer[0] = value;
		buffer[1] = value >> 8;
		break;
	case 8:
		buffer[4] = value_h;
		buffer[5] = value_h >> 8;
		buffer[6] = value_h >> 16;
		buffer[7] = value_h >> 24;
	case 4:
		buffer[0] = value;
		buffer[1] = value >> 8;
		buffer[2] = value >> 16;
		buffer[3] = value >> 24;
		break;
	default:
		LOG_ERROR("unsupported access size: %d", size);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int ndsv5_probe_pc_quick_access(struct target *target, uint64_t *pc_value)
{
	return ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DPC, pc_value);
}
#endif /* _NDS_MEM_Q_ACCESS_ */

int write_memory_bus_v1_opt(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	target_addr_t next_address, end_address;
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DMI_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DMI_SBCS, sbcs);

write_memory_bus_v1_opt_retry:
	LOG_DEBUG("info->dmi_busy_delay=0x%x, info->ac_busy_delay=0x%x", info->dmi_busy_delay, info->ac_busy_delay);
	next_address = address;
	end_address = address + count * size;

	sb_write_address(target, next_address);
	while (next_address < end_address) {
		if (busmode_batch != NULL) {
			riscv_batch_free(busmode_batch);
			busmode_batch = NULL;
		}
		busmode_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		//LOG_DEBUG("nds_jtag_max_scans=0x%x", nds_jtag_max_scans);

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			// if (riscv_batch_full(busmode_batch))
			// check allocated_scans full
			if (busmode_batch->used_scans > (busmode_batch->allocated_scans - 6))
					break;

			const uint8_t *p = buffer + i * size;
			if (size > 12)
				riscv_batch_add_dmi_write(busmode_batch, DMI_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));
			if (size > 8)
				riscv_batch_add_dmi_write(busmode_batch, DMI_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				riscv_batch_add_dmi_write(busmode_batch, DMI_SBDATA1,
						((uint32_t) p[4]) |
						(((uint32_t) p[5]) << 8) |
						(((uint32_t) p[6]) << 16) |
						(((uint32_t) p[7]) << 24));
			uint32_t value = p[0];
			if (size > 2) {
				value |= ((uint32_t) p[2]) << 16;
				value |= ((uint32_t) p[3]) << 24;
			}
			if (size > 1)
				value |= ((uint32_t) p[1]) << 8;
			riscv_batch_add_dmi_write(busmode_batch, DMI_SBDATA0, value);
			log_memory_access(address + i * size, value, size, false);
			next_address += size;
			//LOG_DEBUG("next_address=0x%x", (unsigned int)next_address);
		}

		if (riscv_batch_run(busmode_batch) != ERROR_OK) {
			LOG_DEBUG("riscv_batch_run FAIL, busmode_batch->idle_count=0x%x", (unsigned int)busmode_batch->idle_count);
			increase_dmi_busy_delay(target);
			goto write_memory_bus_v1_opt_retry;
		} else {
			LOG_DEBUG("riscv_batch_run OK");
		}

		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			LOG_DEBUG("DMI_SBCS_SBBUSYERROR");
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);
			increase_dmi_busy_delay(target);
			goto write_memory_bus_v1_opt_retry;
		}
		if (get_field(sbcs, DMI_SBCS_SBERROR)) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			unsigned sb_error = get_field(sbcs, DMI_SBCS_SBERROR);
			NDS32_LOG("<-- DMI_SBCS_SBERROR = 0x%x, address = 0x%lx -->", sb_error, (long unsigned int)address);
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			return ERROR_FAIL;
			//increase_dmi_busy_delay(target);
			//goto write_memory_bus_v1_opt_retry;
		}
	}
	return ERROR_OK;
}

extern INSN_CODE_T_V5* (*gen_get_value_code) (char* name, unsigned index);
extern INSN_CODE_T_V5* (*gen_set_value_code) (char* name, unsigned index);

int nds_ace_enable(struct target *target)
{
	struct riscv_program program;
	riscv_program_init(&program, target);

	// Assembly code used to enable ACE
	//  7d0022f3          	csrr	t0,mmisc_ctl
	//  0012c293          	xori	t0,t0,1
	//  0102e293            ori	t0,t0,16
	//  7d029073          	csrw	mmisc_ctl,t0
	riscv_program_insert(&program, 0x7d0022f3);
	riscv_program_insert(&program, 0x0012c293);
	riscv_program_insert(&program, 0x0102e293);
	riscv_program_insert(&program, 0x7d029073);

	// run the program
	int exec_out = riscv_program_exec(&program, target);

	if (exec_out != ERROR_OK) {
		LOG_ERROR("Unable to execute the program to enable ACR's CSR");
		return exec_out;
	} else
		return ERROR_OK;
};

int nds_ace_get_reg(struct reg *reg)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	bool is_rv64 = (64 == riscv_xlen(target)) ? true : false;
	uint32_t reg_bytes = is_rv64 ? 8ul : 4ul;

	if (isAceCsrEnable == false) {
		nds_ace_enable(target);
		isAceCsrEnable = true;
	}

	// Backup temp register (x5, x6, x7)
	// x5: high part
	// x6: low part
	// x7: ACM's address
	riscv_reg_t s0, s1, s2;
	riscv_get_register(target, &s0, GDB_REGNO_T0);
	riscv_get_register(target, &s1, GDB_REGNO_T1);
	riscv_get_register(target, &s2, GDB_REGNO_T2);

	// Get type_name and register index
	char* type_name = (char*) reg->reg_data_type->id;
	// Format : "%s_%d", type_name, idx
	char* reg_name = (char*) reg->name;
	unsigned int reg_idx = atoi(strrchr(reg_name, '_') + 1);

	LOG_DEBUG("type_name = %s, reg_name = %s, reg_idx = %d, size = %d",
			type_name, reg_name, reg_idx, reg->size);

	// Generate code to read value from ACR/ACM
	INSN_CODE_T_V5* insn_code = gen_get_value_code(type_name, reg_idx);

	// Update the value
	// [NOTE] The pointer of value is updated in following code.
	//        If assigning value to reg->value after these updates,
	//        reg->value would not be the initial address of *value.
	//        So, incorrect value is assigned to reg->value. To avoid
	//        this, we assign *value to reg->value initially.
	char* value = reg->value;

	bool init_acm_addr = false;
	// Execute the code generated by gen_get_value_code() iteratively
	for (unsigned i = 0; i < insn_code->num; i++) {
		// Initialize
		struct riscv_program program;
		riscv_program_init(&program, target);

		// For ACM utility instruction, write memory address to GDB_REGNO_XPR0 + 7
		if (init_acm_addr == false &&
				((insn_code->code + i)->version == acm_io1 ||
				 (insn_code->code + i)->version == acm_io2)) {
			riscv_program_li(&program, GDB_REGNO_T2, reg_idx);
			init_acm_addr = true;
			if (is_rv64) {
				LOG_DEBUG("acm_addr = 0x%016" PRIx64 " feed into $t2", (uint64_t) reg_idx);
			} else {
				LOG_DEBUG("acm_addr= 0x%08" PRIx32 " feed into $t2", (uint32_t) reg_idx);
			}
		}

		// insert utility instruction to program buffer
		unsigned insn = (insn_code->code + i)->insn;
		riscv_program_insert(&program, insn);
		LOG_DEBUG("read utility instruction (offset: %d) = 0x%08" PRIx32, i, insn);
		LOG_DEBUG("read utility instruction version %d", (insn_code->code + i)->version);

		// determine the number of GPRs used to read/write data from/to ACR/ACM
		bool isTwoGPR = false;
		if ((insn_code->code + i)->version == acr_io2 ||
				(insn_code->code + i)->version == acm_io2) {
			isTwoGPR = true;
		}

		// execute the code stored in program buffer
		int exec_out = riscv_program_exec(&program, target);
		if (exec_out != ERROR_OK) {
			LOG_ERROR("Unable to execute ACE utility program");

			// Restore temp register
			riscv_set_register(target, GDB_REGNO_T0, s0);
			riscv_set_register(target, GDB_REGNO_T1, s1);
			riscv_set_register(target, GDB_REGNO_T2, s2);
			return exec_out;
		}

		// read value from program buffer
		if (isTwoGPR == false) {
			riscv_reg_t reg_value;
			riscv_get_register(target, &reg_value, GDB_REGNO_T0);
			memcpy(value, &reg_value, reg_bytes);
			value += reg_bytes;
			if (is_rv64) {
				LOG_DEBUG("reg_value = 0x%016" PRIx64 " read from program buffer", (uint64_t) reg_value);
			} else {
				LOG_DEBUG("reg_value = 0x%08" PRIx32 " read from program buffer", (uint32_t) reg_value);
			}
		} else {
			riscv_reg_t high = 0, low = 0;
			riscv_get_register(target, &high, GDB_REGNO_T0);
			riscv_get_register(target, &low,  GDB_REGNO_T1);     
			if (is_rv64) {
				LOG_DEBUG("reg_value (high) = 0x%016" PRIx64 " read from program buffer", (uint64_t) high);
				LOG_DEBUG("reg_value (low)  = 0x%016" PRIx64 " read from program buffer", (uint64_t) low);
			} else {
				LOG_DEBUG("reg_value (high) = 0x%08" PRIx32 " read from program buffer", (uint32_t) high);
				LOG_DEBUG("reg_value (low)  = 0x%08" PRIx32 " read from program buffer", (uint32_t) low);
			}
			memcpy(value, &low, reg_bytes);
			value += reg_bytes;
			memcpy(value, &high, reg_bytes);
			value += reg_bytes;
		}
	}

	// Restore temp register
	riscv_set_register(target, GDB_REGNO_T0, s0);
	riscv_set_register(target, GDB_REGNO_T1, s1);
	riscv_set_register(target, GDB_REGNO_T2, s2);

	return ERROR_OK;
}

int nds_ace_set_reg(struct reg *reg, unsigned char *val)
{
	int exec_out;
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	bool is_rv64 = (64 == riscv_xlen(target)) ? true : false;
	uint32_t reg_bytes = is_rv64 ? 8ul : 4ul;

	if (isAceCsrEnable == false) {
		nds_ace_enable(target);
		isAceCsrEnable = true;
	}

	// Backup temp register (x5, x6, x7, x28)
	// x5: high part
	// x6: low part
	// x7: ACM's address
	// x28(t3): temp reg to write to xlen(64) gpr
	riscv_reg_t s0, s1, s2, t3;
	riscv_get_register(target, &s0, GDB_REGNO_T0);
	riscv_get_register(target, &s1, GDB_REGNO_T1);
	riscv_get_register(target, &s2, GDB_REGNO_T2);
	if (is_rv64) {
		riscv_get_register(target, &t3, GDB_REGNO_T3);
	}

	// Get acr_name and register index
	char* type_name = (char*) reg->reg_data_type->id;
	// Format : "%s_%d", type_name, idx
	char* reg_name = (char*) reg->name;
	unsigned int reg_idx = atoi(strrchr(reg_name, '_') + 1);

	LOG_DEBUG("type_name = %s, reg_name = %s, reg_idx = %d, size = %d",
			type_name, reg_name, reg_idx, reg->size);

	/* Allocate buffer which is the multiple of register size */
	unsigned int ByteSize = (!reg->size%8) ? reg->size/8 : (reg->size/8) + 1;
	unsigned int rounds = (ByteSize % reg_bytes) ? 
												ByteSize / reg_bytes + 1 : ByteSize / reg_bytes;
	char* buffer = (char*) alloca (rounds * reg_bytes);
	memset(buffer, 0, rounds * reg_bytes);
	memcpy(buffer, val, ByteSize);
	char* value = buffer;	/* this pointer will be increased when extracting value partially */

	// Generate code to write value to ACR/ACM
	INSN_CODE_T_V5* insn_code = gen_set_value_code(type_name, reg_idx);

	bool init_acm_addr = false;
	// Execute the code generated by gen_get_value_code() iteratively
	for (unsigned i = 0; i < insn_code->num; i++) {
		// Initialize
		struct riscv_program program;
		riscv_program_init(&program, target);

		LOG_DEBUG("write utility instruction version %d", (insn_code->code + i)->version);
		// determine the number of GPRs used to read/write data from/to ACR/ACM
		bool isTwoGPR = false;
		if ((insn_code->code + i)->version == acr_io2 ||
				(insn_code->code + i)->version == acm_io2) {
			isTwoGPR = true;
		}

		// For ACM utility instruction, write memory address to GDB_REGNO_XPR0 + 7
		if (init_acm_addr == false &&
				((insn_code->code + i)->version == acm_io1 ||
				 (insn_code->code + i)->version == acm_io2)) {
			riscv_program_li(&program, GDB_REGNO_T2, reg_idx);	/* 2 entry */
			init_acm_addr = true;
			if (is_rv64) {
				// 7 insn entry + ebreak entry fills up program buffer
				exec_out = riscv_program_exec(&program, target);
				if (exec_out != ERROR_OK) {
					LOG_ERROR("Unable to execute program");
					goto error;
				}
				riscv_program_init(&program, target);
				LOG_DEBUG("acm_addr = 0x%016" PRIx64 " feed into $t2", (uint64_t) reg_idx);
			} else {
				LOG_DEBUG("acm_addr = 0x%08" PRIx32 " feed into $t2", (uint32_t) reg_idx);
			}
		}

		// write given value string to S0/1
		if (isTwoGPR == false) {
			// Extract part of value from given value string
			riscv_reg_t reg_value = 0;
			memcpy(&reg_value, value, reg_bytes);
			value += reg_bytes;
			//riscv_program_write_ram(&program, output + 4, 0);
			if (is_rv64) {
				riscv_program_li64(&program, GDB_REGNO_T0, GDB_REGNO_T3, reg_value);	/* 7 entry */ 

				// 7 insn entry + ebreak entry fills up program buffer
				exec_out = riscv_program_exec(&program, target);
				if (exec_out != ERROR_OK) { 
					LOG_ERROR("Unable to execute program");
					goto error;
				}
				riscv_program_init(&program, target);
				LOG_DEBUG("reg_value = 0x%016" PRIx64 " feed into $t0", (uint64_t) reg_value);
			} else {
				riscv_program_li(&program, GDB_REGNO_T0, reg_value);	/* 2 entry */
				LOG_DEBUG("reg_value = 0x%08" PRIx32 " feed into $t0", (uint32_t) reg_value);
			}
		} else {
			// Extract part of value from given value string
			// [NOTE] the order of val is from low bit order
			//        e.g., for value of 0x111222333444555666777888999
			//        the traversing order is from 999 --> 888 --> ...
			riscv_reg_t high = 0, low = 0;
			memcpy(&low, value, reg_bytes);
			value += reg_bytes;
			memcpy(&high, value, reg_bytes);
			value += reg_bytes;

			if (is_rv64) {
				riscv_program_li64(&program, GDB_REGNO_T0, GDB_REGNO_T3, high);	/* 7 entry */
				// 7 insn entry + ebreak entry fills up program buffer
				exec_out = riscv_program_exec(&program, target);
				if (exec_out != ERROR_OK) { 
					LOG_ERROR("Unable to execute program");
					goto error;
				}
				riscv_program_init(&program, target);

				riscv_program_li64(&program, GDB_REGNO_T1, GDB_REGNO_T3, low);	/* 7 entry */
				// 7 insn entry + ebreak entry fills up program buffer
				exec_out = riscv_program_exec(&program, target);
				if (exec_out != ERROR_OK) { 
					LOG_ERROR("Unable to execute program");
					goto error;
				}
				riscv_program_init(&program, target);

				LOG_DEBUG("reg_value = 0x%016" PRIx64 " feed into $t0", (uint64_t) high);
				LOG_DEBUG("reg_value = 0x%016" PRIx64 " feed into $t1", (uint64_t) low);
			} else {
				riscv_program_li(&program, GDB_REGNO_T0, high);	/* 2 entry */
				riscv_program_li(&program, GDB_REGNO_T1, low);	/* 2 entry */
				LOG_DEBUG("reg_value = 0x%08" PRIx32 " feed into $t0", (uint32_t) high);
				LOG_DEBUG("reg_value = 0x%08" PRIx32 " feed into $t1", (uint32_t) low);
			}
		}

		//riscv_program_fence(&program);
		unsigned insn = (insn_code->code + i)->insn;
		riscv_program_insert(&program, insn);
		LOG_DEBUG("write utility instruction (offset:%d) = %x", i, insn);

		// execute the code stored in program buffer
		exec_out = riscv_program_exec(&program, target);
		if (exec_out != ERROR_OK) { 
			LOG_ERROR("Unable to execute program");
			goto error;
		}

		riscv_reg_t high_s0, low_s1;
		riscv_get_register(target, &high_s0, GDB_REGNO_T0);
		riscv_get_register(target, &low_s1,  GDB_REGNO_T1);
		if (is_rv64) {
			LOG_DEBUG("(confirm)reg_value = 0x%016" PRIx64 " feed into $t0", (uint64_t) high_s0);
			LOG_DEBUG("(confirm)reg_value = 0x%016" PRIx64 " feed into $t1", (uint64_t) low_s1);
		} else {
			LOG_DEBUG("(confirm)reg_value = 0x%08" PRIx32 " feed into $t0", (uint32_t) high_s0);
			LOG_DEBUG("(confirm)reg_value = 0x%08" PRIx32 " feed into $t1", (uint32_t) low_s1);
		}
	}

	memcpy(reg->value, val, ByteSize);

	// Restore temp register
	riscv_set_register(target, GDB_REGNO_T0, s0);
	riscv_set_register(target, GDB_REGNO_T1, s1);
	riscv_set_register(target, GDB_REGNO_T2, s2);
	if (is_rv64) {
		riscv_set_register(target, GDB_REGNO_T3, t3);
	}

	return ERROR_OK;

error:
	// Restore temp register
	riscv_set_register(target, GDB_REGNO_T0, s0);
	riscv_set_register(target, GDB_REGNO_T1, s1);
	riscv_set_register(target, GDB_REGNO_T2, s2);
	if (is_rv64) {
		riscv_set_register(target, GDB_REGNO_T3, t3);
	}
	return exec_out;
}

struct reg_arch_type nds_ace_reg_access_type = {
	.get = nds_ace_get_reg,
	.set = nds_ace_set_reg
};

int ndsv5_get_delay_count(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	LOG_DEBUG("dmi_busy_delay=%d", info->dmi_busy_delay);
	return info->dmi_busy_delay;
}

#endif /* _NDS_V5_ONLY_ */
/********************************************************************/

