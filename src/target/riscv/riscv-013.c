/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include <helper/log.h>
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "helper/list.h"
#include "riscv.h"
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

uint32_t nds_is_rvv_0_8;

#if _NDS_V5_ONLY_

#include "ndsv5.h"
#include "ndsv5-013.h"
#include "target/nds32_new/nds32_log.h"
#include <target/smp.h>
#define DIM(x)          (sizeof(x)/sizeof(*x))
static void write_to_buf(uint8_t *buffer, uint64_t value, unsigned size);

static const char *const NDS_MEMORY_ACCESS_NAME[] = {
	"BUS",
	"CPU",
};

bool reset_halt;
unsigned acr_reg_count_v5;
unsigned acr_type_count_v5;

#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
static struct riscv_batch *write_debug_buffer_batch;
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF */

#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
static uint64_t backup_debug_buffer[RISCV_MAX_HARTS][16];
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF */

static bool isAceCsrEnable;
static struct riscv_batch *busmode_batch;
uint32_t nds_sys_bus_supported;

static uint32_t *p_etb_buf_start = NULL, *p_etb_buf_end;
static uint32_t *p_etb_wptr;
uint32_t nds_tracer_on;
uint32_t nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_OFF;
uint32_t nds_tracer_stop_on_wrap;
uint32_t nds_trTeSyncMax = 4;
uint32_t nds_trTeInstMode = 6;
uint32_t nds_teInhibitSrc = 1;
uint64_t nds_tracer_active_id = 0x01;
uint32_t nds_teInstNoAddrDiff;
uint32_t nds_timestamp_on, nds_trTsControl;
uint32_t nds_tracer_multiplexer, nds_tracer_capability;
uint32_t nds_trTeFilteriMatchInst;
int ndsv5_tracer_capability_check(struct target *target);

uint32_t TB_RAM_SIZE = 0x2000;
#define TRACER_TMP_BUFSIZE   0x100000  /* 1MB */

#define TRACER_VERSION       1         /* version number: 0~255 */
#endif /* _NDS_V5_ONLY_ */

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step, bool use_hasel);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int rid);
static int riscv013_set_register(struct target *target, int regid, uint64_t value);
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
static int riscv013_invalidate_cached_debug_buffer(struct target *target);
static int riscv013_execute_debug_buffer(struct target *target);
static void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
static void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a);
static int riscv013_dmi_write_u64_bits(struct target *target);
static void riscv013_fill_dmi_nop_u64(struct target *target, char *buf);
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int riscv013_test_sba_config_reg(struct target *target, target_addr_t legal_address,
		uint32_t num_words, target_addr_t illegal_address, bool run_sbbusyerror_test);
void write_memory_sba_simple(struct target *target, target_addr_t addr, uint32_t *write_data,
		uint32_t write_size, uint32_t sbcs);
void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs);

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

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

#if _NDS_V5_ONLY_
	int ndsv5_hart_count;
#endif


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

	yes_no_maybe_t has_aampostincrement;

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
	assert(info);
	assert(info->version_specific);
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
		if (!dm)
			return NULL;
		dm->abs_chain_position = abs_chain_position;
		dm->current_hartid = -1;
		dm->hart_count = -1;

#if _NDS_V5_ONLY_
		dm->ndsv5_hart_count = -1;
#endif

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
	if (!target_entry) {
		info->dm = NULL;
		return NULL;
	}
	target_entry->target = target;
	list_add(&target_entry->list, &dm->target_list);

	return dm;
}

static uint32_t set_hartsel(uint32_t initial, uint32_t index)
{
	initial &= ~DM_DMCONTROL_HARTSELLO;
	initial &= ~DM_DMCONTROL_HARTSELHI;

	uint32_t index_lo = index & ((1 << DM_DMCONTROL_HARTSELLO_LENGTH) - 1);
	initial |= index_lo << DM_DMCONTROL_HARTSELLO_OFFSET;
	uint32_t index_hi = index >> DM_DMCONTROL_HARTSELLO_LENGTH;
	assert(index_hi < 1 << DM_DMCONTROL_HARTSELHI_LENGTH);
	initial |= index_hi << DM_DMCONTROL_HARTSELHI_OFFSET;

	return initial;
}

static void decode_dmi(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ DM_DMCONTROL, DM_DMCONTROL_HALTREQ, "haltreq" },
		{ DM_DMCONTROL, DM_DMCONTROL_RESUMEREQ, "resumereq" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTRESET, "hartreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_HASEL, "hasel" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELHI, "hartselhi" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELLO, "hartsello" },
		{ DM_DMCONTROL, DM_DMCONTROL_NDMRESET, "ndmreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_DMACTIVE, "dmactive" },
		{ DM_DMCONTROL, DM_DMCONTROL_ACKHAVERESET, "ackhavereset" },
#if _NDS_V5_ONLY_
		{ DM_DMCONTROL, DM_DMCONTROL_SETRESETHALTREQ, "setresethaltreq" },
		{ DM_DMCONTROL, DM_DMCONTROL_CLRRESETHALTREQ, "clrresethaltreq" },
#endif

		{ DM_DMSTATUS, DM_DMSTATUS_IMPEBREAK, "impebreak" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHAVERESET, "allhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHAVERESET, "anyhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRESUMEACK, "allresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRESUMEACK, "anyresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLNONEXISTENT, "allnonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYNONEXISTENT, "anynonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLUNAVAIL, "allunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYUNAVAIL, "anyunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRUNNING, "allrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRUNNING, "anyrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHALTED, "allhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHALTED, "anyhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHENTICATED, "authenticated" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHBUSY, "authbusy" },
		{ DM_DMSTATUS, DM_DMSTATUS_HASRESETHALTREQ, "hasresethaltreq" },
		{ DM_DMSTATUS, DM_DMSTATUS_CONFSTRPTRVALID, "confstrptrvalid" },
		{ DM_DMSTATUS, DM_DMSTATUS_VERSION, "version" },

		{ DM_ABSTRACTCS, DM_ABSTRACTCS_PROGBUFSIZE, "progbufsize" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_BUSY, "busy" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR, "cmderr" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_DATACOUNT, "datacount" },

		{ DM_COMMAND, DM_COMMAND_CMDTYPE, "cmdtype" },

		{ DM_SBCS, DM_SBCS_SBVERSION, "sbversion" },
		{ DM_SBCS, DM_SBCS_SBBUSYERROR, "sbbusyerror" },
		{ DM_SBCS, DM_SBCS_SBBUSY, "sbbusy" },
		{ DM_SBCS, DM_SBCS_SBREADONADDR, "sbreadonaddr" },
		{ DM_SBCS, DM_SBCS_SBACCESS, "sbaccess" },
		{ DM_SBCS, DM_SBCS_SBAUTOINCREMENT, "sbautoincrement" },
		{ DM_SBCS, DM_SBCS_SBREADONDATA, "sbreadondata" },
		{ DM_SBCS, DM_SBCS_SBERROR, "sberror" },
		{ DM_SBCS, DM_SBCS_SBASIZE, "sbasize" },
		{ DM_SBCS, DM_SBCS_SBACCESS128, "sbaccess128" },
		{ DM_SBCS, DM_SBCS_SBACCESS64, "sbaccess64" },
		{ DM_SBCS, DM_SBCS_SBACCESS32, "sbaccess32" },
		{ DM_SBCS, DM_SBCS_SBACCESS16, "sbaccess16" },
		{ DM_SBCS, DM_SBCS_SBACCESS8, "sbaccess8" },
#if _NDS_V5_ONLY_
		/*{ AC_ACCESS_REGISTER, DM_COMMAND_AC_QUICK_ACCESS, "quick" },*/
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_POSTEXEC, "postexec" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_AARSIZE, "size" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_TRANSFER, "transfer" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_WRITE, "write" },
		{ AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_REGNO, "regno" },
		{ DM_DMCS2, DM_DMCS2_HGSELECT, "hgselect" },
		{ DM_DMCS2, DM_DMCS2_HGWRITE, "hgwrite" },
		{ DM_DMCS2, DM_DMCS2_GROUP, "group" },
		{ DM_DMCS2, DM_DMCS2_DMEXTTRIGGER, "dmexttrigger" },
		{ DM_DMCS2, DM_DMCS2_GROUPTYPE, "grouptype" },
#endif /* _NDS_V5_ONLY_ */
	};

	text[0] = 0;
	for (unsigned i = 0; i < ARRAY_SIZE(description); i++) {
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
	uint8_t out_value[4] = { 0 };

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
	/* SYNC TODO
	uint8_t in[num_bytes];
	uint8_t out[num_bytes]; */
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

	memset(in, 0, num_bytes);
	memset(out, 0, num_bytes);

	assert(info->abits != 0);

	/* SYNC TODO
	buf_set_u32(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u32(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u32(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);
	*/
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
	if (idle_count < (int)info->dtmcs_idle)
		idle_count = info->dtmcs_idle;
#endif /* _NDS_V5_ONLY_ */

	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		if (data_in)
			*data_in = ~0;
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
 * @param target
 * @param data_in  The data we received from the target.
 * @param dmi_busy_encountered
 *                 If non-NULL, will be updated to reflect whether DMI busy was
 *                 encountered while executing this operation or not.
 * @param dmi_op   The operation to perform (read/write/nop).
 * @param address  The address argument to that operation.
 * @param data_out The data to send to the target.
 * @param timeout_sec
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

	keep_alive();

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
		LOG_ERROR("[%s] DMI operation didn't complete in %d seconds. The target was "
				"either too slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec);
#else
		LOG_ERROR("[%s] DMI operation didn't complete in %d seconds. The target is "
				"either really slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec);
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
			DM_DMSTATUS, 0, timeout_sec, false, true);
	if (result != ERROR_OK)
		return result;
	int dmstatus_version = get_field(*dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2 (0.13) and 3 (1.0), not "
				"%d (dmstatus=0x%x). This error might be caused by a JTAG "
				"signal issue. Try reducing the JTAG clock speed.",
				get_field(*dmstatus, DM_DMSTATUS_VERSION), *dmstatus);
#if _NDS_V5_ONLY_
		assert(0);
#endif /* _NDS_V5_ONLY_ */
	} else if (authenticated && !get_field(*dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
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
		case 128:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 4);
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
		if (dmi_read(target, abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(*abstractcs, DM_ABSTRACTCS_BUSY) == 0)
			return ERROR_OK;

		if (time(NULL) - start > riscv_command_timeout_sec) {
			info->cmderr = get_field(*abstractcs, DM_ABSTRACTCS_CMDERR);
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
		switch (get_field(command, DM_COMMAND_CMDTYPE)) {
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
			write_debug_buffer_batch = riscv_batch_alloc(target, nds_jtag_max_scans,
					info->dmi_busy_delay + info->ac_busy_delay);
		}
		riscv_batch_add_dmi_write(write_debug_buffer_batch, DM_COMMAND, command);
		size_t index_ABSTRACTCS = riscv_batch_add_dmi_read(write_debug_buffer_batch, DM_ABSTRACTCS);

#if _NDS_MEM_Q_ACCESS_
		if (nds_dmi_quick_access_ena)
			index_ABSTRACTCS = riscv_batch_add_dmi_read(write_debug_buffer_batch, DM_ABSTRACTCS);
#endif /* _NDS_MEM_Q_ACCESS_ */

		uint32_t retry_cnt = 0;
		while (retry_cnt < ndsv5_dmi_busy_retry_times) {
			retry_cnt++;
			if (riscv_batch_run(write_debug_buffer_batch) != ERROR_OK) {
				LOG_DEBUG("riscv_batch_run_FAIL");
				increase_dmi_busy_delay(target);
				riscv013_clear_abstract_error(target);
				write_debug_buffer_batch->idle_count = info->dmi_busy_delay + info->ac_busy_delay;
				if (retry_cnt == ndsv5_dmi_busy_retry_times) {
					LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles",
							ndsv5_dmi_busy_retry_times);
					return ERROR_FAIL;
				}
			} else {
				break;
			}
		}
		cs = riscv_batch_get_dmi_read_data(write_debug_buffer_batch, index_ABSTRACTCS);
		riscv_batch_free(write_debug_buffer_batch);
		write_debug_buffer_batch = NULL;

		if (get_field(cs, DM_ABSTRACTCS_BUSY) != 0)
			wait_for_idle(target, (uint32_t *)&cs);
	} else {
		dmi_write(target, DM_COMMAND, command);
		uint32_t abstractcs = 0;
		wait_for_idle(target, &abstractcs);
		dmi_read(target, &cs, DM_ABSTRACTCS);
	}

	info->cmderr = get_field(cs, DM_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, cs);
		/* Clear the error. */
		dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
		return ERROR_FAIL;
	}
#else /* _NDS_JTAG_SCANS_OPTIMIZE_ */

	if (dmi_write_exec(target, DM_COMMAND, command, false) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t abstractcs = 0;
	int result = wait_for_idle(target, &abstractcs);

	info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0 || result != ERROR_OK) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, abstractcs);
		/* Clear the error. */
		dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
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
			dmi_read(target, &v, DM_DATA0 + offset + 1);
			value |= ((uint64_t) v) << 32;
			/* falls through */
		case 32:
			dmi_read(target, &v, DM_DATA0 + offset);
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
			dmi_write(target, DM_DATA0 + offset + 1, value >> 32);
			/* falls through */
		case 32:
			dmi_write(target, DM_DATA0 + offset, value);
	}
	return ERROR_OK;
}

/**
 * @par size in bits
 */
static uint32_t access_register_command(struct target *target, uint32_t number,
		unsigned size, uint32_t flags)
{
	uint32_t command = set_field(0, DM_COMMAND_CMDTYPE, 0);
	switch (size) {
		case 32:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 2);
			break;
		case 64:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 3);
			break;
		default:
			LOG_ERROR("[%s] %d-bit register %s not supported.",
					target_name(target), size, gdb_regno_name(number));
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
	/* read_abstract_reg_number = number; */
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

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
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

	if (result != ERROR_OK) {
		/* This program might have failed if the program buffer is not
		 * writable. */
		info->progbuf_writable = YNM_NO;
		return ERROR_OK;
	}

	uint32_t written;
	if (dmi_read(target, &written, DM_PROGBUF0) != ERROR_OK)
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
		if (register_read_direct(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
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
	} else {
		*mstatus = 0;
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
	SPACE_DM_DATA,
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
			scratch->memory_space = SPACE_DM_DATA;
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
	return target_free_working_area(target, scratch->area);
}

static int scratch_read64(struct target *target, scratch_mem_t *scratch,
		uint64_t *value)
{
	uint32_t v;
	switch (scratch->memory_space) {
		case SPACE_DM_DATA:
			if (dmi_read(target, &v, DM_DATA0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_DATA1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			if (dmi_read(target, &v, DM_PROGBUF0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_PROGBUF1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {0};
				if (read_memory(target, scratch->debug_address, 4, 2, buffer, 4) != ERROR_OK)
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
		case SPACE_DM_DATA:
			dmi_write(target, DM_DATA0 + scratch->debug_address, value);
			dmi_write(target, DM_DATA1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_PROGBUF:
			dmi_write(target, DM_PROGBUF0 + scratch->debug_address, value);
			dmi_write(target, DM_PROGBUF1 + scratch->debug_address, value >> 32);
			riscv013_invalidate_cached_debug_buffer(target);
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

static bool has_sufficient_progbuf(struct target *target, unsigned size)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

	return info->progbufsize + r->impebreak >= size;
}

/**
 * Immediately write the new value to the requested register. This mechanism
 * bypasses any caches.
 */
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
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
	if (result == ERROR_OK || !has_sufficient_progbuf(target, 2) ||
			!riscv_is_halted(target))
		return result;

	struct riscv_program program;
	riscv_program_init(&program, target);

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	scratch_mem_t scratch;
	bool use_scratch = false;
	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			riscv_supports_extension(target, 'D') &&
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

	} else {
		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D'))
				riscv_program_insert(&program, fmv_d_x(number - GDB_REGNO_FPR0, S0));
			else
				riscv_program_insert(&program, fmv_w_x(number - GDB_REGNO_FPR0, S0));
		} else if (number == GDB_REGNO_VTYPE) {
			if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, csrr(S1, CSR_VL)) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvl(ZERO, S1, S0)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (number == GDB_REGNO_VL) {
			/* "The XLEN-bit-wide read-only vl CSR can only be updated by the
			 * vsetvli and vsetvl instructions, and the fault-only-rst vector
			 * load instruction variants." */
			if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, csrr(S1, CSR_VTYPE)) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvl(ZERO, S0, S1)) != ERROR_OK)
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
		if (register_write_direct(target, GDB_REGNO_S0, value) != ERROR_OK)
			return ERROR_FAIL;
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

	return exec_out;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
#if _NDS_USE_SCRIPT_
	if (ndsv5_script_reg_read(value, number) == ERROR_OK)
		return ERROR_OK;
#endif

	int result = register_read_abstract(target, value, number,
			register_size(target, number));

	if (result != ERROR_OK &&
			has_sufficient_progbuf(target, 2) &&
			number > GDB_REGNO_XPR31) {
		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;

		/* Write program to move data into s0. */

		uint64_t mstatus;
		if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D')
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
			} else if (riscv_supports_extension(target, 'D')) {
				riscv_program_insert(&program, fmv_x_d(S0, number - GDB_REGNO_FPR0));
			} else {
				riscv_program_insert(&program, fmv_x_w(S0, number - GDB_REGNO_FPR0));
			}
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
#if 0
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

static int wait_for_authbusy(struct target *target, uint32_t *dmstatus)
{
	time_t start = time(NULL);
	while (1) {
		uint32_t value;
		if (dmstatus_read(target, &value, false) != ERROR_OK)
			return ERROR_FAIL;
		if (dmstatus)
			*dmstatus = value;
		if (!get_field(value, DM_DMSTATUS_AUTHBUSY))
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

			/* turn-off suppressed hsp exception */
			if (nds32->suppressed_hsp_exception) {
				ndsv5_suppressed_hsp_exception(target, false);
				LOG_DEBUG("Disable suppressed hsp exception");
				nds32->suppressed_hsp_exception = false;
			}

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

typedef enum {
	HALTGROUP,
	RESUMEGROUP
} grouptype_t;
static int set_group(struct target *target, bool *supported, unsigned group, grouptype_t grouptype)
{
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->no_group) {
		LOG_DEBUG("no group ON");
		*supported = false;
		return ERROR_OK;
	}
#endif

	uint32_t write_val = DM_DMCS2_HGWRITE;
	assert(group <= 31);
	write_val = set_field(write_val, DM_DMCS2_GROUP, group);
	write_val = set_field(write_val, DM_DMCS2_GROUPTYPE, (grouptype == HALTGROUP) ? 0 : 1);
	if (dmi_write(target, DM_DMCS2, write_val) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t read_val;
	if (dmi_read(target, &read_val, DM_DMCS2) != ERROR_OK)
		return ERROR_FAIL;
#if _NDS_V5_ONLY_
	if (grouptype == HALTGROUP)
		*supported = get_field(read_val, DM_DMCS2_GROUP) == group;
	else
		*supported = get_field(read_val, DM_DMCS2_GROUPTYPE) == 1;
#else
	*supported = get_field(read_val, DM_DMCS2_GROUP) == group;
#endif
	return ERROR_OK;
}

static int discover_vlenb(struct target *target)
{
	RISCV_INFO(r);
	riscv_reg_t vlenb;
	uint64_t mmsc_cfg, cur_mstatus_VS;
	if (register_read_direct(target, &mmsc_cfg, GDB_REGNO_CSR0 + CSR_MMSC_CFG) != ERROR_OK)
		LOG_ERROR("read mmsc_cfg error");

	if ((mmsc_cfg & 0x1000000000)>>36) {
		/* RVV 0.9 & above */
		cur_mstatus_VS = 0x00000600;
		nds_is_rvv_0_8 = 0;
	} else {
		/* RVV 0.8 */
		cur_mstatus_VS = 0x01800000;
		nds_is_rvv_0_8 = 1;
	}
	uint64_t mstatus;
	if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
		LOG_ERROR("read mstatus error");

	if ((mstatus & cur_mstatus_VS) == 0 &&
	   register_write_direct(target, GDB_REGNO_MSTATUS, set_field(mstatus, cur_mstatus_VS, 1)) != ERROR_OK)
		LOG_ERROR("cannot write mstatus_vs");

	if (register_read_direct(target, &vlenb, GDB_REGNO_VLENB) != ERROR_OK) {
		LOG_WARNING("Couldn't read vlenb for %s; vector register access won't work.",
				target_name(target));
		r->vlenb = 0;
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;
		return ERROR_OK;
	}

	r->vlenb = vlenb;

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	nds32->nds_vector_length = r->vlenb;
	LOG_INFO("Vector support with vlenb=%d", r->vlenb);
	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
		return ERROR_FAIL;
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
		LOG_ERROR("[%s] Unsupported DTM version %d. (dtmcontrol=0x%x)",
				target_name(target), get_field(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	/* TODO: This won't be true if there are multiple DMs. */
	info->index = target->coreid;
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcs_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	/* Reset the Debug Module. */
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->was_reset) {
		dmi_write(target, DM_DMCONTROL, 0);
		dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
		dm->was_reset = true;

		/* The DM gets reset, so forget any cached progbuf entries. */
		riscv013_invalidate_cached_debug_buffer(target);
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

#if _NDS_V5_ONLY_
	/* Avoid using HASEL */
	dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
#else
	dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_HARTSELLO |
			DM_DMCONTROL_HARTSELHI | DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_HASEL);
#endif /* _NDS_V5_ONLY_ */


	uint32_t dmcontrol;
	if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(dmcontrol, DM_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	dm->hasel_supported = get_field(dmcontrol, DM_DMCONTROL_HASEL);

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);
	int dmstatus_version = get_field(dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		/* Error was already printed out in dmstatus_read(). */
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	/* Avoid using HASEL */
	info->hartsellen = 0;
#else
	uint32_t hartsel =
		(get_field(dmcontrol, DM_DMCONTROL_HARTSELHI) <<
		 DM_DMCONTROL_HARTSELLO_LENGTH) |
		get_field(dmcontrol, DM_DMCONTROL_HARTSELLO);
	info->hartsellen = 0;
	while (hartsel & 1) {
		info->hartsellen++;
		hartsel >>= 1;
	}
#endif /* _NDS_V5_ONLY_ */
	LOG_DEBUG("hartsellen=%d", info->hartsellen);

	uint32_t hartinfo;
	if (dmi_read(target, &hartinfo, DM_HARTINFO) != ERROR_OK)
		return ERROR_FAIL;

	info->datasize = get_field(hartinfo, DM_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DM_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DM_HARTINFO_DATAADDR);

	if (!get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", dmstatus);
		return ERROR_FAIL;
	}

	if (dmi_read(target, &info->sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

#if _NDS_V5_ONLY_
	ndsv5_dis_cache_busmode = 0;
	if (info->sbcs & 0x1F) {
		int sb_version = get_field(info->sbcs, DM_SBCS_SBVERSION);
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
	if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
		return ERROR_FAIL;
	info->datacount = get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE);

	LOG_INFO("[%s] datacount=%d progbufsize=%d", target_name(target),
			info->datacount, info->progbufsize);

	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DM_DMSTATUS_IMPEBREAK);

	if (!has_sufficient_progbuf(target, 2)) {
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
			if (get_field(s, DM_DMSTATUS_ANYNONEXISTENT))
				break;
			dm->hart_count = i + 1;

			if (get_field(s, DM_DMSTATUS_ANYHAVERESET))
				dmi_write(target, DM_DMCONTROL,
						set_hartsel(DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_ACKHAVERESET, i));

#if _NDS_V5_ONLY_
			/* Enable halt-on-reset */
			if (nds_halt_on_reset == 1) {
				if (get_field(s, DM_DMSTATUS_HASRESETHALTREQ)) {
					uint32_t control;
					dmi_read(target, &control, DM_DMCONTROL);
					control = set_field(control, DM_DMCONTROL_SETRESETHALTREQ, 1);
					dmi_write(target, DM_DMCONTROL, control);
					LOG_DEBUG("hart [%s] %d: halt-on-reset is on!", target->tap->dotted_name, i);
				} else {
					LOG_ERROR("The Debug Module doesn't supports halt-on-reset functionality!!");
				}
			}
#endif /* _NDS_V5_ONLY_ */


		}

		LOG_DEBUG("Detected %d harts.", dm->hart_count);
	}

	r->current_hartid = target->coreid;

	if (dm->hart_count == 0) {
		LOG_ERROR("No harts found!");
		return ERROR_FAIL;
	}

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */

	if (riscv013_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	bool halted = riscv_is_halted(target);
	if (!halted) {
		if (riscv013_halt_go(target) != ERROR_OK) {
			LOG_ERROR("[%s] Fatal: Hart %d failed to halt during examine()",
					target_name(target), r->current_hartid);
			return ERROR_FAIL;
		}
	}

	/* Without knowing anything else we can at least mess with the
		* program buffer. */
	r->debug_buffer_size = info->progbufsize;

	int result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
	if (result == ERROR_OK)
		r->xlen = 64;
	else {
		r->xlen = 32;
#if _NDS_V5_ONLY_
		riscv013_clear_abstract_error(target);
#endif
	}

	/* Save s0 and s1. The register cache hasn't be initialized yet so we
	 * need to take care of this manually. */
	uint64_t s0, s1;
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK) {
		LOG_ERROR("Fatal: Failed to read s0 from hart %d.", r->current_hartid);
		return ERROR_FAIL;
	}
	if (register_read_direct(target, &s1, GDB_REGNO_S1) != ERROR_OK) {
		LOG_ERROR("Fatal: Failed to read s1 from hart %d.", r->current_hartid);
		return ERROR_FAIL;
	}

	if (register_read_direct(target, &r->misa, GDB_REGNO_MISA)) {
		LOG_ERROR("Fatal: Failed to read MISA from hart %d.", r->current_hartid);
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	if (register_read_direct(target, &r->marchid, (CSR_MARCHID + GDB_REGNO_CSR0)))
		LOG_ERROR("Fatal: Failed to read MARCHID from hart %d.", r->current_hartid);

	if (register_read_direct(target, &r->mhartid, (CSR_MHARTID + GDB_REGNO_CSR0)))
		LOG_ERROR("Fatal: Failed to read MHARTID from hart %d.", r->current_hartid);
#endif

	if (riscv_supports_extension(target, 'V')) {
		if (discover_vlenb(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	/* Now init registers based on what we discovered. */
	if (riscv_init_registers(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Display this as early as possible to help people who are using
	 * really slow simulators. */
	LOG_DEBUG(" hart %d: XLEN=%d, misa=0x%" PRIx64, r->current_hartid, r->xlen,
			r->misa);

	/* Restore s0 and s1. */
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK) {
		LOG_ERROR("Fatal: Failed to write s0 back to hart %d.", r->current_hartid);
		return ERROR_FAIL;
	}
	if (register_write_direct(target, GDB_REGNO_S1, s1) != ERROR_OK) {
		LOG_ERROR("Fatal: Failed to write s1 back to hart %d.", r->current_hartid);
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	/* Init V */
	if (riscv_supports_extension(target, 'V'))
		ndsv5_get_vector_VLMAX(target);

	/* Check if rv32e for target burn*/
	rv32e = false;
	if (riscv_supports_extension(target, 'E')) {
		rv32e = true;
		LOG_DEBUG("target is rv32e");
	}

	if (nds32->reset_halt_as_examine) {
		LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
		target->debug_reason = DBG_REASON_DBGRQ;
		target->state = TARGET_HALTED;
		LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
	} else {
		riscv013_step_or_resume_current_hart(target, false, false);
		target->state = TARGET_RUNNING;
	}

	/* Bug-27408 */
	target_set_examined(target);
#else /* _NDS_V5_ONLY_ */
	if (!halted)
		riscv013_step_or_resume_current_hart(target, false, false);
#endif /* _NDS_V5_ONLY_ */

	if (target->smp) {
		bool haltgroup_supported;
		if (set_group(target, &haltgroup_supported, target->smp, HALTGROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (haltgroup_supported)
			LOG_INFO("Core %d made part of halt group %d.", target->coreid,
					target->smp);
		else
			LOG_INFO("Core %d could not be made part of halt group %d.",
					target->coreid, target->smp);
#if _NDS_V5_ONLY_
		r->group_halt_supported = haltgroup_supported;

		if (set_group(target, &haltgroup_supported, target->smp, RESUMEGROUP) != ERROR_OK)
			return ERROR_FAIL;
		if (haltgroup_supported)
			LOG_INFO("Core %d made part of resume group %d.", target->coreid,
					target->smp);
		else
			LOG_INFO("Core %d could not be made part of resume group %d.",
					target->coreid, target->smp);
		r->group_resume_supported = haltgroup_supported;
#endif /* NDS_V5_ONLY_ */
	}

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_INFO("Examined RISC-V core; found %d harts",
			riscv_count_harts(target));
	LOG_INFO(" hart %d: XLEN=%d, misa=0x%" PRIx64, r->current_hartid, r->xlen,
			r->misa);
	return ERROR_OK;
}

static int riscv013_authdata_read(struct target *target, uint32_t *value, unsigned int index)
{
	if (index > 0) {
		LOG_ERROR("Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	if (wait_for_authbusy(target, NULL) != ERROR_OK)
		return ERROR_FAIL;

	return dmi_read(target, value, DM_AUTHDATA);
}

static int riscv013_authdata_write(struct target *target, uint32_t value, unsigned int index)
{
	if (index > 0) {
		LOG_ERROR("Spec 0.13 only has a single authdata register.");
		return ERROR_FAIL;
	}

	uint32_t before, after;
	if (wait_for_authbusy(target, &before) != ERROR_OK)
		return ERROR_FAIL;

	dmi_write(target, DM_AUTHDATA, value);

	if (wait_for_authbusy(target, &after) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(before, DM_DMSTATUS_AUTHENTICATED) &&
			get_field(after, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_INFO("authdata_write resulted in successful authentication");
		int result = ERROR_OK;
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;
		target_list_t *entry;
		list_for_each_entry(entry, &dm->target_list, list) {
			if (target_examine_one(entry->target) != ERROR_OK)
				result = ERROR_FAIL;
		}
		return result;
	}

	return ERROR_OK;
}

static int riscv013_hart_count(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	assert(dm);
	return dm->hart_count;
}

/* Try to find out the widest memory access size depending on the selected memory access methods. */
static unsigned riscv013_data_bits(struct target *target)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (has_sufficient_progbuf(target, 3))
				return riscv_xlen(target);
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (get_field(info->sbcs, DM_SBCS_SBACCESS128))
				return 128;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS64))
				return 64;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS32))
				return 32;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS16))
				return 16;
			if (get_field(info->sbcs, DM_SBCS_SBACCESS8))
				return 8;
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			/* TODO: Once there is a spec for discovering abstract commands, we can
			 * take those into account as well.  For now we assume abstract commands
			 * support XLEN-wide accesses. */
			return riscv_xlen(target);
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;
	}
	LOG_ERROR("Unable to determine supported data bits on this target. Assuming 32 bits.");
	return 32;
}

COMMAND_HELPER(riscv013_print_info, struct target *target)
{
	RISCV013_INFO(info);

	/* Abstract description. */
	riscv_print_info_line(CMD, "target", "memory.read_while_running8", get_field(info->sbcs, DM_SBCS_SBACCESS8));
	riscv_print_info_line(CMD, "target", "memory.write_while_running8", get_field(info->sbcs, DM_SBCS_SBACCESS8));
	riscv_print_info_line(CMD, "target", "memory.read_while_running16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "target", "memory.write_while_running16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "target", "memory.read_while_running32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "target", "memory.write_while_running32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "target", "memory.read_while_running64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "target", "memory.write_while_running64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "target", "memory.read_while_running128", get_field(info->sbcs, DM_SBCS_SBACCESS128));
	riscv_print_info_line(CMD, "target", "memory.write_while_running128", get_field(info->sbcs, DM_SBCS_SBACCESS128));

	/* Lower level description. */
	riscv_print_info_line(CMD, "dm", "abits", info->abits);
	riscv_print_info_line(CMD, "dm", "progbufsize", info->progbufsize);
	riscv_print_info_line(CMD, "dm", "sbversion", get_field(info->sbcs, DM_SBCS_SBVERSION));
	riscv_print_info_line(CMD, "dm", "sbasize", get_field(info->sbcs, DM_SBCS_SBASIZE));
	riscv_print_info_line(CMD, "dm", "sbaccess128", get_field(info->sbcs, DM_SBCS_SBACCESS128));
	riscv_print_info_line(CMD, "dm", "sbaccess64", get_field(info->sbcs, DM_SBCS_SBACCESS64));
	riscv_print_info_line(CMD, "dm", "sbaccess32", get_field(info->sbcs, DM_SBCS_SBACCESS32));
	riscv_print_info_line(CMD, "dm", "sbaccess16", get_field(info->sbcs, DM_SBCS_SBACCESS16));
	riscv_print_info_line(CMD, "dm", "sbaccess8", get_field(info->sbcs, DM_SBCS_SBACCESS8));

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) == ERROR_OK)
		riscv_print_info_line(CMD, "dm", "authenticated", get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED));

	return 0;
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
	if (register_read_direct(target, vtype, GDB_REGNO_VTYPE) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read_direct(target, vl, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;
	if (nds_is_rvv_0_8 == 1) {
		if (register_write_direct(target, GDB_REGNO_VTYPE, encoded_vsew << 2) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		if (register_write_direct(target, GDB_REGNO_VTYPE, encoded_vsew << 3) != ERROR_OK)
			return ERROR_FAIL;
	}
	*debug_vl = DIV_ROUND_UP(r->vlenb * 8, riscv_xlen(target));
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

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
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

	return result;
}

static int riscv013_set_register_buf(struct target *target,
		int regno, const uint8_t *value)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
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

	return result;
}

static uint32_t sb_sbaccess(unsigned int size_bytes)
{
	switch (size_bytes) {
		case 1:
			return set_field(0, DM_SBCS_SBACCESS, 0);
		case 2:
			return set_field(0, DM_SBCS_SBACCESS, 1);
		case 4:
			return set_field(0, DM_SBCS_SBACCESS, 2);
		case 8:
			return set_field(0, DM_SBCS_SBACCESS, 3);
		case 16:
			return set_field(0, DM_SBCS_SBACCESS, 4);
	}
	assert(0);
	return 0;
}

static int sb_write_address(struct target *target, target_addr_t address,
							bool ensure_success)
{
	RISCV013_INFO(info);
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	/* There currently is no support for >64-bit addresses in OpenOCD. */
	if (sbasize > 96)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS3, 0, false, false);
	if (sbasize > 64)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS2, 0, false, false);
	if (sbasize > 32)
		dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS1, address >> 32, false, false);
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, DM_SBADDRESS0, address,
				  false, ensure_success);
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

static int sba_supports_access(struct target *target, unsigned int size_bytes)
{
	RISCV013_INFO(info);
	switch (size_bytes) {
		case 1:
			return get_field(info->sbcs, DM_SBCS_SBACCESS8);
		case 2:
			return get_field(info->sbcs, DM_SBCS_SBACCESS16);
		case 4:
			return get_field(info->sbcs, DM_SBCS_SBACCESS32);
		case 8:
			return get_field(info->sbcs, DM_SBCS_SBACCESS64);
		case 16:
			return get_field(info->sbcs, DM_SBCS_SBACCESS128);
		default:
			return 0;
	}
}

static int sample_memory_bus_v1(struct target *target,
								struct riscv_sample_buf *buf,
								const riscv_sample_config_t *config,
								int64_t until_ms)
{
	RISCV013_INFO(info);
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	if (sbasize > 64) {
		LOG_ERROR("Memory sampling is only implemented for sbasize <= 64.");
		return ERROR_NOT_IMPLEMENTED;
	}

	if (get_field(info->sbcs, DM_SBCS_SBVERSION) != 1) {
		LOG_ERROR("Memory sampling is only implemented for SBA version 1.");
		return ERROR_NOT_IMPLEMENTED;
	}

	uint32_t sbcs = 0;
	uint32_t sbcs_valid = false;

	uint32_t sbaddress0 = 0;
	bool sbaddress0_valid = false;
	uint32_t sbaddress1 = 0;
	bool sbaddress1_valid = false;

	/* How often to read each value in a batch. */
	const unsigned int repeat = 5;

	unsigned int enabled_count = 0;
	for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
		if (config->bucket[i].enabled)
			enabled_count++;
	}

	while (timeval_ms() < until_ms) {
		/*
		 * batch_run() adds to the batch, so we can't simply reuse the same
		 * batch over and over. So we create a new one every time through the
		 * loop.
		 */
		struct riscv_batch *batch = riscv_batch_alloc(
			target, 1 + enabled_count * 5 * repeat,
			info->dmi_busy_delay + info->bus_master_read_delay);
		if (!batch)
			return ERROR_FAIL;

		unsigned int result_bytes = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					if (!sba_supports_access(target, config->bucket[i].size_bytes)) {
						LOG_ERROR("Hardware does not support SBA access for %d-byte memory sampling.",
								config->bucket[i].size_bytes);
						return ERROR_NOT_IMPLEMENTED;
					}

					uint32_t sbcs_write = DM_SBCS_SBREADONADDR;
					if (enabled_count == 1)
						sbcs_write |= DM_SBCS_SBREADONDATA;
					sbcs_write |= sb_sbaccess(config->bucket[i].size_bytes);
					if (!sbcs_valid || sbcs_write != sbcs) {
						riscv_batch_add_dmi_write(batch, DM_SBCS, sbcs_write);
						sbcs = sbcs_write;
						sbcs_valid = true;
					}

					if (sbasize > 32 &&
							(!sbaddress1_valid ||
							sbaddress1 != config->bucket[i].address >> 32)) {
						sbaddress1 = config->bucket[i].address >> 32;
						riscv_batch_add_dmi_write(batch, DM_SBADDRESS1, sbaddress1);
						sbaddress1_valid = true;
					}
					if (!sbaddress0_valid ||
							sbaddress0 != (config->bucket[i].address & 0xffffffff)) {
						sbaddress0 = config->bucket[i].address;
						riscv_batch_add_dmi_write(batch, DM_SBADDRESS0, sbaddress0);
						sbaddress0_valid = true;
					}
					if (config->bucket[i].size_bytes > 4)
						riscv_batch_add_dmi_read(batch, DM_SBDATA1);
					riscv_batch_add_dmi_read(batch, DM_SBDATA0);
					result_bytes += 1 + config->bucket[i].size_bytes;
				}
			}
		}

		if (buf->used + result_bytes >= buf->size) {
			riscv_batch_free(batch);
			break;
		}

		size_t sbcs_key = riscv_batch_add_dmi_read(batch, DM_SBCS);

		int result = batch_run(target, batch);
		if (result != ERROR_OK)
			return result;

		uint32_t sbcs_read = riscv_batch_get_dmi_read_data(batch, sbcs_key);
		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* Discard this batch (too much hassle to try to recover partial
			 * data) and try again with a larger delay. */
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			dmi_write(target, DM_SBCS, sbcs_read | DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			riscv_batch_free(batch);
			continue;
		}
		if (get_field(sbcs_read, DM_SBCS_SBERROR)) {
			/* The memory we're sampling was unreadable, somehow. Give up. */
			dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR | DM_SBCS_SBERROR);
			riscv_batch_free(batch);
			return ERROR_FAIL;
		}

		unsigned int read = 0;
		for (unsigned int n = 0; n < repeat; n++) {
			for (unsigned int i = 0; i < ARRAY_SIZE(config->bucket); i++) {
				if (config->bucket[i].enabled) {
					assert(i < RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE);
					uint64_t value = 0;
					if (config->bucket[i].size_bytes > 4)
						value = ((uint64_t)riscv_batch_get_dmi_read_data(batch, read++)) << 32;
					value |= riscv_batch_get_dmi_read_data(batch, read++);

					buf->buf[buf->used] = i;
					buf_set_u64(buf->buf + buf->used + 1, 0, config->bucket[i].size_bytes * 8, value);
					buf->used += 1 + config->bucket[i].size_bytes;
				}
			}
		}

		riscv_batch_free(batch);
	}

	return ERROR_OK;
}

static int sample_memory(struct target *target,
						 struct riscv_sample_buf *buf,
						 riscv_sample_config_t *config,
						 int64_t until_ms)
{
	if (!config->enabled)
		return ERROR_OK;

	return sample_memory_bus_v1(target, buf, config, until_ms);
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	RISCV_INFO(generic_info);

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
	generic_info->invalidate_cached_debug_buffer = &riscv013_invalidate_cached_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->authdata_read = &riscv013_authdata_read;
	generic_info->authdata_write = &riscv013_authdata_write;
	generic_info->dmi_read = &dmi_read;
	generic_info->dmi_write = &dmi_write;
	generic_info->read_memory = read_memory;
	generic_info->test_sba_config_reg = &riscv013_test_sba_config_reg;
	generic_info->hart_count = &riscv013_hart_count;
	generic_info->data_bits = &riscv013_data_bits;
	generic_info->print_info = &riscv013_print_info;
	if (!generic_info->version_specific) {
		generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
		if (!generic_info->version_specific)
			return ERROR_FAIL;
	}
	generic_info->sample_memory = sample_memory;
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

	info->has_aampostincrement = YNM_MAYBE;

	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
#if _NDS_V5_ONLY_
	/* Disable halt-on-reset when reset-run */
	if ((nds_halt_on_reset == 1) && (target->reset_halt == 0)) {
		ndsv5_haltonreset(target, 0);
	}

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

	uint32_t control_base = set_field(0, DM_DMCONTROL_DMACTIVE, 1);

#if _NDS_V5_ONLY_
	if (target->reset_halt) {
		/* If 'reset halt', enable haltonreset */
		/* Haltonreset is not supported on a single hart (RTL before 2019/05) */
		ndsv5_haltonreset(target, 1);
	} else {
		/* If 'reset run', disable haltonreset */
		ndsv5_haltonreset(target, 0);
	}
#endif /* _NDS_V5_ONLY_ */

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* Run the user-supplied script if there is one. */
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		 * We must reset them all. */

		/* TODO: Try to use hasel in dmcontrol */

		/* Set haltreq for each hart. */
		uint32_t control = control_base;

		control = set_hartsel(control_base, target->coreid);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		dmi_write(target, DM_DMCONTROL, control);

		/* Assert ndmreset */
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);

	} else {
		/* Reset just this hart. */
		uint32_t control = set_hartsel(control_base, r->current_hartid);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);
	}

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	alive_sleep(nds32->reset_time);
#endif
	target->state = TARGET_RESET;

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	/* The DM might have gotten reset if OpenOCD called us in some reset that
	 * involves SRST being toggled. So clear our cache which may be out of
	 * date. */
	riscv013_invalidate_cached_debug_buffer(target);

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
	control = set_field(control, DM_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
	control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);

	dmi_write(target, DM_DMCONTROL,
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
			if (index != target->coreid)
				continue;
			dmi_write(target, DM_DMCONTROL,
					set_hartsel(control, index));
		} else {
			index = r->current_hartid;
		}

		LOG_DEBUG("Waiting for hart %d to come out of reset.", index);
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


#if _NDS_V5_ONLY_
			/* Bug 28320, Check CPU status before leave deassert-reset */
			if (!get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL)) {
				if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED)) {
					int result_halt;
					if (target->reset_halt) {
						NDS32_LOG(NDS32_MSG_HW_RESET_HOLD_ID, target->tap->dotted_name, index);
						result_halt = ERROR_OK;
					}

					control = set_field(control, DM_DMCONTROL_HALTREQ, 0);
					dmi_write(target, DM_DMCONTROL, control);
					if (result_halt == ERROR_OK)
						break;
					else
						return ERROR_FAIL;
				} else if (get_field(dmstatus, DM_DMSTATUS_ALLRUNNING)) {
					/* Reset run, leave when running */
					if (!target->reset_halt)
						break;
				}

				if (!target->reset_halt && get_field(dmstatus, DM_DMSTATUS_ALLRUNNING))
					break;
			}
#else
			/* Certain debug modules, like the one in GD32VF103
			 * MCUs, violate the specification's requirement that
			 * each hart is in "exactly one of four states" and,
			 * during reset, report harts as both unavailable and
			 * halted/running. To work around this, we check for
			 * the absence of the unavailable state rather than
			 * the presence of any other state. */
			if (!get_field(dmstatus, DM_DMSTATUS_ALLUNAVAIL))
				break;
#endif /* _NDS_V5_ONLY_ */

			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart %d didn't leave reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						index, riscv_reset_timeout_sec, dmstatus);

#if _NDS_V5_ONLY_
				/* For more detailed */
				uint32_t dmcontrol;
				uint32_t abstractcs;
				dmi_read(target, &dmcontrol, DM_DMCONTROL);
				dmi_read(target, &abstractcs, DM_ABSTRACTCS);
				NDS32_LOG("dmcontrol : 0x%x", dmcontrol);
				NDS32_LOG("abstractcs: 0x%x", abstractcs);

				/* Release haltreq when failed to halt */
				control = set_field(control, DM_DMCONTROL_HALTREQ, 0);
				dmi_write(target, DM_DMCONTROL, control);
#endif
				return ERROR_FAIL;
			}
		}

		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_DBGRQ;

		if (get_field(dmstatus, DM_DMSTATUS_ALLHAVERESET)) {
			/* Ack reset. */
			dmi_write(target, DM_DMCONTROL,
					set_hartsel(control, index) |
					DM_DMCONTROL_ACKHAVERESET);
		}

#if _NDS_V5_ONLY_
		/* for reset-run, halt again */
		if (!target->reset_halt) {
			struct nds32_v5 *nds32 = target_to_nds32_v5(target);
			alive_sleep(nds32->boot_time);
			if (riscv013_halt_go(target) != ERROR_OK) {
				LOG_ERROR("[%s] Fatal: Hart %d failed to halt during deassert_reset()",
						target_name(target), r->current_hartid);

				/* Reset target status */
				target->state = TARGET_RUNNING;
				target->debug_reason = DBG_REASON_UNDEFINED;
			}
		}
#endif /* _NDS_V5_ONLY_ */

		if (!target->rtos)
			break;
	}

#if _NDS_V5_ONLY_
	/* Restore halt-on-reset */
	if (nds_halt_on_reset == 1 && target->rtos) {
		/* Haltonreset is not supported on a single hart (RTL before 2019/05) */
		ndsv5_haltonreset(target, 1);
	} else
		ndsv5_haltonreset(target, 0);

	if (target->state == TARGET_HALTED) {
		uint64_t dpc;
		riscv_get_register(target, &dpc, GDB_REGNO_PC);
		NDS_INFO("[%s] hart[%d] halt at 0x%" PRIx64, target->tap->dotted_name,
				riscv_current_hartid(target), dpc);
	}
#endif /* _NDS_V5_ONLY_ */

	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
}

#if _NDS_V5_ONLY_
static int execute_fence_i(struct target *target)
{
	LOG_DEBUG("EXECUTE FENCE.I");
	{
		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		if (nds32->nds_do_fencei == true) {
			if (riscv_program_exec(&program, target) != ERROR_OK)
				LOG_ERROR("Unable to execute pre-fence.i");
			if (!target->smp)
				nds32->nds_do_fencei = false;
		} else
			LOG_DEBUG("Skip to execute pre-fence.i");
	}

	return ERROR_OK;
}
#endif /* _NDS_V5_ONLY_ */


static int execute_fence(struct target *target)
{
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
			value &= 0xffffffffUL;
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
	static int sbdata[4] = { DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3 };
	assert(size <= 16);
	for (int i = (size - 1) / 4; i >= 0; i--) {
		result = dmi_op(target, &value, NULL, DMI_OP_READ, sbdata[i], 0, false, true);
		if (result != ERROR_OK)
			return result;
		buf_set_u32(buffer + i * 4, 0, 8 * MIN(size, 4), value);
		log_memory_access(address + i * 4, value, MIN(size, 4), true);
	}
	return ERROR_OK;
}

static target_addr_t sb_read_address(struct target *target)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
		dmi_read(target, &v, DM_SBADDRESS1);
		address |= v;
		address <<= 32;
	}
	dmi_read(target, &v, DM_SBADDRESS0);
	address |= v;
	return address;
}

static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs)
{
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, sbcs, DM_SBCS) != ERROR_OK)
			return ERROR_FAIL;
		if (!get_field(*sbcs, DM_SBCS_SBBUSY))
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
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5)) {
		/* Read DCSR */
		uint64_t dcsr;
		if (register_read_direct(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;

		/* Read and save MSTATUS */
		if (register_read_direct(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		*mstatus_old = *mstatus;

		/* If we come from m-mode with mprv set, we want to keep mpp */
		if (get_field(dcsr, CSR_DCSR_PRV) < 3) {
			/* MPP = PRIV */
			*mstatus = set_field(*mstatus, MSTATUS_MPP, get_field(dcsr, CSR_DCSR_PRV));

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
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (size != increment) {
		LOG_ERROR("sba v0 reads only support size==increment");
		return ERROR_NOT_IMPLEMENTED;
	}

	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	uint8_t *t_buffer = buffer;
	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	uint32_t access = 0;

	const int DM_SBCS_SBSINGLEREAD_OFFSET = 20;
	const uint32_t DM_SBCS_SBSINGLEREAD = (0x1U << DM_SBCS_SBSINGLEREAD_OFFSET);

	const int DM_SBCS_SBAUTOREAD_OFFSET = 15;
	const uint32_t DM_SBCS_SBAUTOREAD = (0x1U << DM_SBCS_SBAUTOREAD_OFFSET);

	/* ww favorise one off reading if there is an issue */
	if (count == 1) {
		for (uint32_t i = 0; i < count; i++) {
			if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
			dmi_write(target, DM_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DM_SBCS_SBACCESS, size/2);
			access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
			LOG_DEBUG("\r\nread_memory: sab: access:  0x%08x", access);
			dmi_write(target, DM_SBCS, access);
			/* 3) read */
			uint32_t value;
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("\r\nread_memory: sab: value:  0x%08x", value);
			buf_set_u32(t_buffer, 0, 8 * size, value);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_DEBUG("reading block until final address 0x%" PRIx64, fin_addr);
	if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;
	/* set current address */
	dmi_write(target, DM_SBADDRESS0, cur_addr);
	/* 2) write sbaccess=2, sbsingleread,sbautoread,sbautoincrement
	 * size/2 matching the bit access of the spec 0.13 */
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOREAD, 1);
	access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08x", access);
	dmi_write(target, DM_SBCS, access);

	while (cur_addr < fin_addr) {
		LOG_DEBUG("\r\nsab:autoincrement: \r\n size: %d\tcount:%d\taddress: 0x%08"
				PRIx64, size, count, cur_addr);
		/* read */
		uint32_t value;
		if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u32(t_buffer, 0, 8 * size, value);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dmi_write(target, DM_SBCS, 0);
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u32(t_buffer, 0, 8 * size, value);
		}
	}

	uint32_t sbcs;
	if (dmi_read(target, &sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (increment != size && increment != 0) {
		LOG_ERROR("sba v1 reads only support increment of size or 0");
		return ERROR_NOT_IMPLEMENTED;
	}

	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs_write = set_field(0, DM_SBCS_SBREADONADDR, 1);
		sbcs_write |= sb_sbaccess(size);
		if (increment == size)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBAUTOINCREMENT, 1);
		if (count > 1)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, count > 1);
		if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
			return ERROR_FAIL;

		/* This address write will trigger the first read. */
		if (sb_write_address(target, next_address, true) != ERROR_OK)
			return ERROR_FAIL;

		if (info->bus_master_read_delay) {
			jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("Failed to scan idle sequence");
				return ERROR_FAIL;
			}
		}

		/* First value has been read, and is waiting for us to issue a DMI read
		 * to get it. */

		static int sbdata[4] = {DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3};
		assert(size <= 16);
		target_addr_t next_read = address - 1;
		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			for (int j = (size - 1) / 4; j >= 0; j--) {
				uint32_t value;
				unsigned attempt = 0;
				while (1) {
					if (attempt++ > 100) {
						LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								  next_read);
						return ERROR_FAIL;
					}
					keep_alive();
					dmi_status_t status = dmi_scan(target, NULL, &value,
												   DMI_OP_READ, sbdata[j], 0, false);
					if (status == DMI_STATUS_BUSY)
						increase_dmi_busy_delay(target);
					else if (status == DMI_STATUS_SUCCESS)
						break;
					else
						return ERROR_FAIL;
				}
				if (next_read != address - 1) {
					buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
					log_memory_access(next_read, value, MIN(size, 4), true);
				}
				next_read = address + i * size + j * 4;
			}
		}

		uint32_t sbcs_read = 0;
		if (count > 1) {
			uint32_t value;
			unsigned attempt = 0;
			while (1) {
				if (attempt++ > 100) {
					LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								next_read);
					return ERROR_FAIL;
				}
				dmi_status_t status = dmi_scan(target, NULL, &value, DMI_OP_NOP, 0, 0, false);
				if (status == DMI_STATUS_BUSY)
					increase_dmi_busy_delay(target);
				else if (status == DMI_STATUS_SUCCESS)
					break;
				else
					return ERROR_FAIL;
			}
			buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
			log_memory_access(next_read, value, MIN(size, 4), true);

			/* "Writes to sbcs while sbbusy is high result in undefined behavior.
			 * A debugger must not write to sbcs until it reads sbbusy as 0." */
			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;

			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, 0);
			if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Read the last word, after we disabled sbreadondata if necessary. */
		if (!get_field(sbcs_read, DM_SBCS_SBERROR) &&
				!get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			if (read_memory_bus_word(target, address + (count - 1) * size, size,
						buffer + (count - 1) * size) != ERROR_OK)
				return ERROR_FAIL;

			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			if (dmi_write(target, DM_SBCS, sbcs_read | DM_SBCS_SBBUSYERROR) != ERROR_OK)
				return ERROR_FAIL;
			next_address = sb_read_address(target);
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs_read, DM_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			if (dmi_write(target, DM_SBCS, DM_SBCS_SBERROR) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static void log_mem_access_result(struct target *target, bool success, int method, bool read)
{
	RISCV_INFO(r);
	bool warn = false;
	char msg[60];

	/* Compose the message */
	snprintf(msg, 60, "%s to %s memory via %s.",
			success ? "Succeeded" : "Failed",
			read ? "read" : "write",
			(method == RISCV_MEM_ACCESS_PROGBUF) ? "program buffer" :
			(method == RISCV_MEM_ACCESS_SYSBUS) ? "system bus" : "abstract access");

	/* Determine the log message severity. Show warnings only once. */
	if (!success) {
		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			warn = r->mem_access_progbuf_warn;
			r->mem_access_progbuf_warn = false;
		}
		if (method == RISCV_MEM_ACCESS_SYSBUS) {
			warn = r->mem_access_sysbus_warn;
			r->mem_access_sysbus_warn = false;
		}
		if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			warn = r->mem_access_abstract_warn;
			r->mem_access_abstract_warn = false;
		}
	}

	if (warn)
		LOG_WARNING("%s", msg);
	else
		LOG_DEBUG("%s", msg);
}

static bool mem_should_skip_progbuf(struct target *target, target_addr_t address,
		uint32_t size, bool read, char **skip_reason)
{
	assert(skip_reason);

	if (!has_sufficient_progbuf(target, 3)) {
		LOG_DEBUG("Skipping mem %s via progbuf - insufficient progbuf size.",
				read ? "read" : "write");
		*skip_reason = "skipped (insufficient progbuf)";
		return true;
	}
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("Skipping mem %s via progbuf - target not halted.",
				read ? "read" : "write");
		*skip_reason = "skipped (target not halted)";
		return true;
	}
	if (riscv_xlen(target) < size * 8) {
		LOG_DEBUG("Skipping mem %s via progbuf - XLEN (%d) is too short for %d-bit memory access.",
				read ? "read" : "write", riscv_xlen(target), size * 8);
		*skip_reason = "skipped (XLEN too short)";
		return true;
	}
	if (size > 8) {
		LOG_DEBUG("Skipping mem %s via progbuf - unsupported size.",
				read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_DEBUG("Skipping mem %s via progbuf - progbuf only supports %u-bit address.",
				read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}

	return false;
}

static bool mem_should_skip_sysbus(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool read, char **skip_reason)
{
	assert(skip_reason);

	RISCV013_INFO(info);
	if (!sba_supports_access(target, size)) {
		LOG_DEBUG("Skipping mem %s via system bus - unsupported size.",
				read ? "read" : "write");
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	unsigned int sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	if ((sizeof(address) * 8 > sbasize) && (address >> sbasize)) {
		LOG_DEBUG("Skipping mem %s via system bus - sba only supports %u-bit address.",
				read ? "read" : "write", sbasize);
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (read && increment != size && (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0 || increment != 0)) {
		LOG_DEBUG("Skipping mem read via system bus - "
				"sba reads only support size==increment or also size==0 for sba v1.");
		*skip_reason = "skipped (unsupported increment)";
		return true;
	}

	return false;
}

static bool mem_should_skip_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t increment, bool read, char **skip_reason)
{
	assert(skip_reason);

	if (size > 8) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_DEBUG("Skipping mem %s via abstract access - unsupported size: %d bits",
				read ? "read" : "write", size * 8);
		*skip_reason = "skipped (unsupported size)";
		return true;
	}
	if ((sizeof(address) * 8 > riscv_xlen(target)) && (address >> riscv_xlen(target))) {
		LOG_DEBUG("Skipping mem %s via abstract access - abstract access only supports %u-bit address.",
				read ? "read" : "write", riscv_xlen(target));
		*skip_reason = "skipped (too large address)";
		return true;
	}
	if (read && size != increment) {
		LOG_ERROR("Skipping mem read via abstract access - "
				"abstract command reads only support size==increment.");
		*skip_reason = "skipped (unsupported increment)";
		return true;
	}

	return false;
}

/*
 * Performs a memory read using memory access abstract commands. The read sizes
 * supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16 byte
 * aamsize fields in the memory access abstract command.
 */
static int read_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	memset(buffer, 0, count * size);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;

#if _NDS_V5_ONLY_
	uint32_t command;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	/* const addr mode set aampostincrement = 0 */
	if (nds32->nds_const_addr_mode == 0)
		command = access_memory_command(target, false, width, true, false);
	else
		command = access_memory_command(target, false, width, false, false);
#else
	bool use_aampostincrement = info->has_aampostincrement != YNM_NO;
	/* Create the command (physical address, postincrement, read) */
	uint32_t command = access_memory_command(target, false, width, use_aampostincrement, false);
#endif

	/* Execute the reads */
	uint8_t *p = buffer;
	bool updateaddr = true;
	unsigned int width32 = (width < 32) ? 32 : width;
	for (uint32_t c = 0; c < count; c++) {
		/* Update the address if it is the first time or aampostincrement is not supported by the target. */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address + c * size, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during read_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);

		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address = read_abstract_arg(target, 1, riscv_xlen(target));
				if (new_address == address + size) {
					LOG_DEBUG("aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_WARNING("Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, false);
				result = execute_abstract_command(target, command);
				if (result == ERROR_OK) {
					LOG_DEBUG("aampostincrement is not supported on this target.");
					info->has_aampostincrement = YNM_NO;
				}
			}
		}

		if (result != ERROR_OK)
			return result;

		/* Copy arg0 to buffer (rounded width up to nearest 32) */
		riscv_reg_t value = read_abstract_arg(target, 0, width32);
		buf_set_u64(p, 0, 8 * size, value);

		if (info->has_aampostincrement == YNM_YES)
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
	RISCV013_INFO(info);
	int result = ERROR_OK;

	LOG_DEBUG("writing %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;

#if _NDS_V5_ONLY_
	uint32_t command;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	/* const addr mode set aampostincrement = 0 */
	if (nds32->nds_const_addr_mode == 0)
		command = access_memory_command(target, false, width, true, true);
	else
		command = access_memory_command(target, false, width, false, true);
#else
	bool use_aampostincrement = info->has_aampostincrement != YNM_NO;
	/* Create the command (physical address, postincrement, write) */
	uint32_t command = access_memory_command(target, false, width, use_aampostincrement, true);
#endif

	/* Execute the writes */
	const uint8_t *p = buffer;
	bool updateaddr = true;
	for (uint32_t c = 0; c < count; c++) {
		/* Move data to arg0 */
		riscv_reg_t value = buf_get_u64(p, 0, 8 * size);
		result = write_abstract_arg(target, 0, value, riscv_xlen(target));
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to write arg0 during write_memory_abstract().");
			return result;
		}

		/* Update the address if it is the first time or aampostincrement is not supported by the target. */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address + c * size, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during write_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);

		if (info->has_aampostincrement == YNM_MAYBE) {
			if (result == ERROR_OK) {
				/* Safety: double-check that the address was really auto-incremented */
				riscv_reg_t new_address = read_abstract_arg(target, 1, riscv_xlen(target));
				if (new_address == address + size) {
					LOG_DEBUG("aampostincrement is supported on this target.");
					info->has_aampostincrement = YNM_YES;
				} else {
					LOG_WARNING("Buggy aampostincrement! Address not incremented correctly.");
					info->has_aampostincrement = YNM_NO;
				}
			} else {
				/* Try the same access but with postincrement disabled. */
				command = access_memory_command(target, false, width, false, true);
				result = execute_abstract_command(target, command);
				if (result == ERROR_OK) {
					LOG_DEBUG("aampostincrement is not supported on this target.");
					info->has_aampostincrement = YNM_NO;
				}
			}
		}

		if (result != ERROR_OK)
			return result;

		if (info->has_aampostincrement == YNM_YES)
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
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;

	/* Write address to S0. */
	result = register_write_direct(target, GDB_REGNO_S0, address);
	if (result != ERROR_OK)
		return result;

	if (increment == 0 &&
			register_write_direct(target, GDB_REGNO_A0, 0) != ERROR_OK)
		return ERROR_FAIL;

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
		buf_set_u64(buffer, 0, 8 * size, value);
		log_memory_access(address, value, size, true);
		return ERROR_OK;
	}

	if (dmi_write(target, DM_ABSTRACTAUTO,
			1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET) != ERROR_OK)
		goto error;
	/* Read garbage from dmi_data0, which triggers another execution of the
	 * program. Now dmi_data0 contains the first good result, and s1 the next
	 * memory value. */
	if (dmi_read_exec(target, NULL, DM_DATA0) != ERROR_OK)
		goto error;

	/* read_addr is the next address that the hart will read from, which is the
	 * value in s0. */
	unsigned index = 2;
	while (index < count) {
		riscv_addr_t read_addr = address + index * increment;
		LOG_DEBUG("i=%d, count=%d, read_addr=0x%" PRIx64, index, count, read_addr);
		/* The pipeline looks like this:
		 * memory -> s1 -> dm_data0 -> debugger
		 * Right now:
		 * s0 contains read_addr
		 * s1 contains mem[read_addr-size]
		 * dm_data0 contains[read_addr-size*2]
		 */

#if _NDS_JTAG_SCANS_OPTIMIZE_
		struct riscv_batch *batch = riscv_batch_alloc(target, nds_jtag_max_scans,
				info->dmi_busy_delay + info->ac_busy_delay);
#else
		struct riscv_batch *batch = riscv_batch_alloc(target, RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->ac_busy_delay);
#endif

		if (!batch)
			return ERROR_FAIL;

		unsigned reads = 0;
		for (unsigned j = index; j < count; j++) {
			if (size > 4)
				riscv_batch_add_dmi_read(batch, DM_DATA1);
			riscv_batch_add_dmi_read(batch, DM_DATA0);

			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		batch_run(target, batch);

		/* Wait for the target to finish performing the last abstract command,
		 * and update our copy of cmderr. If we see that DMI is busy here,
		 * dmi_busy_delay will be incremented. */
		uint32_t abstractcs;
		if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);

		unsigned next_index;
		unsigned ignore_last = 0;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_index = index + reads;
				break;
			case CMDERR_BUSY:
				LOG_DEBUG("memory read resulted in busy response");

				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);

				dmi_write(target, DM_ABSTRACTAUTO, 0);

				uint32_t dmi_data0, dmi_data1 = 0;
				/* This is definitely a good version of the value that we
				 * attempted to read when we discovered that the target was
				 * busy. */
				if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}
				if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* See how far we got, clobbering dmi_data0. */
				if (increment == 0) {
					uint64_t counter;
					result = register_read_direct(target, &counter, GDB_REGNO_A0);
					next_index = counter;
				} else {
					uint64_t next_read_addr;
					result = register_read_direct(target, &next_read_addr,
												  GDB_REGNO_S0);
					next_index = (next_read_addr - address) / increment;
				}
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
				buf_set_u64(buffer + (next_index - 2) * size, 0, 8 * size, value64);
				log_memory_access(address + (next_index - 2) * size, value64, size, true);

				/* Restore the command, and execute it.
				 * Now DM_DATA0 contains the next value just as it would if no
				 * error had occurred. */
				dmi_write_exec(target, DM_COMMAND, command, true);
				next_index++;

				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

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
		unsigned read = 0;
		assert(index >= 2);
		for (unsigned j = index - 2; j < index + reads; j++) {
			assert(j < count);
#if _NDS_V5_ONLY_
#else
			LOG_DEBUG("index=%d, reads=%d, next_index=%d, ignore_last=%d, j=%d",
				index, reads, next_index, ignore_last, j);
#endif
			if (j + 3 + ignore_last > next_index)
				break;

			status = riscv_batch_get_dmi_read_op(batch, read);
			uint64_t value = riscv_batch_get_dmi_read_data(batch, read);
			read++;
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
			if (size > 4) {
				status = riscv_batch_get_dmi_read_op(batch, read);
				if (status != DMI_STATUS_SUCCESS) {
					LOG_WARNING("Batch memory read encountered DMI error %d. "
							"Falling back on slower reads.", status);
					riscv_batch_free(batch);
					result = ERROR_FAIL;
					goto error;
				}
				value <<= 32;
				value |= riscv_batch_get_dmi_read_data(batch, read);
				read++;
			}
			riscv_addr_t offset = j * size;
			buf_set_u64(buffer + offset, 0, 8 * size, value);
			log_memory_access(address + j * increment, value, size, true);
		}

		index = next_index;

		riscv_batch_free(batch);
	}

	dmi_write(target, DM_ABSTRACTAUTO, 0);

	if (count > 1) {
		/* Read the penultimate word. */
		uint32_t dmi_data0, dmi_data1 = 0;
		if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK)
			return ERROR_FAIL;
		if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK)
			return ERROR_FAIL;
		uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
		buf_set_u64(buffer + size * (count - 2), 0, 8 * size, value64);
		log_memory_access(address + size * (count - 2), value64, size, true);
	}

	/* Read the last word. */
	uint64_t value;
	result = register_read_direct(target, &value, GDB_REGNO_S1);
	if (result != ERROR_OK)
		goto error;
	buf_set_u64(buffer + size * (count-1), 0, 8 * size, value);
	log_memory_access(address + size * (count-1), value, size, true);

	return ERROR_OK;

error:
	dmi_write(target, DM_ABSTRACTAUTO, 0);

	return result;
}

/* Only need to save/restore one GPR to read a single word, and the progbuf
 * program doesn't need to increment. */
static int read_memory_progbuf_one(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	int result = ERROR_FAIL;

	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		goto restore_mstatus;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
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
			goto restore_mstatus;
	}
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		goto restore_mstatus;
	if (riscv_program_write(&program) != ERROR_OK)
		goto restore_mstatus;

	/* Write address to S0, and execute buffer. */
	if (write_abstract_arg(target, 0, address, riscv_xlen(target)) != ERROR_OK)
		goto restore_mstatus;
	uint32_t command = access_register_command(target, GDB_REGNO_S0,
			riscv_xlen(target), AC_ACCESS_REGISTER_WRITE |
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		goto restore_mstatus;

	uint64_t value;
	if (register_read_direct(target, &value, GDB_REGNO_S0) != ERROR_OK)
		goto restore_mstatus;
	buf_set_u64(buffer, 0, 8 * size, value);
	log_memory_access(address, value, size, true);
	result = ERROR_OK;

restore_mstatus:
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			result = ERROR_FAIL;

	return result;
}

/**
 * Read the requested memory, silently handling memory access errors.
 */
static int read_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
#if _NDS_MEM_Q_ACCESS_
	if ((nds_dmi_quick_access) && (!riscv_is_halted(target))) {
		if (riscv_debug_buffer_size(target) >= 7) {
			return ndsv5_read_memory_quick_access(target, address, size, count, buffer);
		}
	}
#endif

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

	/* s0 holds the next address to read from
	 * s1 holds the next data value read
	 * a0 is a counter in case increment is 0
	 */
	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	if (increment == 0 && riscv_save_register(target, GDB_REGNO_A0) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
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

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	if (increment == 0)
		riscv_program_addi(&program, GDB_REGNO_A0, GDB_REGNO_A0, 1);
	else
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, increment);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	result = read_memory_progbuf_inner(target, address, size, count, buffer, increment);

	if (result != ERROR_OK) {
		/* The full read did not succeed, so we will try to read each word individually. */
		/* This will not be fast, but reading outside actual memory is a special case anyway. */
		/* It will make the toolchain happier, especially Eclipse Memory View as it reads ahead. */
		target_addr_t address_i = address;
		uint32_t count_i = 1;
		uint8_t *buffer_i = buffer;

		for (uint32_t i = 0; i < count; i++, address_i += increment, buffer_i += size) {
			/* TODO: This is much slower than it needs to be because we end up
			 * writing the address to read for every word we read. */
			result = read_memory_progbuf_inner(target, address_i, size, count_i, buffer_i, increment);

			/* The read of a single word failed, so we will just return 0 for that instead */
			if (result != ERROR_OK) {
				LOG_DEBUG("error reading single word of %d bytes from 0x%" TARGET_PRIxADDR,
						size, address_i);

				buf_set_u64(buffer_i, 0, 8 * size, 0);
			}
		}
		result = ERROR_OK;
	}

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return result;
}

static int read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (count == 0)
		return ERROR_OK;

	if (size != 1 && size != 2 && size != 4 && size != 8 && size != 16) {
		LOG_ERROR("BUG: Unsupported size for memory read: %d", size);
		return ERROR_FAIL;
	}

	int ret = ERROR_FAIL;
	RISCV_INFO(r);
	RISCV013_INFO(info);

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);

	memset(buffer, 0, sizeof(uint8_t)*count*size);

	if (nds32->nds_const_addr_mode)
		increment = 0;

	if (memory->access_channel == NDS_MEMORY_ACC_CPU) {
		/* CPU mode */
		if (info->progbufsize >= 2) {
			ret = read_memory_progbuf(target, address, size, count, buffer, increment);
			if(ret == ERROR_OK)
				goto read_memory_finish;
		}

		if (nds_dmi_access_mem) {
			ret = read_memory_abstract(target, address, size, count, buffer, increment);
			if(ret == ERROR_OK)
				goto read_memory_finish;
		}
	} else {
		if (nds_sys_bus_supported) {
			/* BUS mode */
			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = read_memory_bus_v0(target, address, size, count, buffer, increment);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = read_memory_bus_v1(target, address, size, count, buffer, increment);

			if(ret == ERROR_OK)
				goto read_memory_finish;
		} else {
			LOG_ERROR("memory access channel: %s, but sysbusaccess:%d", 
					NDS_MEMORY_ACCESS_NAME[memory->access_channel], nds_sys_bus_supported);
			return ERROR_FAIL;
		}
	}

	/* Force using CPU mode again */
	if (info->progbufsize >= 2) {
		ret = read_memory_progbuf(target, address, size, count, buffer, increment);

		if (ret == ERROR_OK)
			goto read_memory_finish;
	}

	ret = read_memory_abstract(target, address, size, count, buffer, increment);

read_memory_finish:
	riscv_flush_registers(target);
	return ret;
#endif

	char *progbuf_result = "disabled";
	char *sysbus_result = "disabled";
	char *abstract_result = "disabled";

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (mem_should_skip_progbuf(target, address, size, true, &progbuf_result))
				continue;

			ret = read_memory_progbuf(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				progbuf_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (mem_should_skip_sysbus(target, address, size, increment, true, &sysbus_result))
				continue;

			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = read_memory_bus_v0(target, address, size, count, buffer, increment);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = read_memory_bus_v1(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				sysbus_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			if (mem_should_skip_abstract(target, address, size, increment, true, &abstract_result))
				continue;

			ret = read_memory_abstract(target, address, size, count, buffer, increment);

			if (ret != ERROR_OK)
				abstract_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;

		log_mem_access_result(target, ret == ERROR_OK, method, true);

		if (ret == ERROR_OK)
			return ret;
	}

	LOG_ERROR("Target %s: Failed to read memory (addr=0x%" PRIx64 ")", target_name(target), address);
	LOG_ERROR("  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
	return ret;


}

static int write_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/*1) write sbaddress: for singlewrite and autoincrement, we need to write the address once*/
	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	dmi_write(target, DM_SBADDRESS0, address);
	int64_t value = 0;
	int64_t access = 0;
	riscv_addr_t offset = 0;
	riscv_addr_t t_addr = 0;
	const uint8_t *t_buffer = buffer + offset;

	/* B.8 Writing Memory, single write check if we write in one go */
	if (count == 1) { /* count is in bytes here */
		value = buf_get_u64(t_buffer, 0, 8 * size);

		access = 0;
		access = set_field(access, DM_SBCS_SBACCESS, size/2);
		dmi_write(target, DM_SBCS, access);
		LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
		LOG_DEBUG("\r\nwrite_memory:SAB: ONE OFF: value 0x%08" PRIx64, value);
		dmi_write(target, DM_SBDATA0, value);
		return ERROR_OK;
	}

	/*B.8 Writing Memory, using autoincrement*/

	access = 0;
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
	dmi_write(target, DM_SBCS, access);

	/*2)set the value according to the size required and write*/
	for (riscv_addr_t i = 0; i < count; ++i) {
		offset = size*i;
		/* for monitoring only */
		t_addr = address + offset;
		t_buffer = buffer + offset;

		value = buf_get_u64(t_buffer, 0, 8 * size);
		LOG_DEBUG("SAB:autoincrement: expected address: 0x%08x value: 0x%08x"
				PRIx64, (uint32_t)t_addr, (uint32_t)value);
		dmi_write(target, DM_SBDATA0, value);
	}
	/*reset the autoincrement when finished (something weird is happening if this is not done at the end*/
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DM_SBCS, access);

	return ERROR_OK;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	int result;

	sb_write_address(target, next_address, true);
	while (next_address < end_address) {
		LOG_DEBUG("transferring burst starting at address 0x%" TARGET_PRIxADDR,
				next_address);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->bus_master_write_delay);
		if (!batch)
			return ERROR_FAIL;

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (riscv_batch_available_scans(batch) < (size + 3) / 4)
				break;

			if (size > 12)
				riscv_batch_add_dmi_write(batch, DM_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));

			if (size > 8)
				riscv_batch_add_dmi_write(batch, DM_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				riscv_batch_add_dmi_write(batch, DM_SBDATA1,
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
			riscv_batch_add_dmi_write(batch, DM_SBDATA0, value);

			log_memory_access(address + i * size, value, size, false);
			next_address += size;
		}

		/* Execute the batch of writes */
		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			return result;

		/* Read sbcs value.
		 * At the same time, detect if DMI busy has occurred during the batch write. */
		bool dmi_busy_encountered;
		if (dmi_op(target, &sbcs, &dmi_busy_encountered, DMI_OP_READ,
				DM_SBCS, 0, false, true) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_busy_encountered)
			LOG_DEBUG("DMI busy encountered during system bus write.");

		/* Wait until sbbusy goes low */
		time_t start = time(NULL);
		while (get_field(sbcs, DM_SBCS_SBBUSY)) {
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
			if (dmi_read(target, &sbcs, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. */
			LOG_DEBUG("Sbbusyerror encountered during system bus write.");
			/* Clear the sticky error flag. */
			dmi_write(target, DM_SBCS, sbcs | DM_SBCS_SBBUSYERROR);
			/* Slow down before trying again. */
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR) || dmi_busy_encountered) {
			/* Recover from the case when the write commands were issued too fast.
			 * Determine the address from which to resume writing. */
			next_address = sb_read_address(target);
			if (next_address < address) {
				/* This should never happen, probably buggy hardware. */
				LOG_DEBUG("unexpected sbaddress=0x%" TARGET_PRIxADDR
						  " - buggy sbautoincrement in hw?", next_address);
				/* Fail the whole operation. */
				return ERROR_FAIL;
			}
			/* Try again - resume writing. */
			continue;
		}

		unsigned int sberror = get_field(sbcs, DM_SBCS_SBERROR);
		if (sberror != 0) {
			/* Sberror indicates the bus access failed, but not because we issued the writes
			 * too fast. Cannot recover. Sbaddress holds the address where the error occurred
			 * (unless sbautoincrement in the HW is buggy).
			 */
			target_addr_t sbaddress = sb_read_address(target);
			LOG_DEBUG("System bus access failed with sberror=%u (sbaddress=0x%" TARGET_PRIxADDR ")",
					  sberror, sbaddress);
			if (sbaddress < address) {
				/* This should never happen, probably buggy hardware.
				 * Make a note to the user not to trust the sbaddress value. */
				LOG_DEBUG("unexpected sbaddress=0x%" TARGET_PRIxADDR
						  " - buggy sbautoincrement in hw?", next_address);
			}
			/* Clear the sticky error flag */
			dmi_write(target, DM_SBCS, DM_SBCS_SBERROR);
			/* Fail the whole operation */
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
#endif

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
	if (riscv_save_register(target, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_save_register(target, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (store, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
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

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
#if _NDS_V5_ONLY_
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
				RISCV_BATCH_ALLOC_SIZE,
				info->dmi_busy_delay + info->ac_busy_delay);
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_ */
		if (!batch)
			goto error;

		/* To write another word, we put it in S1 and execute the program. */
		unsigned start = (cur_addr - address) / size;
		for (unsigned i = start; i < count; ++i) {
			unsigned offset = size*i;
			const uint8_t *t_buffer = buffer + offset;

			uint64_t value = buf_get_u64(t_buffer, 0, 8 * size);

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
					dmi_write(target, DM_DATA1, value >> 32);
				dmi_write(target, DM_DATA0, value);

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
				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				if (size > 4)
					riscv_batch_add_dmi_write(batch, DM_DATA1, value >> 32);
				riscv_batch_add_dmi_write(batch, DM_DATA0, value);
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
				DMI_OP_READ, DM_ABSTRACTCS, 0, false, true);
		if (result != ERROR_OK)
			goto error;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
		if (info->cmderr == CMDERR_NONE && !dmi_busy_encountered) {
			LOG_DEBUG("successful (partial?) memory write");
		} else if (info->cmderr == CMDERR_BUSY || dmi_busy_encountered) {
			if (info->cmderr == CMDERR_BUSY)
				LOG_DEBUG("Memory write resulted in abstract command busy response.");
			else if (dmi_busy_encountered)
				LOG_DEBUG("Memory write resulted in DMI busy response.");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);

			dmi_write(target, DM_ABSTRACTAUTO, 0);
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
	dmi_write(target, DM_ABSTRACTAUTO, 0);

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
	if (size != 1 && size != 2 && size != 4 && size != 8 && size != 16) {
		LOG_ERROR("BUG: Unsupported size for memory write: %d", size);
		return ERROR_FAIL;
	}

	int ret = ERROR_FAIL;
	RISCV_INFO(r);
	RISCV013_INFO(info);

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_memory *memory = &(nds32->memory);
	
	uint32_t *p_word_data = (uint32_t *)buffer;
	NDS_INFO("writing %d words of %d bytes to 0x%08lx = 0x%08x", count, size, (long)address, *p_word_data);

	if (memory->access_channel == NDS_MEMORY_ACC_CPU) {
		/* CPU mode */
		if (info->progbufsize >= 2) {
			ret = write_memory_progbuf(target, address, size, count, buffer);
			if (ret == ERROR_OK)
				goto write_memory_finish;
		}

		if (nds_dmi_access_mem) {
			ret = write_memory_abstract(target, address, size, count, buffer);
			if (ret == ERROR_OK)
				goto write_memory_finish;
		}
	} else {
		if (nds_sys_bus_supported) {
			/* BUS mode */
			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = write_memory_bus_v0(target, address, size, count, buffer);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = write_memory_bus_v1(target, address, size, count, buffer);

			if (ret == ERROR_OK)
				goto write_memory_finish;
		} else {
			LOG_ERROR("memory access channel: %s, but sysbusaccess:%d", 
					NDS_MEMORY_ACCESS_NAME[memory->access_channel], nds_sys_bus_supported);
			return ERROR_FAIL;
		}
	}

	/* Froce using CPU mode again */
	if (info->progbufsize >= 2) {
		ret = write_memory_progbuf(target, address, size, count, buffer);

		if (ret == ERROR_OK)
			goto write_memory_finish;
	}

	ret = write_memory_abstract(target, address, size, count, buffer);

write_memory_finish:
	riscv_flush_registers(target);
	return ret;
#endif


	char *progbuf_result = "disabled";
	char *sysbus_result = "disabled";
	char *abstract_result = "disabled";

	for (unsigned int i = 0; i < RISCV_NUM_MEM_ACCESS_METHODS; i++) {
		int method = r->mem_access_methods[i];

		if (method == RISCV_MEM_ACCESS_PROGBUF) {
			if (mem_should_skip_progbuf(target, address, size, false, &progbuf_result))
				continue;

			ret = write_memory_progbuf(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				progbuf_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_SYSBUS) {
			if (mem_should_skip_sysbus(target, address, size, 0, false, &sysbus_result))
				continue;

			if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
				ret = write_memory_bus_v0(target, address, size, count, buffer);
			else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
				ret = write_memory_bus_v1(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				sysbus_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_ABSTRACT) {
			if (mem_should_skip_abstract(target, address, size, 0, false, &abstract_result))
				continue;

			ret = write_memory_abstract(target, address, size, count, buffer);

			if (ret != ERROR_OK)
				abstract_result = "failed";
		} else if (method == RISCV_MEM_ACCESS_UNSPECIFIED)
			/* No further mem access method to try. */
			break;

		log_mem_access_result(target, ret == ERROR_OK, method, false);

		if (ret == ERROR_OK)
			return ret;
	}

	LOG_ERROR("Target %s: Failed to write memory (addr=0x%" PRIx64 ")", target_name(target), address);
	LOG_ERROR("  progbuf=%s, sysbus=%s, abstract=%s", progbuf_result, sysbus_result, abstract_result);
	return ret;
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

struct target_type riscv013_target = {
#if _NDS_V5_ONLY_
	.name = "riscv_013",
	.resume = &ndsv5_resume,
#else
	.name = "riscv",
#endif /* _NDS_V5_ONLY_ */

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	.poll = &riscv_openocd_poll,
	.halt = &riscv_halt,
	.step = &riscv_openocd_step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.write_memory = write_memory,

	.arch_state = arch_state
};

/*** 0.13-specific implementations of various RISC-V helper functions. ***/
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int rid)
{
	LOG_DEBUG("[%s] reading register %s", target_name(target),
			gdb_regno_name(rid));

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

/* SYNC TODO */
/*
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		*value = -1;
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif
*/

	int result = ERROR_OK;
	if (rid == GDB_REGNO_PC) {
		/* TODO: move this into riscv.c. */
		result = register_read_direct(target, value, GDB_REGNO_DPC);
		LOG_DEBUG("[%d] read PC from DPC: 0x%" PRIx64, target->coreid, *value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		/* TODO: move this into riscv.c. */
		result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		*value = set_field(0, VIRT_PRIV_V, get_field(dcsr, CSR_DCSR_V));
		*value = set_field(*value, VIRT_PRIV_PRV, get_field(dcsr, CSR_DCSR_PRV));
	} else {
		result = register_read_direct(target, value, rid);
		if (result != ERROR_OK)
			*value = -1;
	}

	return result;
}

static int riscv013_set_register(struct target *target, int rid, uint64_t value)
{
	riscv013_select_current_hart(target);
	LOG_DEBUG("[%d] writing 0x%" PRIx64 " to register %s",
			target->coreid, value, gdb_regno_name(rid));

/* TODO SYNC */
/*
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif
*/

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
		register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, get_field(value, VIRT_PRIV_PRV));
		dcsr = set_field(dcsr, CSR_DCSR_V, get_field(value, VIRT_PRIV_V));
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
	if (!dm)
		return ERROR_FAIL;

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
	if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;

#if _NDS_V5_ONLY_
	if (!get_field(dmcontrol, DM_DMCONTROL_DMACTIVE)) {
		LOG_INFO("TARGET WARNING! Debug Module was not become active (dmcontrol=0x%x), forced turn on!",
				dmcontrol);
		dmcontrol = set_field(dmcontrol, DM_DMCONTROL_DMACTIVE, 1);
	}
#endif /* _NDS_V5_ONLY_ */

	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	int result = dmi_write(target, DM_DMCONTROL, dmcontrol);
	dm->current_hartid = r->current_hartid;
	return result;
}

/* Select all harts that were prepped and that are selectable, clearing the
 * prepped flag on the harts that actually were selected. */
static int select_prepped_harts(struct target *target, bool *use_hasel)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
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
		if (!target->smp && total_selected == 1)
			break;
#endif /* _NDS_V5_ONLY_ */
	}

	/* Don't use hasel if we only need to talk to one hart. */
	if (total_selected <= 1) {
		*use_hasel = false;
		return ERROR_OK;
	}

	for (unsigned i = 0; i < hawindow_count; i++) {
		if (dmi_write(target, DM_HAWINDOWSEL, i) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_write(target, DM_HAWINDOW, hawindow[i]) != ERROR_OK)
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
	if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
		return ERROR_FAIL;

	RISCV_INFO(r);
#if _NDS_V5_ONLY_
	LOG_DEBUG("halting [%s] hart %d", target->tap->dotted_name, r->current_hartid);
#else
	LOG_DEBUG("halting hart %d", r->current_hartid);
#endif /* _NDS_V5_ONLY_ */

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	if (use_hasel)
		dmcontrol |= DM_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (riscv_is_halted(target))
			break;

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus;
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

#if _NDS_V5_ONLY_
		if (nds_no_halt_detect) {
			LOG_INFO("Unable to halt [%s] hart %d", target->tap->dotted_name, r->current_hartid);
			LOG_INFO("  dmcontrol=0x%08x", dmcontrol);
			LOG_INFO("  dmstatus =0x%08x", dmstatus);
		} else {
			NDS32_LOG("<-- Unable to halt [%s] hart %d -->", target->tap->dotted_name, r->current_hartid);
			NDS32_LOG("  dmcontrol=0x%08x", dmcontrol);
			NDS32_LOG("  dmstatus =0x%08x", dmstatus);
		}

		if (target->smp) {
			target->hart_unavailable = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", r->current_hartid,
				target->hart_unavailable);
		}

		/* Release haltreq when failed to halt */
		dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DM_DMCONTROL, dmcontrol);

#else /* _NDS_V5_ONLY_ */
		LOG_ERROR("[%s] Unable to halt hart %d. dmcontrol=0x%08x, dmstatus=0x%08x",
				  target_name(target), r->current_hartid, dmcontrol, dmstatus);
#endif /* _NDS_V5_ONLY_ */
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

#if _NDS_V5_ONLY_
	if (target->smp) {
		target->hart_unavailable = 0x0;
		LOG_DEBUG("  hart_unavailable[%d] = 0x%x", r->current_hartid,
				target->hart_unavailable);
	}
#endif

	if (use_hasel) {
		target_list_t *entry;
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;
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
	if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
		return ERROR_FAIL;

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
	if (target->smp) {
		target->hart_unavailable = 0x0;
	}
#endif

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return false;
#if _NDS_V5_ONLY_
	if (get_field(dmstatus, DM_DMSTATUS_ANYUNAVAIL)) {
		LOG_ERROR("[%s] hart %d is unavailiable", target->tap->dotted_name, riscv_current_hartid(target));
		if (target->smp) {
			target->hart_unavailable = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target),
					target->hart_unavailable);
		}
		return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
	}
	if (get_field(dmstatus, DM_DMSTATUS_ANYNONEXISTENT)) {
		LOG_ERROR("[%s] hart %d doesn't exist", target->tap->dotted_name, riscv_current_hartid(target));
		if (target->smp) {
			target->hart_unavailable = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target),
					target->hart_unavailable);
		}
		return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
	}

	if ((target->state != TARGET_HALTED) &&
	    (nds_no_reset_detect == 0) &&
	    (get_field(dmstatus, DM_DMSTATUS_ANYHAVERESET) == 1)) {
		LOG_DEBUG(NDS32_ERRMSG_TARGET_RESET);

		uint32_t dmcontrol;
		dmi_read(target, &dmcontrol, DM_DMCONTROL);
		dmcontrol = set_field(dmcontrol, DM_DMCONTROL_ACKHAVERESET, 1);

		int retry = 0;
		do {
			dmi_write(target, DM_DMCONTROL, dmcontrol);
			dmi_read(target, &dmstatus, DM_DMSTATUS);
			alive_sleep(10);
		} while (get_field(dmstatus, DM_DMSTATUS_ANYHAVERESET) && (retry++ < MAX_RETRY));

		if (retry >= MAX_RETRY) {
			NDS32_LOG_ERROR("<-- TARGET WARNING! Unable to clear havereset on [%s] hart %d. -->",
					target->tap->dotted_name, riscv_current_hartid(target));
		}
	}
#else /* _NDS_V5_ONLY_ */
	if (get_field(dmstatus, DM_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("Hart %d is unavailable.", riscv_current_hartid(target));
	if (get_field(dmstatus, DM_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("Hart %d doesn't exist.", riscv_current_hartid(target));
	if (get_field(dmstatus, DM_DMSTATUS_ANYHAVERESET)) {
		int hartid = riscv_current_hartid(target);
		LOG_INFO("Hart %d unexpectedly reset!", hartid);
		/* TODO: Can we make this more obvious to eg. a gdb user? */
		uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_ACKHAVERESET;
		dmcontrol = set_hartsel(dmcontrol, hartid);
		/* If we had been halted when we reset, request another halt. If we
		 * ended up running out of reset, then the user will (hopefully) get a
		 * message that a reset happened, that the target is running, and then
		 * that it is halted again once the request goes through.
		 */
		if (target->state == TARGET_HALTED)
			dmcontrol |= DM_DMCONTROL_HALTREQ;
		dmi_write(target, DM_DMCONTROL, dmcontrol);
	}
#endif /* _NDS_V5_ONLY_ */

	return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return RISCV_HALT_UNKNOWN;

	LOG_DEBUG("dcsr.cause: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));

	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_EBREAK:
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
	case CSR_DCSR_CAUSE_HALTREQ:
	case CSR_DCSR_CAUSE_RESETHALTREQ:
		return RISCV_HALT_INTERRUPT;
	case CSR_DCSR_CAUSE_GROUP:
		return RISCV_HALT_GROUP;
	}

#if _NDS_DISABLE_ABORT_
	NDS32_LOG("Unknown DCSR cause field: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));
	NDS32_LOG("  dcsr=0x%" PRIx32, (uint32_t) dcsr);
#endif
	LOG_ERROR("Unknown DCSR cause field: 0x%" PRIx64, get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%" PRIx32, (uint32_t) dcsr);
	return RISCV_HALT_UNKNOWN;
}

int riscv013_write_debug_buffer(struct target *target, unsigned index, riscv_insn_t data)
{
#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
	if (nds_jtag_scans_optimize > 0) {
		RISCV013_INFO(info);
		if (write_debug_buffer_batch == NULL) {
			write_debug_buffer_batch = riscv_batch_alloc(target, nds_jtag_max_scans,
					info->dmi_busy_delay + info->ac_busy_delay);
		}

		if (index >= info->progbufsize)
			riscv_batch_add_dmi_write(write_debug_buffer_batch, DM_DATA0 + index - info->progbufsize, data);
		else
			riscv_batch_add_dmi_write(write_debug_buffer_batch, DM_PROGBUF0 + index, data);
		return ERROR_OK;
	}
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF */

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (dm->progbuf_cache[index] != data) {
		if (dmi_write(target, DM_PROGBUF0 + index, data) != ERROR_OK)
			return ERROR_FAIL;
		dm->progbuf_cache[index] = data;
	} else {
		LOG_DEBUG("cache hit for 0x%" PRIx32 " @%d", data, index);
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
	} else {
		uint64_t return_val = backup_debug_buffer[riscv_current_hartid(target)][index];
		LOG_DEBUG("%s, return_val=0x%lx", __func__, (long unsigned int)return_val);
		return return_val;
	}
#endif /* _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF */

	uint32_t value;
	dmi_read(target, &value, DM_PROGBUF0 + index);
	return value;
}

int riscv013_invalidate_cached_debug_buffer(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "Invalidating progbuf cache");
	for (unsigned int i = 0; i < 15; i++)
		dm->progbuf_cache[i] = 0;
	return ERROR_OK;
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

	uint32_t sbaccess128 = get_field(info->sbcs, DM_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DM_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DM_SBCS_SBACCESS32);
	uint32_t sbaccess16 = get_field(info->sbcs, DM_SBCS_SBACCESS16);
	uint32_t sbaccess8 = get_field(info->sbcs, DM_SBCS_SBACCESS8);

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

	uint32_t sbaccess128 = get_field(info->sbcs, DM_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DM_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DM_SBCS_SBACCESS32);

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
	dmi_read(target, &sbcs_orig, DM_SBCS);

	uint32_t sbcs = sbcs_orig;
	bool test_passed;

	int max_sbaccess = get_max_sbaccess(target);

	if (max_sbaccess == -1) {
		LOG_ERROR("System Bus Access not supported in this config.");
		return ERROR_FAIL;
	}

	if (get_field(sbcs, DM_SBCS_SBVERSION) != 1) {
		LOG_ERROR("System Bus Access unsupported SBVERSION (%d). Only version 1 is supported.",
				get_field(sbcs, DM_SBCS_SBVERSION));
		return ERROR_FAIL;
	}

	uint32_t num_sbdata_regs = get_num_sbdata_regs(target);
	assert(num_sbdata_regs);

	uint32_t rd_buf[num_sbdata_regs];

	/* Test 1: Simple write/read test */
	test_passed = true;
	sbcs = set_field(sbcs_orig, DM_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DM_SBCS, sbcs);

	uint32_t test_patterns[4] = {0xdeadbeef, 0xfeedbabe, 0x12345678, 0x08675309};
	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DM_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DM_SBCS, sbcs);

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
	sbcs = set_field(sbcs_orig, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DM_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DM_SBCS, sbcs);

		dmi_write(target, DM_SBADDRESS0, legal_address);
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
			dmi_write(target, DM_SBDATA0, i);
		}

		read_sbcs_nonbusy(target, &sbcs);

		dmi_write(target, DM_SBADDRESS0, legal_address);

		uint32_t val;
		sbcs = set_field(sbcs, DM_SBCS_SBREADONDATA, 1);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &val, DM_SBDATA0); /* Dummy read to trigger first system bus read */
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
			dmi_read(target, &val, DM_SBDATA0);
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

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 2);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 3: Illegal address read test PASSED.");
		else
			LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to clear to 0.");
	} else {
		LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to set error code.");
	}

	/* Test 4: Write to illegal address */
	write_memory_sba_simple(target, illegal_address, test_patterns, 1, sbcs_orig);

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 2);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
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
	uint32_t sbaccess128 = get_field(sbcs_orig, DM_SBCS_SBACCESS128);

	if (sbaccess128) {
		LOG_INFO("System Bus Access Test 5: SBCS sbaccess error test PASSED, all sbaccess sizes supported.");
	} else {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBACCESS, 4);

		write_memory_sba_simple(target, legal_address, test_patterns, 1, sbcs);

		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 4) {
			sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 4);
			dmi_write(target, DM_SBCS, sbcs);
			dmi_read(target, &rd_val, DM_SBCS);
			if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
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
	sbcs = set_field(sbcs_orig, DM_SBCS_SBACCESS, 1);

	write_memory_sba_simple(target, legal_address+1, test_patterns, 1, sbcs);

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 3) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 3);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
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
		sbcs = set_field(sbcs_orig, DM_SBCS_SBREADONADDR, 1);
		dmi_write(target, DM_SBCS, sbcs);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DM_SBDATA0, 0xdeadbeef);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DM_SBADDRESS0, legal_address);

		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBBUSYERROR)) {
			sbcs = set_field(sbcs_orig, DM_SBCS_SBBUSYERROR, 1);
			dmi_write(target, DM_SBCS, sbcs);
			dmi_read(target, &rd_val, DM_SBCS);
			if (get_field(rd_val, DM_SBCS_SBBUSYERROR) == 0)
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

	uint32_t sba_size = get_field(info->sbcs, DM_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_no_readonaddr = set_field(sbcs, DM_SBCS_SBREADONADDR, 0);
	dmi_write(target, DM_SBCS, sbcs_no_readonaddr);

	for (uint32_t i = 0; i < sba_size/32; i++) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DM_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DM_SBADDRESS3, masked_addr);
	}

	/* Write SBDATA registers starting with highest address, since write to
	 * SBDATA0 triggers write */
	for (int i = write_size-1; i >= 0; i--)
		dmi_write(target, DM_SBDATA0+i, write_data[i]);
}

void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs)
{
	RISCV013_INFO(info);

	uint32_t rd_sbcs;
	uint32_t masked_addr;

	uint32_t sba_size = get_field(info->sbcs, DM_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_readonaddr = set_field(sbcs, DM_SBCS_SBREADONADDR, 1);
	dmi_write(target, DM_SBCS, sbcs_readonaddr);

	/* Write addresses starting with highest address register */
	for (int i = sba_size/32-1; i >= 0; i--) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DM_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DM_SBADDRESS3, masked_addr);
	}

	read_sbcs_nonbusy(target, &rd_sbcs);

	for (uint32_t i = 0; i < read_size; i++)
		dmi_read(target, &(rd_buf[i]), DM_SBDATA0+i);
}

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int maybe_execute_fence_i(struct target *target)
{
	if (has_sufficient_progbuf(target, 3))
#if _NDS_V5_ONLY_
		return execute_fence_i(target);
#else
		return execute_fence(target);
#endif
	return ERROR_OK;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	if (maybe_execute_fence_i(target) != ERROR_OK)
		return ERROR_FAIL;

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, riscv_ebreakm);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, riscv_ebreaks);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, riscv_ebreaku);

#if _NDS_V5_ONLY_
	extern bool v5_stepie;
	if (v5_stepie == true)
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 1);
	else
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 0);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if ((nds32->attached == false) && (nds32->target_burn_attached == false)) {
		LOG_DEBUG("nds32->attached: %x, nds32->target_burn_attached: %x", 
				nds32->attached, nds32->target_burn_attached);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 0);
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE , 0);
	}

	/* Disable SMP group resume */
	RISCV_INFO(r);
	if ((step || nds32->target_burn_attached) && target->smp && r->group_resume_supported) {
		LOG_DEBUG("Disable SMP group resume before stepping");
		bool haltgroup_supported;
		set_group(target, &haltgroup_supported, 0, RESUMEGROUP);
	}
#endif /* _NDS_V5_ONLY_ */

	if (riscv_set_register(target, GDB_REGNO_DCSR, dcsr) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_flush_registers(target) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
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

	if (riscv_flush_registers(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	if (use_hasel)
		dmcontrol |= DM_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HASEL, 0);
	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (get_field(dmstatus, DM_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0)
			continue;

		dmi_write(target, DM_DMCONTROL, dmcontrol);
#if _NDS_V5_ONLY_
		/* e-15336, when resuming, clear issued_halt */
		target->halt_issued = false;


		/* Disable SMP group resume */
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		if ((step || nds32->target_burn_attached) && target->smp && r->group_resume_supported) {
			LOG_DEBUG("Re-enable SMP group resume after stepping");
			bool haltgroup_supported;
			set_group(target, &haltgroup_supported, target->smp, RESUMEGROUP);
		}
#endif /* _NDS_V5_ONLY_ */
		return ERROR_OK;
	}

	dmi_write(target, DM_DMCONTROL, dmcontrol);

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
	dmi_read(target, &abstractcs, DM_ABSTRACTCS);
	while (get_field(abstractcs, DM_ABSTRACTCS_BUSY)) {
		dmi_read(target, &abstractcs, DM_ABSTRACTCS);

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
	dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
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

	dmi_write(target, DM_DATA0, reg_value);
	if (riscv_xlen(target) == 64)
		dmi_write(target, DM_DATA1, reg_value >> 32);

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
			if (((ndsv5_dmi_abstractcs & DM_ABSTRACTCS_CMDERR) >> DM_ABSTRACTCS_CMDERR_OFFSET) ==
					CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, set_csr_reg_quick_access_retry");
				goto ndsv5_set_csr_reg_quick_access_retry;
			}
		}
	}

	LOG_DEBUG("reg_num: 0x%x, value: 0x%lx", reg_num, reg_value);
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
			if (((ndsv5_dmi_abstractcs & DM_ABSTRACTCS_CMDERR) >> DM_ABSTRACTCS_CMDERR_OFFSET) ==
					CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, get_csr_reg_quick_access_retry");
				goto ndsv5_get_csr_reg_quick_access_retry;
			}
		}
	}

	uint32_t value = 0;
	dmi_read(target, &value, DM_DATA0);
	uint32_t value_h = 0;
	if (riscv_xlen(target) == 64) {
		dmi_read(target, &value_h, DM_DATA1);
		LOG_DEBUG("value_h: 0x%x", value_h);
	}
	uint64_t reg_value = value_h;
	reg_value = (reg_value << 32) | value;
	*preg_value = reg_value;
	LOG_DEBUG("reg_num: 0x%x, value: 0x%lx", reg_num, *preg_value);
	return ERROR_OK;
}

int ndsv5_haltonreset(struct target *target, int enable)
{
	if (enable == 1)
		LOG_DEBUG("Set setresethaltreq");
	else
		LOG_DEBUG("Set clrresethaltreq");

	RISCV_INFO(r);
	int hartid_bak = r->current_hartid;

	/* Correct current dm current_hartid */
	dm013_info_t *dm = get_dm(target);
	uint32_t tmp_dmcontrol;
	dmi_read(target, &tmp_dmcontrol, DM_DMCONTROL);
	dm->current_hartid = get_field(tmp_dmcontrol, DM_DMCONTROL_HARTSELLO);

	riscv013_select_current_hart(target);

	uint32_t s;
	dmi_read(target, &s, DM_DMSTATUS);
	if (get_field(s, DM_DMSTATUS_HASRESETHALTREQ)) {
		uint32_t control;
		dmi_read(target, &control, DM_DMCONTROL);

		if (enable == 1)
			control = set_field(control, DM_DMCONTROL_SETRESETHALTREQ, 1);
		else
			control = set_field(control, DM_DMCONTROL_CLRRESETHALTREQ, 1);
		dmi_write(target, DM_DMCONTROL, control);

		if (enable == 1)
			LOG_DEBUG("hart %d: halt-on-reset is on!", target->coreid);
		else
			LOG_DEBUG("hart %d: halt-on-reset is off!", target->coreid);

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
	dmi_write(target, DM_DMCONTROL, 0);
	dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
	dmi_read(target, &control, DM_DMCONTROL);

	/* check existence harts */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DM_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
		dmi_write(target, DM_DMCONTROL, control);
		dmi_read(target, &control, DM_DMCONTROL);

		dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (get_field(dmstatus, DM_DMSTATUS_ANYNONEXISTENT)) {
			user_def_hart_count = i;
			LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
			break;
		}
	}

	/* Assert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DM_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);

		/* SETRESETHALTREQ can halt on 0x80000000 for AMP/SMP all harts */
		control = set_field(control, DM_DMCONTROL_SETRESETHALTREQ, 1);
		control = set_field(control, DM_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DM_DMCONTROL, control);
		dmi_read(target, &control, DM_DMCONTROL);
	}

	/* Assert ndmreset */
	control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
	dmi_write(target, DM_DMCONTROL, control);

	alive_sleep(nds32->reset_time);
	dmi_read(target, &dmstatus, DM_DMSTATUS);

	/* Deassert reset */
	for (i = 0; i < user_def_hart_count; ++i) {
		time_t start = time(NULL);
		control = 0;

		/* Clear the reset, but make sure haltreq is still set */
		control = set_field(control, DM_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
		control = set_field(control, DM_DMCONTROL_ACKHAVERESET, 1);
		control = set_field(control, DM_DMCONTROL_CLRRESETHALTREQ, 1);
		control = set_field(control, DM_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DM_DMCONTROL, control);
		dmi_read(target, &control, DM_DMCONTROL);

		do {
			dmi_read(target, &dmstatus, DM_DMSTATUS);
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart didn't halt coming out of reset in %ds; "
				"dmstatus=0x%x; "
				"Increase the timeout with riscv set_reset_timeout_sec.",
				riscv_reset_timeout_sec, dmstatus);

				/* Release haltreq when failed to halt */
				control = set_field(control, DM_DMCONTROL_HALTREQ, 0);
				dmi_write(target, DM_DMCONTROL, control);

				return ERROR_FAIL;
			}
		} while (get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0);

		control = set_field(control, DM_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DM_DMCONTROL, control);
		dmi_read(target, &control, DM_DMCONTROL);
		dmi_read(target, &dmstatus, DM_DMSTATUS);
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

	uint64_t abstractcs = riscv_batch_get_dmi_read_data(ndsv5_access_memory_pack_batch, index_access_mem_batch_ABSTRACTCS);
	if (get_field(abstractcs, DM_ABSTRACTCS_CMDERR) != 0) {
		riscv013_clear_abstract_error(target);
		uint32_t cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
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
		case 64:
			riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_DATA0 + offset + 1, value >> 32);
			riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_DATA0 + offset, value);
			break;
		case 32:
			riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_DATA0 + offset, value);
			break;
		default:
			LOG_ERROR("Unsupported xlen: %d", xlen);
			return ERROR_FAIL;
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
	riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_COMMAND, command);
	riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DM_ABSTRACTCS);
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
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_DATA0 + index - info->progbufsize, data);
	else
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_PROGBUF0 + index, data);
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
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DM_ABSTRACTCS);
		/* move S1 to DATA0 */
		ndsv5_access_s0s1_direct_pack(target, GDB_REGNO_S1, address + offset, 0);

		/* read value in DATA0 */
		size_t index_data0 = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DM_DATA0);

		result = ndsv5_access_memory_pack_batch_run(target, 0);

		uint64_t dmi_out = riscv_batch_get_dmi_read_data(ndsv5_access_memory_pack_batch, index_data0);
		value = dmi_out;
		write_to_buf(buffer + offset, value, size);
		buf_set_u64(buffer + offset, 0, 8 * size, value); ///TODO!!??

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
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DM_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DM_ABSTRACTCS);
	}
	return ERROR_OK;
}

int ndsv5_vector_restore_vtype_vl(struct target *target, uint64_t reg_vl, uint64_t reg_vtype)
{
	int result;

	register_write_direct(target, GDB_REGNO_S0, reg_vl);
	register_write_direct(target, GDB_REGNO_S1, reg_vtype);
	LOG_DEBUG("reg_vtype: 0x%lx", (long unsigned int)reg_vtype);

	/* Restore vtype & vl */
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_vsetvl(&program, GDB_REGNO_S0, GDB_REGNO_S1);
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
	if (register_read_direct(target, &mmsc_cfg, GDB_REGNO_CSR0 + CSR_MMSC_CFG) != ERROR_OK)
		LOG_ERROR("read mmsc_cfg error");

	if ((mmsc_cfg & 0x1000000000)>>36)
		MSTATUS_VS = 0x00000600;
	else
		MSTATUS_VS = 0x01800000;

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
		vector_vl = (nds32->nds_vector_length/vector_SEW);
		if (register_write_direct(target, GDB_REGNO_S0, vector_vl) != ERROR_OK)
			return ERROR_FAIL;
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

	nds32->nds_vector_SEW = vector_SEW;
	nds32->nds_vector_vl = (uint32_t)vector_vl;
	LOG_DEBUG("nds32->nds_vector_SEW: 0x%x, nds32->nds_vector_vl: 0x%x", nds32->nds_vector_SEW, nds32->nds_vector_vl);
	LOG_DEBUG("nds32->nds_vector_length: 0x%x", nds32->nds_vector_length);

	/* Restore vtype & vl */
	ndsv5_vector_restore_vtype_vl(target, reg_vl, reg_vtype);

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

	if (register_write_direct(target, GDB_REGNO_S0, nds32->nds_vector_vl) != ERROR_OK)
		return ERROR_FAIL;
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
	ndsv5_vector_restore_vtype_vl(target, reg_vl, reg_vtype);
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

	if (register_write_direct(target, GDB_REGNO_S0, nds32->nds_vector_vl) != ERROR_OK)
		return ERROR_FAIL;
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
	ndsv5_vector_restore_vtype_vl(target, reg_vl, reg_vtype);
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
		uint32_t sbcs = set_field(0, DM_SBCS_SBREADONADDR, 1);
		sbcs |= sb_sbaccess(size);
		sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
		sbcs = set_field(sbcs, DM_SBCS_SBREADONDATA, count > 1);
		dmi_write(target, DM_SBCS, sbcs);

		/* This address write will trigger the first read. */
		sb_write_address(target, next_address, true);

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
		/* LOG_DEBUG("nds_jtag_max_scans=0x%x", nds_jtag_max_scans); */

		if_last = 0;
		read_cnt = 0;
		batch_index = 0;
		cur_address = next_address;
		for (i = (next_address - address) / size; i < count; i++) {
			if (busmode_batch->used_scans > (busmode_batch->allocated_scans - 8))
				if_last = 1;
			else if (i == (count-1))
				if_last = 1;

			if (if_last) {
				sbcs = set_field(sbcs, DM_SBCS_SBREADONDATA, 0);
				/* dmi_write(target, DM_SBCS, sbcs); */
				riscv_batch_add_dmi_write(busmode_batch, DM_SBCS, sbcs);
			}

			if (size > 12)
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DM_SBDATA3);
			if (size > 8)
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DM_SBDATA2);
			if (size > 4)
				pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DM_SBDATA1);
			pindex_read[batch_index++] = riscv_batch_add_dmi_read(busmode_batch, DM_SBDATA0);
			read_cnt++;
			next_address += size;
			if (if_last)
				break;
		}

		if (riscv_batch_run(busmode_batch) != ERROR_OK) {
			LOG_DEBUG("riscv_batch_run FAIL, busmode_batch->idle_count=0x%x", (unsigned int)busmode_batch->idle_count);
			increase_dmi_busy_delay(target);
			goto read_memory_bus_v1_opt_retry;
		} else {
			LOG_DEBUG("riscv_batch_run OK");
		}

		batch_index = 0;
		for (i = 0; i < read_cnt; i++) {
			riscv_addr_t offset = cur_address - address;
			if (size > 12) {
				dmi_out = riscv_batch_get_dmi_read_data(busmode_batch, pindex_read[batch_index++]);
				value = dmi_out;
				write_to_buf(buffer + offset + i * size + 12, value, 4);
				log_memory_access(cur_address + i * size + 12, value, 4, true);
			}
			if (size > 8) {
				dmi_out = riscv_batch_get_dmi_read_data(busmode_batch, pindex_read[batch_index++]);
				value = dmi_out;
				write_to_buf(buffer + offset + i * size + 8, value, 4);
				log_memory_access(cur_address + i * size + 8, value, 4, true);
			}
			if (size > 4) {
				dmi_out = riscv_batch_get_dmi_read_data(busmode_batch, pindex_read[batch_index++]);
				value = dmi_out;
				write_to_buf(buffer + offset + i * size + 4, value, 4);
				log_memory_access(cur_address + i * size + 4, value, 4, true);
			}
			dmi_out = riscv_batch_get_dmi_read_data(busmode_batch, pindex_read[batch_index++]);
			value = dmi_out;
			write_to_buf(buffer + offset + i * size, value, MIN(size, 4));
			log_memory_access(cur_address + i * size, value, 4, true);
		}

		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			LOG_DEBUG("DM_SBCS_SBBUSYERROR");
			dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR);
			increase_dmi_busy_delay(target);
			goto read_memory_bus_v1_opt_retry;
		}
		if (get_field(sbcs, DM_SBCS_SBERROR)) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			unsigned sb_error = get_field(sbcs, DM_SBCS_SBERROR);
			NDS32_LOG("<-- DM_SBCS_SBERROR = 0x%x, address = 0x%lx -->", sb_error, (long unsigned int)address);
			dmi_write(target, DM_SBCS, DM_SBCS_SBERROR);
			return ERROR_FAIL;
			/*
			increase_dmi_busy_delay(target);
			goto read_memory_bus_v1_opt_retry;
			*/
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
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
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
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  /* data0 */
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  /* data2 */
	}
	dmi_write(target, DM_DATA0, read_addr);
	dmi_write(target, DM_DATA1, read_addr >> 32);

	if (riscv_program_exec(&program, target) != ERROR_OK) {
		/* quick_access mode, if target state from freerun to halt */
		if (nds_dmi_quick_access_ena) {
			if (((nds_dmi_abstractcs & DM_ABSTRACTCS_CMDERR) >> DM_ABSTRACTCS_CMDERR_OFFSET) == CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, read_memory_retry");
				goto ndsv5_read_memory_quick_access_retry;
			}
		}
	}
	uint32_t value;
	dmi_read(target, &value, DM_DATA0);
	LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08lx", address, (long)value);
	uint32_t value_h = 0;
	if (size == 8) {
		dmi_read(target, &value_h, DM_DATA1);
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
	case 4:
		buffer[0] = value;
		buffer[1] = value >> 8;
		buffer[2] = value >> 16;
		buffer[3] = value >> 24;
		break;
	case 8:
		buffer[0] = value;
		buffer[1] = value >> 8;
		buffer[2] = value >> 16;
		buffer[3] = value >> 24;
		buffer[4] = value_h;
		buffer[5] = value_h >> 8;
		buffer[6] = value_h >> 16;
		buffer[7] = value_h >> 24;
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
	sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

write_memory_bus_v1_opt_retry:
	LOG_DEBUG("info->dmi_busy_delay=0x%x, info->ac_busy_delay=0x%x", info->dmi_busy_delay, info->ac_busy_delay);
	next_address = address;
	end_address = address + count * size;

	sb_write_address(target, next_address, true);
	while (next_address < end_address) {
		if (busmode_batch != NULL) {
			riscv_batch_free(busmode_batch);
			busmode_batch = NULL;
		}
		busmode_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		/* LOG_DEBUG("nds_jtag_max_scans=0x%x", nds_jtag_max_scans); */

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			/* if (riscv_batch_full(busmode_batch)) */
			/* check allocated_scans full */
			if (busmode_batch->used_scans > (busmode_batch->allocated_scans - 6))
					break;

			const uint8_t *p = buffer + i * size;
			if (size > 12)
				riscv_batch_add_dmi_write(busmode_batch, DM_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));
			if (size > 8)
				riscv_batch_add_dmi_write(busmode_batch, DM_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				riscv_batch_add_dmi_write(busmode_batch, DM_SBDATA1,
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
			riscv_batch_add_dmi_write(busmode_batch, DM_SBDATA0, value);
			log_memory_access(address + i * size, value, size, false);
			next_address += size;
			/* LOG_DEBUG("next_address=0x%x", (unsigned int)next_address); */
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

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			LOG_DEBUG("DM_SBCS_SBBUSYERROR");
			dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR);
			increase_dmi_busy_delay(target);
			goto write_memory_bus_v1_opt_retry;
		}
		if (get_field(sbcs, DM_SBCS_SBERROR)) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			unsigned sb_error = get_field(sbcs, DM_SBCS_SBERROR);
			NDS32_LOG("<-- DM_SBCS_SBERROR = 0x%x, address = 0x%lx -->", sb_error, (long unsigned int)address);
			dmi_write(target, DM_SBCS, DM_SBCS_SBERROR);
			return ERROR_FAIL;
			/*
			increase_dmi_busy_delay(target);
			goto write_memory_bus_v1_opt_retry;
			*/
		}
	}
	return ERROR_OK;
}

extern INSN_CODE_T_V5 *(*gen_get_value_code) (char *name, unsigned index);
extern INSN_CODE_T_V5 *(*gen_set_value_code) (char *name, unsigned index);

int nds_ace_enable(struct target *target)
{
	struct riscv_program program;
	riscv_program_init(&program, target);

	/* Assembly code used to enable ACE
	 *  7d0022f3 csrr t0,mmisc_ctl
	 *  0012c293 xori t0,t0,1
	 *  0102e293 ori  t0,t0,16
	 *  7d029073 csrw mmisc_ctl,t0
	 */
	riscv_program_insert(&program, 0x7d0022f3);
	riscv_program_insert(&program, 0x0012c293);
	riscv_program_insert(&program, 0x0102e293);
	riscv_program_insert(&program, 0x7d029073);

	/* run the program */
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

	/* Backup temp register (x5, x6, x7)
	 * x5: high part
	 * x6: low part
	 * x7: ACM's address
	 */
	riscv_reg_t s0, s1, s2;
	riscv_get_register(target, &s0, GDB_REGNO_T0);
	riscv_get_register(target, &s1, GDB_REGNO_T1);
	riscv_get_register(target, &s2, GDB_REGNO_T2);

	/* Get type_name and register index */
	char* type_name = (char *) reg->reg_data_type->id;
	/* Format : "%s_%d", type_name, idx */
	char* reg_name = (char *) reg->name;
	unsigned int reg_idx = atoi(strrchr(reg_name, '_') + 1);

	LOG_DEBUG("type_name = %s, reg_name = %s, reg_idx = %d, size = %d",
			type_name, reg_name, reg_idx, reg->size);

	/* Generate code to read value from ACR/ACM */
	INSN_CODE_T_V5 *insn_code = gen_get_value_code(type_name, reg_idx);

	/** Update the value
	 * [NOTE] The pointer of value is updated in following code.
	 *       If assigning value to reg->value after these updates,
	 *       reg->value would not be the initial address of *value.
	 *       So, incorrect value is assigned to reg->value. To avoid
	 *       this, we assign *value to reg->value initially.
	 */
	char *value = (char *) reg->value;

	bool init_acm_addr = false;
	/* Execute the code generated by gen_get_value_code() iteratively */
	for (unsigned i = 0; i < insn_code->num; i++) {
		/* Initialize */
		struct riscv_program program;
		riscv_program_init(&program, target);

		/* For ACM utility instruction, write memory address to GDB_REGNO_XPR0 + 7 */
		if (init_acm_addr == false &&
				((insn_code->code + i)->version == acm_io1 ||
				 (insn_code->code + i)->version == acm_io2)) {
			riscv_program_li(&program, GDB_REGNO_T2, reg_idx);
			init_acm_addr = true;
			if (is_rv64)
				LOG_DEBUG("acm_addr = 0x%016" PRIx64 " feed into $t2", (uint64_t) reg_idx);
			else
				LOG_DEBUG("acm_addr= 0x%08" PRIx32 " feed into $t2", (uint32_t) reg_idx);
		}

		/* insert utility instruction to program buffer */
		unsigned insn = (insn_code->code + i)->insn;
		riscv_program_insert(&program, insn);
		LOG_DEBUG("read utility instruction (offset: %d) = 0x%08" PRIx32, i, insn);
		LOG_DEBUG("read utility instruction version %d", (insn_code->code + i)->version);

		/* determine the number of GPRs used to read/write data from/to ACR/ACM */
		bool isTwoGPR = false;
		if ((insn_code->code + i)->version == acr_io2 ||
				(insn_code->code + i)->version == acm_io2) {
			isTwoGPR = true;
		}

		/* execute the code stored in program buffer */
		int exec_out = riscv_program_exec(&program, target);
		if (exec_out != ERROR_OK) {
			LOG_ERROR("Unable to execute ACE utility program");

			/* Restore temp register */
			riscv_set_register(target, GDB_REGNO_T0, s0);
			riscv_set_register(target, GDB_REGNO_T1, s1);
			riscv_set_register(target, GDB_REGNO_T2, s2);
			return exec_out;
		}

		/* read value from program buffer */
		if (isTwoGPR == false) {
			riscv_reg_t reg_value;
			riscv_get_register(target, &reg_value, GDB_REGNO_T0);
			memcpy(value, &reg_value, reg_bytes);
			value += reg_bytes;
			if (is_rv64)
				LOG_DEBUG("reg_value = 0x%016" PRIx64 " read from program buffer", (uint64_t) reg_value);
			else
				LOG_DEBUG("reg_value = 0x%08" PRIx32 " read from program buffer", (uint32_t) reg_value);
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

	/* Restore temp register */
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

	/* Backup temp register (x5, x6, x7, x28)
	 * x5: high part
	 * x6: low part
	 * x7: ACM's address
	 * x28(t3): temp reg to write to xlen(64) gpr
	 */
	riscv_reg_t s0, s1, s2, t3;
	riscv_get_register(target, &s0, GDB_REGNO_T0);
	riscv_get_register(target, &s1, GDB_REGNO_T1);
	riscv_get_register(target, &s2, GDB_REGNO_T2);
	if (is_rv64)
		riscv_get_register(target, &t3, GDB_REGNO_T3);

	/* Get acr_name and register index */
	char *type_name = (char *) reg->reg_data_type->id;
	/* Format : "%s_%d", type_name, idx */
	char *reg_name = (char *) reg->name;
	unsigned int reg_idx = atoi(strrchr(reg_name, '_') + 1);

	LOG_DEBUG("type_name = %s, reg_name = %s, reg_idx = %d, size = %d",
			type_name, reg_name, reg_idx, reg->size);

	/* Allocate buffer which is the multiple of register size */
	unsigned int ByteSize = (!reg->size%8) ? reg->size/8 : (reg->size/8) + 1;
	unsigned int rounds = (ByteSize % reg_bytes) ? (ByteSize / reg_bytes + 1) : (ByteSize / reg_bytes);
	char *buffer = (char *) alloca(rounds * reg_bytes);
	memset(buffer, 0, rounds * reg_bytes);
	memcpy(buffer, val, ByteSize);
	char *value = buffer;	/* this pointer will be increased when extracting value partially */

	/* Generate code to write value to ACR/ACM */
	INSN_CODE_T_V5 *insn_code = gen_set_value_code(type_name, reg_idx);

	bool init_acm_addr = false;
	/* Execute the code generated by gen_get_value_code() iteratively */
	for (unsigned i = 0; i < insn_code->num; i++) {
		/* Initialize */
		struct riscv_program program;
		riscv_program_init(&program, target);

		LOG_DEBUG("write utility instruction version %d", (insn_code->code + i)->version);
		/* determine the number of GPRs used to read/write data from/to ACR/ACM */
		bool isTwoGPR = false;
		if ((insn_code->code + i)->version == acr_io2 ||
				(insn_code->code + i)->version == acm_io2) {
			isTwoGPR = true;
		}

		/* For ACM utility instruction, write memory address to GDB_REGNO_XPR0 + 7 */
		if (init_acm_addr == false &&
				((insn_code->code + i)->version == acm_io1 ||
				 (insn_code->code + i)->version == acm_io2)) {
			riscv_program_li(&program, GDB_REGNO_T2, reg_idx);	/* 2 entry */
			init_acm_addr = true;
			if (is_rv64) {
				/* 7 insn entry + ebreak entry fills up program buffer */
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

		/* write given value string to S0/1 */
		if (isTwoGPR == false) {
			/* Extract part of value from given value string */
			riscv_reg_t reg_value = 0;
			memcpy(&reg_value, value, reg_bytes);
			value += reg_bytes;
			/* riscv_program_write_ram(&program, output + 4, 0); */
			if (is_rv64) {
				riscv_program_li64(&program, GDB_REGNO_T0, GDB_REGNO_T3, reg_value);	/* 7 entry */

				/* 7 insn entry + ebreak entry fills up program buffer */
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
			/* Extract part of value from given value string
			 * [NOTE] the order of val is from low bit order
			 *        e.g., for value of 0x111222333444555666777888999
			 *        the traversing order is from 999 --> 888 --> ...
			 */
			riscv_reg_t high = 0, low = 0;
			memcpy(&low, value, reg_bytes);
			value += reg_bytes;
			memcpy(&high, value, reg_bytes);
			value += reg_bytes;

			if (is_rv64) {
				riscv_program_li64(&program, GDB_REGNO_T0, GDB_REGNO_T3, high);	/* 7 entry */
				/* 7 insn entry + ebreak entry fills up program buffer */
				exec_out = riscv_program_exec(&program, target);
				if (exec_out != ERROR_OK) {
					LOG_ERROR("Unable to execute program");
					goto error;
				}
				riscv_program_init(&program, target);

				riscv_program_li64(&program, GDB_REGNO_T1, GDB_REGNO_T3, low);	/* 7 entry */
				/* 7 insn entry + ebreak entry fills up program buffer */
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

		/* riscv_program_fence(&program); */
		unsigned insn = (insn_code->code + i)->insn;
		riscv_program_insert(&program, insn);
		LOG_DEBUG("write utility instruction (offset:%d) = %x", i, insn);

		/* execute the code stored in program buffer */
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

	/* Restore temp register */
	riscv_set_register(target, GDB_REGNO_T0, s0);
	riscv_set_register(target, GDB_REGNO_T1, s1);
	riscv_set_register(target, GDB_REGNO_T2, s2);
	if (is_rv64)
		riscv_set_register(target, GDB_REGNO_T3, t3);

	return ERROR_OK;

error:
	/* Restore temp register */
	riscv_set_register(target, GDB_REGNO_T0, s0);
	riscv_set_register(target, GDB_REGNO_T1, s1);
	riscv_set_register(target, GDB_REGNO_T2, s2);
	if (is_rv64)
		riscv_set_register(target, GDB_REGNO_T3, t3);

	return exec_out;
}

struct reg_arch_type nds_ace_reg_access_type = {
	.get = nds_ace_get_reg,
	.set = nds_ace_set_reg
};

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

int ndsv5_get_delay_count(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	LOG_DEBUG("dmi_busy_delay=%d", info->dmi_busy_delay);
	return info->dmi_busy_delay;
}

/* tracer functions */
static uint32_t trace_sel;
static uint32_t selected_encoder(uint32_t offset)
{
	uint32_t encoder_addr = (DMI_TEADDRESSBASE + (trace_sel * 0x800) + offset);
	return encoder_addr;
}

static uint32_t tracer_select_hart(uint32_t hartsel)
{
	trace_sel = hartsel;
	LOG_DEBUG("trace_sel=%d", trace_sel);
	return 0;
}

static uint32_t tracer_activate_encoder(struct target *target)
{
	uint32_t  te_ctrl_reg, te_info_reg, tf_info_reg;
	uint32_t  ts_ctrl_reg;
	uint32_t  te_inst_features_reg;
	uint32_t  xtrigger_in_ctrl_reg;
	uint32_t  xtrigger_out_ctrl_reg;
	uint32_t  atb_ctrl_reg;
	uint32_t  timeout_limit;
	uint32_t  timeout_counter;
	uint32_t  tmux_ctrl_reg;

	riscv013_info_t *info = get_info(target);
	/* info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS); */
	if (info->abits <= 0x7) {
		LOG_DEBUG("DBG_API:ERROR:value of dtmcs.ABITS (%d) is smaller than or equal to 7", info->abits);
		LOG_DEBUG("no trace support is present in the target");
		return 0;
	}
	LOG_DEBUG("DBG_API:activate TMUX_ITTMUXCTRL %d", trace_sel);
	dmi_read(target, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
	tmux_ctrl_reg |= (0x01 << trace_sel);
	dmi_write(target, TMUX_ITTMUXCTRL, tmux_ctrl_reg);
	dmi_read(target, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
	LOG_DEBUG("DBG_API: tmux_ctrl_reg 0x%x", tmux_ctrl_reg);

	LOG_DEBUG("DBG_API:activate encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	te_ctrl_reg |= DMI_TECONTROL_teActive;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power up */
	timeout_counter = 0;
	timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
		if (te_ctrl_reg & DMI_TECONTROL_teActive)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teActive to be set for encoder");

	/* Read initial trace encoder registers */
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	dmi_read(target, &te_inst_features_reg, selected_encoder(DMI_TEINSTFEATURES));
	dmi_read(target, &ts_ctrl_reg, selected_encoder(DMI_TSCONTROL));
	dmi_read(target, &xtrigger_in_ctrl_reg, selected_encoder(DMI_XTRIGINCONTROL));
	dmi_read(target, &xtrigger_out_ctrl_reg, selected_encoder(DMI_XTRIGOUTCONTROL));
	dmi_read(target, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	dmi_read(target, &te_info_reg, selected_encoder(DMI_TEIMPL));
	dmi_read(target, &tf_info_reg, DMI_TFIMPL);
	LOG_DEBUG("DBG_API:te_info_reg = 0x%x, tf_info_reg = 0x%x", te_info_reg, tf_info_reg);
	LOG_DEBUG("DBG_API:te_ctrl_reg = 0x%x, te_inst_features_reg = 0x%x", te_ctrl_reg, te_inst_features_reg);
	LOG_DEBUG("DBG_API:ts_ctrl_reg = 0x%x, xtrigger_in_ctrl_reg = 0x%x", ts_ctrl_reg, xtrigger_in_ctrl_reg);
	LOG_DEBUG("DBG_API:xtrigger_out_ctrl_reg = 0x%x, atb_ctrl_reg = 0x%x", xtrigger_out_ctrl_reg, atb_ctrl_reg);
	return 0;
}

static uint32_t tracer_deactivate_encoder(struct target *target)
{
	uint32_t  te_ctrl_reg;
	uint32_t  timeout_limit;
	uint32_t  timeout_counter;

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	LOG_DEBUG("DBG_API:deactivate encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	te_ctrl_reg &= ~DMI_TECONTROL_teActive;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	timeout_counter = 0;
	timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
		if ((te_ctrl_reg & DMI_TECONTROL_teActive) == 0)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teActive to be cleared for encoder");

	return 0;
}

static uint32_t tracer_enable_encoder(struct target *target)
{
	uint32_t  te_ctrl_reg;
	uint32_t  timeout_limit;
	uint32_t  timeout_counter;

	LOG_DEBUG("DBG_API:enable encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	te_ctrl_reg |= DMI_TECONTROL_teEnable;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	timeout_counter = 0;
	timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
		if (te_ctrl_reg & DMI_TECONTROL_teEnable)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teEnable to be set for encoder");

	return 0;
}

static uint32_t tracer_disable_encoder(struct target *target)
{
	uint32_t  te_ctrl_reg;
	uint32_t  timeout_limit;
	uint32_t  timeout_counter;

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	LOG_DEBUG("DBG_API:disable encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	te_ctrl_reg &= ~DMI_TECONTROL_teEnable;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	timeout_counter = 0;
	timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
		if ((te_ctrl_reg & DMI_TECONTROL_teEnable) == 0)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teEnable to be cleared for encoder");

	return 0;
}

static uint32_t tracer_set_itracing_mode(struct target *target,
	uint32_t  call_stack_en,     /* teInstFeatures[3]: teInstEnCallStack */
	uint32_t  inst_no_addr_diff, /* teInstFeatures[0]: teInstNoAddrDiff */
	uint32_t  inhibit_src,       /* teControl[15]:     teInhibitSrc */
	uint32_t  stall_en,          /* teControl[13]:     teInstStallEn */
	uint32_t  itrigger_en,       /* teControl[11]:     teInstTrigEn */
	uint32_t  trace_sync_extra,  /* teControl[9]:      ndsSyncExtra */
	uint32_t  trace_context,     /* teControl[8]:      ndsTracePID */
	uint32_t  trace_priv,        /* teControl[7]:      ndsTracePRIV */
	uint32_t  te_inst_mode)      /* teControl[6:4]:    teInstMode */
{
	uint32_t te_ctrl_reg;
	uint32_t te_inst_features_reg;

	/* New spec mode: 0/3/6 */
	if (te_inst_mode == 7) {
		te_inst_mode = 6;
		call_stack_en = 1;
	}

	LOG_DEBUG("DBG_API:set teInstEnCallStack to %d, teInstStallEn to %d, teInstTrigEn to %d, "
	    "ndsSyncExtra to %d, ndsTracePID to %d, ndsTracePRIV to %d, teInstMode to %d for encoder%d",
	    call_stack_en, stall_en, itrigger_en, trace_sync_extra, trace_context, trace_priv, te_inst_mode, trace_sel);
	LOG_DEBUG("DBG_API:set teInhibitSrc to %d (1 to disable)", inhibit_src);

	dmi_read(target, &te_inst_features_reg, selected_encoder(DMI_TEINSTFEATURES));
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));

	te_inst_features_reg &= ~(DMI_TEINSTFEATURES_teInstNoAddrDiff | DMI_TEINSTFEATURES_teInstEnCallStack);
	if (call_stack_en)
		te_inst_features_reg |= (DMI_TEINSTFEATURES_teInstEnCallStack);
	if (inst_no_addr_diff)
		te_inst_features_reg |= (DMI_TEINSTFEATURES_teInstNoAddrDiff);
	te_inst_features_reg |= (DMI_TEINSTFEATURES_trTeInstExtendAddrMSB);  /* extended MSB to 64bits */

	te_ctrl_reg &= ~(DMI_TECONTROL_teEnable);
	if (inhibit_src)
		te_ctrl_reg |= (DMI_TECONTROL_teInhibitSrc);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_teInhibitSrc);
	if (stall_en)
		te_ctrl_reg |= (DMI_TECONTROL_teInstStallEn);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_teInstStallEn);
	if (itrigger_en)
		te_ctrl_reg |= (DMI_TECONTROL_teInstTrigEn);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_teInstTrigEn);
	if (trace_sync_extra)
		te_ctrl_reg |= (DMI_TECONTROL_ndsSyncExtra);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_ndsSyncExtra);
	if (trace_context)
		te_ctrl_reg |= (DMI_TECONTROL_ndsTracePID);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_ndsTracePID);
	if (trace_priv)
		te_ctrl_reg |= (DMI_TECONTROL_ndsTracePRIV);
	else
		te_ctrl_reg &= ~(DMI_TECONTROL_ndsTracePRIV);

	te_ctrl_reg &= ~(DMI_TECONTROL_teInstMode_MASK);
	te_ctrl_reg |= ((te_inst_mode << DMI_TECONTROL_teInstMode_SHIFT) & DMI_TECONTROL_teInstMode_MASK);
	dmi_write(target, selected_encoder(DMI_TEINSTFEATURES), te_inst_features_reg);
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_set_sync_mode(struct target *target,
	uint32_t  sync_mode,      /* teControl[17:16]: teSyncMode */
	uint32_t  max_interval)   /* teControl[23:20]: teSyncMax */
{
	uint32_t te_ctrl_reg;

	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));
	te_ctrl_reg &= ~DMI_TECONTROL_teSyncMode_MASK;
	te_ctrl_reg &= ~DMI_TECONTROL_teSyncMax_MASK;
	te_ctrl_reg |= ((sync_mode << DMI_TECONTROL_teSyncMode_SHIFT) & DMI_TECONTROL_teSyncMode_MASK);
	te_ctrl_reg |= ((max_interval << DMI_TECONTROL_teSyncMax_SHIFT) & DMI_TECONTROL_teSyncMax_MASK);
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_set_filter(struct target *target,
		uint32_t filtermatchinst)
{
	LOG_DEBUG("Setting filter");

	/* Enable filter control */
	uint32_t trTeFilterControl_0;
	dmi_read(target, &trTeFilterControl_0, selected_encoder(DMI_TEFILTER));
	trTeFilterControl_0 |= 0x3; /* trTeFilterEnable = 1, trTeFilterMatchPrivilege = 1 */
	dmi_write(target, selected_encoder(DMI_TEFILTER), trTeFilterControl_0);
	dmi_read(target, &trTeFilterControl_0, selected_encoder(DMI_TEFILTER));
	LOG_DEBUG("trTeFilterControl_0: 0x%x", trTeFilterControl_0);

	/* Set trTeFilteriMatchInst */
	uint32_t trTeFilterMatchInst_0;
	dmi_read(target, &trTeFilterMatchInst_0, selected_encoder(DMI_TEFILTERMATCH0));
	trTeFilterMatchInst_0 = filtermatchinst;
	dmi_write(target, selected_encoder(DMI_TEFILTERMATCH0), trTeFilterMatchInst_0);
	dmi_read(target, &trTeFilterMatchInst_0, selected_encoder(DMI_TEFILTERMATCH0));
	LOG_DEBUG("trTeFilterMatchInst_0: 0x%x", trTeFilterMatchInst_0);

	return 0;
}


static uint32_t tracer_set_timestamp_mode(struct target *target,
	uint32_t  mode_enable,         /* tsControl[0]  : tsActive */
	uint32_t  counter_enable,      /* tsControl[1]  : tsCount */
	uint32_t  stop_on_debug,       /* tsControl[3]  : tsDebug */
	uint32_t  msg_type,            /* tsControl[6:4]: tsType */
	uint32_t  prescale,            /* tsControl[9:8]: tsPrescale */
	uint32_t  nds_timestam_select) /* tsControl[15] : ndsTimestampSelect */
{
	uint32_t ts_ctrl_reg;

	LOG_DEBUG("DBG_API:set tsActive to %d, tsCount to %d, tsDebug to %d, tsType to %d, tsPrescale to %d, ndsTimestampSelect to %d for encoder%d",
		mode_enable, counter_enable, stop_on_debug, msg_type, prescale, nds_timestam_select, trace_sel);

	dmi_read(target, &ts_ctrl_reg, selected_encoder(DMI_TSCONTROL));
	ts_ctrl_reg &= ~(DMI_TSCONTROL_tsActive|DMI_TSCONTROL_tsCount|DMI_TSCONTROL_tsDebug);
	if (mode_enable)
		ts_ctrl_reg |= (DMI_TSCONTROL_tsActive);
	if (counter_enable)
		ts_ctrl_reg |= (DMI_TSCONTROL_tsCount);
	if (stop_on_debug)
		ts_ctrl_reg |= (DMI_TSCONTROL_tsDebug);

	ts_ctrl_reg &= ~DMI_TSCONTROL_tsType_MASK;
	ts_ctrl_reg |= ((msg_type << DMI_TSCONTROL_tsType_SHIFT) & DMI_TSCONTROL_tsType_MASK);
	ts_ctrl_reg &= ~DMI_TSCONTROL_tsPrescale_MASK;
	ts_ctrl_reg |= ((prescale << DMI_TSCONTROL_tsPrescale_SHIFT) & DMI_TSCONTROL_tsPrescale_MASK);
	ts_ctrl_reg &= ~DMI_TSCONTROL_ndsTimestampSelect_MASK;
	ts_ctrl_reg |=
		((nds_timestam_select << DMI_TSCONTROL_ndsTimestampSelect_SHIFT) & DMI_TSCONTROL_ndsTimestampSelect_MASK);

	dmi_write(target, selected_encoder(DMI_TSCONTROL), ts_ctrl_reg);
	dmi_read(target, &ts_ctrl_reg, selected_encoder(DMI_TSCONTROL));
	LOG_DEBUG("DBG_API: after setting %x", ts_ctrl_reg);
	return 0;
}

static uint32_t tracer_enable_itracing(struct target *target)
{
	uint32_t te_ctrl_reg;

	LOG_DEBUG("DBG_API:enable instruction trace for encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));

	te_ctrl_reg |= DMI_TECONTROL_teInstTracing;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_disable_itracing(struct target *target)
{
	uint32_t te_ctrl_reg;

	LOG_DEBUG("DBG_API:disable instruction trace for encoder%d", trace_sel);
	dmi_read(target, &te_ctrl_reg, selected_encoder(DMI_TECONTROL));

	te_ctrl_reg &= ~DMI_TECONTROL_teInstTracing;
	dmi_write(target, selected_encoder(DMI_TECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_reset_timestamp(struct target *target)
{
	uint32_t ts_ctrl_reg;

	LOG_DEBUG("DBG_API:reset timestamp for encoder%d", trace_sel);
	dmi_read(target, &ts_ctrl_reg, selected_encoder(DMI_TSCONTROL));
	ts_ctrl_reg |= (DMI_TSCONTROL_tsReset);
	dmi_write(target, selected_encoder(DMI_TSCONTROL), ts_ctrl_reg);
	return 0;
}
/*
static uint64_t tracer_get_timestamp(struct target *target)
{
	uint64_t	timestamp;
	uint32_t	tsUpper;
	uint32_t	tsUpper2;
	uint32_t	tsLower;

	while (1) {
		dmi_read(target, &tsUpper, selected_encoder(DMI_TSUPPER));
		dmi_read(target, &tsLower, selected_encoder(DMI_TSLOWER));
		dmi_read(target, &tsUpper2, selected_encoder(DMI_TSUPPER));
		if (tsUpper2 == tsUpper)
			break;
	}
	timestamp = tsUpper;
	timestamp <<= 32;
	timestamp |= tsLower;
	LOG_DEBUG("DBG_API:encoder%d: timestamp=0x%lx", trace_sel, timestamp);
	return timestamp;
}
*/
static int tracer_set_atbid(struct target *target, uint32_t atbid)
{
	uint32_t    atb_ctrl_reg;
	if ((atbid == 0) || (atbid >= 0x70)) {
		LOG_DEBUG("DBG_API:ERROR:atbID %d is invalid", atbid);
		return -1;
	}
	dmi_read(target, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	atb_ctrl_reg &= ~DMI_ATBCONTROL_atbId_MASK;
	atb_ctrl_reg |= ((atbid << DMI_ATBCONTROL_atbId_SHIFT) & DMI_ATBCONTROL_atbId_MASK);
	dmi_write(target, selected_encoder(DMI_ATBCONTROL), atb_ctrl_reg);
	return 0;
}
/*
static uint32_t tracer_get_atbid(struct target *target)
{
	uint32_t atb_ctrl_reg;
	uint32_t atbid;

	dmi_read(target, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	atbid = (atb_ctrl_reg & DMI_ATBCONTROL_atbId_MASK) >> DMI_ATBCONTROL_atbId_SHIFT;
	LOG_DEBUG("DBG_API:get atbId 0x%x for encoder%0d", atbid, trace_sel);
	return atbid;
}
*/
static uint32_t tracer_activate_tbuf(struct target *target)
{
	uint32_t tf_ctrl_reg;
	uint32_t timeout_limit;
	uint32_t timeout_counter;

	riscv013_info_t *info = get_info(target);
	/* info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS); */
	if (info->abits <= 0x7) {
		LOG_DEBUG("DBG_API:ERROR:value of dtmcs.ABITS (%d) is smaller than or equal to 7", info->abits);
		LOG_DEBUG("no trace support is present in the target");
		return 0;
	}
	LOG_DEBUG("DBG_API:activate tbuf");
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	tf_ctrl_reg |= DMI_TFCONTROL_atbActive;
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	timeout_limit   = 50;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
		if (tf_ctrl_reg & DMI_TFCONTROL_atbActive)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfActive to be set");

	/* Read initial trace buffer registers ??? */
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	return 0;
}

static uint32_t tracer_deactivate_tbuf(struct target *target)
{
	uint32_t tf_ctrl_reg;
	uint32_t timeout_limit;
	uint32_t timeout_counter;

	LOG_DEBUG("DBG_API:deactivate tbuf");
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	tf_ctrl_reg &= ~DMI_TFCONTROL_atbActive;
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	timeout_limit   = 50;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
		if ((tf_ctrl_reg & DMI_TFCONTROL_atbActive) == 0)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfActive to be cleared");

	return 0;
}

static uint32_t tracer_tbuf_enable_recording(struct target *target)
{
	uint32_t tf_ctrl_reg, teWrap;
	uint32_t timeout_limit;
	uint32_t timeout_counter;

	LOG_DEBUG("DBG_API:tbuf:enable recording");
#if 0
	dmi_write(target, DMI_TERAMRP, 0x0);
	/* dummy read fifo (clear fifo) */
	uint32_t fifo_words, i, etb_data, etb_rptr;
	fifo_words = TB_RAM_SIZE;
	fifo_words >>= 2;
	for (i = 0; i < fifo_words; i++) {
		dmi_read(target, &etb_data, DMI_TERAMDATA);
		dmi_read(target, &etb_rptr, DMI_TERAMRP);
	  LOG_DEBUG("etb_rptr = 0x%x", etb_rptr);
	}
#endif

	/* reset teRamWP before recording */
	uint32_t teramlimitlow;
	dmi_read(target, &teramlimitlow, DMI_TFRAMLIMIT);
	TB_RAM_SIZE = teramlimitlow + 4;
	LOG_DEBUG("bufsize(etb_wptr) = 0x%x(%d)", TB_RAM_SIZE, TB_RAM_SIZE);
	dmi_write(target, DMI_TERAMWP, 0);
	timeout_limit   = 100;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &teWrap, DMI_TERAMWP);
		if (teWrap == 0)
			break;
		timeout_counter++;
	}

	dmi_write(target, DMI_TERAMRP, 0x0);

	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	tf_ctrl_reg |= DMI_TFCONTROL_tfEnable;
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	timeout_limit   = 100;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
		if (tf_ctrl_reg & DMI_TFCONTROL_tfEnable)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfEnable to be set");

	return 0;
}

static uint32_t tracer_tbuf_disable_recording(struct target *target)
{
	uint32_t tf_ctrl_reg;
	uint32_t timeout_limit;
	uint32_t timeout_counter;

	LOG_DEBUG("DBG_API:tbuf:disable recording");
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	tf_ctrl_reg &= ~DMI_TFCONTROL_tfEnable;
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	timeout_limit   = 100;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
		if ((tf_ctrl_reg & DMI_TFCONTROL_tfEnable) == 0)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfEnable to be cleared");

	/* Poll tfControl.tfEmpty in all Trace Funnels. Wait until all are 1 */
	timeout_limit   = 1000;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
		if (tf_ctrl_reg & DMI_TFCONTROL_tfEmpty)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfEmpty to be set after disabling tbuf");

	return 0;
}

static uint32_t tracer_tbuf_set_trace_format(struct target *target, uint32_t format)
{
	uint32_t tf_ctrl_reg;

	LOG_DEBUG("DBG_API:tbuf:recording format is set to %d", format);
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	tf_ctrl_reg &= ~DMI_TFCONTROL_teFormat_MASK;
	tf_ctrl_reg |= ((format << DMI_TFCONTROL_teFormat_SHIFT) & DMI_TFCONTROL_teFormat_MASK);
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	return 0;
}

static uint32_t tracer_tbuf_set_stop_on_wrap(struct target *target, uint32_t mode)
{
	uint32_t tf_ctrl_reg;

	LOG_DEBUG("DBG_API:tbuf:tfStopOnWrap is set to %d", mode);
	dmi_read(target, &tf_ctrl_reg, DMI_TFCONTROL);
	if (mode)
		tf_ctrl_reg |= DMI_TFCONTROL_tfStopOnWrap;
	else
		tf_ctrl_reg &= ~DMI_TFCONTROL_tfStopOnWrap;
	dmi_write(target, DMI_TFCONTROL, tf_ctrl_reg);

	return 0;
}

static uint32_t tracer_tbuf_get_teWrap(struct target *target)
{
	uint32_t teWrap, ifWrap;

	dmi_read(target, &teWrap, DMI_TERAMWP);
	ifWrap = (teWrap & DMI_TERAMWP_teWrap);
	return ifWrap;
}

static uint32_t tracer_enable_multiplexer(struct target *target)
{
	uint32_t  trFunnel_ctrl_reg, trFunnel_Impl_reg;
	if (nds_tracer_multiplexer == 0)
		return 0;

	dmi_read(target, &trFunnel_Impl_reg, TMUX_trFunnelImpl);
	LOG_DEBUG("DBG_API:tracer_enable_multiplexer, trFunnelImpl = 0x%x", trFunnel_Impl_reg);
	dmi_read(target, &trFunnel_ctrl_reg, TMUX_trFunnelControl);
	trFunnel_ctrl_reg |= TMUX_trFunnelControl_trFunnelActive;
	trFunnel_ctrl_reg |= TMUX_trFunnelControl_trFunnelEnable;
	dmi_write(target, TMUX_trFunnelControl, trFunnel_ctrl_reg);

	uint32_t  tmux_ctrl_reg;
	dmi_read(target, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
	LOG_DEBUG("DBG_API: tmux_ctrl_reg 0x%x", tmux_ctrl_reg);
	return 0;
}

static int ndsv5_tracer_buffer_init(void)
{
	if (p_etb_buf_start) {
		free(p_etb_buf_start);
		p_etb_buf_start = NULL;
	}
	p_etb_buf_start = (uint32_t *)malloc(TRACER_TMP_BUFSIZE);
	p_etb_buf_end = p_etb_buf_start;
	p_etb_buf_end += (TRACER_TMP_BUFSIZE >> 2);

	/* reset to buffer start */
	p_etb_wptr = p_etb_buf_start;
	return 0;
}

static int ndsv5_tracer_buffer_free(void)
{
	if (p_etb_buf_start) {
		free(p_etb_buf_start);
		p_etb_buf_start = NULL;
	}
	return 0;
}

uint32_t ndsv5_tracer_setting(struct target *target)
{
	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	/* Activate trace encoder and set register */
	tracer_activate_encoder(target);

	/* Set teInstMode to 3, ndsTracePRIV to 0, ndsTracePID to 0, ndsSyncExtra to 0,
		teInstTrigEn to 1, teInstStallEn to 0, teInstEnCallStack to 0 */
	tracer_set_itracing_mode(target, 0, nds_teInstNoAddrDiff, nds_teInhibitSrc, 0, 1, 0, 0, 0, nds_trTeInstMode);

	if (nds_tracer_action == CSR_MCONTROL_ACTION_TRACE_OFF)
		tracer_enable_itracing(target);

	/* Set teSyncMode to 1, teSyncMax to 4 */
	tracer_set_sync_mode(target, 0x1, nds_trTeSyncMax);

	if (nds_trTeFilteriMatchInst)
		tracer_set_filter(target, nds_trTeFilteriMatchInst);

	tracer_reset_timestamp(target);

	if (nds_timestamp_on) {
		uint32_t tsDebug, tsType, tsPrescale, tsSelect;
		if (nds_trTsControl == 0) {
			/* Activate timestamp and enable internal timestamp counter, set tsDebug to 0,
				tsType to 1(externel), tsPrescale to 0, ndsTimestampSelect to 1(trTsEnable) */
			tsDebug = 0;
			tsType = 3;
			tsPrescale = 0;
			tsSelect = 1;
		} else {
			if (nds_trTsControl & DMI_TSCONTROL_tsDebug)
				tsDebug = 1;
			else
				tsDebug = 0;
			tsType = (nds_trTsControl & DMI_TSCONTROL_tsType_MASK) >> DMI_TSCONTROL_tsType_SHIFT;
			tsPrescale = (nds_trTsControl & DMI_TSCONTROL_tsPrescale_MASK) >> DMI_TSCONTROL_tsPrescale_SHIFT;
			tsSelect =
				(nds_trTsControl & DMI_TSCONTROL_ndsTimestampSelect_MASK) >> DMI_TSCONTROL_ndsTimestampSelect_SHIFT;
		}
		tracer_set_timestamp_mode(target, 1, 1, tsDebug, tsType, tsPrescale, tsSelect);
	}

	tracer_set_atbid(target, 1);

	/* Activate trace buffer and set its register */
	tracer_activate_tbuf(target);
	tracer_tbuf_set_trace_format(target, NEXUS_TRACE_FORMAT);
	if (nds_tracer_stop_on_wrap == 0)
		tracer_tbuf_set_stop_on_wrap(target, 0);
	else
		tracer_tbuf_set_stop_on_wrap(target, 1);

	/* Enable trace buffer for recording */
	tracer_tbuf_enable_recording(target);

	/* Enable trace encoder and wait for trace-on event */
	tracer_enable_encoder(target);

	return 0;
}

uint32_t ndsv5_tracer_all_cores_setting(void)
{
	if (nds_tracer_on != 0)
		return 0;
	ndsv5_tracer_buffer_init();

	struct target *target = all_targets;
	uint32_t coreid;
	if (nds_tracer_capability == 0)
		ndsv5_tracer_capability_check(target);

	if (nds_tracer_capability != 1) {
		LOG_DEBUG("TB_DEVARCH != 0x4200");
		return 0;
	}

	for (target = all_targets; target; target = target->next) {
		if (target->smp) {
			struct target_list *tlist;
			foreach_smp_target(tlist, target->smp_targets) {
				struct target *t = tlist->target;
				riscv_info_t *r = riscv_info(t);
				coreid = r->current_hartid;
				LOG_DEBUG("smp-coreid: %d ", coreid);
				if (((0x01 << coreid) & nds_tracer_active_id) != 0) {
					tracer_enable_multiplexer(t);
					ndsv5_tracer_setting(t);
				}
			}
		} else {
			RISCV_INFO(r);
			coreid = r->current_hartid;
			LOG_DEBUG("amp-coreid: %d ", coreid);
			if (((0x01 << coreid) & nds_tracer_active_id) != 0) {
				tracer_enable_multiplexer(target);
				ndsv5_tracer_setting(target);
			}
		}
	}

	nds_tracer_on = 1;
	return 0;
}

uint32_t tracer_disable_encoder_all(void)
{
	struct target *target = all_targets;
	uint32_t coreid;

	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_info_t *r = riscv_info(t);
			coreid = r->current_hartid;
			if (((0x01 << coreid) & nds_tracer_active_id) != 0)
				tracer_disable_encoder(t);
		}
		return 0;
	}

	for (target = all_targets; target; target = target->next) {
		RISCV_INFO(r);
		coreid = r->current_hartid;
		if (((0x01 << coreid) & nds_tracer_active_id) != 0)
			tracer_disable_encoder(target);
	}
	return 0;
}

uint32_t tracer_deactivate_encoder_all(void)
{
	struct target *target = all_targets;
	uint32_t coreid;

	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_info_t *r = riscv_info(t);
			coreid = r->current_hartid;
			if (((0x01 << coreid) & nds_tracer_active_id) != 0)
				tracer_deactivate_encoder(t);
		}
		return 0;
	}
	for (target = all_targets; target; target = target->next) {
		RISCV_INFO(r);
		coreid = r->current_hartid;
		if (((0x01 << coreid) & nds_tracer_active_id) != 0)
			tracer_deactivate_encoder(target);
	}
	return 0;
}

uint32_t ndsv5_tracer_disable(struct target *target)
{
	ndsv5_tracer_buffer_free();

	/* Disable instruction tracing */
	tracer_disable_itracing(target);

	/* Disable trace encoder and trace buffer */
	tracer_disable_encoder_all();
	tracer_tbuf_disable_recording(target);

	/* Deactivate trace encoder and trace buffer */
	tracer_deactivate_encoder_all();

	tracer_deactivate_tbuf(target);

	nds_tracer_on = 0;
	return 0;
}

static int ndsv5_tracer_read_etb(struct target *target)
{
	uint32_t etb_rptr, etb_wptr, etb_data;
	uint32_t fifo_words, i;

	/* Disable instruction tracing */
	tracer_disable_itracing(target);

	/* Disable trace encoder and trace buffer */
	tracer_disable_encoder_all();
	tracer_tbuf_disable_recording(target);

	nds_tracer_on = 0;

	dmi_read(target, &etb_wptr, DMI_TERAMWP);
	dmi_read(target, &etb_rptr, DMI_TERAMRP);
	LOG_DEBUG("1.etb_wptr = 0x%x, etb_rptr = 0x%x", etb_wptr, etb_rptr);

	if (etb_wptr & DMI_TERAMWP_teWrap) {
		LOG_DEBUG("teWrap");
		etb_wptr &= ~DMI_TERAMWP_teWrap;
		etb_rptr = etb_wptr;
		fifo_words = TB_RAM_SIZE;
	} else {
		etb_rptr = 0;
		fifo_words = etb_wptr;
	}
	fifo_words >>= 2;
	dmi_write(target, DMI_TERAMRP, etb_rptr);
	dmi_read(target, &etb_wptr, DMI_TERAMWP);
	dmi_read(target, &etb_rptr, DMI_TERAMRP);
	LOG_DEBUG("2.etb_wptr = 0x%x, etb_rptr = 0x%x", etb_wptr, etb_rptr);

	/* copy pkt from ETB to tmp buffer */
	uint32_t *pcurr_wptr = (uint32_t *)p_etb_wptr;
	pcurr_wptr += fifo_words;
	LOG_DEBUG("p_etb_wptr = 0x%lx, p_etb_buf_end = 0x%lx, fifo_words = 0x%x",
			(unsigned long)p_etb_wptr, (unsigned long)p_etb_buf_end, fifo_words);
	if (pcurr_wptr >= p_etb_buf_end) {
		LOG_DEBUG("PKT BUF FULL !! need to dump file");
		return ERROR_FAIL;
	} else {
		for (i = 0; i < fifo_words; i++) {
			dmi_read(target, &etb_data, DMI_TERAMDATA);
			*p_etb_wptr++ = etb_data;
		}
		dmi_read(target, &etb_wptr, DMI_TERAMWP);
		dmi_read(target, &etb_rptr, DMI_TERAMRP);

		LOG_DEBUG("3.etb_wptr = 0x%x, etb_rptr = 0x%x, fifo_words = 0x%x", etb_wptr, etb_rptr, fifo_words);
	}

	dmi_write(target, DMI_TERAMWP, 0x0);
	return ERROR_OK;
}

int ndsv5_tracer_dumpfile(struct target *target, char *pFileName)
{
	char filename[2048];
	/* pkt output path depend on log file path */

	memset(filename, 0, sizeof(filename));
	if (ndsv5_dump_trace_folder) {
		LOG_DEBUG("dump_trace_folder: %s", ndsv5_dump_trace_folder);
		strcpy(filename, ndsv5_dump_trace_folder);
	}
	strcat(filename, pFileName);
	LOG_INFO("Dump pkt to %s", filename);

	uint32_t total_pkt_bytes = 0;
	FILE *pPacketFile = NULL;
	pPacketFile = fopen(filename, "wb");
	if (pPacketFile == NULL)
		return ERROR_FAIL;

	/* Prepare Parameter
	 * byte[0]: srcbits
	 * byte[1]: teInstNoAddrDiff
	 * byte[2]: VALEN
	 * byte[3]: version

	 (1) srcbits 6
	 SRC Source of Message. Field Width(4)
	 Hart index or trace ID. This field is used for multi-hart/core trace

	 (2) teInstNoAddrDiff = 0
	 UADDR Unique Portion of Branch Target Address
	 This field is produced by XOR-ing the branch target address with the most recent FADDR/UADDR/PC.
	 When trTeInstNoAddrDiff is set, no XOR is performed and this field becomes FADDR.

	 (3) VALEN, RISC-V ISA defines 3 different virtual memory addressing modes: Sv39, Sv48 and Sv57.
	 In each of these modes the most significant bit (38, 47 or 56) is extended on all higher bits.
	 It means that there is no need to send full 64-bit addresses and only report 39/48 or 57 bits of an address.
	 */

	unsigned char parameter[4];
	uint32_t idx;

	if (nds_teInhibitSrc == 0)
		parameter[0] = 6;
	else
		parameter[0] = 0;
	parameter[1] = (unsigned char) nds_teInstNoAddrDiff;
	parameter[2] = 57;
	parameter[3] = TRACER_VERSION;
	/* Write parameters into file */
	for (idx = 0; idx < 4; idx++)
		fputc(parameter[idx], pPacketFile);

	char *pbuf_start = (char *)p_etb_buf_start;
	/* get data from ETB */
	ndsv5_tracer_read_etb(target);

	if (p_etb_wptr != p_etb_buf_start) {
		total_pkt_bytes = (p_etb_wptr - p_etb_buf_start);
		total_pkt_bytes <<= 2;
	}
	LOG_DEBUG("p_etb_wptr = 0x%lx, total_pkt_bytes = 0x%x",
		(unsigned long)p_etb_wptr, total_pkt_bytes);

	if (total_pkt_bytes)
		fwrite(pbuf_start, 1, total_pkt_bytes, pPacketFile);
	else
		LOG_DEBUG("Buffer empty!!");

	fclose(pPacketFile);

	/* copy another ntracer.log */
	char log_filename[256], write_line[32];
	char *pLogName = (char *)&log_filename[0];
	memcpy(&log_filename[0], pFileName, strlen((char *)pFileName));
	memcpy(&log_filename[strlen((char *)pFileName)], ".log", 5);
	pPacketFile = fopen(pLogName, "wb");

	uint32_t i, *pData;
	/* Write parameters into file */
	pData = (uint32_t *)&parameter[0];
	sprintf(&write_line[0], "%08x\n", *pData++);
	fwrite(&write_line[0], 1, strlen(&write_line[0]), pPacketFile);

	/* Write packets into file */
	pData = (uint32_t *)pbuf_start;
	for (i = 0; i < total_pkt_bytes/4; i++) {
		sprintf(&write_line[0], "%08x\n", *pData++);
		fwrite(&write_line[0], 1, strlen(&write_line[0]), pPacketFile);
	}
	fclose(pPacketFile);
	/* reset to buffer start */
	p_etb_wptr = p_etb_buf_start;

	return ERROR_OK;
}

int ndsv5_tracer_polling(struct target *target)
{
	if (nds_tracer_on == 0)
		return ERROR_FAIL;
	LOG_DEBUG("ndsv5_tracer_polling...");
	if (tracer_tbuf_get_teWrap(target) == 0) {
		LOG_DEBUG("ndsv5_tracer_polling exit");
		return ERROR_OK;
	}
	if (nds_tracer_stop_on_wrap == 0) {
		LOG_DEBUG("teWrap but nds_tracer_stop_on_wrap=0");
		return ERROR_OK;
	}

	struct target_type *tt = get_target_type(target);
	if (tt->halt(target) != ERROR_OK)
		LOG_ERROR("tt->halt() ERROR");
	target->debug_reason = DBG_REASON_TRACE_BUFFULL;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	LOG_DEBUG("DBG_REASON_TRACE_BUFFULL, %d", target->debug_reason);
	return ERROR_OK;
}

int ndsv5_tracer_capability_check(struct target *target)
{
	uint32_t  devarch_reg;
	dmi_read(target, &devarch_reg, TB_DEVARCH);
	LOG_DEBUG("TB_DEVARCH = 0x%x", devarch_reg);
	/* TB_DEVARCH, 15:0 ARCHID Architecture ID RO 0x4200 */
	if ((devarch_reg & 0xFFFF) == 0x4200) {
		dmi_read(target, &devarch_reg, TMUX_DEVARCH);
		LOG_DEBUG("TMUX_DEVARCH = 0x%x", devarch_reg);
		if ((devarch_reg & 0xFFFF) == 0x4D00)
			nds_tracer_multiplexer = 1;
		nds_tracer_capability = 1;
		return ERROR_OK;
	}
	nds_tracer_capability = 0xFF;
	return ERROR_FAIL;
}


/* ============================================================================= */
/*    packet parser                                                              */
/*         input: packet file (pkt.log)                                          */
/*         output: packet-decoded file (pkt.log-de)                              */
/* ============================================================================= */
#define TRACER_DECODE_MSG(x)  LOG_DEBUG x
#define PACKET_BUF_SIZE       0x100000  /* 1MB */

char *gpTcodeName[] = {
	"0",
	"1",
	"OwnershipTrace",
	"DirectBranch",
	"IndirectBranch",
	"5", "6", "7",
	"Error",
	"ProgramTraceSync",
	"10",
	"DirectBranchSync",
	"IndirectBranchSync",
	"13", "14", "15", "16", "17", "18", "19",
	"20", "21", "22", "23", "24", "25", "26",
	"ResourceFull",
	"IndirectBranchHist",
	"IndirectBranchHistSync",
	"30", "31", "32",
	"ProgramCorrelation",
	"",
};

char *gpSyncName[] = {
	"0",
	"Exit_from_Reset",
	"Period_Msg",
	"Exit_from_Debug",
	"4",
	"Trace_Enable",
	"Watchpoint",
	"FIFO_Overrun",
	"8",
	"Exit_from_Power_down",
	"",
};

char *gpEVcodeName[] = {
	"Entry_Debug_mode",
	"Entry_LowPower_mode",
	"2", "3",
	"Program_Trace_Disabled",
	"Process_ID_Change",
	"6", "7",
	"Privilege_Level_Change",
	"9", "10", "11", "12", "13", "14",
	"Entry_Prohibited_Region",
	"",
};

static char pkt_decoded_filename[256];

static FILE *gpPktDecodedFile;
static char pkt_decoded_data[256];
unsigned int teSrcBits = 6;
unsigned int teInstNoAddrDiff;
unsigned long recent_PC[64];

unsigned long ndsv5_get_variable_length_field(unsigned char **pSrcPkt, unsigned int PktSize)
{
	unsigned char *pCurrPkt = (unsigned char *)*pSrcPkt;
	unsigned char cur_byte;
	unsigned int shift_bits = 0;
	unsigned long ret_data = 0;

	while (PktSize) {
		cur_byte = *pCurrPkt;
		ret_data |= ((cur_byte >> 2) << shift_bits);
		shift_bits += 6;
		if (((cur_byte & MSEO_MASK) == MSEO_END_MSG) ||
			  ((cur_byte & MSEO_MASK) == MSEO_END_FIELD)) {
			break;
		}
		pCurrPkt++;
		PktSize--;
	}
	*pSrcPkt = pCurrPkt;
	return ret_data;
}

static unsigned int ndsv5_tracer_decode_pkt(unsigned char *pSrcPkt, unsigned int PktSize, char *p_text)
{
	unsigned char *pCurrPkt = (unsigned char *)pSrcPkt;
	unsigned char cur_byte;
	unsigned int if_start = 0, t_code;
	unsigned int src_id = 0, sync_id = 0, icnt = 0, btype = 0;
	unsigned long context = 0;
	unsigned int rcode = 0, RDATA = 0, hist = 0, evcode = 0, cdf = 0;
	unsigned long addrs = 0, xor_addrs, field_data;
	unsigned int decoded_bytes = 0;

	while (1) {
		cur_byte = *pCurrPkt++;
		if ((pCurrPkt - pSrcPkt) > PktSize)
			break;

		if (((cur_byte & MSEO_MASK) == MSEO_START) && (if_start == 0)) {
			if_start = 1;
			t_code = abs(cur_byte >> 2);

			if (teSrcBits)
				src_id = (unsigned int)(*pCurrPkt++ >> 2);

			if ((pCurrPkt - pSrcPkt) > PktSize)
				break;

			/* ProgramTraceSync & IndirectBranch & DirectBranch ... */
			if ((t_code == TCODE_ProgramTraceSync) ||
				(t_code == TCODE_IndirectBranch) ||
				(t_code == TCODE_DirectBranch) ||
				(t_code == TCODE_ResourceFull) ||
				(t_code == TCODE_OwnershipTrace) ||
				(t_code == TCODE_ProgramCorrelation) ||
				(t_code == TCODE_IndirectBranchHist) ||
				(t_code == TCODE_DirectBranchSync) ||
				(t_code == TCODE_IndirectBranchSync) ||
				(t_code == TCODE_IndirectBranchHistSync)) {

				field_data = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));

				if (t_code == TCODE_ProgramTraceSync) {
					sync_id = (unsigned int)(field_data & 0x0F);
				  icnt    = (unsigned int)((field_data >> 4) & 0xFFFF);
				  pCurrPkt++;
				  if ((pCurrPkt - pSrcPkt) > PktSize)
						break;
				  addrs = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));

					recent_PC[src_id] = addrs;
					/* Full PC Address (without bit[0] */
					addrs <<= 1;
				} else if ((t_code == TCODE_DirectBranchSync) ||
					(t_code == TCODE_IndirectBranchSync) ||
					(t_code == TCODE_IndirectBranchHistSync)) {
					sync_id = (unsigned int)(field_data & 0x0F);
					if ((t_code == TCODE_IndirectBranchSync) || (t_code == TCODE_IndirectBranchHistSync)) {
						btype = (unsigned int)((field_data >> 4) & 0x03);
						pCurrPkt++;
					  if ((pCurrPkt - pSrcPkt) > PktSize)
							break;
					  icnt = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));
					} else {
						icnt = (unsigned int)((field_data >> 4) & 0xFFFF);
					}
					pCurrPkt++;
				  if ((pCurrPkt - pSrcPkt) > PktSize)
						break;
				  addrs = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));

				  if (t_code == TCODE_IndirectBranchHistSync) {
						pCurrPkt++;
						if ((pCurrPkt - pSrcPkt) > PktSize)
							break;
						hist = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));
					}
				} else if ((t_code == TCODE_IndirectBranch) ||
					(t_code == TCODE_IndirectBranchHist)) {
					btype = (unsigned int)(field_data & 0x03);
					icnt  = (unsigned int)((field_data >> 2) & 0xFFFF);
					pCurrPkt++;
					if ((pCurrPkt - pSrcPkt) > PktSize)
						break;
					xor_addrs = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));
					if (teInstNoAddrDiff == 1)
						addrs = xor_addrs;
					else
						addrs = (recent_PC[src_id] ^ xor_addrs);

					recent_PC[src_id] = addrs;
					/* Full PC Address (without bit[0] */
					addrs <<= 1;
					if (t_code == TCODE_IndirectBranchHist) {
						pCurrPkt++;
						if ((pCurrPkt - pSrcPkt) > PktSize)
							break;
						hist = ndsv5_get_variable_length_field(&pCurrPkt, (PktSize - (pCurrPkt - pSrcPkt)));
					}
				} else if (t_code == TCODE_DirectBranch)
					icnt = (unsigned int)(field_data & 0xFFFF);
				else if (t_code == TCODE_OwnershipTrace)
					context = (unsigned long)(field_data & 0x1FFFF);
				else if (t_code == TCODE_ResourceFull) {
					rcode = (unsigned int)(field_data & 0xf);
					RDATA = (unsigned int)(field_data >> 4);
				} else if (t_code == TCODE_ProgramCorrelation) {
					evcode = (unsigned int)(field_data & 0x0F);
					cdf    = (unsigned int)((field_data >> 4) & 0x3);
					icnt = (unsigned int)(field_data >> 6);
				}

				/* ICNT => Number of 16-bit half-word instruction data executed. */
				icnt <<= 1;

				if (p_text) {
					/* IndirectBranch */
					if (t_code == TCODE_IndirectBranch)
						sprintf(p_text, "\n%s src: %d  btype: %d  icnt: 0x%x  addrs: 0x%lx ",
						    gpTcodeName[t_code], src_id, btype, icnt, addrs);
					/* IndirectBranchSync */
					else if (t_code == TCODE_IndirectBranchSync)
						sprintf(p_text, "\n%s src: %d  sync: %d(%s)  btype: %d  icnt: 0x%x  addrs: 0x%lx ",
						    gpTcodeName[t_code], src_id, sync_id, gpSyncName[sync_id], btype, icnt, addrs);
					/* DirectBranch */
					else if (t_code == TCODE_DirectBranch)
						sprintf(p_text, "\n%s src: %d  icnt: 0x%x ",
						    gpTcodeName[t_code], src_id, icnt);
					/* DirectBranchSync */
					else if (t_code == TCODE_DirectBranchSync)
						sprintf(p_text, "\n%s src: %d  sync: %d(%s)  icnt: 0x%x ",
						    gpTcodeName[t_code], src_id, sync_id, gpSyncName[sync_id], icnt);
					/* OwnershipTrace */
					else if (t_code == TCODE_OwnershipTrace)
						sprintf(p_text, "\n%s src: %d  process: 0x%lx ",
						    gpTcodeName[t_code], src_id, context);
					/* ResourceFull */
					else if (t_code == TCODE_ResourceFull) {
						if (rcode == 1)
							sprintf(p_text, "\n%s src: %d  rcode: %d  hist: 0x%x ",
						    gpTcodeName[t_code], src_id, rcode, RDATA);
						else
							sprintf(p_text, "\n%s src: %d  rcode: %d  RDATA: 0x%x ",
						    gpTcodeName[t_code], src_id, rcode, RDATA);
					}
					/* IndirectBranchHist */
					else if (t_code == TCODE_IndirectBranchHist)
						sprintf(p_text, "\n%s src: %d  btype: %d  icnt: 0x%x  addrs: 0x%lx  hist: 0x%x ",
						    gpTcodeName[t_code], src_id, btype, icnt, addrs, hist);
					/* IndirectBranchHistSync */
					else if (t_code == TCODE_IndirectBranchHistSync)
						sprintf(p_text, "\n%s src: %d  sync: %d(%s)  btype: %d  icnt: 0x%x  addrs: 0x%lx  hist: 0x%x ",
						    gpTcodeName[t_code], src_id, sync_id, gpSyncName[sync_id], btype, icnt, addrs, hist);
					/* ProgramCorrelation */
					else if (t_code == TCODE_ProgramCorrelation)
						sprintf(p_text, "\n%s src: %d  evcode: %d(%s)  icnt: 0x%x  cdf: %d ",
						    gpTcodeName[t_code], src_id, evcode, gpEVcodeName[evcode], icnt, cdf);
					/* ProgramTraceSync */
					else
						sprintf(p_text, "\n%s src: %d  sync: %d(%s)  icnt: 0x%x  addrs: 0x%lx ",
						    gpTcodeName[t_code], src_id, sync_id, gpSyncName[sync_id], icnt, addrs);
				}
				if ((*pCurrPkt & MSEO_MASK) == MSEO_END_MSG)
					return (unsigned int)(pCurrPkt - pSrcPkt + 1);
			}

		} else if (((cur_byte & MSEO_MASK) == MSEO_END_MSG) && (if_start)) {
			/* finish */
			break;
		}

	}
	decoded_bytes = (pCurrPkt - pSrcPkt);
	if (decoded_bytes > PktSize)
		return 0;
	return decoded_bytes;
}

unsigned int ndsv5_tracer_read_logfile(unsigned char *curr_buf, FILE *pPacketFile)
{
	unsigned int pkt_value = 0, *dst_buf, read_size = 0;
	char pPktBuf[64];
	int result;

	dst_buf = (unsigned int *)curr_buf;

	/* get data from .log file */
	while (fgets(pPktBuf, 64, pPacketFile) != NULL) {
		result = sscanf(pPktBuf, "%x", &pkt_value);
		if (result != 1)
			break;
		*dst_buf++ = pkt_value;
		read_size += 4;
	}
	return read_size;
}

int ndsv5_tracer_decode_pktfile(char *pPktFileName)
{
	unsigned long buffer_size = PACKET_BUF_SIZE;
	unsigned long read_size = 0, decoded_bytes = 0;
	unsigned int cur_idx = 0, i;
	unsigned char cur_data;
	unsigned char *pPktBuf, *curr_buf;
	char *pOutFileName = (char *)&pkt_decoded_filename[0];
	FILE *pPacketFile = NULL;
	char *p_text = &pkt_decoded_data[0];
	char tmp_text[256];
	char *p_tmp_text = &tmp_text[0];

	memcpy(&pkt_decoded_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&pkt_decoded_filename[strlen((char *)pPktFileName)], "-de", 4);
	pPacketFile = fopen(pPktFileName, "rb");
	gpPktDecodedFile = fopen(pOutFileName, "wb");
	pPktBuf = (unsigned char *)malloc(PACKET_BUF_SIZE);
	if (pPktBuf == NULL) {
		TRACER_DECODE_MSG(("ERROR!! packet buffer !!"));
		return -1;
	}

	curr_buf = pPktBuf;
	if (pPacketFile) {
		char *ret;
		ret = strstr(pPktFileName, ".log");
		if (ret)
			/* get data from .log file */
			read_size = ndsv5_tracer_read_logfile(curr_buf, pPacketFile);
		else
			/* get data from raw-pkt file */
			read_size = fread(curr_buf, 1, buffer_size, pPacketFile);

		/* decode Parameter
		* byte[0]: srcbits
		* byte[1]: teInstNoAddrDiff
		* byte[2]: VALEN
		* byte[3]: version
		*/
		teSrcBits = curr_buf[0];
		teInstNoAddrDiff  = curr_buf[1];
		curr_buf += 4;

		/* TRACER_DECODE_MSG(("read_size = 0x%lx, curr_buf[0] = 0x%x\n", read_size, curr_buf[0])); */
		while (read_size) {
			p_text[0] = 0;
			decoded_bytes = ndsv5_tracer_decode_pkt(curr_buf, read_size, p_text);
			/*TRACER_DECODE_MSG(("\n:pkt: [%d] ", cur_idx));*/
			p_tmp_text = &tmp_text[0];
			sprintf(p_tmp_text, "\n:pkt: [%d] ", cur_idx);
			p_tmp_text += strlen((char *)p_tmp_text);
			for (i = 0; i < decoded_bytes; i++) {
				cur_data = (unsigned char)curr_buf[i];
				sprintf(p_tmp_text, "%02x", cur_data);
				p_tmp_text += 2;
			}
			/* TRACER_DECODE_MSG(("%s", p_text)); */
			fwrite((char *)&tmp_text[0], 1, strlen((char *)tmp_text), gpPktDecodedFile);

			if (decoded_bytes) {
				if (gpPktDecodedFile)
					fwrite((char *)pkt_decoded_data, 1, strlen((char *)pkt_decoded_data), gpPktDecodedFile);

			} else {
				TRACER_DECODE_MSG(("ERROR!! packet decode ERROR !!"));
				break;
			}
			/* TRACER_DECODE_MSG(("read_size=0x%lx, decoded_bytes=0x%lx", read_size, decoded_bytes)); */
			if (read_size >= decoded_bytes)
				read_size -= decoded_bytes;
			else
				read_size = 0;
			/* if (cur_idx >= 4090)
				break; */
			curr_buf += decoded_bytes;
			cur_idx += decoded_bytes;
		}

	}

	if (pPacketFile)
		fclose(pPacketFile);
	if (gpPktDecodedFile)
		fclose(gpPktDecodedFile);
	free(pPktBuf);

	return 0;
}

int ndsv5013_hart_count(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return 1;

	if (dm->ndsv5_hart_count < 0) {
		RISCV_INFO(r);
		int bak_hartid = r->current_hartid;
		for (int i = 0; i < RISCV_MAX_HARTS; ++i) {
			r->current_hartid = i;
			if (riscv013_select_current_hart(target) != ERROR_OK)
				return 1;

			uint32_t s;
			if (dmstatus_read(target, &s, true) != ERROR_OK)
				return 1;
			if (get_field(s, DM_DMSTATUS_ANYNONEXISTENT))
				break;
			dm->ndsv5_hart_count = i + 1;
		}
		r->current_hartid = bak_hartid;
		if (riscv013_select_current_hart(target) != ERROR_OK)
			return 1;
	}
	return dm->ndsv5_hart_count;
}

#endif /* _NDS_V5_ONLY_ */
/********************************************************************/

