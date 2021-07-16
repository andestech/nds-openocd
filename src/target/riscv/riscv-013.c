/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

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
#include "helper/list.h"
#include "riscv.h"
#include "rtos/riscv_debug.h"
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

#if _NDS_V5_ONLY_
#include "target/nds32_v5.h"
#include "target/nds32_v5_ace.h"
static bool isAceCsrEnable = false;
struct riscv_batch *busmode_batch = NULL;
uint32_t nds_sys_bus_supported = 0;
uint32_t ndsv5_system_bus_access = 0;
extern uint32_t nds_dmi_access_mem;
extern uint32_t nds_halt_on_reset;
uint32_t read_abstract_reg_number, write_abstract_reg_number;
int ndsv5_get_vector_VLMAX(struct target *target);

#if _NDS_DMI_CHECK_TIMEOUT_
struct timeval begin_time;
struct timeval end_time;
extern uint32_t v5_count_to_check_dm;
static double timeval_diff_ms(struct timeval *a_timeval_begin, struct timeval *a_timeval_end);
static int dmi_check_timeout(struct target *target);
#endif

#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
struct riscv_batch *write_debug_buffer_batch = NULL;
#endif

#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
uint64_t backup_debug_buffer[RISCV_MAX_HARTS][16];
#endif

#if _NDS_MEM_Q_ACCESS_
extern uint32_t nds_dmi_quick_access;
uint32_t nds_dmi_abstractcs = 0;
uint32_t nds_dmi_quick_access_ena = 0;
int ndsv5_read_memory_quick_access(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
#endif

extern uint32_t nds_jtag_max_scans;
extern uint32_t nds_dmi_busy_retry_times;
//#define DMI_BUSY_RETRY  100
void decode_csr(char *text, unsigned address, unsigned data);
#define MAX_RETRY  3
#endif

#define DMI_DATA1 (DMI_DATA0 + 1)
#define DMI_DATA2 (DMI_DATA0 + 2)
#define DMI_DATA3 (DMI_DATA0 + 3)
#define DMI_PROGBUF1 (DMI_PROGBUF0 + 1)

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target, bool step);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int hid, int rid);
static int riscv013_set_register(struct target *target, int hartid, int regid, uint64_t value);
static void riscv013_select_current_hart(struct target *target);
static int riscv013_halt_current_hart(struct target *target);
static int riscv013_resume_current_hart(struct target *target);
static int riscv013_step_current_hart(struct target *target);
static int riscv013_on_halt(struct target *target);
static int riscv013_on_step(struct target *target);
static int riscv013_on_resume(struct target *target);
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
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);

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
#endif
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

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

typedef enum {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
} yes_no_maybe_t;

#if _NDS_V5_ONLY_
typedef struct {
	struct list_head list;
	int abs_chain_position;
	/* Indicates we already reset this DM, so don't need to do it again. */
	bool was_reset;
	/* Targets that are connected to this DM. */
	struct list_head target_list;
	/* The currently selected hartid on this DM. */
	int current_hartid;
} dm013_info_t;

typedef struct {
	struct list_head list;
	struct target *target;
} target_list_t;
#endif


typedef struct {
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

	bool need_strict_step;

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

#if _NDS_V5_ONLY_
	/* DM that provides access to this target. */
	dm013_info_t *dm;
#endif

} riscv013_info_t;

LIST_HEAD(dm_list);

void decode_dmi(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ, "haltreq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_RESUMEREQ, "resumereq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HARTRESET, "hartreset" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_ACKHAVERESET, "ackhavereset" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HASEL, "hasel" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_HARTSELLO, "hartsel" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_SETRESETHALTREQ, "setresethaltreq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_CLRRESETHALTREQ, "clrresethaltreq" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_NDMRESET, "ndmreset" },
		{ DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE, "dmactive" },

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
		{ DMI_COMMAND, DMI_COMMAND_CONTROL, "control" },

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
		{ DMI_COMMAND_AC_ACCESS_REGISTER, DMI_COMMAND_AC_QUICK_ACCESS, "quick" },
		{ DMI_COMMAND_AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_POSTEXEC, "postexec" },
		{ DMI_COMMAND_AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_AARSIZE, "size" },
		{ DMI_COMMAND_AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_TRANSFER, "transfer" },
		{ DMI_COMMAND_AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_WRITE, "write" },
		{ DMI_COMMAND_AC_ACCESS_REGISTER, AC_ACCESS_REGISTER_REGNO, "regno" },

		{ DMI_DMCS2, DMI_DMCS2_HGSELECT, "hgselect" },
		{ DMI_DMCS2, DMI_DMCS2_HGWRITE, "hgwrite" },
		{ DMI_DMCS2, DMI_DMCS2_HALTGROUP, "haltgroup" },
		{ DMI_DMCS2, DMI_DMCS2_EXTTRIGGER, "exttrigger" },
#endif
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

#if _NDS_V5_ONLY_
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

static void dump_field(const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_INFO)
		return;

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;
	unsigned int op_address = in_address;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %08x @%02x -> %s %08x @%02x",
			field->num_bits,
			op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address);

	char in_text[500] = {0};
	char append_text[500] = {0};
	decode_dmi(in_text, in_address, in_data);
	if (in_address == DMI_COMMAND) {
		decode_dmi(append_text, DMI_COMMAND_AC_ACCESS_REGISTER, in_data);
	}
	
	if( in_text[0] != 0x0 || append_text[0] != 0x0 )
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "\t%s: %s %s", dmi_reg_string[op_address], in_text, append_text);
}
#else
static void dump_field(const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

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
			"%db %s %08x @%02x -> %s %08x @%02x",
			field->num_bits,
			op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address);

	char out_text[500];
	char in_text[500];
	decode_dmi(out_text, out_address, out_data);
	decode_dmi(in_text, in_address, in_data);
	if (in_text[0] || out_text[0]) {
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "%s -> %s",
				out_text, in_text);
	}
}
#endif

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv013_info_t *) info->version_specific;
}

/**
 *  * Return the DM structure for this target. If there isn't one, find it in the
 *   * global list of DMs. If it's not in there, then create one and initialize it
 *    * to 0.
 *     */
static dm013_info_t *get_dm(struct target *target)
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
		dm = calloc(1, sizeof(dm013_info_t));
		dm->abs_chain_position = abs_chain_position;
		dm->current_hartid = -1;
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

/*** Utility functions. ***/

static void select_dmi(struct target *target)
{
	static uint8_t ir_dmi[1] = {DTM_DMI};
	struct scan_field field = {
		.num_bits = target->tap->ir_length,
		.out_value = ir_dmi,
		.in_value = NULL,
		.check_value = NULL,
		.check_mask = NULL
	};

	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
}

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
	LOG_DEBUG("dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

/**
 * exec: If this is set, assume the scan results in an execution, so more
 * run-test/idle cycles may be required.
 */
static dmi_status_t dmi_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dmi_op_t op, uint16_t address_out, uint64_t data_out,
		bool exec)
{
	riscv013_info_t *info = get_info(target);
	uint8_t in[8] = {0};
	uint8_t out[8];
	struct scan_field field = {
		.num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH,
		.out_value = out,
		.in_value = in
	};

	assert(info->abits != 0);

	buf_set_u64(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u64(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u64(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int idle_count = info->dmi_busy_delay;
	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);
#if _NDS_V5_ONLY_
	else 
		jtag_add_runtest(info->dtmcontrol_idle, TAP_IDLE);
#endif

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		return DMI_STATUS_FAILED;
	}

	if (data_in)
		*data_in = buf_get_u64(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);

#if _NDS_V5_ONLY_
	uint32_t in_value = buf_get_u32(in, 0, 32);
	if (in_value == 0xFFFFFFFF) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
	}
#endif
	dump_field(&field);

#if _NDS_DMI_CHECK_TIMEOUT_
	dmi_status_t dmi_stat = buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
	if (dmi_stat == DMI_STATUS_BUSY) {
		dmi_stat = dmi_check_timeout(target);
	}
	return dmi_stat;
#endif
	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

static uint64_t dmi_read(struct target *target, uint16_t address)
{
	select_dmi(target);
#if 0//_NDS_USE_SCRIPT_
	uint64_t dmi_read_data = 0;
	int result = ndsv5_script_dmi_read(address, &dmi_read_data);
	LOG_DEBUG("dmi_read = 0x%" PRIx64, dmi_read_data);
	if (result == ERROR_OK) {
		return dmi_read_data;
	}
#endif

	dmi_status_t status;
	uint16_t address_in;

	unsigned i = 0;

	/* This first loop ensures that the read request was actually sent
	 * to the target. Note that if for some reason this stays busy,
	 * it is actually due to the previous dmi_read or dmi_write. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday (&begin_time, NULL);
#endif

	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, NULL, NULL, DMI_OP_READ, address, 0,
				false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed read from 0x%x, status=%d", address, status);
			break;
		}
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed read from 0x%x; status=%d", address, status);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Failed read from 0x%x; status=%d", address, status);
#endif
		return ~0ULL;
	}

	/* This second loop ensures that we got the read
	 * data back. Note that NOP can result in a 'busy' result as well, but
	 * that would be noticed on the next DMI access we do. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday (&begin_time, NULL);
#endif

	uint64_t value;
	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, &address_in, &value, DMI_OP_NOP, address, 0,
				false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed read (NOP) at 0x%x, status=%d", address, status);
			break;
		}
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed read (NOP) from 0x%x; value=0x%" PRIx64 ", status=%d",
				address, value, status);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Failed read (NOP) from 0x%x; value=0x%" PRIx64 ", status=%d",
				address, value, status);
#endif
		return ~0ULL;
	}

	return value;
}

static int dmi_write(struct target *target, uint16_t address, uint64_t value)
{
	select_dmi(target);
#if 0//_NDS_USE_SCRIPT_
	LOG_DEBUG("script write 0x%" PRIx64 " to 0x%x", value, address);
	int result = ndsv5_script_dmi_write(address, value);
	if (result == ERROR_OK) {
		return;
	}
#endif

	dmi_status_t status = DMI_STATUS_BUSY;
	unsigned i = 0;

	/* The first loop ensures that we successfully sent the write request. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday (&begin_time, NULL);
#endif

	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, NULL, NULL, DMI_OP_WRITE, address, value,
				address == DMI_COMMAND);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed write to 0x%x, status=%d", address, status);
			break;
		}
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed write to 0x%x;, status=%d",
			address, status);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Failed write to 0x%x;, status=%d",
			address, status);
#endif
		return ERROR_FAIL;
	}

	/* The second loop isn't strictly necessary, but would ensure that the
	 * write is complete/ has no non-busy errors before returning from this
	 * function. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday (&begin_time, NULL);
#endif

	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, NULL, NULL, DMI_OP_NOP, address, 0,
				false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed write (NOP) at 0x%x, status=%d", address, status);
			break;
		}
	}
	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("failed to write (NOP) 0x%" PRIx64 " to 0x%x; status=%d", value, address, status);
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("failed to write (NOP) 0x%" PRIx64 " to 0x%x; status=%d", value, address, status);
#endif
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
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
		*abstractcs = dmi_read(target, DMI_ABSTRACTCS);

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

			LOG_ERROR("Timed out after %ds waiting for busy to go low (abstractcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					*abstractcs);
			return ERROR_FAIL;
		}
	}
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	RISCV013_INFO(info);
	LOG_DEBUG("command=0x%x", command);
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
#endif

		uint32_t retry_cnt = 0;
		while (retry_cnt < nds_dmi_busy_retry_times) {
			retry_cnt ++;
			if (riscv_batch_run(write_debug_buffer_batch) != ERROR_OK) {
				LOG_DEBUG("riscv_batch_run_FAIL");
				increase_dmi_busy_delay(target);
				riscv013_clear_abstract_error(target);
				write_debug_buffer_batch->idle_count = info->dmi_busy_delay + info->ac_busy_delay;
				if (retry_cnt == nds_dmi_busy_retry_times) {
		            LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
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
		cs = dmi_read(target, DMI_ABSTRACTCS);
	}
#else
	dmi_write(target, DMI_COMMAND, command);

	{
		uint32_t abstractcs = 0;
		wait_for_idle(target, &abstractcs);
	}
	uint32_t cs = dmi_read(target, DMI_ABSTRACTCS);
#endif

	info->cmderr = get_field(cs, DMI_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0) {
		LOG_ERROR("command 0x%x failed; abstractcs=0x%x", command, cs);
		/* Clear the error. */
		dmi_write(target, DMI_ABSTRACTCS, set_field(0, DMI_ABSTRACTCS_CMDERR,
					info->cmderr));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static riscv_reg_t read_abstract_arg(struct target *target, unsigned index)
{
	riscv_reg_t value = 0;
	unsigned xlen = riscv_xlen(target);
#if _NDS_V5_ONLY_
	if ((read_abstract_reg_number >= GDB_REGNO_FPR0) && (read_abstract_reg_number <= GDB_REGNO_FPR31)) {
		xlen = 64;
	}
#endif

	unsigned offset = index * xlen / 32;
	switch (xlen) {
		default:
			LOG_ERROR("Unsupported xlen: %d", xlen);
			return ~0;
		case 64:
			value |= ((uint64_t) dmi_read(target, DMI_DATA0 + offset + 1)) << 32;
		case 32:
			value |= dmi_read(target, DMI_DATA0 + offset);
	}
	return value;
}

static int write_abstract_arg(struct target *target, unsigned index,
		riscv_reg_t value)
{
	unsigned xlen = riscv_xlen(target);
#if _NDS_V5_ONLY_
	if ((write_abstract_reg_number >= GDB_REGNO_FPR0) && (write_abstract_reg_number <= GDB_REGNO_FPR31)) {
		xlen = 64;
	}
#endif

	unsigned offset = index * xlen / 32;
	switch (xlen) {
		default:
			LOG_ERROR("Unsupported xlen: %d", xlen);
			return ~0;
		case 64:
			dmi_write(target, DMI_DATA0 + offset + 1, value >> 32);
		case 32:
			dmi_write(target, DMI_DATA0 + offset, value);
	}
	return ERROR_OK;
}

/**
 * @size in bits
 */
static uint32_t access_register_command(uint32_t number, unsigned size,
		uint32_t flags)
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

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_read_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_read_csr_supported)
		return ERROR_FAIL;

#if _NDS_IDE_MESSAGE_
	if (number > GDB_REGNO_CSR4095)
		NDS32_LOG("Unsupported register (enum gdb_regno)(%d)", number);
#endif
	
	uint64_t mstatus;
	if ((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) { 
		if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if ((mstatus & MSTATUS_FS) == 0)
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
							set_field(mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	if ((number >= GDB_REGNO_FPR0) && (number <= GDB_REGNO_FPR31)) {
		size = 64;
	}
	read_abstract_reg_number = number;
#endif

	uint32_t command = access_register_command(number, size,
			AC_ACCESS_REGISTER_TRANSFER);

	int result = execute_abstract_command(target, command);

	//if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0)
	//	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
	//		return ERROR_FAIL;
	
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
	} else {
		if (value)
			*value = read_abstract_arg(target, 0);
	}

	if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;

	return result;
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
#endif

	uint64_t mstatus;
	if ((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) { 
		if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if ((mstatus & MSTATUS_FS) == 0)
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
							set_field(mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	if ((number >= GDB_REGNO_FPR0) && (number <= GDB_REGNO_FPR31)) {
		size = 64;
	}
	write_abstract_reg_number = number;
#endif

	uint32_t command = access_register_command(number, size,
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);

	if (write_abstract_arg(target, 0, value) != ERROR_OK)
		return ERROR_FAIL;

	int result = execute_abstract_command(target, command);
	
	//if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0) 
	//	if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
	//		return ERROR_FAIL;
	
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
	}

	if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;

	return result;
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
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK)
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

	uint32_t written = dmi_read(target, DMI_PROGBUF0);
	if (written == (uint32_t) info->progbuf_address) {
		LOG_INFO("progbuf is writable at 0x%" TARGET_PRIxADDR,
				info->progbuf_address);
		info->progbuf_writable = YNM_YES;

	} else {
		LOG_INFO("progbuf is not writeable at 0x%" TARGET_PRIxADDR,
				info->progbuf_address);
		info->progbuf_writable = YNM_NO;
	}

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
} scratch_mem_t;

/**
 * Find some scratch memory to be used with the given program.
 */
static int scratch_find(struct target *target,
		scratch_mem_t *scratch,
		struct riscv_program *program,
		unsigned size_bytes)
{
	riscv013_info_t *info = get_info(target);

	riscv_addr_t alignment = 1;
	while (alignment < size_bytes)
		alignment *= 2;

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

	if (examine_progbuf(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Allow for ebreak at the end of the program. */
	unsigned program_size = (program->instruction_count + 1) * 4;
	scratch->hart_address = (info->progbuf_address + program_size + alignment - 1) &
		~(alignment - 1);
	if ((size_bytes + scratch->hart_address - info->progbuf_address + 3) / 4 >=
			info->progbufsize) {
		scratch->memory_space = SPACE_DMI_PROGBUF;
		scratch->debug_address = (scratch->hart_address - info->progbuf_address) / 4;
		return ERROR_OK;
	}

	if (riscv_use_scratch_ram) {
		scratch->hart_address = (riscv_scratch_ram_address + alignment - 1) &
			~(alignment - 1);
		scratch->memory_space = SPACE_DMI_RAM;
		scratch->debug_address = scratch->hart_address;
		return ERROR_OK;
	}

	LOG_ERROR("Couldn't find %d bytes of scratch RAM to use. Please configure "
			"an address with 'riscv set_scratch_ram'.", size_bytes);
	return ERROR_FAIL;
}

static int scratch_read64(struct target *target, scratch_mem_t *scratch,
		uint64_t *value)
{
	switch (scratch->memory_space) {
		case SPACE_DMI_DATA:
			*value = dmi_read(target, DMI_DATA0 + scratch->debug_address);
			*value |= ((uint64_t) dmi_read(target, DMI_DATA1 +
						scratch->debug_address)) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			*value = dmi_read(target, DMI_PROGBUF0 + scratch->debug_address);
			*value |= ((uint64_t) dmi_read(target, DMI_PROGBUF1 +
						scratch->debug_address)) << 32;
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

static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
#if _NDS_V5_ONLY_
	char in_text[500];
	in_text[0] = 0x0;
	decode_csr(in_text, number, value);
	NDS_INFO("[%s] hart[%d] reg[%s] <- 0x%" PRIx64 " %s", target->tap->dotted_name, riscv_current_hartid(target),
			gdb_regno_name(number), value, in_text);
#else
	LOG_DEBUG("[%d] reg[0x%x] <- 0x%" PRIx64, riscv_current_hartid(target),
			number, value);
#endif

#if _NDS_USE_SCRIPT_
	if (ndsv5_script_reg_write(number, value) == ERROR_OK) {
		return ERROR_OK;
	}
#endif
	int result = register_write_abstract(target, number, value,
			riscv_xlen(target));
	if (result == ERROR_OK)
		return ERROR_OK;

	struct riscv_program program;
	riscv_program_init(&program, target);

	uint64_t s0;
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
#if _NDS_V5_ONLY_
	uint64_t mstatus;
	if ((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) {
		if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if ((mstatus & MSTATUS_FS) == 0)
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
	}
#endif
	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			riscv_supports_extension(target, 'D') &&
			riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register, so
		 * we need to use some scratch RAM. */
		riscv_program_insert(&program, fld(number - GDB_REGNO_FPR0, S0, 0));

		scratch_mem_t scratch;
		if (scratch_find(target, &scratch, &program, 8) != ERROR_OK)
			return ERROR_FAIL;

		if (register_write_direct(target, GDB_REGNO_S0, scratch.hart_address)
				!= ERROR_OK)
			return ERROR_FAIL;

		if (scratch_write64(target, &scratch, value) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		if (register_write_direct(target, GDB_REGNO_S0, value) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, 'D'))
				riscv_program_insert(&program, fmv_d_x(number - GDB_REGNO_FPR0, S0));
			else
				riscv_program_insert(&program, fmv_w_x(number - GDB_REGNO_FPR0, S0));
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

#if _NDS_V5_ONLY_
	if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0) 
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;
#endif

	/* Restore S0. */
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return exec_out;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
#if _NDS_USE_SCRIPT_
	if (ndsv5_script_reg_read(value, number) == ERROR_OK) {
		return ERROR_OK;
	}
#endif

	int result = register_read_abstract(target, value, number,
			riscv_xlen(target));
	
	if (result != ERROR_OK) {
		assert(number != GDB_REGNO_S0);

		result = ERROR_OK;

		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		uint64_t s0;
		if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;

		/* Write program to move data into s0. */
		
		uint64_t mstatus;
		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
				return ERROR_FAIL;
			if ((mstatus & MSTATUS_FS) == 0)
				if (register_write_direct(target, GDB_REGNO_MSTATUS,
							set_field(mstatus, MSTATUS_FS, 1)) != ERROR_OK)
					return ERROR_FAIL;

			if (riscv_supports_extension(target, 'D') && riscv_xlen(target) < 64) {
				/* There are no instructions to move all the bits from a
				 * register, so we need to use some scratch RAM. */
				riscv_program_insert(&program, fsd(number - GDB_REGNO_FPR0, S0,
							0));

				if (scratch_find(target, &scratch, &program, 8) != ERROR_OK)
					return ERROR_FAIL;
				use_scratch = true;

				if (register_write_direct(target, GDB_REGNO_S0,
							scratch.hart_address) != ERROR_OK)
					return ERROR_FAIL;
			} else if (riscv_supports_extension(target, 'D')) {
				riscv_program_insert(&program, fmv_x_d(S0, number - GDB_REGNO_FPR0));
			} else {
				riscv_program_insert(&program, fmv_x_w(S0, number - GDB_REGNO_FPR0));
			}
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			if (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3)) {
				if (register_read_direct(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
					return ERROR_FAIL;
				if ((mstatus & MSTATUS_FS) == 0)
					if (register_write_direct(target, GDB_REGNO_MSTATUS,
							set_field(mstatus, MSTATUS_FS, 1)) != ERROR_OK)
						return ERROR_FAIL;
			}
			riscv_program_csrr(&program, S0, number);
		} else {
			LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
			return ERROR_FAIL;
		}

		/* Execute program. */
		result = riscv_program_exec(&program, target);

		if (use_scratch) {
			if (scratch_read64(target, &scratch, value) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			/* Read S0 */
			if (register_read_direct(target, value, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (((number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) || (number > GDB_REGNO_CSR0 && number <= (GDB_REGNO_CSR0 + 3))) && (mstatus & MSTATUS_FS) == 0) 
			if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
				return ERROR_FAIL;

		/* Restore S0. */
		if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (result == ERROR_OK) {
#if _NDS_V5_ONLY_
		char in_text[500];
		in_text[0] = 0x0;
		decode_csr(in_text, number, *value);
		NDS_INFO("[%s] hart[%d] reg[%s] = 0x%" PRIx64 " %s", target->tap->dotted_name, riscv_current_hartid(target),
				gdb_regno_name(number), *value, in_text);
#else
		LOG_DEBUG("[%d] reg[0x%x] = 0x%" PRIx64, riscv_current_hartid(target),
				number, *value);
#endif
	}

	return result;
}

/*** OpenOCD target functions. ***/

#if _NDS_V5_ONLY_
unsigned acr_reg_count_v5 = 0;
unsigned acr_type_count_v5 = 0;
extern uint32_t v5_dmi_busy_delay_count;
#endif

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;

#if _NDS_V5_ONLY_
	// initialize ACE
	extern unsigned* global_acr_reg_count_v5;
	extern unsigned* global_acr_type_count_v5;

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
	generic_info->select_current_hart = &riscv013_select_current_hart;
	generic_info->is_halted = &riscv013_is_halted;
	generic_info->halt_current_hart = &riscv013_halt_current_hart;
	generic_info->resume_current_hart = &riscv013_resume_current_hart;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->on_halt = &riscv013_on_halt;
	generic_info->on_resume = &riscv013_on_resume;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->read_debug_buffer = &riscv013_read_debug_buffer;
	generic_info->write_debug_buffer = &riscv013_write_debug_buffer;
	generic_info->execute_debug_buffer = &riscv013_execute_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;
	riscv013_info_t *info = get_info(target);

	info->progbufsize = -1;

#if _NDS_V5_ONLY_
	info->dmi_busy_delay = v5_dmi_busy_delay_count;
#else
	info->dmi_busy_delay = 0;
#endif
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

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (target_was_examined(target)) {
		if (nds32->attached) {
			LOG_DEBUG("deinit_target(): gdb_detach process, resume dcsr");
			if (target->state != TARGET_HALTED) {
				target_halt(target);
			}

			// clear all breakpoints & watchpoints
			breakpoint_clear_target(nds32->target);
			watchpoint_clear_target(nds32->target);

			nds32->gdb_run_mode = RUN_MODE_DEBUG;
			nds32->attached = false;        // Set attached to false before resume

			//free run in debug mode
			target_resume(target, 1, 0, 0, 0);
		}
	}
#endif

	free(info->version_specific);
	info->version_specific = NULL;
}

#if _NDS_V5_ONLY_
extern uint32_t ndsv5_dis_cache_busmode;
extern bool rv32e;
bool reset_halt = false;
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

	// reset the Debug Module
	dmi_write(target, DMI_DMCONTROL, 0);
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	dmi_read(target, DMI_DMCONTROL);

	// check existence harts
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, DMI_DMCONTROL);

		dmstatus = dmi_read(target, DMI_DMSTATUS);
		if ( get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT) ) {
			user_def_hart_count = i;
			LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
			break;
		}
	}

	// Assert reset
	for (i = 0; i < user_def_hart_count; ++i) {
		control = 0;
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, i);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);
		// SETRESETHALTREQ can halt on 0x80000000 for AMP/SMP all harts
		control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
		control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);

		dmi_write(target, DMI_DMCONTROL, control);
		dmi_read(target, DMI_DMCONTROL);
	}
	/* Assert ndmreset */
	control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
	dmi_write(target, DMI_DMCONTROL, control);

	alive_sleep(nds32->reset_time);
	dmi_read(target, DMI_DMSTATUS);

	// Deassert reset
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
		dmi_read(target, DMI_DMCONTROL);

		do {
			dmstatus = dmi_read(target, DMI_DMSTATUS);
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
		dmi_read(target, DMI_DMCONTROL);
		dmi_read(target, DMI_DMSTATUS);
	}
	reset_halt = true;
	return ERROR_OK;
}

static int set_haltgroup(struct target *target, bool *supported)
{
	uint32_t write = set_field(DMI_DMCS2_HGWRITE, DMI_DMCS2_HALTGROUP, target->group);
	if (dmi_write(target, DMI_DMCS2, write) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t read = dmi_read(target, DMI_DMCS2);
	*supported = get_field(read, DMI_DMCS2_HALTGROUP) == (unsigned) target->group;
	return ERROR_OK;
}
#endif

static int examine(struct target *target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	LOG_DEBUG("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
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
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	if (get_field(dmstatus, DMI_DMSTATUS_VERSION) != 2) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2, not %d "
				"(dmstatus=0x%x)", get_field(dmstatus, DMI_DMSTATUS_VERSION), dmstatus);
		return ERROR_FAIL;
	}

	/* Reset the Debug Module. */
#if _NDS_V5_ONLY_
	// -H: reset as init
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (!target_was_examined(target)) {
		if (nds32->reset_halt_as_examine) {
			if (nds_script_custom_reset_halt) {
				ndsv5_script_do_custom_reset(target, nds_script_custom_reset_halt);
			} else if (!reset_halt){
				ndsv5_reset_halt_as_examine(target);
			}
			NDS32_LOG(NDS32_MSG_HW_RESET_HOLD);
		}
	}

	 /* reset info->dmi_busy_delay=info->dtmcontrol_idle
	    if info->dmi_busy_delay was incease by the 1st dmi_scan*/
	info->dmi_busy_delay = v5_dmi_busy_delay_count;
	LOG_DEBUG("info->dmi_busy_delay: 0x%08x", info->dmi_busy_delay);

	dm013_info_t *dm = get_dm(target);
	if (!dm->was_reset) {
		dmi_write(target, DMI_DMCONTROL, 0);
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
		dm->was_reset = true;
		LOG_DEBUG("Reset DM done!!");
	}
	dmstatus = dmi_read(target, DMI_DMSTATUS);
#else
	dmi_write(target, DMI_DMCONTROL, 0);
#endif
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);

	uint32_t hartinfo = dmi_read(target, DMI_HARTINFO);

	LOG_DEBUG("dmcontrol: 0x%08x", dmcontrol);
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);
	LOG_DEBUG("hartinfo:  0x%08x", hartinfo);

	info->datasize = get_field(hartinfo, DMI_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DMI_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DMI_HARTINFO_DATAADDR);


	if (!get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	if (!get_field(dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dmcontrol=0x%x", dmcontrol);
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	//The upstream was removed to check UNAVAIL and NONEXISTENT
#else
	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {
		LOG_ERROR("The hart is unavailable.");
		return ERROR_FAIL;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
		LOG_ERROR("The hart doesn't exist.");
		return ERROR_FAIL;
	}
#endif

#if _NDS_V5_ONLY_
	ndsv5_dis_cache_busmode = 1;
	info->sbcs = dmi_read(target, DMI_SBCS);
	if (info->sbcs & 0x1F) {
		// DMI_SBCS_SBACCESS8|DMI_SBCS_SBACCESS16|DMI_SBCS_SBACCESS32|
		// DMI_SBCS_SBACCESS64|DMI_SBCS_SBACCESS128
		int sb_version = get_field(info->sbcs, DMI_SBCS_SBVERSION);
		if ((sb_version == 0) || (sb_version == 1)) {
			//according to eticket 16199, default no support system bus access, so ndsv5_system_bus_access default value is 0
			if (ndsv5_system_bus_access == 1) {
				nds_sys_bus_supported = 1;
				ndsv5_dis_cache_busmode = 0;
			}
		}
	}
	LOG_DEBUG("info->sbcs = 0x%x, nds_sys_bus_supported = 0x%x", (int)info->sbcs, nds_sys_bus_supported);
#endif

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE);

	/* Before doing anything else we must first enumerate the harts. */
	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DMI_DMSTATUS_IMPEBREAK);

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */
#if _NDS_V5_ONLY_
	int user_def_hart_count = (int)target->corenums;
	if (user_def_hart_count == 0)
		user_def_hart_count = RISCV_MAX_HARTS;
	LOG_DEBUG("user_def_hart_count = 0x%x", (int)user_def_hart_count);
	for (int i = 0; i < user_def_hart_count; ++i) {
#else
	for (int i = 0; i < RISCV_MAX_HARTS; ++i) {
#endif
		if (!riscv_rtos_enabled(target) && i != target->coreid)
			continue;

		r->current_hartid = i;
		riscv013_select_current_hart(target);

		uint32_t s = dmi_read(target, DMI_DMSTATUS);
		if (get_field(s, DMI_DMSTATUS_ANYNONEXISTENT))
			break;
		r->hart_count = i + 1;

#if _NDS_V5_ONLY_
		if(get_field(s, DMI_DMSTATUS_ANYHAVERESET)) {
			uint32_t control = dmi_read(target, DMI_DMCONTROL);
			control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
			dmi_write(target, DMI_DMCONTROL, control);
		}

		if (!riscv_is_halted(target)) {
			int result = riscv013_halt_current_hart(target);
			if (result != ERROR_OK) {
				if (riscv_rtos_enabled(target)) {
					if(i != 0) {
						LOG_DEBUG("riscv_rtos_enabled, riscv013_halt_current_hart %d FAIL, but return OK", i);
						return ERROR_OK;
					}
				}
				r->debug_buffer_size[i] = info->progbufsize;
				r->xlen[i] = 0;
				/* Now init registers based on what we discovered. */
				//if (riscv_init_registers(target) != ERROR_OK)
				//	return ERROR_FAIL;
				LOG_DEBUG("riscv013_halt_current_hart FAIL return");
				return ERROR_FAIL;
				//return ERROR_OK;
			}
		}

		/// Enable halt-on-reset
		if( nds_halt_on_reset == 1 ) {
			if(get_field(s, DMI_DMSTATUS_HASRESETHALTREQ)) {
				uint32_t control = dmi_read(target, DMI_DMCONTROL);
				control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
				dmi_write(target, DMI_DMCONTROL, control);
				LOG_DEBUG("hart [%s] %d: halt-on-reset is on!", target->tap->dotted_name, i);
			} else {
				LOG_ERROR("The Debug Module doesn't supports halt-on-reset functionality!!");
			}
		}

#else
		if (!riscv_is_halted(target))
			riscv013_halt_current_hart(target);
#endif

		/* Without knowing anything else we can at least mess with the
		 * program buffer. */
		r->debug_buffer_size[i] = info->progbufsize;

		int result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
		if (result == ERROR_OK)
			r->xlen[i] = 64;
		else {
#if _NDS_V5_ONLY_
			riscv013_clear_abstract_error(target);
#endif
			r->xlen[i] = 32;
		}

		register_read_direct(target, &r->misa, GDB_REGNO_MISA);

#if _NDS_V5_ONLY_
		if (r->misa & (0x01 << 21)) {
			LOG_DEBUG("Vector Extension support");
			ndsv5_get_vector_VLMAX(target);
		}
#endif

		/* Now init registers based on what we discovered. */
		if (riscv_init_registers(target) != ERROR_OK)
			return ERROR_FAIL;

		/* Display this as early as possible to help people who are using
		 * really slow simulators. */
#if _NDS_V5_ONLY_
		LOG_DEBUG(" [%s] hart %d: XLEN=%d, misa=0x%" PRIx64, target->tap->dotted_name, i, r->xlen[i],
				r->misa);
#else
		LOG_DEBUG(" hart %d: XLEN=%d, misa=0x%" PRIx64, i, r->xlen[i],
				r->misa);
#endif

#if _NDS_V5_ONLY_
		/* Check if rv32e for target burn*/
		rv32e = false;
		if (r->misa & 0x10) {
			rv32e = true;
			LOG_DEBUG("target is rv32e");
		}

		if (target->group) {
			bool haltgroup_supported;
			LOG_DEBUG("group:%d\n", target->group);
			if (set_haltgroup(target, &haltgroup_supported) != ERROR_OK)
				return ERROR_FAIL;
			if (haltgroup_supported) {
				LOG_INFO("Core %d made part of halt group %d.", target->coreid,
						target->group);
			} else {
				LOG_ERROR("Core %d could not be made part of halt group %d.",
						target->coreid, target->group);
			}
		}
#endif
	}

	LOG_DEBUG("Enumerated %d harts", r->hart_count);

	/* Then we check the number of triggers availiable to each hart. */
	riscv_enumerate_triggers(target);

#if _NDS_V5_ONLY_
	if(nds32->reset_halt_as_examine) {
		LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
		target->debug_reason = DBG_REASON_DBGRQ;
		target->state = TARGET_HALTED;
		LOG_DEBUG("reset_halt_as_examine, target->state: 0x%08x", target->state);
	} else {
		riscv_resume_all_harts(target);
		target->state = TARGET_RUNNING;
	}
#else
	/* Resumes all the harts, so the debugger can later pause them. */
	/* TODO: Only do this if the harts were halted to start with. */
	riscv_resume_all_harts(target);
	target->state = TARGET_RUNNING;
#endif

	target_set_examined(target);

	if (target->rtos)
		riscv_update_threads(target->rtos);

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_INFO("Examined RISC-V core; found %d harts",
			riscv_count_harts(target));
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (riscv_hart_enabled(target, i)) {
#if _NDS_V5_ONLY_
			LOG_INFO(" [%s] hart %d: XLEN=%d, %d triggers", target->tap->dotted_name, i, r->xlen[i],
					r->trigger_count[i]);
#else
			LOG_INFO(" hart %d: XLEN=%d, %d triggers", i, r->xlen[i],
					r->trigger_count[i]);
#endif
		} else {
#if _NDS_V5_ONLY_
			LOG_INFO(" [%s] hart %d: currently disabled", target->tap->dotted_name, i);
#else
			LOG_INFO(" hart %d: currently disabled", i);
#endif
		}
	}
	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
#if _NDS_V5_ONLY_
	// Disable halt-on-reset when reset-run
	if((nds_halt_on_reset == 1) && (target->reset_halt == 0) ) {
		ndsv5_haltonreset(target, 0);
	}

	if ((nds_script_custom_reset_halt) && (target->reset_halt == 1)) {
		ndsv5_script_do_custom_reset(target, nds_script_custom_reset_halt);
		return ERROR_OK;
	} else if ((nds_script_custom_reset) && (target->reset_halt == 0)) {
		ndsv5_script_do_custom_reset(target, nds_script_custom_reset);
		return ERROR_OK;
	}
#endif
	RISCV_INFO(r);

	select_dmi(target);

	uint32_t control_base = set_field(0, DMI_DMCONTROL_DMACTIVE, 1);

#if _NDS_V5_ONLY_
	if (target->reset_halt) {
		// single core no execute haltonreset: bitmap built before 2019/5 : setresethaltreq no work(hardware issue)
		if (target->rtos)
			ndsv5_haltonreset(target, 1);
	} else
		ndsv5_haltonreset(target, 0);
#endif

	if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		 * We must reset them all. */

		/* TODO: Try to use hasel in dmcontrol */

		/* Set haltreq/resumereq for each hart. */
		uint32_t control = control_base;
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			if (!riscv_hart_enabled(target, i))
				continue;

			control = set_field(control_base, DMI_DMCONTROL_HARTSELLO, i);
			control = set_field(control, DMI_DMCONTROL_HALTREQ,
					target->reset_halt ? 1 : 0);
			dmi_write(target, DMI_DMCONTROL, control);
		}

		/* Assert ndmreset */
		control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DMI_DMCONTROL, control);

	} else {
		/* Reset just this hart. */
		uint32_t control = set_field(control_base, DMI_DMCONTROL_HARTSELLO,
				r->current_hartid);
		control = set_field(control, DMI_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DMI_DMCONTROL_HARTRESET, 1);
		dmi_write(target, DMI_DMCONTROL, control);

		/* Read back to check if hartreset is supported. */
		uint32_t rb = dmi_read(target, DMI_DMCONTROL);
		if (!get_field(rb, DMI_DMCONTROL_HARTRESET)) {
			/* Use ndmreset instead. That will reset the entire device, but
			 * that's probably what OpenOCD wants anyway. */
			control = set_field(control, DMI_DMCONTROL_HARTRESET, 0);
			control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
			dmi_write(target, DMI_DMCONTROL, control);
		}
	}

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	alive_sleep(nds32->reset_time);
#endif
	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
	RISCV_INFO(r);
	RISCV013_INFO(info);
	select_dmi(target);

	LOG_DEBUG("%d", r->current_hartid);
	uint32_t control = 0;

#if _NDS_V5_ONLY_
	isAceCsrEnable = false;
	if ((nds_script_custom_reset_halt) && (target->reset_halt == 1)) {
		//ndsv5_script_do_custom_reset(target, nds_script_custom_reset_halt);
	} else if ((nds_script_custom_reset) && (target->reset_halt == 0)) {
		//ndsv5_script_do_custom_reset(target, nds_script_custom_reset);
	}
	else
#endif
	{
		/* Clear the reset, but make sure haltreq is still set */
		control = set_field(control, DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
		control = set_field(control, DMI_DMCONTROL_HARTSELLO, r->current_hartid);
		control = set_field(control, DMI_DMCONTROL_DMACTIVE, 1);

#if _NDS_V5_ONLY_
		control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);
#endif
		dmi_write(target, DMI_DMCONTROL, control);
	}

	uint32_t dmstatus;
	int dmi_busy_delay = info->dmi_busy_delay;

#if _NDS_V5_ONLY_
#else
	time_t start = time(NULL);
#endif

#if _NDS_V5_ONLY_
	// The following is modified from Sifive github @ 2019/03/13 version, 
	// please remove when sync successful
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		time_t start = time(NULL);
		int index = i;
		control = set_field(control, DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
		if (target->rtos) {
			if (!riscv_hart_enabled(target, index))
				continue;

			control = set_field(control, DMI_DMCONTROL_HARTSELLO, index);
			dmi_write(target, DMI_DMCONTROL, control);
		} else {
			index = r->current_hartid;
		}

		if (target->reset_halt) {
			LOG_DEBUG("Waiting for [%s] hart %d to halt out of reset.", target->tap->dotted_name, index);

			do {
				dmstatus = dmi_read(target, DMI_DMSTATUS);
				if (time(NULL) - start > riscv_reset_timeout_sec) {
					LOG_ERROR("Hart didn't halt coming out of reset in %ds; "
							"dmstatus=0x%x; "
							"Increase the timeout with riscv set_reset_timeout_sec.",
							riscv_reset_timeout_sec, dmstatus);
					return ERROR_FAIL;
				}
			} while (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0);
			target->state = TARGET_HALTED;

			control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
			dmi_write(target, DMI_DMCONTROL, control);

			if (target->reset_halt)
				NDS32_LOG(NDS32_MSG_HW_RESET_HOLD_ID, target->tap->dotted_name, index);
		} else {
			LOG_DEBUG("Waiting for [%s] hart %d to run out of reset.", target->tap->dotted_name, index);

			do {
				dmstatus = dmi_read(target, DMI_DMSTATUS);
				if (get_field(dmstatus, DMI_DMSTATUS_ANYHALTED) ||
						get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {

					NDS32_LOG("Unexpected [%s] hart %d status during reset. dmstatus=0x%x", target->tap->dotted_name, target->coreid,
							dmstatus);

					// For more detailed
					uint32_t dmcontrol  = dmi_read(target, DMI_DMCONTROL);
					uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
					NDS32_LOG("dmcontrol : 0x%x", dmcontrol);
					NDS32_LOG("abstractcs: 0x%x", abstractcs);
					return ERROR_FAIL;

				}
				if (time(NULL) - start > riscv_reset_timeout_sec) {
					LOG_ERROR("Hart didn't run coming out of reset in %ds; "
							"dmstatus=0x%x; "
							"Increase the timeout with riscv set_reset_timeout_sec.",
							riscv_reset_timeout_sec, dmstatus);
					return ERROR_FAIL;
				}
			} while (get_field(dmstatus, DMI_DMSTATUS_ALLRUNNING) == 0);

			//Halt again
			struct nds32_v5 *nds32 = target_to_nds32_v5(target);
			alive_sleep(nds32->boot_time);
			riscv013_halt_current_hart(target);
		}

		int retry = 0;
		if (get_field(dmstatus, DMI_DMSTATUS_ALLHAVERESET)) {
			control = set_field(control, DMI_DMCONTROL_HARTSELLO, index);
			control = set_field(control, DMI_DMCONTROL_ACKHAVERESET, 1);

			// Ack reset
			do {
				dmi_write(target, DMI_DMCONTROL, control);
				dmstatus = dmi_read(target, DMI_DMSTATUS);
			} while( get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET) && (retry++ < MAX_RETRY) );
		}
		if(retry >= MAX_RETRY ) {
			NDS32_LOG_ERROR("<-- TARGET WARNING! Unable to clear havereset on [%s] hart %d. -->", target->tap->dotted_name, index);
		}

	}

	// Restore halt-on-reset
	if(nds_halt_on_reset == 1) {
		// single core no execute haltonreset: bitmap built before 2019/5 : setresethaltreq no work(hardware issue)
		if (target->rtos)
			ndsv5_haltonreset(target, 1);
	} else
		ndsv5_haltonreset(target, 0);
	info->dmi_busy_delay = dmi_busy_delay;

	// Reset again if necessary
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	uint32_t cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
	while (cmderr == CMDERR_EXCEPTION) {
		LOG_DEBUG("cmderr == CMDERR_EXCEPTION reset again !!\n");
		printf("cmderr == CMDERR_EXCEPTION reset again !!\n");
		/* Clear the error status. */
		dmi_write(target, DMI_ABSTRACTCS, abstractcs & DMI_ABSTRACTCS_CMDERR);
		dtmcontrol_scan(target, DTM_DTMCS_DMIRESET|DTM_DTMCS_DMIHARDRESET);
		assert_reset(target);
		deassert_reset(target);
		abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		if (cmderr == 0) {
			break;
		}
	}
	LOG_DEBUG("exit reset\n");
	//printf("exit reset\n");

	return ERROR_OK;
#else
	if (target->reset_halt) {
		LOG_DEBUG("Waiting for hart to be halted.");
		do {
			dmstatus = dmi_read(target, DMI_DMSTATUS);
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart didn't halt coming out of reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						riscv_reset_timeout_sec, dmstatus);
				return ERROR_FAIL;
			}
			target->state = TARGET_HALTED;
		} while (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0);

		control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
		dmi_write(target, DMI_DMCONTROL, control);
	} else {
		LOG_DEBUG("Waiting for hart to be running.");
		do {
			dmstatus = dmi_read(target, DMI_DMSTATUS);
			if (get_field(dmstatus, DMI_DMSTATUS_ANYHALTED) ||
				get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {
				LOG_ERROR("Unexpected hart status during reset. dmstatus=0x%x",
						dmstatus);
				return ERROR_FAIL;
			}
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart didn't run coming out of reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						riscv_reset_timeout_sec, dmstatus);
				return ERROR_FAIL;
			}
		} while (get_field(dmstatus, DMI_DMSTATUS_ALLRUNNING) == 0);
		target->state = TARGET_RUNNING;
	}
	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
#endif
}

/**
 * @size in bytes
 */
static void write_to_buf(uint8_t *buffer, uint64_t value, unsigned size)
{
	switch (size) {
		case 8:
			buffer[7] = value >> 56;
			buffer[6] = value >> 48;
			buffer[5] = value >> 40;
			buffer[4] = value >> 32;
		case 4:
			buffer[3] = value >> 24;
			buffer[2] = value >> 16;
		case 2:
			buffer[1] = value >> 8;
		case 1:
			buffer[0] = value;
			break;
		default:
			assert(false);
	}
}

static int execute_fence(struct target *target)
{
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence(&program);
	int result = riscv_program_exec(&program, target);
	if (result != ERROR_OK)
		LOG_ERROR("Unable to execute fence");
	return result;
}

static void log_memory_access(target_addr_t address, uint64_t value,
		unsigned size_bytes, bool read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%0%d" PRIx64,
			address, read ? "read" : "write", size_bytes * 2);
	value &= (((uint64_t) 0x1) << (size_bytes * 8)) - 1;
	LOG_DEBUG(fmt, value);
}

/* Read the relevant sbdata regs depending on size, and put the results into
 * buffer. */
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint32_t value;
	if (size > 12) {
		//if (dmi_read(target, &value, DMI_SBDATA3) != ERROR_OK)
		//	return ERROR_FAIL;
		value = dmi_read(target, DMI_SBDATA3);
		write_to_buf(buffer + 12, value, 4);
		log_memory_access(address + 12, value, 4, true);
	}
	if (size > 8) {
		//if (dmi_read(target, &value, DMI_SBDATA2) != ERROR_OK)
		//	return ERROR_FAIL;
		value = dmi_read(target, DMI_SBDATA2);
		write_to_buf(buffer + 8, value, 4);
		log_memory_access(address + 8, value, 4, true);
	}
	if (size > 4) {
		//if (dmi_read(target, &value, DMI_SBDATA1) != ERROR_OK)
		//	return ERROR_FAIL;
		value = dmi_read(target, DMI_SBDATA1);
		write_to_buf(buffer + 4, value, 4);
		log_memory_access(address + 4, value, 4, true);
	}
	//if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
	//	return ERROR_FAIL;
	value = dmi_read(target, DMI_SBDATA0);
	write_to_buf(buffer, value, MIN(size, 4));
	log_memory_access(address, value, MIN(size, 4), true);
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
		//dmi_read(target, &v, DMI_SBADDRESS1);
		v = dmi_read(target, DMI_SBADDRESS1);
		address |= v;
		address <<= 32;
#endif
	}
	//dmi_read(target, &v, DMI_SBADDRESS0);
	v = dmi_read(target, DMI_SBADDRESS0);
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
		//if (dmi_read(target, sbcs, DMI_SBCS) != ERROR_OK)
		//	return ERROR_FAIL;
		*sbcs = dmi_read(target, DMI_SBCS);
		if (!get_field(*sbcs, DMI_SBCS_SBBUSY))
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec, *sbcs);
			return ERROR_FAIL;
		}
	}
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
			//if (dmi_read(target, &access, DMI_SBCS) != ERROR_OK)
			//	return ERROR_FAIL;
			access = dmi_read(target, DMI_SBCS);
			dmi_write(target, DMI_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DMI_SBCS_SBACCESS, size/2);
			access = set_field(access, DMI_SBCS_SBSINGLEREAD, 1);
			LOG_DEBUG("\r\nread_memory: sab: access:  0x%08x", access);
			dmi_write(target, DMI_SBCS, access);
			/* 3) read */
			uint32_t value;
			//if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
			//	return ERROR_FAIL;
			value = dmi_read(target, DMI_SBDATA0);
			LOG_DEBUG("\r\nread_memory: sab: value:  0x%08x", value);
			write_to_buf(t_buffer, value, size);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_DEBUG("reading block until final address 0x%" PRIx64, fin_addr);
	//if (dmi_read(target, &access, DMI_SBCS) != ERROR_OK)
	//	return ERROR_FAIL;
	access = dmi_read(target, DMI_SBCS);
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
		//if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
		//	return ERROR_FAIL;
		value = dmi_read(target, DMI_SBDATA0);
		write_to_buf(t_buffer, value, size);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dmi_write(target, DMI_SBCS, 0);
			//if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
			//	return ERROR_FAIL;
			value = dmi_read(target, DMI_SBDATA0);
			write_to_buf(t_buffer, value, size);
		}
	}

	return ERROR_OK;
}

#if _NDS_V5_ONLY_
static int read_memory_bus_v1_opt(struct target *target, target_addr_t address,
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
			if (size > 12) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + i * size + 12, value, 4);
				log_memory_access(cur_address + i * size + 12, value, 4, true);
			}
			if (size > 8) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + i * size + 8, value, 4);
				log_memory_access(cur_address + i * size + 8, value, 4, true);
			}
			if (size > 4) {
				dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
				value = get_field(dmi_out, DTM_DMI_DATA);
				write_to_buf(buffer + i * size + 4, value, 4);
				log_memory_access(cur_address + i * size + 4, value, 4, true);
			}
			dmi_out = riscv_batch_get_dmi_read(busmode_batch, pindex_read[batch_index++]);
			value = get_field(dmi_out, DTM_DMI_DATA);
			write_to_buf(buffer + i * size, value, MIN(size, 4));
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
#endif

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

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

		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			read_memory_bus_word(target, address + i * size, size,
					buffer + i * size);
		}

		sbcs = set_field(sbcs, DMI_SBCS_SBREADONDATA, 0);
		dmi_write(target, DMI_SBCS, sbcs);

		read_memory_bus_word(target, address + (count - 1) * size, size,
				buffer + (count - 1) * size);

		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);
			next_address = sb_read_address(target);
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs, DMI_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

/**
 * Read the requested memory, taking care to execute every read exactly once,
 * even if cmderr=busy is encountered.
 */
static int ndsv5_read_memory_abstract_access(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("Abstract Memory Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	RISCV013_INFO(info);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	uint32_t run_program = 0;
	uint32_t ndsv5_read_memory_retry_cnt = 0;
read_memory_abstract_retry:
	LOG_DEBUG("info->dmi_busy_delay=0x%x, info->ac_busy_delay=0x%x", info->dmi_busy_delay, info->ac_busy_delay);

	run_program = set_field(run_program, AC_ACCESS_REGISTER_CMDTYPE, 2);

	switch (size) {
		case 1:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 0);
			break;
		case 2:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 1);
			break;
		case 4:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 2);
			break;
		//case 8: //Batch unsupport read dobule-world
		//	run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 3);
		//	break;
		default:
			LOG_ERROR("unsupported access size: %d", size);
			return ERROR_FAIL;
	};

	//set AAMVIRTUAL 0, because of Bug 18878:only support aamvirtual=0,
	//hardware does not implement aamvirtual=1, but does not means abstract command only use pa address
	//if mprv enabled (implement on riscv.c:ndsv5_virtual_to_physical function) , abstract command can process va address.
	run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMVIRTUAL, 0);

	//const addr mode set aampostincrement=0
	if (nds32->nds_const_addr_mode == 0)
		run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMPOSTINCREMENT, 1);
	else
		run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMPOSTINCREMENT, 0);

	run_program = set_field(run_program, AC_ACCESS_MEMORY_WRITE, 0);

	// Set address based on DXLEN(XLEN)
	unsigned int xlen = riscv_xlen(target);
	if (xlen == 64) {
		dmi_write(target, DMI_DATA2, address);
		dmi_write(target, DMI_DATA3, address >> 32);
	} else {
		dmi_write(target, DMI_DATA1, address);
	}

	if( execute_abstract_command(target, run_program) != ERROR_OK ) {
		LOG_ERROR("Abstract memory access fail");
		return ERROR_FAIL;
	}

	// First read has just triggered. Result is in s1.
	dmi_write(target, DMI_ABSTRACTAUTO,
			1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

	// read_addr is the next address that the hart will read from, which is the
	// value in data0
	riscv_addr_t read_addr = address + size;
	riscv_addr_t receive_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	//unsigned skip = 1;
	while (read_addr < fin_addr) {
		LOG_DEBUG("read_addr=0x%" PRIx64 ", receive_addr=0x%" PRIx64
			", fin_addr=0x%" PRIx64, read_addr, receive_addr, fin_addr);

		LOG_DEBUG("creating burst to read from 0x%" TARGET_PRIxADDR
			" up to 0x%" TARGET_PRIxADDR, read_addr, fin_addr);
		assert(read_addr >= address && read_addr < fin_addr);

		struct riscv_batch *batch = riscv_batch_alloc(target, nds_jtag_max_scans,
			info->dmi_busy_delay + info->ac_busy_delay);

		size_t reads = 0;
		for (riscv_addr_t addr = read_addr; addr < fin_addr; addr += size) {
			riscv_batch_add_dmi_read(batch, DMI_DATA0);
			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		if (riscv_batch_run(batch) != ERROR_OK) {
			riscv013_clear_abstract_error(target);
			dmi_write(target, DMI_ABSTRACTAUTO, 0);
			riscv_batch_free(batch);
			increase_dmi_busy_delay(target);
			if (ndsv5_read_memory_retry_cnt < nds_dmi_busy_retry_times) {
				ndsv5_read_memory_retry_cnt ++;
				goto read_memory_abstract_retry;
			} else {
				LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
				return ERROR_FAIL;
			}
		}

		// Wait for the target to finish performing the last abstract command,
		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);

		unsigned cmderr = info->cmderr;
		riscv_addr_t next_read_addr;
		uint32_t dmi_data0 = -1;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_read_addr = read_addr + reads * size;
				break;
			case CMDERR_BUSY:
				LOG_ERROR("memory read resulted in busy response");

				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);
				dmi_write(target, DMI_ABSTRACTAUTO, 0);

				riscv_batch_free(batch);
				if (ndsv5_read_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_read_memory_retry_cnt ++;
					goto read_memory_abstract_retry;
				} else {
					LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
					return ERROR_FAIL;
				}

				// This is definitely a good version of the value that we
				// attempted to read when we discovered that the target was
				// busy.
				dmi_data0 = dmi_read(target, DMI_DATA0);

				// Clobbers DMI_DATA0.
				if (size == 8) {
					dmi_write(target, DMI_DATA2, next_read_addr);
					dmi_write(target, DMI_DATA3, next_read_addr >> 32);
				} else {
					dmi_write(target, DMI_DATA1, next_read_addr);
				}

				if( execute_abstract_command(target, run_program) != ERROR_OK ) {
					LOG_ERROR("Abstract memory access fail");
					return ERROR_FAIL;
				}

				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);
				break;
			default:
				LOG_ERROR("error when reading memory, abstractcs=0x%08lx", (long)abstractcs);
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv013_clear_abstract_error(target);
				riscv_batch_free(batch);
				return ERROR_FAIL;
		}

		// Now read whatever we got out of the batch.
		for (size_t i = 0; i < reads; i++) {
			if (read_addr >= next_read_addr) {
				break;
			}

			read_addr += size;

			riscv_addr_t offset = receive_addr - address;
			uint64_t dmi_out = riscv_batch_get_dmi_read(batch, i);
			uint32_t value = get_field(dmi_out, DTM_DMI_DATA);
			write_to_buf(buffer + offset, value, size);
			LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08x", receive_addr, value);

			receive_addr += size;
		}
		riscv_batch_free(batch);

		if (cmderr == CMDERR_BUSY) {
			riscv_addr_t offset = receive_addr - address;
			write_to_buf(buffer + offset, dmi_data0, size);
			LOG_ERROR("M[0x%" TARGET_PRIxADDR "] reads 0x%08x", receive_addr,
				dmi_data0);
			read_addr += size;
			receive_addr += size;
		}
	}

	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (count >= 1) {
		// Read the last word
		uint64_t value = dmi_read(target, DMI_DATA0);
		write_to_buf(buffer + receive_addr - address, value, size);
		LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%" PRIx64, receive_addr, value);
		receive_addr += size;
	}

	return ERROR_OK;
}

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

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			size, address);

	select_dmi(target);
	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */
	uint64_t s0, s1;
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read_direct(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	// Write the program (load, increment)
	struct riscv_program program;
#if _NDS_BATCH_RUN_RETRY_
uint32_t ndsv5_read_memory_retry_cnt = 0;
sifive_read_memory_retry:
#endif
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
#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->nds_const_addr_mode == 0)
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#else
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#endif

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	riscv_program_write(&program);

	// Write address to S0, and execute buffer.
	if (register_write_direct(target, GDB_REGNO_S0, address) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t command = access_register_command(GDB_REGNO_S1, riscv_xlen(target),
				AC_ACCESS_REGISTER_TRANSFER |
				AC_ACCESS_REGISTER_POSTEXEC);

#if _NDS_V5_ONLY_
	if (execute_abstract_command(target, command) != ERROR_OK) {
		//restore $s0/$s1 here
		riscv_set_register(target, GDB_REGNO_S0, s0);
		riscv_set_register(target, GDB_REGNO_S1, s1);
		return ERROR_FAIL;
	}
#else
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;
#endif

	// First read has just triggered. Result is in s1.

	dmi_write(target, DMI_ABSTRACTAUTO,
			1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

	// read_addr is the next address that the hart will read from, which is the
	// value in s0.
	riscv_addr_t read_addr = address + size;
	// The next address that we need to receive data for.
	riscv_addr_t receive_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	unsigned skip = 1;
	while (read_addr < fin_addr) {
		LOG_DEBUG("read_addr=0x%" PRIx64 ", receive_addr=0x%" PRIx64
				", fin_addr=0x%" PRIx64, read_addr, receive_addr, fin_addr);
		// The pipeline looks like this:
		// memory -> s1 -> dm_data0 -> debugger
		// It advances every time the debugger reads dmdata0.
		// So at any time the debugger has just read mem[s0 - 3*size],
		// dm_data0 contains mem[s0 - 2*size]
		// s1 contains mem[s0-size]

		LOG_DEBUG("creating burst to read from 0x%" TARGET_PRIxADDR
				" up to 0x%" TARGET_PRIxADDR, read_addr, fin_addr);
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
			riscv_batch_add_dmi_read(batch, DMI_DATA0);

			reads++;
			if (riscv_batch_full(batch))
				break;
		}
#if _NDS_BATCH_RUN_RETRY_
		if (riscv_batch_run(batch) != ERROR_OK) {
		    riscv013_clear_abstract_error(target);
			dmi_write(target, DMI_ABSTRACTAUTO, 0);
			riscv_batch_free(batch);
			increase_dmi_busy_delay(target);
			if (ndsv5_read_memory_retry_cnt < nds_dmi_busy_retry_times) {
				ndsv5_read_memory_retry_cnt ++;
				goto sifive_read_memory_retry;
			}
            else {
				riscv_set_register(target, GDB_REGNO_S0, s0);
				riscv_set_register(target, GDB_REGNO_S1, s1);
		        LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
                return ERROR_FAIL;
            }
		}
#else
		riscv_batch_run(batch);
#endif
		// Wait for the target to finish performing the last abstract command,
		// and update our copy of cmderr.
		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);

		unsigned cmderr = info->cmderr;
		riscv_addr_t next_read_addr;
		uint32_t dmi_data0 = -1;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_read_addr = read_addr + reads * size;
				break;
			case CMDERR_BUSY:
				LOG_ERROR("memory read resulted in busy response");

				/*
				 * If you want to exercise this code path, apply the following patch to spike:
--- a/riscv/debug_module.cc
+++ b/riscv/debug_module.cc
@@ -1,3 +1,5 @@
+#include <unistd.h>
+
 #include <cassert>
 
 #include "debug_module.h"
@@ -398,6 +400,15 @@ bool debug_module_t::perform_abstract_command()
       // Since the next instruction is what we will use, just use nother NOP
       // to get there.
       write32(debug_abstract, 1, addi(ZERO, ZERO, 0));
+
+      if (abstractauto.autoexecdata &&
+          program_buffer[0] == 0x83 &&
+          program_buffer[1] == 0x24 &&
+          program_buffer[2] == 0x04 &&
+          program_buffer[3] == 0 &&
+          rand() < RAND_MAX / 10) {
+        usleep(1000000);
+      }
     } else {
       write32(debug_abstract, 1, ebreak());
     }
				 */
				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);

				dmi_write(target, DMI_ABSTRACTAUTO, 0);
#if _NDS_BATCH_RUN_RETRY_
			    riscv_batch_free(batch);
				if (ndsv5_read_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_read_memory_retry_cnt ++;
					goto sifive_read_memory_retry;
				}
                else {
				    riscv_set_register(target, GDB_REGNO_S0, s0);
				    riscv_set_register(target, GDB_REGNO_S1, s1);
		            LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
					return ERROR_FAIL;
                }
#endif

				// This is definitely a good version of the value that we
				// attempted to read when we discovered that the target was
				// busy.
				dmi_data0 = dmi_read(target, DMI_DATA0);

				// Clobbers DMI_DATA0.
				if (register_read_direct(target, &next_read_addr, GDB_REGNO_S0) != ERROR_OK)
					return ERROR_FAIL;
				// Restore the command, and execute it.
				// Now DMI_DATA0 contains the next value just as it would if no
				// error had occurred.
				dmi_write(target, DMI_COMMAND, command);

				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);
				break;
			default:
				LOG_ERROR("error when reading memory, abstractcs=0x%08lx", (long)abstractcs);
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv_set_register(target, GDB_REGNO_S0, s0);
				riscv_set_register(target, GDB_REGNO_S1, s1);
				riscv013_clear_abstract_error(target);
				riscv_batch_free(batch);
				return ERROR_FAIL;
		}

		// Now read whatever we got out of the batch.
		for (size_t i = 0; i < reads; i++) {
			if (read_addr >= next_read_addr) {
				break;
			}

			read_addr += size;

			if (skip > 0) {
				skip--;
				continue;
			}

			riscv_addr_t offset = receive_addr - address;
			uint64_t dmi_out = riscv_batch_get_dmi_read(batch, i);
			uint32_t value = get_field(dmi_out, DTM_DMI_DATA);
			write_to_buf(buffer + offset, value, size);
			LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08x", receive_addr,
					value);

			receive_addr += size;
		}
		riscv_batch_free(batch);

		if (cmderr == CMDERR_BUSY) {
			riscv_addr_t offset = receive_addr - address;
			write_to_buf(buffer + offset, dmi_data0, size);
			LOG_ERROR("M[0x%" TARGET_PRIxADDR "] reads 0x%08x", receive_addr,
					dmi_data0);
			read_addr += size;
			receive_addr += size;
		}
	}

	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (count > 1) {
		// Read the penultimate word.
		uint64_t value = dmi_read(target, DMI_DATA0);
		write_to_buf(buffer + receive_addr - address, value, size);
		LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%" PRIx64, receive_addr, value);
		receive_addr += size;
	}

	// Read the last word.
	uint64_t value;
	if (register_read_direct(target, &value, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	write_to_buf(buffer + receive_addr - address, value, size);
	LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%" PRIx64, receive_addr, value);
	receive_addr += size;

	riscv_set_register(target, GDB_REGNO_S0, s0);
	riscv_set_register(target, GDB_REGNO_S1, s1);
	return ERROR_OK;
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
		result = ndsv5_read_memory_abstract_access(target, address, size, count, buffer);
		if(result == ERROR_OK)
			goto read_memory_finish;
	}
#else
	if (info->progbufsize >= 2)
		return read_memory_progbuf(target, address, size, count, buffer);
#endif

	//according to eticket 16199, default no support system bus access, so ndsv5_system_bus_access default value is 0
	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 0 && ndsv5_system_bus_access == 1)
			return read_memory_bus_v0(target, address, size, count, buffer);
		else if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 1 && ndsv5_system_bus_access == 1) {
#if _NDS_V5_ONLY_
			if (nds_jtag_scans_optimize > 0) {
				result = read_memory_bus_v1_opt(target, address, size, count, buffer);
				goto read_memory_finish;
			} else {
				result = read_memory_bus_v1(target, address, size, count, buffer);
				goto read_memory_finish;
			}
#else
			return read_memory_bus_v1(target, address, size, count, buffer);
#endif
		}
	}
#if _NDS_V5_ONLY_
	if (info->progbufsize >= 2) {
		result = read_memory_progbuf(target, address, size, count, buffer);
		goto read_memory_finish;
	}
#else
	if (info->progbufsize >= 2)
		return read_memory_progbuf(target, address, size, count, buffer);
#endif

	LOG_ERROR("Don't know how to read memory on this target.");
	return ERROR_FAIL;

#if _NDS_V5_ONLY_
read_memory_finish:
	if (result == ERROR_OK) {
		uint32_t *p_word_data = (uint32_t *)buffer;
		NDS_INFO("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR " = 0x%08x", count,
			size, address, *p_word_data);
	}
	return result;
#endif
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

#if _NDS_V5_ONLY_
static int write_memory_bus_v1_opt(struct target *target, target_addr_t address,
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
#endif

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DMI_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DMI_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	sb_write_address(target, next_address);
	while (next_address < end_address) {
		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (size > 12)
				dmi_write(target, DMI_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));
			if (size > 8)
				dmi_write(target, DMI_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				dmi_write(target, DMI_SBDATA1,
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
			dmi_write(target, DMI_SBDATA0, value);
			log_memory_access(address + i * size, value, size, false);

			if (info->bus_master_write_delay) {
				jtag_add_runtest(info->bus_master_write_delay, TAP_IDLE);
				if (jtag_execute_queue() != ERROR_OK) {
					LOG_ERROR("Failed to scan idle sequence");
					return ERROR_FAIL;
				}
			}
		}
		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);
			next_address = sb_read_address(target);
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs, DMI_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int ndsv5_write_memory_abstract_access(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	LOG_DEBUG("Abstract Memory Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);

	uint32_t run_program = 0;
	uint32_t ndsv5_write_memory_retry_cnt = 0;
write_memory_abstract_retry:
	LOG_DEBUG("info->dmi_busy_delay=0x%x, info->ac_busy_delay=0x%x", info->dmi_busy_delay, info->ac_busy_delay);

	run_program = set_field(run_program, AC_ACCESS_REGISTER_CMDTYPE, 2);

	switch (size) {
		case 1:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 0);
			break;
		case 2:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 1);
			break;
		case 4:
			run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 2);
			break;
		//case 8: //Batch unsupport read dobule-world
		//	run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMSIZE, 3);
		//	break;
		default:
			LOG_ERROR("unsupported access size: %d", size);
			return ERROR_FAIL;
	};

	//set AAMVIRTUAL 0, because of Bug 18878:only support aamvirtual=0,
	//hardware does not implement aamvirtual=1, but does not means abstract command only use pa address
	//if mprv enabled(implement on riscv.c:ndsv5_virtual_to_physical function) , abstract command can process va address.
	run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMVIRTUAL, 0);

	//const addr mode set aampostincrement=0
	if (nds32->nds_const_addr_mode == 0)
		run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMPOSTINCREMENT, 1);
	else
		run_program = set_field(run_program, AC_ACCESS_MEMORY_AAMPOSTINCREMENT, 0);

	run_program = set_field(run_program, AC_ACCESS_MEMORY_WRITE, 1);

	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	bool setup_needed = true;
	LOG_DEBUG("writing until final address 0x%016" PRIx64, fin_addr);
	while (cur_addr < fin_addr) {
		LOG_DEBUG("transferring burst starting at address 0x%016" PRIx64,
				cur_addr);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				nds_jtag_max_scans,
				info->dmi_busy_delay + info->ac_busy_delay);

		unsigned start = (cur_addr - address) / size;
		for (unsigned i = start; i < count; ++i) {
			unsigned offset = size*i;
			const uint8_t *t_buffer = buffer + offset;

			uint32_t value;
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
			cur_addr += size;

			if (setup_needed) {
				// Set address based on DXLEN(XLEN)
				unsigned int xlen = riscv_xlen(target);
				if (xlen == 64) {
					dmi_write(target, DMI_DATA2, address + offset);
					dmi_write(target, DMI_DATA3, (address + offset) >> 32);
				} else {
					dmi_write(target, DMI_DATA1, address + offset);
				}

				// Write value.
				dmi_write(target, DMI_DATA0, value);

				if( execute_abstract_command(target, run_program) != ERROR_OK ) {
					LOG_ERROR("Abstract memory access fail");
					return ERROR_FAIL;
				}

				// Turn on autoexec
				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				riscv_batch_add_dmi_write(batch, DMI_DATA0, value);
				if (riscv_batch_full(batch))
					break;
			}
		}

		if ((batch) && (batch->used_scans != 0)) {
			if (riscv_batch_run(batch) != ERROR_OK) {
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv_batch_free(batch);
				increase_dmi_busy_delay(target);
				riscv013_clear_abstract_error(target);
				if (ndsv5_write_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_write_memory_retry_cnt ++;
					goto write_memory_abstract_retry;
				}
				else {
					LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
					return ERROR_FAIL;
				}
			}
		}
		riscv_batch_free(batch);

		// Note that if the scan resulted in a Busy DMI response, it
		// is this read to abstractcs that will cause the dmi_busy_delay
		// to be incremented if necessary.

		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory write");
				break;
			case CMDERR_BUSY:
				LOG_ERROR("memory write resulted in busy response");
				riscv013_clear_abstract_error(target);
				increase_ac_busy_delay(target);

				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				if (ndsv5_write_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_write_memory_retry_cnt ++;
					goto write_memory_abstract_retry;
				} else {
					LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
					return ERROR_FAIL;
				}

				if (size == 8) {
					cur_addr =  dmi_read(target, DMI_DATA2);
					cur_addr |= dmi_read(target, DMI_DATA3) << 32;
				} else {
					cur_addr = dmi_read(target, DMI_DATA1);
				}

				setup_needed = true;
				break;

			default:
				LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv013_clear_abstract_error(target);
				return ERROR_FAIL;
		}
	}

	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	LOG_DEBUG("writing %d words of %d bytes to 0x%08lx", count, size, (long)address);

	select_dmi(target);

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */

	uint64_t s0, s1;
	if (register_read_direct(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read_direct(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	// Write the program (store, increment)
	struct riscv_program program;
#if _NDS_BATCH_RUN_RETRY_
uint32_t ndsv5_write_memory_retry_cnt = 0;
sifive_write_memory_retry:
#endif
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

#if _NDS_V5_ONLY_
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->nds_const_addr_mode == 0)
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#else
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
#endif

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
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
#else
		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				32,
				info->dmi_busy_delay + info->ac_busy_delay);
#endif

		/* To write another word, we put it in S1 and execute the program. */
		unsigned start = (cur_addr - address) / size;
		for (unsigned i = start; i < count; ++i) {
			unsigned offset = size*i;
			const uint8_t *t_buffer = buffer + offset;

			uint32_t value;
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
			cur_addr += size;

			if (setup_needed) {
				if (register_write_direct(target, GDB_REGNO_S0,
							address + offset) != ERROR_OK)
					return ERROR_FAIL;

				// Write value.
				dmi_write(target, DMI_DATA0, value);

				// Write and execute command that moves value into S1 and
				// executes program buffer.
				uint32_t command = access_register_command(GDB_REGNO_S1, 32, 
						AC_ACCESS_REGISTER_POSTEXEC |
						AC_ACCESS_REGISTER_TRANSFER |
						AC_ACCESS_REGISTER_WRITE);
				int result = execute_abstract_command(target, command);

#if _NDS_V5_ONLY_
				if (result != ERROR_OK) {
					//restore $s0/$s1 here
					riscv_set_register(target, GDB_REGNO_S0, s0);
					riscv_set_register(target, GDB_REGNO_S1, s1);
					return result;
				}
#else
				if (result != ERROR_OK)
					return result;
#endif

				// Turn on autoexec
				dmi_write(target, DMI_ABSTRACTAUTO,
						1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				riscv_batch_add_dmi_write(batch, DMI_DATA0, value);
				if (riscv_batch_full(batch))
					break;
			}
		}
#if _NDS_BATCH_RUN_RETRY_
		if ((batch) && (batch->used_scans != 0)) {
			if (riscv_batch_run(batch) != ERROR_OK) {
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv_batch_free(batch);
				increase_dmi_busy_delay(target);
				riscv013_clear_abstract_error(target);
				if (ndsv5_write_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_write_memory_retry_cnt ++;
					goto sifive_write_memory_retry;
				}
                else {
				    riscv_set_register(target, GDB_REGNO_S0, s0);
				    riscv_set_register(target, GDB_REGNO_S1, s1);
		            LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
                    return ERROR_FAIL;
                }
			}
		}
#else
		riscv_batch_run(batch);
#endif
		riscv_batch_free(batch);

		// Note that if the scan resulted in a Busy DMI response, it
		// is this read to abstractcs that will cause the dmi_busy_delay
		// to be incremented if necessary.

		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		info->cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory write");
				break;
			case CMDERR_BUSY:
				LOG_ERROR("memory write resulted in busy response");
				riscv013_clear_abstract_error(target);
				increase_ac_busy_delay(target);

				dmi_write(target, DMI_ABSTRACTAUTO, 0);
#if _NDS_BATCH_RUN_RETRY_
				//riscv_batch_free(batch); already done after riscv_batch_run
				if (ndsv5_write_memory_retry_cnt < nds_dmi_busy_retry_times) {
					ndsv5_write_memory_retry_cnt ++;
					goto sifive_write_memory_retry;
				}
                else {
				    riscv_set_register(target, GDB_REGNO_S0, s0);
				    riscv_set_register(target, GDB_REGNO_S1, s1);
		            LOG_ERROR("please set dmi_busy_retry_times > %d to increase delay cycles", nds_dmi_busy_retry_times);
                    return ERROR_FAIL;
                }
#endif
				if (register_read_direct(target, &cur_addr, GDB_REGNO_S0) != ERROR_OK)
					return ERROR_FAIL;
				setup_needed = true;
				break;

			default:
				LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
				dmi_write(target, DMI_ABSTRACTAUTO, 0);
				riscv013_clear_abstract_error(target);
				riscv_set_register(target, GDB_REGNO_S0, s0);
				riscv_set_register(target, GDB_REGNO_S1, s1);
				return ERROR_FAIL;
		}
	}

	dmi_write(target, DMI_ABSTRACTAUTO, 0);

	if (register_write_direct(target, GDB_REGNO_S1, s1) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
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
		if(ndsv5_write_memory_abstract_access(target, address, size, count, buffer) == ERROR_OK)
			return ERROR_OK;
	}

#endif

	//according to eticket 16199, default no support system bus access, so ndsv5_system_bus_access default value is 0
	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 0 && ndsv5_system_bus_access == 1) {
			return write_memory_bus_v0(target, address, size, count, buffer);
		}
		else if (get_field(info->sbcs, DMI_SBCS_SBVERSION) == 1 && ndsv5_system_bus_access == 1) {
#if _NDS_V5_ONLY_
			if (nds_jtag_scans_optimize > 0) {
				return write_memory_bus_v1_opt(target, address, size, count, buffer);
			}
#endif
			return write_memory_bus_v1(target, address, size, count, buffer);
		}
	}
	if (info->progbufsize >= 2) {
		return write_memory_progbuf(target, address, size, count, buffer);
	}
	LOG_ERROR("Don't know how to write memory on this target.");
	return ERROR_FAIL;
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
	.halt = &riscv_openocd_halt,
	.resume = &riscv_openocd_resume,
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
	LOG_DEBUG("reading register %s on [%s] hart %d", gdb_regno_name(rid), target->tap->dotted_name, hid);
#else
	LOG_DEBUG("reading register %s on hart %d", gdb_regno_name(rid), hid);
#endif

	riscv_set_current_hartid(target, hid);
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		*value = -1;
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif

	int result = ERROR_OK;
	if (rid <= GDB_REGNO_XPR31) {
		result = register_read_direct(target, value, rid);
	} else if (rid == GDB_REGNO_PC) {
		result = register_read_direct(target, value, GDB_REGNO_DPC);
		LOG_DEBUG("read PC from DPC: 0x%016" PRIx64, *value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		buf_set_u64((unsigned char *)value, 0, 8, get_field(dcsr, CSR_DCSR_PRV));
	} else {
		result = register_read_direct(target, value, rid);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to read register %d", rid);
			*value = -1;
		}
	}

	return result;
}

static int riscv013_set_register(struct target *target, int hid, int rid, uint64_t value)
{
#if _NDS_V5_ONLY_
	LOG_DEBUG("writing 0x%" PRIx64 " to register %s on [%s] hart %d", value,
			gdb_regno_name(rid), target->tap->dotted_name, hid);
#else
	LOG_DEBUG("writing 0x%" PRIx64 " to register %s on hart %d", value,
			gdb_regno_name(rid), hid);
#endif

	riscv_set_current_hartid(target, hid);
#if _NDS_V5_ONLY_
	if (!riscv_hart_enabled(target, hid)) {
		LOG_DEBUG("hart %d is unavailable, do nothing", hid);
		return ERROR_OK;
	}
#endif

	if (rid <= GDB_REGNO_XPR31) {
		return register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_DEBUG("writing PC to DPC: 0x%016" PRIx64, value);
		register_write_direct(target, GDB_REGNO_DPC, value);
		uint64_t actual_value;
		register_read_direct(target, &actual_value, GDB_REGNO_DPC);
		LOG_DEBUG("  actual DPC written: 0x%016" PRIx64, actual_value);
		if (value != actual_value) {
			LOG_ERROR("Written PC (0x%" PRIx64 ") does not match read back "
					"value (0x%" PRIx64 ")", value, actual_value);
			return ERROR_FAIL;
		}
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, value);
		return register_write_direct(target, GDB_REGNO_DCSR, dcsr);
	} else {
		return register_write_direct(target, rid, value);
	}

	return ERROR_OK;
}

static void riscv013_select_current_hart(struct target *target)
{
	RISCV_INFO(r);

	uint64_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
#if _NDS_V5_ONLY_
	if( !get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE) ) {
		NDS32_LOG("<-- TARGET WARNING! DMACTIVE has been pulse to low, turn-on DM again. -->");
		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_DMACTIVE, 1);
	}
#endif
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HARTSELLO, r->current_hartid);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
}

static int riscv013_halt_current_hart(struct target *target)
{
	RISCV_INFO(r);

#if _NDS_V5_ONLY_
	LOG_DEBUG("halting [%s] hart %d", target->tap->dotted_name, r->current_hartid);
#else
	LOG_DEBUG("halting hart %d", r->current_hartid);
#endif

	if (riscv_is_halted(target))
		LOG_ERROR("Hart %d is already halted!", r->current_hartid);

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HALTREQ, 1);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
#if _NDS_V5_ONLY_
	for (size_t i = 0; i < MAX_RETRY; ++i)
		if (riscv_is_halted(target))
			break;
#else
	for (size_t i = 0; i < 256; ++i)
		if (riscv_is_halted(target))
			break;
#endif

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
		dmcontrol = dmi_read(target, DMI_DMCONTROL);

#if _NDS_V5_ONLY_
		NDS32_LOG("<-- Unable to halt [%s] hart %d -->", target->tap->dotted_name, r->current_hartid);
		NDS32_LOG("  dmcontrol=0x%08x", dmcontrol);
		NDS32_LOG("  dmstatus =0x%08x", dmstatus);
		if (riscv_rtos_enabled(target)) {
			target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0xff;
			LOG_DEBUG("  hart_unavailable[%d] = 0x%x", riscv_current_hartid(target), target->rtos->hart_unavailable[riscv_current_hartid(target)]);
		}
#else
		LOG_ERROR("unable to halt hart %d", r->current_hartid);
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
#endif
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
	return ERROR_OK;
}

static int riscv013_resume_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, false);
}

static int riscv013_step_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, true);
}

static int riscv013_on_resume(struct target *target)
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

extern uint32_t nds_no_crst_detect;
static bool riscv013_is_halted(struct target *target)
{
	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);

#if _NDS_V5_ONLY_
	if (riscv_rtos_enabled(target)) {
		target->rtos->hart_unavailable[riscv_current_hartid(target)] = 0x0;
	}
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
#else
	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("hart %d is unavailiable", riscv_current_hartid(target));
	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("hart %d doesn't exist", riscv_current_hartid(target));
#endif

#if _NDS_V5_ONLY_
	if ((target->state != TARGET_HALTED) &&
	    (nds_no_crst_detect == 0) &&
	    (get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET) == 1) ) {
		LOG_DEBUG(NDS32_ERRMSG_TARGET_RESET);

		uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_ACKHAVERESET, 1);

		int retry = 0;
		do {
			dmi_write(target, DMI_DMCONTROL, dmcontrol);
			dmstatus = dmi_read(target, DMI_DMSTATUS);
			alive_sleep(10);
		} while( get_field(dmstatus, DMI_DMSTATUS_ANYHAVERESET) && (retry++ < MAX_RETRY) );

		if(retry >= MAX_RETRY) {
			NDS32_LOG_ERROR("<-- TARGET WARNING! Unable to clear havereset on [%s] hart %d. -->", target->tap->dotted_name, riscv_current_hartid(target));
		}
	}
#endif

	return get_field(dmstatus, DMI_DMSTATUS_ALLHALTED);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return RISCV_HALT_UNKNOWN;

	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_SWBP:
	case CSR_DCSR_CAUSE_TRIGGER:
		return RISCV_HALT_BREAKPOINT;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_DEBUGINT:
	case CSR_DCSR_CAUSE_HALT:
		return RISCV_HALT_INTERRUPT;
	}

	LOG_ERROR("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%016lx", (long)dcsr);
#if _NDS_DISABLE_ABORT_
	NDS32_LOG("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	NDS32_LOG("  dcsr=0x%016lx", (long)dcsr);
#endif
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
#endif
	return dmi_write(target, DMI_PROGBUF0 + index, data);
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
#else
	return dmi_read(target, DMI_PROGBUF0 + index);
#endif
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

#if _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF
	if (write_debug_buffer_batch) {
		LOG_DEBUG("with batch");
		riscv_batch_add_dmi_write(write_debug_buffer_batch, DMI_COMMAND, run_program);
		size_t index_ABSTRACTCS = riscv_batch_add_dmi_read(write_debug_buffer_batch, DMI_ABSTRACTCS);

#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
		size_t index_debug_buf[16];
		for (size_t i = 0; i < riscv_debug_buffer_size(target); ++i)
			if (i >= riscv_debug_buffer_size(target))	///!? - p->data_count
				index_debug_buf[i] = riscv_batch_add_dmi_read(write_debug_buffer_batch, DMI_PROGBUF0 + i);
#endif

riscv013_execute_debug_buffer_retry:
		if (riscv_batch_run(write_debug_buffer_batch) != ERROR_OK) {
			increase_dmi_busy_delay(target);
			write_debug_buffer_batch->idle_count ++;
			goto riscv013_execute_debug_buffer_retry;
		}

		uint64_t dmi_out = riscv_batch_get_dmi_read(write_debug_buffer_batch, index_ABSTRACTCS);
		uint64_t cs = buf_get_u64((uint8_t *)&dmi_out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

#if _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF
		for (size_t i = 0; i < riscv_debug_buffer_size(target); ++i)
			if (i >= riscv_debug_buffer_size(target)) {	///!? - p->data_count
				uint64_t tmp_debug_buf_value = riscv_batch_get_dmi_read(write_debug_buffer_batch, index_debug_buf[i]);
				backup_debug_buffer[riscv_current_hartid(target)][i] = buf_get_u64((uint8_t *)&tmp_debug_buf_value, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
				//LOG_DEBUG("backup_debug_buffer[] = 0x%lx", backup_debug_buffer[riscv_current_hartid(target)][i]);
			}
#endif

		riscv_batch_free(write_debug_buffer_batch);
		write_debug_buffer_batch = NULL;
		LOG_DEBUG("index_ABSTRACTCS=0x%x, dmi_out=0x%lx, cs=0x%lx", (unsigned int)index_ABSTRACTCS, (long unsigned int)dmi_out, (long unsigned int)cs);

#if _NDS_MEM_Q_ACCESS_
		nds_dmi_abstractcs = cs;
		if (get_field(cs, DMI_ABSTRACTCS_CMDERR) != 0) {
			if (nds_dmi_quick_access_ena) {
				if ( ((nds_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) == CMDERR_HALT_RESUME) {
					// do NOT output this message to IDE, only in log files
					LOG_ERROR("unable to execute program: (abstractcs=0x%lx)", (long unsigned int)cs);
					riscv013_clear_abstract_error(target);
					dmi_read(target, DMI_DMSTATUS);
					return ERROR_FAIL;
				}
			}
		}
#endif

		LOG_DEBUG("(cs=0x%lx)", (long unsigned int)cs);
		if (get_field(cs, DMI_ABSTRACTCS_CMDERR) != 0) {
			LOG_ERROR("unable to execute program: (abstractcs=0x%lx)", (long unsigned int)cs);
			riscv013_clear_abstract_error(target);
			dmi_read(target, DMI_DMSTATUS);
			return ERROR_FAIL;
		}
		return ERROR_OK;
	}
	//else
	LOG_DEBUG("without batch");
#endif
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

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK)
		LOG_ERROR("Unable to execute fence.i");

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	int result = register_read_direct(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 1);

#if _NDS_V5_ONLY_
	extern bool v5_stepie;
	if(v5_stepie == true) {
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 1);
	} else {
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE, 0);
	}


	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if((nds32->attached == false) && (nds32->target_burn_attached == false)) {
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 0);
		dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 0);
		dcsr = set_field(dcsr, CSR_DCSR_STEPIE , 0);
	}
#endif

	return riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
}

static int riscv013_step_or_resume_current_hart(struct target *target, bool step)
{
	RISCV_INFO(r);

#if _NDS_V5_ONLY_
	LOG_DEBUG("resuming [%s] hart %d (for step?=%d)", target->tap->dotted_name, r->current_hartid, step);
#else
	LOG_DEBUG("resuming hart %d (for step?=%d)", r->current_hartid, step);
#endif
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart %d is not halted!", r->current_hartid);
		return ERROR_FAIL;
	}

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK) {
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("resuming [%s] hart %d (for step?=%d) fail", target->tap->dotted_name, r->current_hartid, step);
#endif
		return ERROR_FAIL;
	}

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_RESUMEREQ, 1);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);

	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
		if (get_field(dmstatus, DMI_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0)
			continue;

		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_RESUMEREQ, 0);
		dmi_write(target, DMI_DMCONTROL, dmcontrol);
#if _NDS_V5_ONLY_
		// e-15336, when resuming, clear issued_halt
		target->halt_issued = false;
#endif
		return ERROR_OK;
	}

	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	dmcontrol = dmi_read(target, DMI_DMCONTROL);
	LOG_ERROR("unable to resume hart %d", r->current_hartid);
	LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	if (step) {
		LOG_ERROR("  was stepping, halting");
		riscv013_halt_current_hart(target);
		return ERROR_OK;
	}

#if _NDS_DISABLE_ABORT_
	NDS32_LOG("resuming [%s] hart %d (for step?=%d) fail", target->tap->dotted_name, r->current_hartid, step);
#endif

	return ERROR_FAIL;
}

static void riscv013_clear_abstract_error(struct target *target)
{
	/* Wait for busy to go away. */
	time_t start = time(NULL);
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY)) {
		abstractcs = dmi_read(target, DMI_ABSTRACTCS);

		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("abstractcs.busy is not going low after %d seconds "
					"(abstractcs=0x%x). The target is either really slow or "
					"broken. You could increase the timeout with riscv "
					"set_reset_timeout_sec.",
					riscv_command_timeout_sec, abstractcs);
			break;
		}
	}
	/* Clear the error status. */
	dmi_write(target, DMI_ABSTRACTCS, abstractcs & DMI_ABSTRACTCS_CMDERR);
}

#if _NDS_V5_ONLY_
uint32_t nds_without_announce = 0;

//struct reg_arch_type riscv_reg_arch_type_013 = {
//	.get = register_get,
//	.set = register_set
//};

int riscv013_poll_wo_announce(struct target *target)
{
	//return poll_target(target, false);
	nds_without_announce = 1;
	return riscv_openocd_poll(target);
}

int update_trigger_config(struct target *target)
{
	LOG_DEBUG("===== Begin =====");

	RISCV_INFO(r);
	//riscv013_info_t *info = get_info(target);

	int i;
	for ( i = 0; i < riscv_count_triggers(target); i++) {

		if (r->trigger_unique_id[i] != -1) {
			/// Is really should to check???
			continue;
		}

		register_write_direct(target, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		register_read_direct(target, &tdata1, GDB_REGNO_TDATA1);
		
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));
		if( type != 2 ) 
			continue;


		//if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
		//	// Trigger is already in use, presumably by user code.
		//	continue;
		//}

		tdata1 &= ~MCONTROL_DMODE(riscv_xlen(target));
		tdata1 = set_field(tdata1, MCONTROL_ACTION,  MCONTROL_ACTION_DEBUG_EXCEPTION);
		tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);

		tdata1 &= ~MCONTROL_M;
		tdata1 &= ~MCONTROL_H;
		tdata1 &= ~MCONTROL_S;
		tdata1 &= ~MCONTROL_U;

		tdata1 &= ~MCONTROL_EXECUTE;
		tdata1 &= ~MCONTROL_LOAD;
		tdata1 &= ~MCONTROL_STORE;

		register_write_direct(target, GDB_REGNO_TDATA1, tdata1);

		uint64_t tdata1_rb;
		register_read_direct(target, &tdata1_rb, GDB_REGNO_TDATA1);
		LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

		if (tdata1 != tdata1_rb) {
			LOG_DEBUG("Trigger %d doesn't support what we need; After writing 0x%"
					PRIx64 " to tdata1 it contains 0x%" PRIx64,
					i, tdata1, tdata1_rb);
			continue;
		}
	
	}

	LOG_DEBUG("===== END =====");
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
	uint32_t value = dmi_read(target, DMI_DATA0);
	LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] reads 0x%08lx", address, (long)value);
	uint32_t value_h = 0;
	if (size == 8) {
		value_h = dmi_read(target, DMI_DATA1);
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
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
	}
	riscv_program_insert(&program, csrrw(GDB_REGNO_ZERO, GDB_REGNO_S0, reg_num - GDB_REGNO_CSR0));
	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	} else {
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	}

	int result = riscv_program_exec(&program, target);
	nds_dmi_quick_access = nds_dmi_quick_access_bak;
	if (result != ERROR_OK) {
		// quick_access mode, if target state from freerun to halt
		if (nds_dmi_quick_access_ena) {
			if ( ((nds_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) == CMDERR_HALT_RESUME) {
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

	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	}
	riscv_program_insert(&program, csrrs(GDB_REGNO_S0, GDB_REGNO_ZERO, reg_num - GDB_REGNO_CSR0));
	if (riscv_xlen(target) == 64) {
		riscv_program_insert(&program, sd(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
		riscv_program_insert(&program, ld(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	} else {
		riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_ZERO, 192));  // data0
		riscv_program_insert(&program, lw(GDB_REGNO_S0, GDB_REGNO_ZERO, 200));  // data2
	}

	int result = riscv_program_exec(&program, target);
	nds_dmi_quick_access = nds_dmi_quick_access_bak;
	if (result != ERROR_OK) {
		// quick_access mode, if target state from freerun to halt
		if (nds_dmi_quick_access_ena) {
			if ( ((nds_dmi_abstractcs & DMI_ABSTRACTCS_CMDERR) >> DMI_ABSTRACTCS_CMDERR_OFFSET) == CMDERR_HALT_RESUME) {
				LOG_ERROR("quick_access fail, get_csr_reg_quick_access_retry");
				goto ndsv5_get_csr_reg_quick_access_retry;
			}
		}
	}
	uint64_t value = dmi_read(target, DMI_DATA0);
	LOG_DEBUG("value: 0x%lx", (long unsigned int)value);
	uint64_t value_h = 0;
	if (riscv_xlen(target) == 64) {
		value_h = dmi_read(target, DMI_DATA1);
		LOG_DEBUG("value_h: 0x%lx", (long unsigned int)value_h);
	}
	uint64_t reg_value =  (value_h << 32) | value;
	*preg_value = reg_value;
	return ERROR_OK;
}

int ndsv5_probe_pc_quick_access(struct target *target, uint64_t *pc_value)
{
	return ndsv5_get_csr_reg_quick_access(target, GDB_REGNO_DPC, pc_value);
}
#endif

#if _NDS_DMI_CHECK_TIMEOUT_
static double timeval_diff_ms(struct timeval *a_timeval_begin, struct timeval *a_timeval_end)
{
	return (double)((a_timeval_end->tv_usec - a_timeval_begin->tv_usec)/1000 + 
		        (a_timeval_end->tv_sec - a_timeval_begin->tv_sec)  *1000   );
}

static int check_idcode(struct target *target)
{
	struct scan_field field;
	uint8_t in_value[4];

	jtag_add_ir_scan(target->tap, &select_idcode, TAP_IDLE);
	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("IDCODE: 0x0 -> 0x%x", in);

	if( (in == 0xFFFFFFFF) ||
	    (in == 0x00000000) || 
	    ((in&0x1) == 0   )    ) {	///< bit0 of IDCODE should be 1.
		LOG_ERROR("check idcode failed: 0x%08x", in);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int dmi_check_timeout(struct target *target)
{
	gettimeofday (&end_time, NULL);
	if( timeval_diff_ms(&begin_time, &end_time) >= v5_count_to_check_dm ) {
		LOG_ERROR("dmi_read timeout!! checking DM alive!!");

		if( check_idcode(target) != ERROR_OK ) {
			NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
			exit(-1);
		} else {
			return DMI_STATUS_CHECK_TIMEOUT;
		}
	}
	return DMI_STATUS_BUSY;
}

#endif

#endif

#if _NDS_V5_ONLY_
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
	} else {    
		return ERROR_OK;
	}
};

int nds_ace_get_reg(struct reg *reg) 
{
	struct target *target = (struct target *) reg->arch_info;
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
	struct target *target = (struct target *) reg->arch_info;
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

int ndsv5_reexamine(struct target *target)
{
	LOG_DEBUG("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	//if (riscv_init_registers(target) != ERROR_OK)
	//	return ERROR_FAIL;

	RISCV_INFO(r);
	r->current_hartid = target->coreid;
	riscv013_select_current_hart(target);

	return examine(target);
}

void decode_csr(char *text, unsigned address, unsigned data)
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
		{ GDB_REGNO_DCSR, DCSR_STEPIE, "stepie" },
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

int ndsv5_haltonreset(struct target *target, int enable)
{
	if( enable == 1 )
		LOG_DEBUG("Set setresethaltreq");
	else
	 	LOG_DEBUG("Set clrresethaltreq");

	RISCV_INFO(r);
	int hartid_bak = r->current_hartid;
	for(int i = 0; i < r->hart_count; i++) {
		r->current_hartid = i;
		riscv013_select_current_hart(target);

		uint32_t s = dmi_read(target, DMI_DMSTATUS);
		if(get_field(s, DMI_DMSTATUS_HASRESETHALTREQ)) {
			uint32_t control = dmi_read(target, DMI_DMCONTROL);

			if(enable == 1)
				control = set_field(control, DMI_DMCONTROL_SETRESETHALTREQ, 1);
			else
				control = set_field(control, DMI_DMCONTROL_CLRRESETHALTREQ, 1);
			dmi_write(target, DMI_DMCONTROL, control);

			if(enable == 1)
				LOG_DEBUG("hart %d: halt-on-reset is on!", i);
			else
				LOG_DEBUG("hart %d: halt-on-reset is off!", i);

		}

	}

	r->current_hartid = hartid_bak;
	return ERROR_OK;
}

struct riscv_batch *ndsv5_access_memory_pack_batch = NULL;
size_t index_access_mem_batch_ABSTRACTCS;
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

	uint32_t command = access_register_command(number, riscv_xlen(target),
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
		//LOG_DEBUG("%p: debug_buffer[%02x] = DASM(0x%08x)", program, i, program->debug_buffer[i]);
		if (ndsv5_write_debug_buffer_pack(program->target, i,
					program->debug_buffer[i]) != ERROR_OK)
			return ERROR_FAIL;
	}
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
		if (ndsv5_retry_cnt < nds_dmi_busy_retry_times) {
			ndsv5_retry_cnt ++;
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

int ndsv5_write_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	riscv013_info_t *info = get_info(target);
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
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		//} else if (riscv_batch_full(ndsv5_access_memory_pack_batch)) {
		} else if (ndsv5_access_memory_pack_batch->used_scans > (ndsv5_access_memory_pack_batch->allocated_scans - 12)) {
			result = ndsv5_access_memory_pack_batch_run(target, 1);
			if (result != ERROR_OK) {
				LOG_ERROR("access_memory_pack_batch FAIL !!");
				return ERROR_FAIL;
			}
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
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

		// Write value.
		ndsv5_write_abstract_arg_pack(target, 0, value);

		// Write and execute command that moves value into S1 and
		// executes program buffer.
		uint32_t command = access_register_command(GDB_REGNO_S1, 32,
				AC_ACCESS_REGISTER_POSTEXEC |
				AC_ACCESS_REGISTER_TRANSFER |
				AC_ACCESS_REGISTER_WRITE);
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_ABSTRACTCS);
	}
	return ERROR_OK;
}

int ndsv5_read_memory_progbuf_pack(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	riscv013_info_t *info = get_info(target);
	uint32_t i;
	uint64_t value;
	int result;

	for (i = 0; i < count; i++) {
		unsigned offset = size*i;

		if (ndsv5_access_memory_pack_batch == NULL) {
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
		//} else if (riscv_batch_full(ndsv5_access_memory_pack_batch)) {
		} else if (ndsv5_access_memory_pack_batch->used_scans > (ndsv5_access_memory_pack_batch->allocated_scans - 15)) {
			result = ndsv5_access_memory_pack_batch_run(target, 1);
			if (result != ERROR_OK) {
				LOG_ERROR("access_memory_pack_batch FAIL !!");
				return ERROR_FAIL;
			}
			ndsv5_access_memory_pack_batch = riscv_batch_alloc(target, nds_jtag_max_scans, info->dmi_busy_delay + info->ac_busy_delay);
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

		// Write and execute command that moves value into S1 and
		// executes program buffer.
		uint32_t command = access_register_command(GDB_REGNO_S1, riscv_xlen(target),
				AC_ACCESS_REGISTER_POSTEXEC |
				AC_ACCESS_REGISTER_TRANSFER);
		riscv_batch_add_dmi_write(ndsv5_access_memory_pack_batch, DMI_COMMAND, command);
		index_access_mem_batch_ABSTRACTCS = riscv_batch_add_dmi_read(ndsv5_access_memory_pack_batch, DMI_ABSTRACTCS);
		// move S1 to DATA0
		ndsv5_access_s0s1_direct_pack(target, GDB_REGNO_S1, address + offset, 0);

		// read value in DATA0
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

int ndsv5_vector_restore_vtype_vl(struct target *target, uint64_t reg_vtype)
{
	int result;

	register_write_direct(target, GDB_REGNO_S0, reg_vtype);
	LOG_DEBUG("reg_vtype: 0x%lx", (long unsigned int)reg_vtype);

	// Restore vtype & vl
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

// 1. The debugger can first set the SEW to XLEN by using the vsetvli instruction.
// 2. Then the debugger can use CSRR to read the vtype CSR to check the vill bit.
// 3. If the vill bit is set, then the debugger set the SEW to (SEW lenth of last vsetvli)/2 again.
// 4. Continue to step 2 and step 3 until the vill bit is not set.
// To get  VLMAX:
// vsetvli s1, x0, vtypei
int ndsv5_get_vector_VLMAX(struct target *target)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t vector_SEW = 64;
	uint64_t vector_vl = 0;
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vtype_tmp=0, reg_vl_tmp=0;
	int result;
	unsigned xlen = riscv_xlen(target);

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

		//if (vector_vl != 0) {  // need check if vtype.vill == 1 ?
		if ((reg_vtype_tmp & (0x01 << (xlen-1)) ) == 0) {  // check if vtype.vill == 1
			break;
		} else {
			vector_SEW = (vector_SEW >> 1);
		}
	}
	nds32->nds_vector_length = vector_SEW * vector_vl;
	nds32->nds_vector_SEW = vector_SEW;
	nds32->nds_vector_vl = (uint32_t)vector_vl;
	LOG_DEBUG("nds32->nds_vector_SEW: 0x%x, nds32->nds_vector_vl: 0x%x", nds32->nds_vector_SEW, nds32->nds_vector_vl);
	LOG_DEBUG("nds32->nds_vector_length: 0x%x", nds32->nds_vector_length);

	// Restore vtype & vl
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);

	riscv_get_register(target, &reg_vtype, CSR_VTYPE + GDB_REGNO_CSR0);
	riscv_get_register(target, &reg_vl, CSR_VL + GDB_REGNO_CSR0);
	LOG_DEBUG("new_reg_vtype: 0x%lx, new_reg_vl: 0x%lx", (long unsigned int)reg_vtype, (long unsigned int)reg_vl);
	return ERROR_OK;
}

// A. Backup vtype CSRs
// B. Use vsetvli to set vtype(vsew is set to XLEN).
// C. Repeat vext.x.v VLEN/XLEN times to read the vector regieter.
//    vext.x.v rd, vs2, rs1  # rd = vs2[rs1]
// D. Use vsetvl to restore vtype
int ndsv5_get_vector_register(struct target *target, enum gdb_regno r, char *pRegValue)
{
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vstart = 0;
	uint64_t vector_vl = 0, vector_value = 0;
	uint32_t i, j;
	int result;
	char *p_vector_value;
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

	register_write_direct(target, CSR_VSTART + GDB_REGNO_CSR0, 0);  // set vstart to 0
	for (i=0; i < nds32->nds_vector_vl; i++) {
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
		//LOG_DEBUG("vector_value = 0x%lx", (long unsigned int)vector_value);
		for (j=0; j < (nds32->nds_vector_SEW/8); j++) {
			*pRegValue++ = *p_vector_value++;
		}
	}
	// Restore vtype & vl
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);
	riscv_set_register(target, CSR_VSTART + GDB_REGNO_CSR0, reg_vstart);

	return ERROR_OK;
}

// A. Backup vstart, vl, vtype CSRs
// B. Set vstart to 0.
// C. Use vsetvli to set vl and vtype(vsew is set to XLEN, vl is set to VLEN/XLEN,  and vlmul is set to 0(no grouping)). 
//    vsetvli rd, rs1, vtypei # rd = new vl, rs1 = AVL, vtypei = new vtype setting 
//    Repeat vslide1down VLEN/XLEN times to write the vector register.
//    vslide1down.vx vd, vs2, rs1, vm      # vd[i] = vs2[i+1], vd[vl-1]=x[rs1]
// D. Use vsetvl to restore vl and vtype
//    vsetvl  rd, rs1, rs2    # rd = new vl, rs1 = AVL, rs2 = new vtype value 
// E. Restore vstart CSR
int ndsv5_set_vector_register(struct target *target, enum gdb_regno r, char *pRegValue)
{
	uint64_t reg_vtype = 0, reg_vl = 0, reg_vstart = 0;
	uint64_t vector_vl = 0, vector_value = 0;
	uint32_t i, j;
	int result;
	char *p_vector_value;
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

	register_write_direct(target, CSR_VSTART + GDB_REGNO_CSR0, 0);  // set vstart to 0
	for (i=0; i < nds32->nds_vector_vl; i++) {
		p_vector_value = (char *)&vector_value;
		for (j=0; j < (nds32->nds_vector_SEW/8); j++) {
			 *p_vector_value++ = *pRegValue++;
		}
		register_write_direct(target, GDB_REGNO_S0, vector_value);

		riscv_program_init(&program, target);
		riscv_program_vslide1down_vx(&program, r, r, GDB_REGNO_S0);
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK) {
			LOG_ERROR("riscv_program_vslide1down_vx ERROR !!");
			return ERROR_FAIL;
		}
	}
	// Restore vtype & vl & vstart
	ndsv5_vector_restore_vtype_vl(target, reg_vtype);
	riscv_set_register(target, CSR_VSTART + GDB_REGNO_CSR0, reg_vstart);

	return ERROR_OK;
}

#endif

