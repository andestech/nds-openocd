#ifndef RISCV_H
#define RISCV_H

struct riscv_program;

#include <stdint.h>
#include "opcodes.h"
#include "gdb_regs.h"

/* The register cache is staticly allocated. */
#define RISCV_MAX_HARTS 64
#define RISCV_MAX_REGISTERS 5000
#define RISCV_MAX_TRIGGERS 32
#define RISCV_MAX_HWBPS 16

#define DEFAULT_COMMAND_TIMEOUT_SEC		2

#if _NDS_V5_ONLY_
	#define DEFAULT_RESET_TIMEOUT_SEC		15
#else
	#define DEFAULT_RESET_TIMEOUT_SEC		30
#endif

extern struct target_type riscv011_target;
extern struct target_type riscv013_target;

/*
 * Definitions shared by code supporting all RISC-V versions.
 */
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;
typedef uint64_t riscv_addr_t;

enum riscv_halt_reason {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_BREAKPOINT,
	RISCV_HALT_SINGLESTEP,
	RISCV_HALT_UNKNOWN
};

typedef struct {
	unsigned dtm_version;

	riscv_reg_t misa;

	struct command_context *cmd_ctx;
	void *version_specific;

	/* The number of harts on this system. */
	int hart_count;

	/* The hart that the RTOS thinks is currently being debugged. */
	int rtos_hartid;

	/* The hart that is currently being debugged.  Note that this is
	 * different than the hartid that the RTOS is expected to use.  This
	 * one will change all the time, it's more of a global argument to
	 * every function than an actual */
	int current_hartid;

	/* Enough space to store all the registers we might need to save. */
	/* FIXME: This should probably be a bunch of register caches. */
	uint64_t saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];
	bool valid_saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];

	/* The register cache points into here. */
	uint64_t reg_cache_values[RISCV_MAX_REGISTERS];

	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;

	/* It's possible that each core has a different supported ISA set. */
	int xlen[RISCV_MAX_HARTS];

	/* The number of triggers per hart. */
	unsigned trigger_count[RISCV_MAX_HARTS];

	/* For each physical trigger, contains -1 if the hwbp is available, or the
	 * unique_id of the breakpoint/watchpoint that is using it.
	 * Note that in RTOS mode the triggers are the same across all harts the
	 * target controls, while otherwise only a single hart is controlled. */
	int trigger_unique_id[RISCV_MAX_HWBPS];

	/* The number of entries in the debug buffer. */
	int debug_buffer_size[RISCV_MAX_HARTS];

	/* This avoids invalidating the register cache too often. */
	bool registers_initialized;

	/* This hart contains an implicit ebreak at the end of the program buffer. */
	bool impebreak;

	/* Helper functions that target the various RISC-V debug spec
	 * implementations. */
	int (*get_register)(struct target *target,
		riscv_reg_t *value, int hid, int rid);
	int (*set_register)(struct target *, int hartid, int regid,
			uint64_t value);
	void (*select_current_hart)(struct target *);
	bool (*is_halted)(struct target *target);
	int (*halt_current_hart)(struct target *);
	int (*resume_current_hart)(struct target *target);
	int (*step_current_hart)(struct target *target);
	int (*on_halt)(struct target *target);
	int (*on_resume)(struct target *target);
	int (*on_step)(struct target *target);
	enum riscv_halt_reason (*halt_reason)(struct target *target);
	int (*write_debug_buffer)(struct target *target, unsigned index,
			riscv_insn_t d);
	riscv_insn_t (*read_debug_buffer)(struct target *target, unsigned index);
	int (*execute_debug_buffer)(struct target *target);
	int (*dmi_write_u64_bits)(struct target *target);
	void (*fill_dmi_write_u64)(struct target *target, char *buf, int a, uint64_t d);
	void (*fill_dmi_read_u64)(struct target *target, char *buf, int a);
	void (*fill_dmi_nop_u64)(struct target *target, char *buf);
} riscv_info_t;

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
extern int riscv_command_timeout_sec;

/* Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
extern int riscv_reset_timeout_sec;

extern bool riscv_use_scratch_ram;
extern uint64_t riscv_scratch_ram_address;

/* Everything needs the RISC-V specific info structure, so here's a nice macro
 * that provides that. */
static inline riscv_info_t *riscv_info(const struct target *target) __attribute__((unused));
static inline riscv_info_t *riscv_info(const struct target *target)
{ return target->arch_info; }
#define RISCV_INFO(R) riscv_info_t *R = riscv_info(target);

extern uint8_t ir_dtmcontrol[1];
extern struct scan_field select_dtmcontrol;
extern uint8_t ir_dbus[1];
extern struct scan_field select_dbus;
extern uint8_t ir_idcode[1];
extern struct scan_field select_idcode;

/*** OpenOCD Interface */
int riscv_openocd_poll(struct target *target);

int riscv_openocd_halt(struct target *target);

int riscv_openocd_resume(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution
);

int riscv_openocd_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints
);

int riscv_openocd_assert_reset(struct target *target);
int riscv_openocd_deassert_reset(struct target *target);

/*** RISC-V Interface ***/

/* Initializes the shared RISC-V structure. */
void riscv_info_init(struct target *target, riscv_info_t *r);

/* Run control, possibly for multiple harts.  The _all_harts versions resume
 * all the enabled harts, which when running in RTOS mode is all the harts on
 * the system. */
int riscv_halt_all_harts(struct target *target);
int riscv_halt_one_hart(struct target *target, int hartid);
int riscv_resume_all_harts(struct target *target);
int riscv_resume_one_hart(struct target *target, int hartid);

/* Steps the hart that's currently selected in the RTOS, or if there is no RTOS
 * then the only hart. */
int riscv_step_rtos_hart(struct target *target);

bool riscv_supports_extension(struct target *target, char letter);

/* Returns XLEN for the given (or current) hart. */
int riscv_xlen(const struct target *target);
int riscv_xlen_of_hart(const struct target *target, int hartid);

bool riscv_rtos_enabled(const struct target *target);

/* Sets the current hart, which is the hart that will actually be used when
 * issuing debug commands. */
void riscv_set_current_hartid(struct target *target, int hartid);
int riscv_current_hartid(const struct target *target);

/*** Support functions for the RISC-V 'RTOS', which provides multihart support
 * without requiring multiple targets.  */

/* When using the RTOS to debug, this selects the hart that is currently being
 * debugged.  This doesn't propogate to the hardware. */
void riscv_set_all_rtos_harts(struct target *target);
void riscv_set_rtos_hartid(struct target *target, int hartid);

/* Lists the number of harts in the system, which are assumed to be
 * concecutive and start with mhartid=0. */
int riscv_count_harts(struct target *target);

/* Returns TRUE if the target has the given register on the given hart.  */
bool riscv_has_register(struct target *target, int hartid, int regid);

/* Returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
int riscv_set_register(struct target *target, enum gdb_regno i, riscv_reg_t v);
int riscv_set_register_on_hart(struct target *target, int hid, enum gdb_regno rid, uint64_t v);
int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r);
int riscv_get_register_on_hart(struct target *target, riscv_reg_t *value,
		int hartid, enum gdb_regno regid);

/* Checks the state of the current hart -- "is_halted" checks the actual
 * on-device register. */
bool riscv_is_halted(struct target *target);
enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid);

/* Returns the number of triggers availiable to either the current hart or to
 * the given hart. */
int riscv_count_triggers(struct target *target);
int riscv_count_triggers_of_hart(struct target *target, int hartid);

/* These helper functions let the generic program interface get target-specific
 * information. */
size_t riscv_debug_buffer_size(struct target *target);

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index);
int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn);
int riscv_execute_debug_buffer(struct target *target);

void riscv_fill_dmi_nop_u64(struct target *target, char *buf);
void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a);
int riscv_dmi_write_u64_bits(struct target *target);

/* Invalidates the register cache. */
void riscv_invalidate_register_cache(struct target *target);

/* Returns TRUE when a hart is enabled in this target. */
bool riscv_hart_enabled(struct target *target, int hartid);

int riscv_enumerate_triggers(struct target *target);

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint);

int riscv_init_registers(struct target *target);

#if _NDS_V5_ONLY_
#include "target/nds32_log.h"

enum{
    CONFIRM_SET_FTDI_DIVID,
    CONFIRM_DTM_CONNECTIVITY,
    CONFIRM_DM_DOMAIN,
    //CONFIRM_DM_TRST_WORKING,
    //CONFIRM_DM_SRST_NOT_AFFECT_JTAG,
    CONFIRM_DM_RESET_HOLD,
    CONFIRM_DM_PROGRAM_BUFFER,
    CONFIRM_DM_MEMORY_ON_CPU,
    CONFIRM_DM_END
};

#define _NDS_DMI_CHECK_TIMEOUT_            1
#define _NDS_SUPPORT_WITHOUT_ANNOUNCING_   1
#define _NDS_DISABLE_ABORT_                1
#define _NDS_IDE_MESSAGE_                  1
#define _NDS_JTAG_SCANS_OPTIMIZE_          1

#define _NDS_BUILD_NO_WARNING_             1
#define _NDS_SMALL_PROGBUF_8WORDS_         1
#define _NDS_BATCH_RUN_RETRY_              1
#define _NDS_RW_MEMORY_64BITS_             1
#define _NDS_MEM_Q_ACCESS_                 1

#if _NDS_JTAG_SCANS_OPTIMIZE_
#define _NDS_JTAG_SCANS_OPTIMIZE_EXE_PBUF  1
#define _NDS_JTAG_SCANS_OPTIMIZE_R_PBUF    1
#endif	//_NDS_JTAG_SCANS_OPTIMIZE_

#define _NDS_USE_SCRIPT_                 1

int ndsv5_openocd_poll_one_hart(struct target *target, int hartid);
int ndsv5_openocd_halt_one_hart(struct target *target, int hartid);
int ndsv5_openocd_resume_one_hart(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		int hartid
);

extern void bus_mode_on(struct target *target, uint64_t *reg_value_backup);
extern void bus_mode_off(struct target *target, uint64_t reg_value);
extern struct reg *ndsv5_get_reg_by_CSR(struct target *target, uint32_t csr_id);
extern uint64_t ndsv5_get_register_value(struct reg *reg);
extern void ndsv5_set_register_value(struct reg *reg, uint64_t reg_value);

extern uint32_t nds_jtag_scans_optimize;
extern const char * const dmi_reg_string[];
extern void decode_dmi(char *text, unsigned address, unsigned data);
extern void ndsv5_decode_progbuf(char *text, uint32_t cur_instr);
extern void riscv_select_current_hart(struct target *target);
extern int ndsv5_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical);
extern int ndsv5_get_physical_address(struct target *target, target_addr_t address, target_addr_t *physical);
extern int ndsv5_haltonreset(struct target *target, int enable);
#endif	//_NDS_V5_ONLY_

#endif
