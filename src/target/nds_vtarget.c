#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target/algorithm.h"
#include "target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "register.h"
#include "breakpoints.h"
#include "helper/time_support.h"
#include "nds_vtarget_common.h"
#include "nds_vtarget.h"
#include "jtag/interfaces.h"
#include "nds32_log.h"
#include "rtos/riscv_debug.h"


#define PGSHIFT             12

/** global variable **/
extern char *gpBitFieldFileName;
#define FILE_V5_BIT_FIELD      "ndsv5_tdesc_bitfield.xml"
char *gpBitFieldFileNameVtarget = (char *)FILE_V5_BIT_FIELD;

uint64_t vtarget_reg_misa_value;
uint32_t vtarget_ena_hit_exception;

uint8_t vtarget_ir_dtmcontrol[1] = {DTMCONTROL};
struct scan_field vtarget_select_dtmcontrol = {
	.in_value = NULL,
	.out_value = vtarget_ir_dtmcontrol
};
uint8_t vtarget_ir_dbus[1] = {DBUS};
struct scan_field vtarget_select_dbus = {
	.in_value = NULL,
	.out_value = vtarget_ir_dbus
};
uint8_t vtarget_ir_idcode[1] = {0x1};
struct scan_field vtarget_select_idcode = {
	.in_value = NULL,
	.out_value = vtarget_ir_idcode
};

/** memory **/
uint32_t vtarget_system_bus_access;
static uint32_t vtarget_force_word_access;
uint32_t vtarget_force_aligned_access;

/** static variable **/
/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
static int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;
static uint32_t vtarget_sys_bus_supported;
static struct reg_arch_type *vtarget_p_riscv_reg_arch_type;

struct {
	uint16_t low, high;
} *vtarget_expose_csr;

/** extern variable **/
extern struct jtag_interface *jtag_interface;

/**
 * Prototype
 */
/*** target functions ***/
static int vtarget_init_target(struct command_context *cmd_ctx,
		struct target *target);
static void vtarget_deinit_target(struct target *target);
static int vtarget_examine(struct target *target);
static int vtarget_poll(struct target *target);

int vtarget_reexamine(struct target *target);
int vtarget_init_reg(struct target *target);
static int vtarget_arch_state(struct target *target);

/*** riscv functions ***/
static int riscv_examine(struct target *target);
static int riscv_init_registers(struct target *target);
static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);
static int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r);

/*** helper functions ***/
static uint32_t dtmcontrol_scan(struct target *target, uint32_t out);
static void decode_dmi(char *text, unsigned address, unsigned data);
static void dump_field(const struct scan_field *field);
static void select_dmi(struct target *target);
static void increase_dmi_busy_delay(struct target *target);
static dmi_status_t dmi_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dmi_op_t op, uint16_t address_out, uint64_t data_out,
		bool exec);
static uint64_t dmi_read(struct target *target, uint16_t address);
static int dmi_write(struct target *target, uint16_t address, uint64_t value);
static int register_get(struct reg *reg);
static int register_set(struct reg *reg, uint8_t *buf);
static int cmp_csr_info(const void *p1, const void *p2);

/** memory functions **/
static int vtarget_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
static int vtarget_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer);
static uint32_t sb_sbaccess(unsigned size_bytes);
static target_addr_t sb_read_address(struct target *target);
static int sb_write_address(struct target *target, target_addr_t address);
static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs);
static void log_memory_access(target_addr_t address, uint64_t value,
		unsigned size_bytes, bool read);
static void write_to_buf(uint8_t *buffer, uint64_t value, unsigned size);

/** memory nothing functions **/
static int write_memory_bus_v1_opt(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{ return ERROR_OK; }
static int read_memory_bus_v1_opt(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{ return ERROR_OK; }

static struct reg_arch_type vtarget_riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

/* In addition to the ones in the standard spec, we'll also expose additional
 * CSRs in this list.
 * The list is either NULL, or a series of ranges (inclusive), terminated with
 * 1,0. */
struct {
	uint16_t low, high;
} *expose_csr;

/** target functions **/
/**
 *
 *
 */
static int vtarget_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("vtarget_init_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info)
		return ERROR_FAIL;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->cmd_ctx = cmd_ctx;

	vtarget_select_dtmcontrol.num_bits = target->tap->ir_length;
	vtarget_select_dbus.num_bits = target->tap->ir_length;
	vtarget_select_idcode.num_bits = target->tap->ir_length;

	return ERROR_OK;
}

static void vtarget_deinit_target(struct target *target)
{
	LOG_DEBUG("vtarget_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);
	if (target_was_examined(target)) {
		if (nds32->attached) {
			LOG_DEBUG("deinit_target(): gdb_detach process, resume dcsr");
			nds32->gdb_run_mode = RUN_MODE_DEBUG;
			nds32->attached = false;        /* Set attached to false before resume */
		}
	}

	free(info);
	target->arch_info = NULL;
}

static int vtarget_examine(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	uint32_t retry_cnt = 0;
	uint32_t dtmcontrol = 0;
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
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

	if (target_was_examined(target))
		return ERROR_OK;

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("  version=0x%x", info->dtm_version);

	info->progbufsize = -1;
	info->dmi_busy_delay = vtarget_dmi_busy_delay_count;
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

	if (riscv_examine(target) != ERROR_OK)
		return ERROR_FAIL;

	vtarget_handle_examine(target);
	return ERROR_OK;
}

static int vtarget_poll(struct target *target)
{
	return ERROR_OK;
}

int vtarget_reexamine(struct target *target)
{
	LOG_DEBUG("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);

	RISCV_INFO(r);
	r->current_hartid = target->coreid;

	return riscv_examine(target);
}

static char BitFileName_64[512];
static uint32_t nds32_redirect_64bitfilename;
int vtarget_init_reg(struct target *target)
{
	NDS_INFO("%s, [%s] coreid=%d", __func__, target->tap->dotted_name, target->coreid);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	switch (info->dtm_version) {
		case 0:
		case 1:
			vtarget_p_riscv_reg_arch_type = (struct reg_arch_type *)&vtarget_riscv_reg_arch_type;
			break;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return ERROR_FAIL;
	}

	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);
	gpBitFieldFileName = gpBitFieldFileNameVtarget;
	if (vtarget_xlen == 64) {
		nds32->gmon_64_bit = 1;
		if (nds32_redirect_64bitfilename == 0) {
			char *pBitFileName_64 = (char *)&BitFileName_64[0];
			char *pfilename = pBitFileName_64;
			strcpy(pfilename, gpBitFieldFileName);
			char *curr_str = strstr(pfilename, ".xml");
			strcpy(curr_str, "_64");
			curr_str += 3;
			strcpy(curr_str, ".xml");
			curr_str += 4;
			*curr_str = 0;
			gpBitFieldFileName = (char *)pBitFileName_64;
			gpBitFieldFileNameVtarget = gpBitFieldFileName;
			nds32_redirect_64bitfilename = 1;
		}
		NDS_INFO("gpBitFieldFileName: %s", gpBitFieldFileName);
	} else
		nds32->gmon_64_bit = 0;
	NDS_INFO("nds32->gmon_64_bit: 0x%x", nds32->gmon_64_bit);

	return ERROR_OK;
}

static int vtarget_arch_state(struct target *target)
{
	return ERROR_OK;
}

/*** riscv functions ***/
static int riscv_examine(struct target *target)
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

	RISCV_INFO(info);
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	if (get_field(dmstatus, DMI_DMSTATUS_VERSION) != 2) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2, not %d "
				"(dmstatus=0x%x)", get_field(dmstatus, DMI_DMSTATUS_VERSION), dmstatus);
		return ERROR_FAIL;
	}

	/** reset info->dmi_busy_delay=info->dtmcontrol_idle
	 *  if info->dmi_busy_delay was incease by the 1st dmi_scan */
	info->dmi_busy_delay = vtarget_dmi_busy_delay_count;
	LOG_DEBUG("info->dmi_busy_delay: 0x%08x", info->dmi_busy_delay);

	dmstatus = dmi_read(target, DMI_DMSTATUS);

	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);


	LOG_DEBUG("dmcontrol: 0x%08x", dmcontrol);
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);


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

	/** The upstream was removed to check UNAVAIL and NONEXISTENT **/

	info->sbcs = dmi_read(target, DMI_SBCS);
	if (info->sbcs & 0x1F) {
		/** DMI_SBCS_SBACCESS8|DMI_SBCS_SBACCESS16|DMI_SBCS_SBACCESS32|
		    DMI_SBCS_SBACCESS64|DMI_SBCS_SBACCESS128 **/
		int sb_version = get_field(info->sbcs, DMI_SBCS_SBVERSION);
		if ((sb_version == 0) || (sb_version == 1)) {
			/* according to eticket 16199, default no support system bus access,
			 * so vtarget_system_bus_access default value is 0 */
			if (vtarget_system_bus_access == 1)
				vtarget_sys_bus_supported = 1;
		}
	}
	LOG_DEBUG("info->sbcs = 0x%x, vtarget_sys_bus_supported = 0x%x", (int)info->sbcs, vtarget_sys_bus_supported);

	/** Check that abstract data registers are accessible. **/
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progbufsize = 0;

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */
	info->current_hartid = 0;
	info->xlen = vtarget_xlen;

	riscv_get_register(target, &info->misa, GDB_REGNO_MISA);

	/* Now init registers based on what we discovered. */
	if (riscv_init_registers(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Display this as early as possible to help people who are using
	 * really slow simulators. */
	LOG_DEBUG(" [%s] hart 0: XLEN=%d, misa=0x%" PRIx64, target->tap->dotted_name, info->xlen,
			info->misa);

	target->state = TARGET_RUNNING;
	target_set_examined(target);

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_INFO("Examined RISC-V core; found 1 harts");

	return ERROR_OK;
}

static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	RISCV_INFO(r);
	LOG_DEBUG("reg_class=%d", reg_class);
	LOG_DEBUG("rtos_hartid=%d current_hartid=%d", r->rtos_hartid, r->current_hartid);

	if (!target_was_examined(target)) {
		NDS_INFO("target was NOT examined");
		vtarget_examine(target);
	}

	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 32;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = GDB_REGNO_COUNT;
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
	return ERROR_OK;
}

struct csr_info {
	unsigned number;
	const char *name;
};

static int cmp_csr_info(const void *p1, const void *p2)
{
	return (int) (((struct csr_info *)p1)->number) - (int) (((struct csr_info *)p2)->number);
}

static int riscv_init_registers(struct target *target)
{
	RISCV_INFO(info);

	if (target->reg_cache) {
		if (target->reg_cache->reg_list)
			free(target->reg_cache->reg_list);
		free(target->reg_cache);
	}

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	target->reg_cache->name = "RISC-V Registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;
	target->reg_cache->reg_list = calloc(GDB_REGNO_COUNT, sizeof(struct reg));

	const unsigned int max_reg_name_len = 12;
	if (info->reg_names)
		free(info->reg_names);

	info->reg_names = calloc(1, GDB_REGNO_COUNT * max_reg_name_len);

	char *reg_name = info->reg_names;

	/* for gdb8.x , feature name :cpu, fpu, csr,virtual */
	static struct reg_feature feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};
	static struct reg_feature feature_fpu = {
		.name = "org.gnu.gdb.riscv.fpu"
	};
	static struct reg_feature feature_csr = {
		.name = "org.gnu.gdb.riscv.csr"
	};
	static struct reg_feature feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
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
	static struct reg_data_type type_ieee_double = {
		.type = REG_TYPE_IEEE_DOUBLE,
		.id = "ieee_double"
	};
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

	struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "riscv/encoding.h"
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
		r->type = &vtarget_riscv_reg_arch_type;
		r->arch_info = target;
		r->number = number;
		r->size = vtarget_xlen;

		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
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
			r->name = vtarget_gpr_and_fpu_name[number];

			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number == GDB_REGNO_PC) {
			sprintf(reg_name, "pc");
			r->group = "general";
			r->feature = &feature_cpu;
			r->reg_data_type = &type_code_ptr;
		} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			r->reg_data_type = &type_ieee_double;
			r->name = vtarget_gpr_and_fpu_name[number];
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
			/* tmp-for-gdb-tdesc */
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
					r->exist = true;
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
					r->exist = true;
					break;
			}

			/* skip for 64 bit bitfield xml */
			switch (csr_number) {
				case CSR_SCOUNTEREN:
				case CSR_MCOUNTEREN:
				case CSR_MCOUNTINHIBIT:
				case CSR_MCOUNTERINTEN:
				case CSR_MCOUNTERWEN:
				case CSR_MCOUNTERMASK_M:
				case CSR_MCOUNTERMASK_S:
				case CSR_MCOUNTERMASK_U:
				case CSR_SCOUNTERINTEN:
				case CSR_SCOUNTERMASK_M:
				case CSR_SCOUNTERMASK_S:
				case CSR_SCOUNTERMASK_U:
				case CSR_MCOUNTEROVF:
				case CSR_SCOUNTEROVF:
				case CSR_SCOUNTINHIBIT:
				case CSR_DCSR:
					r->size = 32;
					break;
			}

			if (!r->exist && vtarget_expose_csr) {
				for (unsigned i = 0; vtarget_expose_csr[i].low <= vtarget_expose_csr[i].high; i++) {
					if (csr_number >= vtarget_expose_csr[i].low && csr_number <= vtarget_expose_csr[i].high) {
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
		} else if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31) {
			sprintf(reg_name, "v%d", number - GDB_REGNO_V0);
			r->exist = false;
			r->group = "general";
			r->feature = &feature_vector;

			struct nds_vtarget *nds32 = target_to_nds_vtarget(target);
			/* LOG_DEBUG("nds32->nds_vector_length: %d", nds32->nds_vector_length); */
			if (nds32->nds_vector_length == 128) {
				r->reg_data_type = &type_vec128;
				r->size = 128;
			} else if (nds32->nds_vector_length == 256) {
				r->reg_data_type = &type_vec256;
				r->size = 256;
			} else {
			/* if (nds32->nds_vector_length == 512) { */
				r->reg_data_type = &type_vec512;
				r->size = 512;
			}
		}
		if (reg_name[0])
			r->name = reg_name;
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + GDB_REGNO_COUNT * max_reg_name_len);
		r->value = &info->reg_cache_values[number];
	}

	/* redirect all CSRs (r->name/r->exist) to NDS define */
	extern int vtarget_redefine_CSR_name(struct target *target);
	extern int vtarget_redefine_GPR_FPU_name(struct target *target);
	vtarget_redefine_CSR_name(target);
	vtarget_redefine_GPR_FPU_name(target);

	struct nds_vtarget *nds32 = target_to_nds_vtarget(target);
	nds32->execute_register_init = true;

	return ERROR_OK;
}

static int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r)
{
	switch (r) {
		case GDB_REGNO_MISA:
			*value = (0x01 << 23) | (0x01 << 12) | (0x01 << 2) | (0x01 << 8);
			break;
		case GDB_REGNO_MSTATUS:
			*value = 0x0;
			break;
		case GDB_REGNO_PC:
			*value = 0xca;
			break;
		case GDB_REGNO_SP:
			*value = 0x8000000;
			break;
		default:
			*value = 0x0;
			LOG_DEBUG("Other registers [0] reg[0x%x] = 0x%" PRIx64, r, *value);
			break;
	}

	return ERROR_OK;
}

/*** helper functions ***/
static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4];

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &vtarget_select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dmi. */
	jtag_add_ir_scan(target->tap, &vtarget_select_dbus, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
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

static const char * const dmi_reg_string[] = {
	"",                  /* 0x00 */
	"",                  /* 0x01 */
	"",                  /* 0x02 */
	"",                  /* 0x03 */
	"DMI_DATA0",         /* 0x04 */
	"DMI_DATA1",         /* 0x05 */
	"DMI_DATA2",         /* 0x06 */
	"DMI_DATA3",         /* 0x07 */
	"DMI_DATA4",         /* 0x08 */
	"DMI_DATA5",         /* 0x09 */
	"DMI_DATA6",         /* 0x0a */
	"DMI_DATA7",         /* 0x0b */
	"DMI_DATA8",         /* 0x0c */
	"DMI_DATA9",         /* 0x0d */
	"DMI_DATA10",        /* 0x0e */
	"DMI_DATA11",        /* 0x0f */
	"DMI_DMCONTROL",     /* 0x10 */
	"DMI_DMSTATUS",      /* 0x11 */
	"DMI_HARTINFO",      /* 0x12 */
	"DMI_HALTSUM",       /* 0x13 */
	"DMI_HAWINDOWSEL",   /* 0x14 */
	"DMI_HAWINDOW",      /* 0x15 */
	"DMI_ABSTRACTCS",    /* 0x16 */
	"DMI_COMMAND",       /* 0x17 */
	"DMI_ABSTRACTAUTO",  /* 0x18 */
	"DMI_DEVTREEADDR0",  /* 0x19 */
	"DMI_DEVTREEADDR1",  /* 0x1a */
	"DMI_DEVTREEADDR2",  /* 0x1b */
	"DMI_DEVTREEADDR3",  /* 0x1c */
	"",                  /* 0x1d */
	"",                  /* 0x1e */
	"",                  /* 0x1f */
	"DMI_PROGBUF0",      /* 0x20 */
	"DMI_PROGBUF1",      /* 0x21 */
	"DMI_PROGBUF2",      /* 0x22 */
	"DMI_PROGBUF3",      /* 0x23 */
	"DMI_PROGBUF4",      /* 0x24 */
	"DMI_PROGBUF5",      /* 0x25 */
	"DMI_PROGBUF6",      /* 0x26 */
	"DMI_PROGBUF7",      /* 0x27 */
	"DMI_PROGBUF8",      /* 0x28 */
	"DMI_PROGBUF9",      /* 0x29 */
	"DMI_PROGBUF10",     /* 0x2a */
	"DMI_PROGBUF11",     /* 0x2b */
	"DMI_PROGBUF12",     /* 0x2c */
	"DMI_PROGBUF13",     /* 0x2d */
	"DMI_PROGBUF14",     /* 0x2e */
	"DMI_PROGBUF15",     /* 0x2f */
	"DMI_AUTHDATA",      /* 0x30 */
	"",                  /* 0x31 */
	"",                  /* 0x32 */
	"",                  /* 0x33 */
	"",                  /* 0x34 */
	"",                  /* 0x35 */
	"",                  /* 0x36 */
	"",                  /* 0x37 */
	"DMI_SBCS",          /* 0x38 */
	"DMI_SBADDRESS0",    /* 0x39 */
	"DMI_SBADDRESS1",    /* 0x3a */
	"DMI_SBADDRESS2",    /* 0x3b */
	"DMI_SBDATA0",       /* 0x3c */
	"DMI_SBDATA1",       /* 0x3d */
	"DMI_SBDATA2",       /* 0x3e */
	"DMI_SBDATA3",       /* 0x3f */
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
	if (in_text[0] != 0x0 || append_text[0] != 0x0)
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "\t%s: %s %s",
				dmi_reg_string[op_address], in_text, append_text);
}

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

static void increase_dmi_busy_delay(struct target *target)
{
	RISCV_INFO(info);
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
	RISCV_INFO(info);
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
	else
		jtag_add_runtest(info->dtmcontrol_idle, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		return DMI_STATUS_FAILED;
	}

	if (data_in)
		*data_in = buf_get_u64(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);

	uint32_t in_value = buf_get_u32(in, 0, 32);
	if (in_value == 0xFFFFFFFF) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
	}
	dump_field(&field);

#if _NDS_DMI_CHECK_TIMEOUT_
	dmi_status_t dmi_stat = buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
	if (dmi_stat == DMI_STATUS_BUSY)
		dmi_stat = dmi_check_timeout(target);
	return dmi_stat;
#endif
	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

static uint64_t dmi_read(struct target *target, uint16_t address)
{
	select_dmi(target);

	dmi_status_t status;
	uint16_t address_in;

	unsigned i = 0;

	/* This first loop ensures that the read request was actually sent
	 * to the target. Note that if for some reason this stays busy,
	 * it is actually due to the previous dmi_read or dmi_write. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday(&begin_time, NULL);
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
	gettimeofday(&begin_time, NULL);
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

	dmi_status_t status = DMI_STATUS_BUSY;
	unsigned i = 0;

	/* The first loop ensures that we successfully sent the write request. */
#if _NDS_DMI_CHECK_TIMEOUT_
	gettimeofday(&begin_time, NULL);
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
	gettimeofday(&begin_time, NULL);
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

struct reg *vtarget_get_reg_by_CSR(struct target *target, uint32_t csr_id)
{
	char *reg_name = vtarget_get_CSR_name(target, csr_id);
	if (reg_name == NULL) {
		LOG_DEBUG("get reg_name ERROR");
		return NULL;
	}

	struct reg *reg = register_get_by_name(target->reg_cache, reg_name, 1);
	if (reg == NULL) {
		LOG_DEBUG("get reg ERROR");
		return NULL;
	}
	return reg;
}

/** memory functions **/
int vtarget_get_buffer_access_size(uint64_t start_addr, uint32_t bufsize,
		uint32_t *pcurr_size, uint32_t *paccess_size)
{
	uint32_t access_size = 4;  /* default word access */
	uint32_t curr_size = bufsize;

	if (curr_mem_access_attr_index == 0) {
		*pcurr_size = curr_size;
		*paccess_size = access_size;
		return ERROR_OK;
	}
	struct nds32_mem_access_attr *pmem_attr = (struct nds32_mem_access_attr *)&all_mem_access_attr[0];
	/* search from tail to head */
	pmem_attr += curr_mem_access_attr_index;

	for (uint32_t i = 0; i < curr_mem_access_attr_index; i++) {
		if (pmem_attr->access_size == 0)
			break;
		if ((start_addr >= pmem_attr->lowAddr) &&
		    (start_addr <= pmem_attr->highAddr)) {
			if ((start_addr + (bufsize - 1)) >= pmem_attr->highAddr)
				curr_size = (pmem_attr->highAddr - start_addr) + 1;
			access_size = pmem_attr->access_size;
			break;
		}
		pmem_attr--;
	}

	*pcurr_size = curr_size;
	*paccess_size = access_size;
	return ERROR_OK;
}

static int vtarget_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("%s", __func__);
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);

	uint32_t i;
	if ((vtarget_force_aligned_access == 1) && ((address % size) != 0)) {
		/* if 013, unaligned access */
		uint64_t align_addr = 0;
		uint32_t data_val1 = 0, data_val2 = 0;

		for (i = 0; i < count; i++) {
			align_addr = (address & ~0x03);
			read_memory(target, align_addr, 4, 1, (uint8_t *)&data_val1);
			read_memory(target, align_addr+4, 4, 1, (uint8_t *)&data_val2);
			LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", data_val1=0x%x, data_val2=0x%x", align_addr, data_val1, data_val2);
			*buffer++ = (data_val1 >> 16) & 0xff;
			*buffer++ = (data_val1 >> 24) & 0xff;
			*buffer++ = (data_val2 & 0xff);
			*buffer++ = (data_val2 >> 8) & 0xff;
			address += 4;
		}
		return ERROR_OK;
	}

	if (vtarget_force_word_access == 1) {
		for (i = 0; i < count; i++) {
			read_memory(target, address, size, 1, buffer);
			address += size;
			buffer += size;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, readsize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	vtarget_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
	if (((total_size % access_size) != 0) ||
		((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return read_memory(target, address, access_size, access_cnt, buffer);
	}

	while (total_size) {
		vtarget_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
		access_cnt = readsize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = read_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

static int vtarget_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("%s", __func__);
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);

	uint32_t i;
	if ((vtarget_force_aligned_access == 1) && ((address % size) != 0)) {
		/* if 013, unaligned access */
		for (i = 0; i < count; i++) {
			write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
			write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, writesize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	vtarget_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
	if (((total_size % access_size) != 0) ||
	    ((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return write_memory(target, address, access_size, access_cnt, buffer);
	}

	while (total_size) {
		vtarget_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
		access_cnt = writesize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = write_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV_INFO(info);
	int result = ERROR_FAIL;

	/* according to eticket 16199, default no support system bus access,
	 * so vtarget_system_bus_access default value is 0 */
	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {

		if (vtarget_jtag_scans_optimize > 0) {
			result = read_memory_bus_v1_opt(target, address, size, count, buffer);
			goto read_memory_finish;
		} else {
			result = read_memory_bus_v1(target, address, size, count, buffer);
			goto read_memory_finish;
		}
	}

	LOG_ERROR("Don't know how to read memory on this target.");
	return ERROR_FAIL;

read_memory_finish:
	if (result == ERROR_OK) {
		uint32_t *p_word_data = (uint32_t *)buffer;
		NDS_INFO("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR " = 0x%08x", count,
			size, address, *p_word_data);
	}

	return result;
}

static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV_INFO(info);

	uint32_t *p_word_data = (uint32_t *)buffer;
	NDS_INFO("writing %d words of %d bytes to 0x%08lx = 0x%08x", count, size, (long)address, *p_word_data);

	/* according to eticket 16199, default no support system bus access,
	 * so vtarget_system_bus_access default value is 0 */
	if ((get_field(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)) {

		if (vtarget_jtag_scans_optimize > 0)
			return write_memory_bus_v1_opt(target, address, size, count, buffer);

		return write_memory_bus_v1(target, address, size, count, buffer);
	}

	LOG_ERROR("Don't know how to write memory on this target.");
	return ERROR_FAIL;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	RISCV_INFO(info);
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

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("start, count=0x%x, size=0x%x", count, size);
	dmi_read(target, DMI_SBCS);
	RISCV_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs = set_field(0, DMI_SBCS_SBREADONADDR, 1);
		sbcs |= sb_sbaccess(size);
		sbcs = set_field(sbcs, DMI_SBCS_SBAUTOINCREMENT, 1);
		sbcs = set_field(sbcs, DMI_SBCS_SBREADONDATA, count > 1);
		dmi_write(target, DMI_SBCS, sbcs);
		if (read_sbcs_nonbusy(target, &sbcs) != ERROR_OK)
			return ERROR_FAIL;

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

		/* disable autoread */
		dmi_write(target, DMI_SBCS, 0);

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
		/* if (dmi_read(target, &value, DMI_SBDATA3) != ERROR_OK)
			return ERROR_FAIL; */
		value = dmi_read(target, DMI_SBDATA3);
		write_to_buf(buffer + 12, value, 4);
		log_memory_access(address + 12, value, 4, true);
	}
	if (size > 8) {
		/* if (dmi_read(target, &value, DMI_SBDATA2) != ERROR_OK)
			return ERROR_FAIL; */
		value = dmi_read(target, DMI_SBDATA2);
		write_to_buf(buffer + 8, value, 4);
		log_memory_access(address + 8, value, 4, true);
	}
	if (size > 4) {
		/*if (dmi_read(target, &value, DMI_SBDATA1) != ERROR_OK)
			return ERROR_FAIL; */
		value = dmi_read(target, DMI_SBDATA1);
		write_to_buf(buffer + 4, value, 4);
		log_memory_access(address + 4, value, 4, true);
	}
	/* if (dmi_read(target, &value, DMI_SBDATA0) != ERROR_OK)
		return ERROR_FAIL; */
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
	RISCV_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DMI_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
#if BUILD_TARGET64
		/* dmi_read(target, &v, DMI_SBADDRESS1); */
		v = dmi_read(target, DMI_SBADDRESS1);
		address |= v;
		address <<= 32;
#endif
	}
	/* dmi_read(target, &v, DMI_SBADDRESS0); */
	v = dmi_read(target, DMI_SBADDRESS0);
	address |= v;
	return address;
}

static int sb_write_address(struct target *target, target_addr_t address)
{
	RISCV_INFO(info);
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

struct target_type nds_vtarget_target = {
	.name = "nds_vtarget",

	.init_target = vtarget_init_target,
	.deinit_target = vtarget_deinit_target,
	.examine = vtarget_examine,

	/* poll current target status */
	.poll = vtarget_poll,

	/* memory access */
	.read_memory = vtarget_read_memory,
	.write_memory = vtarget_write_memory,

	/* gdb */
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.arch_state = vtarget_arch_state,

	/* monitor command */
	.commands = vtarget_command_handlers,

	/* target create */
	.target_create = vtarget_target_create,
};
