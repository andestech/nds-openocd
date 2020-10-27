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
#include "riscv/riscv.h"
#include "nds32_v5.h"
#include "jtag/interfaces.h"
#include "nds32_log.h"

extern struct jtag_interface *jtag_interface;

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
 * 		cache_get...
 * 		cache_set...
 * 		cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

// Constants for legacy SiFive hardware breakpoints.
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

#if 0 // define in riscv.c
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
#endif
struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

int ndsv5_poll(struct target *target);

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
/*
struct target_type *get_target_type(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	switch (info->dtm_version) {
		case 0:
			return &riscv011_target;
		case 1:
			return &riscv013_target;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			printf("Unsupported DTM version: %d", info->dtm_version);
			fflush(stdout);
			exit(-1);
			return NULL;
	}
}
*/
extern uint64_t nds_support_syscall_id[];
static int riscv_step_virtual_hosting_checking(struct target *target)
{
	uint32_t cur_instr = 0;
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	if (target_read_memory(target, reg_pc_value, 4, 1, (uint8_t *)&cur_instr) != ERROR_OK) {
		LOG_ERROR("can't read memory: 0x%" TARGET_PRIxADDR, reg_pc_value);
		return ERROR_FAIL;
	}
	LOG_DEBUG("reg_pc_value = 0x%" TARGET_PRIxADDR ", cur_instr=0x%x", reg_pc_value, cur_instr);
	cur_instr &= MASK_C_EBREAK;
	if (cur_instr == MATCH_C_EBREAK) {
		/* ebreak, check a7 value */
		struct reg *reg_a7 = register_get_by_name(target->reg_cache, gpr_and_fpu_name[17], 1); //a7
		reg_a7->type->get(reg_a7);
		uint64_t reg_a7_value = buf_get_u64(reg_a7->value, 0, reg_a7->size);
		uint32_t i;
		for (i = 0; i < NDS_EBREAK_NUMS; i++) {
			if (reg_a7_value == nds_support_syscall_id[i])
				break;
		}
		if (i == NDS_EBREAK_NUMS) {
			/* NOT syscall_id (maybe target break insn. ) */
			return ERROR_FAIL;
		}
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		nds32->active_syscall_id = (uint32_t)reg_a7_value;	///< WARNING: potential issue on target64
		nds32->hit_syscall = true;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

int ndsv5_step_check(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->hit_syscall == true) {
		/* skip ebreak */
		struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
		reg_pc->type->get(reg_pc);
		uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
		int ebreak_length = ndsv5_get_ebreak_length(target, reg_pc_value);
		reg_pc_value += ebreak_length;  // "ebreak" length
		reg_pc->type->set(reg_pc, (uint8_t *)&reg_pc_value);
		nds32->hit_syscall = false;
		LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, reg_pc_value);
	}
	if (nds32->virtual_hosting_ctrl_c == true) {
		LOG_DEBUG("virtual_hosting_ctrl_c = true");
		nds32->virtual_hosting_ctrl_c = false;
		return ERROR_FAIL;
	}

	if (riscv_step_virtual_hosting_checking(target) == ERROR_OK) {
		LOG_DEBUG("it's virtual hosting, skip stepping !!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int ndsv5_reset_target(struct target *target, enum target_reset_mode reset_mode)
{
	//struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	int reset_halt_bak = target->reset_halt;
	
	if( reset_mode == RESET_HALT )
		target->reset_halt = 1;
	else
		target->reset_halt = 0;

	CHECK_RETVAL(target->type->assert_reset(target));
	CHECK_RETVAL(target->type->deassert_reset(target));

	target->reset_halt = reset_halt_bak;
	return ERROR_OK;
}

int ndsv5_srst_reset_target(struct target *target)
{
	LOG_DEBUG("Initializing with hard TRST+SRST reset");
	int retval;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

//	jtag_add_reset(1, 0);   /* TAP_RESET */
//	if (jtag_reset_config & RESET_HAS_SRST) {
//		jtag_add_reset(1, 1);
//		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) == 0)
//			jtag_add_reset(0, 1);
//	}
//	jtag_add_reset(0, 0);
//	retval = jtag_execute_queue();
//	if (retval != ERROR_OK) {
//		LOG_ERROR("Reset execute queue failed!!");
//		return retval;
//	}

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (jtag_reset_config & RESET_HAS_SRST) {
		jtag_add_reset(1, 1);
	}
	alive_sleep(1000);

	// deassert_reset
	jtag_add_reset(0, 0);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("Reset execute queue failed!!");
		return retval;
	}
	alive_sleep(nds32->reset_time);

	return ERROR_OK;
}

int ndsv5_resume_check(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	if (nds32->hit_syscall == true) {
		/* skip ebreak */
		struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
		reg_pc->type->get(reg_pc);
		uint64_t reg_pc_value = buf_get_u64(reg_pc->value, 0, reg_pc->size);
		int ebreak_length = ndsv5_get_ebreak_length(target, reg_pc_value);
		reg_pc_value += ebreak_length;  // "ebreak" length
		reg_pc->type->set(reg_pc, (uint8_t *)&reg_pc_value);
		nds32->hit_syscall = false;
		LOG_DEBUG("next_pc: 0x%" TARGET_PRIxADDR, reg_pc_value);
	}
	if (nds32->virtual_hosting_ctrl_c == true) {
		LOG_DEBUG("virtual_hosting_ctrl_c = true");
		nds32->virtual_hosting_ctrl_c = false;
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

extern int nds32_get_buffer_access_size(uint64_t start_addr, uint32_t bufsize,
		uint32_t *pcurr_size, uint32_t *paccess_size);
extern uint32_t nds_force_word_access;
extern uint32_t nds_force_aligned_access;
int ndsv5_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	riscv_info_t *info = (riscv_info_t *)target->arch_info;

	/* check if 011, or  word_access/aligned_access disable */
	if ((info->dtm_version == 0) ||
		((nds_force_word_access == 0) && (nds_force_aligned_access == 0))) {
		return tt->read_memory(target, address, size, count, buffer);
	}
	uint32_t i;
	if ((nds_force_aligned_access == 1) && ((address % size) != 0)) {
		/* if 013, unaligned access */
		uint64_t align_addr = 0;
		uint32_t data_val1 = 0, data_val2 = 0;

		for (i = 0; i < count; i++) {
			align_addr = (address & ~0x03);
			tt->read_memory(target, align_addr, 4, 1, (uint8_t *)&data_val1);
			tt->read_memory(target, align_addr+4, 4, 1, (uint8_t *)&data_val2);
			LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", data_val1=0x%x, data_val2=0x%x", align_addr, data_val1, data_val2);
			*buffer++ = (data_val1 >> 16) & 0xff;
			*buffer++ = (data_val1 >> 24) & 0xff;
			*buffer++ = (data_val2 & 0xff);
			*buffer++ = (data_val2 >> 8) & 0xff;
			address += 4;
		}
		return ERROR_OK;
	}

	if (nds_force_word_access == 1) {
		for (i = 0; i < count; i++) {
			tt->read_memory(target, address, size, 1, buffer);
			address += size;
			buffer += size;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, readsize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	nds32_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
	if (((total_size % access_size) != 0) ||
		((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return tt->read_memory(target, address, access_size, access_cnt, buffer);
	}

	while(total_size) {
		nds32_get_buffer_access_size(start_addr, total_size, &readsize, &access_size);
		access_cnt = readsize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = tt->read_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

int ndsv5_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	riscv_info_t *info = (riscv_info_t *)target->arch_info;

	/* check if 011, or  word_access/aligned_access disable */
	if ((info->dtm_version == 0) || (nds_force_aligned_access == 0)) {
		return tt->write_memory(target, address, size, count, buffer);
	}
	uint32_t i;
	if ((nds_force_aligned_access == 1) && ((address % size) != 0) ) {
		/* if 013, unaligned access */
		for (i = 0; i < count; i++) {
			tt->write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
			tt->write_memory(target, address, 2, 1, buffer);
			buffer += 2;
			address += 2;
		}
		return ERROR_OK;
	}
	uint32_t access_size = 4, writesize = 0, access_cnt = 0;
	uint64_t start_addr = address;
	uint32_t total_size = (size * count);
	nds32_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
	if (((total_size % access_size) != 0) ||
		((address % access_size) != 0)) {
		access_size = size;
		access_cnt = count;
		LOG_DEBUG("path-I, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);
		return tt->write_memory(target, address, access_size, access_cnt, buffer);
	}

	while(total_size) {
		nds32_get_buffer_access_size(start_addr, total_size, &writesize, &access_size);
		access_cnt = writesize/access_size;
		LOG_DEBUG("path-II, access_size=0x%x, access_cnt=0x%x", access_size, access_cnt);

		int retval = tt->write_memory(target, address, access_size, access_cnt, buffer);
		if (retval != ERROR_OK)
			return retval;
		total_size -= (access_size * access_cnt);
		start_addr += (access_size * access_cnt);
		buffer += (access_size * access_cnt);
	}
	return ERROR_OK;
}

int ndsv5_readwrite_byte(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer, bool if_write)
{
	LOG_DEBUG("addr=0x%" TARGET_PRIxADDR ", size=0x%x, count=0x%x", address, size, count);
	struct target_type *tt = get_target_type(target);
	int retval;

	if (if_write)
		retval = tt->write_memory(target, address, size, count, buffer);
	else
		retval = tt->read_memory(target, address, size, count, (uint8_t *) buffer);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

//struct reg_feature ndsv5_reg_feature[REG_COUNT];
//struct reg_data_type ndsv5_reg_data_type[REG_COUNT];
#if 0
int ndsv5_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	LOG_DEBUG("%s", __func__);
	uint32_t reg_nums = 0;

  extern unsigned* global_acr_reg_count_v5;
  unsigned acr_reg_count_v5 = 0;
  if (global_acr_reg_count_v5 != 0 && reg_class == REG_CLASS_ALL) {
    acr_reg_count_v5 = *global_acr_reg_count_v5;
  }

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			reg_nums = 32;
			break;
		case REG_CLASS_ALL:
			reg_nums = REG_COUNT;
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}
	*reg_list = calloc(REG_COUNT + acr_reg_count_v5, sizeof(struct reg *));
	if (!*reg_list) {
		return ERROR_FAIL;
	}

	for (uint32_t i = 0; i < reg_nums; i++) {
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
		//(*reg_list)[i]->feature = malloc(sizeof(struct reg_feature));
		//(*reg_list)[i]->feature = &ndsv5_reg_feature[i];
		//(*reg_list)[i]->feature->name = "org.gnu.gdb.riscv.cpu";
		//(*reg_list)[i]->feature->name = "";
		//(*reg_list)[i]->group = "general";
		//(*reg_list)[i]->reg_data_type = malloc(sizeof(struct reg_data_type));
		(*reg_list)[i]->reg_data_type = &ndsv5_reg_data_type[i];
		if ((*reg_list)[i]->size == 64) {
			(*reg_list)[i]->reg_data_type->type = REG_TYPE_UINT64;
			(*reg_list)[i]->reg_data_type->id = "uint64";
		} else {
			(*reg_list)[i]->reg_data_type->type = REG_TYPE_UINT32;
			(*reg_list)[i]->reg_data_type->id = "uint32";
		}
	}
	if (reg_nums == REG_COUNT) {
		(*reg_list)[REG_PRIV]->reg_data_type->type = REG_TYPE_UINT8;
		(*reg_list)[REG_PRIV]->reg_data_type->id = "uint8";
	}

  // All parameters are set up in init_target() of riscv-013.c
  for (uint32_t i = reg_nums; i < reg_nums + acr_reg_count_v5; i++) {
    (*reg_list)[i] = &target->reg_cache->reg_list[i];
  }

	*reg_list_size = reg_nums + acr_reg_count_v5;
  LOG_DEBUG("reg_list_size =  %d", *reg_list_size);

	return ERROR_OK;
}
#endif

extern int ndsv5_gdb_fileio_write_memory(struct target *target, target_addr_t address, uint32_t *psize, uint8_t **pbuffer);
int ndsv5_write_buffer(struct target *target, target_addr_t address, uint32_t writesize, const uint8_t *buffer)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t total_size = writesize;
	uint8_t *write_buffers = (uint8_t *)buffer;

	LOG_DEBUG("writing buffer of %" PRIi32 " byte at " TARGET_ADDR_FMT, writesize, address);
	if (writesize == 0)
		return ERROR_OK;

	if (nds32->hit_syscall) {
	    ndsv5_gdb_fileio_write_memory(target, address, &total_size, &write_buffers);
	    LOG_DEBUG("gdb_fileio_write_memory, total_size=%d", total_size);
	}


	/* Copy from target.c */
	uint32_t count = total_size;
	uint32_t size;

	/* Align up to maximum 4 bytes. The loop condition makes sure the next pass
	 *          * will have something to do with the size we leave to it. */
	for (size = 1; size < 4 && count >= size * 2 + (address & size); size *= 2) {
	    if (address & size) {
		int retval = target_write_memory(target, address, size, 1, write_buffers);
		if (retval != ERROR_OK)
		  return retval;
		address += size;
		count -= size;
		write_buffers += size;
	    }
	}

	/* Write the data with as large access size as possible. */
	for (; size > 0; size /= 2) {
	    uint32_t aligned = count - count % size;
	    if (aligned > 0) {
		int retval = target_write_memory(target, address, size, aligned / size, write_buffers);
		if (retval != ERROR_OK)
		  return retval;
		address += aligned;
		count -= aligned;
		write_buffers += aligned;
	    }
	}

	return ERROR_OK;
}

struct cache_element {
	uint64_t pa;
	uint64_t cacheline[32];
	uint8_t dirty;
	uint8_t valid;
	uint8_t lock;
	uint8_t inval;
	uint8_t shared;
	uint8_t exclusive;
	uint8_t modified;
};
/* command value */
#define L1D_WBINVAL_ALL 6
#define L1D_WB_ALL 7
#define L1D_IX_RTAG 19
#define L1D_IX_RDATA 20
#define L1D_INVAL_ALL 23
#define L1I_IX_INVAL 24
#define L1I_IX_RTAG 27
#define L1I_IX_RDATA 28
/* This is the number of cache entries. User should change it if necessary. */
#define CACHE_SET_NUM 0x1000
#define CACHE_WAY_NUM 0x8
struct cache_element ce[CACHE_SET_NUM][CACHE_WAY_NUM];
#define NDS_PAGE_SHIFT      12          // for PAGE_SIZE 4KB
#define NDS_PAGE_SIZE       (1UL << NDS_PAGE_SHIFT)
#define CCTL_mskTAG_32 0xffffff		/* palen : 34bit  TAG:PA[palen-1:10 */
#define CCTL_mskTAG_64 0x3fffffffffffff	/* palen : 64bit  TAG:PA[palen-1:10] */

int ndsv5_init_cache(struct target *target)
{
	LOG_DEBUG("ndsv5_init_cache");

	riscv_select_current_hart(target);

	uint8_t seth = 0;
	uint64_t value_cr1 = 0;
	uint64_t value_cr2 = 0;
	struct reg *reg_micm_cfg, *reg_mdcm_cfg;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *icache = &nds32->memory.icache;
	struct nds32_v5_cache *dcache = &nds32->memory.dcache;

	reg_micm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
	reg_mdcm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MDCM_CFG);
	if ((reg_micm_cfg == NULL) || (reg_mdcm_cfg == NULL))
		return ERROR_FAIL;

	value_cr1 = ndsv5_get_register_value(reg_micm_cfg);
	value_cr2 = ndsv5_get_register_value(reg_mdcm_cfg);

	seth = (uint8_t)((value_cr1 >> 24) & 0x1);
	icache->set = value_cr1 & 0x7;
	if (seth) {
		icache->log2_set = 5 - icache->set;
		icache->set = 32 >> icache->set;
	} else {
		icache->log2_set = icache->set + 6;
		icache->set = 64 << icache->set;
	}
	icache->way = ((value_cr1 >> 3) & 0x7) + 1;
	icache->line_size = (value_cr1 >> 6) & 0x7;
	if (icache->line_size != 0) {
		icache->log2_line_size = icache->line_size + 2;
		icache->line_size = 8 << (icache->line_size - 1);
	} else {
		icache->log2_line_size = 0;
	}

	LOG_DEBUG("\ticache set: %lld, way: %lld, line size: %lld, "
			"log2(set): %lld, log2(line_size): %lld",
			icache->set, icache->way, icache->line_size,
			icache->log2_set, icache->log2_line_size);

	seth = (uint8_t)((value_cr2 >> 24) & 0x1);
	dcache->set = value_cr2 & 0x7;
	if (seth) {
		dcache->log2_set = 5 - dcache->set;
		dcache->set = 32 >> dcache->set;
	} else {
		dcache->log2_set = dcache->set + 6;
		dcache->set = 64 << dcache->set;
	}
	dcache->way = ((value_cr2 >> 3) & 0x7) + 1;
	dcache->line_size = (value_cr2 >> 6) & 0x7;
	if (dcache->line_size != 0) {
		dcache->log2_line_size = dcache->line_size + 2;
		dcache->line_size = 8 << (dcache->line_size - 1);
	} else {
		dcache->log2_line_size = 0;
	}

	LOG_DEBUG("\tdcache set: %lld, way: %lld, line size: %lld, "
			"log2(set): %lld, log2(line_size): %lld",
			dcache->set, dcache->way, dcache->line_size,
			dcache->log2_set, dcache->log2_line_size);

	return ERROR_OK;
}

int ndsv5_dump_cache(struct target *target, unsigned int cache_type, const char* filename)
{
	LOG_DEBUG("Dump Cache");

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand, *reg_mcctldata, *reg_mmsc_cfg, *reg_marchid;
	FILE *pFile; 
	uint64_t idx, way, ra, tag, i, maskTAG, read_tag_cmd, read_data_cmd, reg_marchid_value;
	uint64_t sets, ways, line_size, set_bits, line_bits, way_offset;
	uint64_t cache_index, word_num, word_size;
	struct nds32_v5_cache *cache;
	uint64_t total_cache;
	uint64_t now_cache = 0;
	int xlen, shift_bit, mesi;
	bool idx_auto = false, new_tagformat = false;

	pFile = fopen( filename, "w" );
	if( NULL == pFile ) {
		LOG_ERROR("Error!! Can't open file to write");
		return ERROR_FAIL;
	}

	if (ndsv5_init_cache(target) != ERROR_OK)
		return ERROR_FAIL;

	if ( cache_type == ICACHE ) {
		cache = &nds32->memory.icache;
		read_tag_cmd = L1I_IX_RTAG;
		read_data_cmd = L1I_IX_RDATA;
	} else if (cache_type == DCACHE) {
		cache = &nds32->memory.dcache;
		read_tag_cmd = L1D_IX_RTAG;
		read_data_cmd = L1D_IX_RDATA;
	} else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	xlen = riscv_xlen(target);
	if (xlen == 32)
		maskTAG = CCTL_mskTAG_32;
	else if (xlen == 64)
		maskTAG = CCTL_mskTAG_64;
	else {
		LOG_ERROR("%s not supported xlen:%d", __func__, xlen);
		return ERROR_FAIL;
	}

	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	reg_mcctldata = ndsv5_get_reg_by_CSR(target, CSR_MCCTLDATA);
	reg_mmsc_cfg = ndsv5_get_reg_by_CSR(target, CSR_MMSC_CFG);
	if ((reg_mcctlbeginaddr == NULL) || (reg_mcctlcommand == NULL) || (reg_mcctldata == NULL) || (reg_mmsc_cfg == NULL))
		return ERROR_FAIL;

	//if vcctl=1, index will auto count
	if (ndsv5_get_register_value(reg_mmsc_cfg) & 0x40000) {
		idx_auto = true;
		LOG_DEBUG("VCCTL=1, AUTO COUNT INDEX");
	}

	ways = cache->way;
	sets = cache->set;
	set_bits = cache->log2_set;
	line_bits = cache->log2_line_size;
	line_size = cache->line_size;
	way_offset = set_bits + line_bits;
	LOG_DEBUG("Way:%lld, Set:%lld, Line Size:%lld", ways, sets, line_size);

	/* check cpu id == 0x45 and dcache, tag no used bit = shift_bit,
	 * shift bit = log2 line_size + log2 set; others, shift bit = 10 */
	reg_marchid = ndsv5_get_reg_by_CSR(target, CSR_MARCHID);
	reg_marchid_value = ndsv5_get_register_value(reg_marchid);
	if (((reg_marchid_value & 0xff) == 0x45) && (cache_type == DCACHE)) {
		new_tagformat = true;
		shift_bit = line_bits + set_bits;
	} else
		shift_bit = 10;

	/* Index Example Format for CCTL Index Type Operation for 64bit icache
	 * same with 32bit i/dcache(from Roger email) */
	if ((xlen == 64) && (cache_type == ICACHE))
		word_size = 4;
	else
		word_size = xlen / 8;
	word_num = line_size / word_size;
	total_cache = ways * sets * word_num;
	LOG_DEBUG( "Total cache:%lld", total_cache); 

	/* READ TAG/DATA COMMAND :auto count idx or no auto count idx */
	if (idx_auto) {
		/* READ TAG COMMAND */
		ndsv5_set_register_value(reg_mcctlbeginaddr, 0);
		if (new_tagformat) {
			for (idx = 0; idx < sets; idx++) {
				for (way = 0; way < ways; way++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
					tag = ndsv5_get_register_value(reg_mcctldata);
					LOG_DEBUG("tag: %lld", tag);

					mesi = tag & 0x7;
					ce[idx][way].inval = (mesi == 0);
					ce[idx][way].shared = (mesi == 1);
					ce[idx][way].exclusive = (mesi == 3);
					ce[idx][way].modified = (mesi == 7);

					ce[idx][way].lock = (uint8_t)(tag & 0x8) >> 3;
					ce[idx][way].pa = (tag >> 4) << shift_bit ;
				}
			}
		} else {
			for (idx = 0; idx < sets; idx++) {
				for (way = 0; way < ways; way++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
					tag = ndsv5_get_register_value(reg_mcctldata);
					LOG_DEBUG("tag: %lld", tag);

					ce[idx][way].valid = (uint8_t)((tag & (1 << (xlen - 1))) >> (xlen - 1));
					ce[idx][way].lock = (uint8_t)((tag & (1 << (xlen - 2))) >> (xlen - 2));
					ce[idx][way].dirty = (uint8_t)((tag & (1 << (xlen - 3))) >> (xlen - 3));
					ce[idx][way].pa = (tag & maskTAG) << shift_bit;
				}
			}
		}
		/* READ DATA COMMAND */
		for (i = 0; i < word_num; i++) {
			ndsv5_set_register_value(reg_mcctlbeginaddr, (i * word_size));
			for (idx = 0; idx < sets; idx++) {
				for (way = 0; way < ways; way++) {
					ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
					ce[idx][way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
				}
			}
			NDS32_LOG_R("Dump Progressing...%lld%%", ((sets*ways*(i+1)*100)/total_cache));
		}
	} else {
		for (idx = 0; idx < sets; idx++) {
			for (way = 0; way < ways; way++) {
				//READ TAG COMMAND
				ra = (way << way_offset) | (idx << line_bits);
				ndsv5_set_register_value(reg_mcctlbeginaddr, ra);
				ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
				tag = ndsv5_get_register_value(reg_mcctldata);

				if (new_tagformat) {
					mesi = tag & 0x7;
					ce[idx][way].inval = (mesi == 0);
					ce[idx][way].shared = (mesi == 1);
					ce[idx][way].exclusive = (mesi == 3);
					ce[idx][way].modified = (mesi == 7);

					ce[idx][way].lock = (uint8_t)(tag & 0x8) >> 3;
					ce[idx][way].pa = (tag >> 4) << shift_bit ;
				} else {
					ce[idx][way].valid = (uint8_t)((tag & (1 << (xlen - 1))) >> (xlen - 1));
					ce[idx][way].lock = (uint8_t)((tag & (1 << (xlen - 2))) >> (xlen - 2));
					ce[idx][way].dirty = (uint8_t)((tag & (1 << (xlen - 3))) >> (xlen - 3));
					ce[idx][way].pa = (tag & maskTAG) << shift_bit;
				}

				/* READ DATA COMMAND */
				for (i = 0; i < word_num; i++) {
					cache_index = (ra | (i * word_size));
					ndsv5_set_register_value(reg_mcctlbeginaddr, cache_index);
					ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
					ce[idx][way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
				}
				now_cache += word_num;
				NDS32_LOG_R("Dump Progressing...%lld%%", ((now_cache*100)/total_cache));
			}
		}
	}

	/* print 32bit, 64bit icache, 64bit dcache */
	char *fmt_str = " %8llx";
	char *fmt_str1 = "%08llx ";
	char *fmt_str2 = "%8s %4s %4s %1s %1s %1s";
	char *fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x ";
	if (xlen == 64) {
		if (cache_type == DCACHE) {
			fmt_str = " %16llx";
			fmt_str1 = "%016llx ";
		}
		fmt_str2 = "%16s %4s %4s %1s %1s %1s";
		fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x ";
	}
	if (new_tagformat) {
		fmt_str2 = "%8s %4s %4s %1s %1s %1s %1s %1s";
		fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x %01x %01x ";
		if (xlen == 64) {
			fmt_str2 = "%16s %4s %4s %1s %1s %1s %1s %1s";
			fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x %01x %01x ";
		}
	}

	fprintf(pFile, "dump %s\n", cache_type ? "DCACHE":"ICACHE");
	if (new_tagformat)
		fprintf(pFile, fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		fprintf(pFile, fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		fprintf(pFile, fmt_str, (i * word_size));
	fprintf(pFile, "\n");
	for (idx = 0; idx < sets; idx++) {
		for (way = 0; way < ways; way++) {
			if (new_tagformat)
				fprintf(pFile, fmt_str3,
					ce[idx][way].pa | (idx * line_size),
					idx,
					way,
					ce[idx][way].inval,
					ce[idx][way].shared,
					ce[idx][way].exclusive,
					ce[idx][way].modified,
					ce[idx][way].lock);
			else
				fprintf(pFile, fmt_str3,
					ce[idx][way].pa | (idx * line_size),
					idx,
					way,
					ce[idx][way].valid,
					ce[idx][way].dirty,
					ce[idx][way].lock);
			for (i = 0; i < word_num; i++)
				fprintf(pFile, fmt_str1, ce[idx][way].cacheline[i]);
			fprintf(pFile, "\n");
		}
	}

	NDS32_LOG("\nDump Finish!!");
	LOG_DEBUG("\nDump Finish!!");
	fclose(pFile);
	return ERROR_OK;
}

int ndsv5_dump_cache_va(struct target *target, unsigned int cache_type, uint64_t va)
{
	LOG_DEBUG( "Dump Cache" );

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand, *reg_mcctldata, *reg_marchid;
	uint64_t idx, way, ra, tag, i, maskTAG, read_tag_cmd, read_data_cmd, reg_marchid_value;
	uint64_t sets, ways, line_size, line_bits, set_bits, way_offset;
	uint64_t cache_index, word_num, word_size;
	struct nds32_v5_cache *cache;
	struct cache_element ces[8];    // maximum mdicm_cfg.DWAY is 8
	int xlen, shift_bit, mesi;
	bool new_tagformat = false;

	if (ndsv5_init_cache(target) != ERROR_OK)
		return ERROR_FAIL;

	if( cache_type == ICACHE ) {
		cache = &nds32->memory.icache;
		read_tag_cmd = L1I_IX_RTAG;
		read_data_cmd = L1I_IX_RDATA;
	} else if (cache_type == DCACHE) {
		cache = &nds32->memory.dcache;
		read_tag_cmd = L1D_IX_RTAG;
		read_data_cmd = L1D_IX_RDATA;
	} else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	xlen = riscv_xlen(target);
	if (xlen == 32)
		maskTAG = CCTL_mskTAG_32;
	else if (xlen == 64)
		maskTAG = CCTL_mskTAG_64;
	else {
		LOG_ERROR("%s not supported xlen:%d", __func__, xlen);
		return ERROR_FAIL;
	}

	ways = cache->way;
	sets = cache->set;
	line_size = cache->line_size;
	line_bits = cache->log2_line_size;
	set_bits = cache->log2_set;
	way_offset = set_bits + line_bits;

	uint64_t pa = va;
	ndsv5_get_physical_address(target, va, &pa);
	LOG_DEBUG( "physical address:0x%llx", pa);
	//if dcache use pa to index; if icache use va to index
	if( cache_type == DCACHE ) {
		idx = (pa & (((1 << set_bits) - 1) << line_bits));
	} else {
		idx = (va & (((1 << set_bits) - 1) << line_bits));
	}

	LOG_DEBUG("Way:%lld, Set:%lld, Line Size:%lld", ways, sets, line_size);

	/* check cpu id == 0x45 and dcache, tag no used bit = shift_bit,
	 * shift bit = log2 line_size + log2 set; others, shift bit = 10 */
	reg_marchid = ndsv5_get_reg_by_CSR(target, CSR_MARCHID);
	reg_marchid_value = ndsv5_get_register_value(reg_marchid);
	if (((reg_marchid_value & 0xff) == 0x45) && (cache_type == DCACHE)) {
		new_tagformat = true;
		shift_bit = line_bits + set_bits;
	} else
		shift_bit = 10;

	/* Index Example Format for CCTL Index Type Operation for 64bit icache
	 * same with 32bit i/dcache(from Roger email) */
	if ((xlen == 64) && (cache_type == ICACHE))
		word_size = 4;
	else
		word_size = xlen / 8;
	word_num = line_size / word_size;

	//check which way is the dump data for user
	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	reg_mcctldata = ndsv5_get_reg_by_CSR(target, CSR_MCCTLDATA);
	if ((reg_mcctlbeginaddr == NULL) || (reg_mcctlcommand == NULL) || (reg_mcctldata == NULL))
		return ERROR_FAIL;

	for( way = 0; way < ways; way++ ) {
		// Read Tag first
		ra = (way << way_offset) | idx;

		// Read TAG command
		ndsv5_set_register_value(reg_mcctlbeginaddr, ra);
		ndsv5_set_register_value(reg_mcctlcommand, read_tag_cmd);
		tag = ndsv5_get_register_value(reg_mcctldata);
		LOG_DEBUG( "tag: %lld", tag);
		if (new_tagformat) {
			mesi = tag & 0x7;
			ces[way].inval = (mesi == 0);
			ces[way].shared = (mesi == 1);
			ces[way].exclusive = (mesi == 3);
			ces[way].modified = (mesi == 7);

			ces[way].lock = (uint8_t)(tag & 0x8) >> 3;
			ces[way].pa = (tag >> 4) << shift_bit ;
		} else {
			ces[way].valid = (uint8_t)((tag & (1 << (xlen - 1))) >> (xlen - 1));
			ces[way].lock = (uint8_t)((tag & (1 << (xlen - 2))) >> (xlen - 2));
			ces[way].dirty = (uint8_t)((tag & (1 << (xlen - 3))) >> (xlen - 3));
			ces[way].pa = (tag & maskTAG) << shift_bit;
		}
		for( i = 0; i < word_num; i++ ) {
			cache_index = (ra | (i * word_size));
			// Read DATA command
			ndsv5_set_register_value(reg_mcctlbeginaddr, cache_index);
			ndsv5_set_register_value(reg_mcctlcommand, read_data_cmd);
			ces[way].cacheline[i] = ndsv5_get_register_value(reg_mcctldata);
		}
	}

	/* print 32bit, 64bit icache, 64bit dcache */
	char *fmt_str = " %8llx";
	char *fmt_str1 = "%08llx ";
	char *fmt_str2 = "%8s %4s %4s %1s %1s %1s";
	char *fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x ";
	if (xlen == 64) {
		if (cache_type == DCACHE) {
			fmt_str = " %16llx";
			fmt_str1 = "%016llx ";
		}
		fmt_str2 = "%16s %4s %4s %1s %1s %1s";
		fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x ";
	}
	if (new_tagformat) {
		fmt_str2 = "%8s %4s %4s %1s %1s %1s %1s %1s";
		fmt_str3 = "%08llx %04llx %04llx %01x %01x %01x %01x %01x ";
		if (xlen == 64) {
			fmt_str2 = "%16s %4s %4s %1s %1s %1s %1s %1s";
			fmt_str3 = "%016llx %04llx %04llx %01x %01x %01x %01x %01x ";
		}
	}

	NDS32_LOG_LF("dump %s\n", cache_type ? "DCACHE" : "ICACHE");
	if (new_tagformat)
		NDS32_LOG_LF(fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		NDS32_LOG_LF(fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		NDS32_LOG_LF(fmt_str, (i * word_size));
	NDS32_LOG_LF("\n");
	for (way = 0; way < ways; way++) {
		if (new_tagformat)
			NDS32_LOG_LF(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].inval,
				ces[way].shared,
				ces[way].exclusive,
				ces[way].modified,
				ces[way].lock);
		else
			NDS32_LOG_LF(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock);
		for (i = 0; i < word_num; i++)
			NDS32_LOG_LF(fmt_str1, ces[way].cacheline[i]);
		NDS32_LOG_LF("\n");
	}

	LOG_INFO("dump %s\n", cache_type ? "DCACHE" : "ICACHE");
	if (new_tagformat)
		LOG_INFO(fmt_str2, "ADDRESS", "SET", "WAY", "I", "S", "E", "M", "L");
	else
		LOG_INFO(fmt_str2, "ADDRESS", "SET", "WAY", "V", "D", "L");
	for (i = 0; i < word_num; i++)
		LOG_INFO(fmt_str, (i * word_size));
	LOG_INFO("\n");
	for (way = 0; way < ways; way++) {
		if (new_tagformat)
			LOG_INFO(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].inval,
				ces[way].shared,
				ces[way].exclusive,
				ces[way].modified,
				ces[way].lock);
		else
			LOG_INFO(fmt_str3,
				ces[way].pa | idx,
				idx >> line_bits,
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock);
		for (i = 0; i < word_num; i++)
			LOG_INFO(fmt_str1, ces[way].cacheline[i]);
		LOG_INFO("\n");
	}
	return ERROR_OK;
}

int ndsv5_enableornot_cache(struct target *target, unsigned int cache_type, const char* enableornot)
{
	LOG_DEBUG( "enable or disable Cache" );

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	struct nds32_v5_cache *cache;

	struct reg *reg_mcache_ctl;
	uint64_t enable_bit = 0x1, new_value;

	reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_DEBUG( "Cannot read mcache_ctl" );
		return ERROR_FAIL;
	}

	if (cache_type == DCACHE)
		enable_bit = enable_bit << 1;

	// enable/disable icache/dcache
	if (strcmp(enableornot,"enable") == 0)
		new_value = ndsv5_get_register_value(reg_mcache_ctl) | enable_bit;
	else
		new_value = ndsv5_get_register_value(reg_mcache_ctl) & (~enable_bit);

	ndsv5_set_register_value(reg_mcache_ctl, new_value);

	// check enable/disable icache/dcache PASS or FAIL
	new_value = ndsv5_get_register_value(reg_mcache_ctl);
	LOG_DEBUG("reg_mcache_ctl : %llx", new_value);

	// for set icache/dcache->enable=true/false
	cache = cache_type ? &nds32->memory.dcache : &nds32->memory.icache;

	if ((new_value & enable_bit) != 0) {
		if (strcmp(enableornot,"disable") == 0) {
			LOG_ERROR("Unable disable %s", cache_type ? "DCACHE" : "ICACHE");
			return ERROR_FAIL;
		}
		cache->enable = true;
	} else {
		if (strcmp(enableornot,"enable") == 0) {
			LOG_ERROR("Unable enable %s", cache_type ? "DCACHE" : "ICACHE");
			return ERROR_FAIL;
		}
		cache->enable = false;
	}

	return ERROR_OK;
}

int ndsv5_dcache_wb(struct target *target)
{
	LOG_DEBUG( "ndsv5_dcache_writeback" );

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR( "Cannot read mcache_ctl" );
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support writeback");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;
	// 7: L1D_WB_ALL
	ndsv5_set_register_value(reg_mcctlcommand, L1D_WB_ALL);

	return ERROR_OK;
}

int ndsv5_dcache_invalidate(struct target *target)
{
	LOG_DEBUG( "ndsv5_dcache_invalidate" );

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR( "Cannot read mcache_ctl" );
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support invalidate");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;
	// 23: L1D_INVAL_ALL
	ndsv5_set_register_value(reg_mcctlcommand, L1D_INVAL_ALL);

	return ERROR_OK;
}

int ndsv5_dcache_wb_invalidate(struct target *target)
{
	LOG_DEBUG( "ndsv5_dcache_wb_invalidate" );

	riscv_select_current_hart(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR( "Cannot read mcache_ctl" );
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x2) == 0) {
		LOG_DEBUG("Data cache disabled, NOT support wb_invalidate");
		return ERROR_OK;
	}

	struct reg *reg_mcctlcommand;
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if (reg_mcctlcommand == NULL)
		return ERROR_FAIL;
	// 6: L1D_WBINVAL_ALL
	ndsv5_set_register_value(reg_mcctlcommand, L1D_WBINVAL_ALL);

	return ERROR_OK;
}

int ndsv5_icache_invalidate(struct target *target)
{
	LOG_DEBUG( "ndsv5_icache_invalidate" );

	riscv_select_current_hart(target);

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);

	struct reg *reg_mcache_ctl = ndsv5_get_reg_by_CSR(target, CSR_MCACHE_CTL);
	if (reg_mcache_ctl == NULL) {
		LOG_ERROR( "Cannot read mcache_ctl" );
		return ERROR_FAIL;
	}
	if ((ndsv5_get_register_value(reg_mcache_ctl) & 0x1) == 0) {
		LOG_DEBUG("Instruction cache disabled, NOT support invalidate");
		return ERROR_OK;
	}

	uint64_t set, way, way_offset, index;
	struct nds32_v5_cache *cache = &nds32->memory.icache;
	struct reg *reg_mcctlbeginaddr, *reg_mcctlcommand;

	way_offset = cache->log2_set + cache->log2_line_size;
	reg_mcctlbeginaddr = ndsv5_get_reg_by_CSR(target, CSR_MCCTLBEGINADDR);
	reg_mcctlcommand = ndsv5_get_reg_by_CSR(target, CSR_MCCTLCOMMAND);
	if ((reg_mcctlbeginaddr == NULL) || (reg_mcctlcommand == NULL))
		return ERROR_FAIL;

	for ( set = 0; set < cache->set; set++ ) {
		for ( way = 0; way < cache->way; way++ ) {
			index = ((way << way_offset) | (set << cache->log2_line_size));
			ndsv5_set_register_value(reg_mcctlbeginaddr, index);
			// 24: L1I_IX_INVAL
			ndsv5_set_register_value(reg_mcctlcommand, L1I_IX_INVAL);
		}
	}

	return ERROR_OK;
}
