/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for AndesCore NCETRACE200, including following components:
 * - TraceEncoder (NCETENC200)
 * - TraceMultiplexer (NCETMUX200)
 * - TraceBuffer (NCETBUF200)
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <helper/log.h>
#include <target/target.h>
#include <target/smp.h>
#include <target/nds32_new/nds32_log.h>
#include <jtag/interface.h>

#include "riscv.h"
#include "debug_defines.h"
#include "ndsv5.h"
#include "debug_defines_ncetrace.h"


/*******************************************************
 * Global variable
 *******************************************************/
uint32_t nds_tracer_on;
uint32_t nds_tracer_action = CSR_MCONTROL_ACTION_TRACE_OFF;
uint32_t nds_tracer_stop_on_wrap;
uint32_t nds_trTeSyncMax = 4;
uint32_t nds_teSyncMode = 1;
uint32_t nds_trTeInstMode = 6;
uint32_t nds_teInhibitSrc = 1;
uint64_t nds_tracer_active_id = 0x01;
uint32_t nds_teInstNoAddrDiff;
uint32_t nds_timestamp_on, nds_trTsControl;
uint32_t nds_trTeFilteriMatchInst;
uint32_t nds_trTeFilterMatchValueContext;
uint32_t nds_trTeFilterMatchMaskContext = 0x3;
uint64_t nds_trRamStart;
uint64_t ndsv5_trace_ram_size = 4 * 1024; /* Default: 4K */
uint32_t ndsv5_mpsse_t2 = -1;
uint32_t TB_RAM_SIZE = 0x2000;
uint32_t ndsv5_TsCompMode;
enum ndsv5_trace_mem_mode ndsv5_trace_mem_mode = NDSV5_TRACE_MEM_SRAM;
uint32_t nds_trTeInstExtendAddrMSB;

int ndsv5_tracer_capability_check(struct target *target);
int ndsv5_tracer_smem_capability_check(struct target *target);
int ndsv5_tracer_pib_capability_check(struct target *target);

extern uint32_t nds_sys_bus_supported;

#define TRACER_VERSION       2         /* version number: 0~255 */
#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define TRACER_HEADER_VERSION_V1              1
#define TRACER_HEADER_PREFIX_BYTES            4
#define TRACER_HEADER_V1_BYTES                4
#define TRACER_HEADER_LEN24_MAX               0xFFFFFFu
#define TRACER_HEADER_V2_LEGACY_PAYLOAD_LEN   4
#define TRACER_HEADER_V2_LEGACY_TOTAL_BYTES   7
#define TRACER_TLV_TAG_SRCBITS                1
#define TRACER_TLV_TAG_INST_NO_ADDR_DIFF      2
#define TRACER_TLV_TAG_VALEN                  3
#define TRACER_TLV_TAG_TS_COMP_MODE           4
#define TRACER_TLV_TAG_PADDING                0xFE
#define TRACER_TLV_U8_LEN                     1

struct ndsv5_tracer_header {
	unsigned char version;
	unsigned char srcbits;
	unsigned char inst_no_addr_diff;
	unsigned char valen;
	unsigned char ts_comp_mode;
};

static int ndsv5_tracer_header_write_tlv_u8(unsigned char *buf, size_t buf_size,
	size_t *offset, unsigned char tag, unsigned char value)
{
	if (!buf || !offset || (*offset + 3) > buf_size)
		return ERROR_FAIL;

	buf[*offset] = tag;
	buf[*offset + 1] = TRACER_TLV_U8_LEN;
	buf[*offset + 2] = value;
	*offset += 3;

	return ERROR_OK;
}

static int ndsv5_tracer_build_header_v2(const struct ndsv5_tracer_header *header,
	unsigned char *buf, size_t buf_size, size_t *header_bytes)
{
	size_t offset = TRACER_HEADER_PREFIX_BYTES;
	uint32_t header_length;

	if (!header || !buf || !header_bytes || buf_size < TRACER_HEADER_PREFIX_BYTES)
		return ERROR_FAIL;

	buf[3] = header->version;

	if (ndsv5_tracer_header_write_tlv_u8(buf, buf_size, &offset,
		TRACER_TLV_TAG_SRCBITS, header->srcbits) != ERROR_OK)
		return ERROR_FAIL;

	if (ndsv5_tracer_header_write_tlv_u8(buf, buf_size, &offset,
		TRACER_TLV_TAG_INST_NO_ADDR_DIFF, header->inst_no_addr_diff) != ERROR_OK)
		return ERROR_FAIL;

	if (ndsv5_tracer_header_write_tlv_u8(buf, buf_size, &offset,
		TRACER_TLV_TAG_VALEN, header->valen) != ERROR_OK)
		return ERROR_FAIL;

	if (ndsv5_tracer_header_write_tlv_u8(buf, buf_size, &offset,
		TRACER_TLV_TAG_TS_COMP_MODE, header->ts_comp_mode) != ERROR_OK)
		return ERROR_FAIL;

	/*
	 * Keep v2 header 4-byte aligned so current .log word-based dump/read
	 * path keeps correct packet boundary.
	 */
	while ((offset & 0x3) != 0) {
		if (ndsv5_tracer_header_write_tlv_u8(buf, buf_size, &offset,
			TRACER_TLV_TAG_PADDING, 0) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (offset <= 3 || (offset - 3) > TRACER_HEADER_LEN24_MAX)
		return ERROR_FAIL;

	header_length = (uint32_t)(offset - 3); /* counted from byte[3] */
	buf[0] = (unsigned char)(header_length & 0xFF);
	buf[1] = (unsigned char)((header_length >> 8) & 0xFF);
	buf[2] = (unsigned char)((header_length >> 16) & 0xFF);
	*header_bytes = offset;

	return ERROR_OK;
}

static int ndsv5_tracer_parse_header_v2_tlv_payload(const unsigned char *payload,
	size_t payload_bytes, struct ndsv5_tracer_header *header)
{
	unsigned int have_srcbits = 0;
	unsigned int have_inst_no_addr_diff = 0;
	unsigned int have_valen = 0;

	header->ts_comp_mode = 0;

	while (payload_bytes) {
		unsigned char tag, tlv_len;

		if (payload_bytes < 2)
			return ERROR_FAIL;

		tag = payload[0];
		tlv_len = payload[1];
		payload += 2;
		payload_bytes -= 2;

		if (tlv_len > payload_bytes)
			return ERROR_FAIL;

		switch (tag) {
		case TRACER_TLV_TAG_SRCBITS:
			if (tlv_len != TRACER_TLV_U8_LEN)
				return ERROR_FAIL;
			header->srcbits = payload[0];
			have_srcbits = 1;
			break;
		case TRACER_TLV_TAG_INST_NO_ADDR_DIFF:
			if (tlv_len != TRACER_TLV_U8_LEN)
				return ERROR_FAIL;
			header->inst_no_addr_diff = payload[0];
			have_inst_no_addr_diff = 1;
			break;
		case TRACER_TLV_TAG_VALEN:
			if (tlv_len != TRACER_TLV_U8_LEN)
				return ERROR_FAIL;
			header->valen = payload[0];
			have_valen = 1;
			break;
		case TRACER_TLV_TAG_TS_COMP_MODE:
			if (tlv_len != TRACER_TLV_U8_LEN)
				return ERROR_FAIL;
			header->ts_comp_mode = payload[0];
			break;
		default:
			/* Skip unknown TLV for forward compatibility. */
			break;
		}

		payload += tlv_len;
		payload_bytes -= tlv_len;
	}

	if (!have_srcbits || !have_inst_no_addr_diff || !have_valen)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int ndsv5_tracer_parse_header(const unsigned char *buf, size_t buf_size,
	struct ndsv5_tracer_header *header, size_t *header_bytes)
{
	uint32_t header_length;
	size_t total_header_bytes;
	size_t payload_bytes;

	if (!buf || !header || !header_bytes || buf_size < TRACER_HEADER_V1_BYTES)
		return ERROR_FAIL;

	header->version = buf[3];

	if (header->version == TRACER_HEADER_VERSION_V1) {
		header->srcbits = buf[0];
		header->inst_no_addr_diff = buf[1];
		header->valen = buf[2];
		header->ts_comp_mode = 0;
		*header_bytes = TRACER_HEADER_V1_BYTES;
		return ERROR_OK;
	}

	if (header->version != TRACER_VERSION)
		return ERROR_FAIL;

	header_length = (uint32_t)buf[0] |
		((uint32_t)buf[1] << 8) |
		((uint32_t)buf[2] << 16);

	if (header_length < 1)
		return ERROR_FAIL;

	total_header_bytes = (size_t)header_length + 3;
	if (total_header_bytes > buf_size)
		return ERROR_FAIL;

	/* Transitional compatibility: old v2 positional payload. */
	if (header_length == TRACER_HEADER_V2_LEGACY_PAYLOAD_LEN) {
		if (total_header_bytes < TRACER_HEADER_V2_LEGACY_TOTAL_BYTES)
			return ERROR_FAIL;
		header->srcbits = buf[4];
		header->inst_no_addr_diff = buf[5];
		header->valen = buf[6];
		header->ts_comp_mode = 0;
		*header_bytes = total_header_bytes;
		return ERROR_OK;
	}

	payload_bytes = (size_t)header_length - 1; /* exclude version byte */
	if (ndsv5_tracer_parse_header_v2_tlv_payload(buf + TRACER_HEADER_PREFIX_BYTES,
		payload_bytes, header) != ERROR_OK)
		return ERROR_FAIL;

	*header_bytes = total_header_bytes;
	return ERROR_OK;
}



/*******************************************************
 * Static Global variable
 *******************************************************/
static bool nds_trace_new_bitmap;
static uint32_t *p_etb_buf_start, *p_etb_buf_end;
static uint32_t *p_etb_wptr;
static void tracer_tbuf_set_ram_mode(struct target *target);
static int ndsv5_tracer_buffer_init(void);
static uint32_t nds_tracer_multiplexer, nds_tracer_capability;

static int component_read(struct target *target, enum ndsv5_component_id id, uint32_t *value, uint64_t addr)
{
	if (ndsv5_comps[id].addr_type == NDSV5_COMP_ADDR_DMI) {
		RISCV_INFO(r);
		if (!r) {
			LOG_ERROR("riscv_info is NULL!");
			return ERROR_FAIL;
		}

		if (r->dmi_read)
			return r->dmi_read(target, value, (addr >> 2));
	} else {
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		struct nds32_v5_memory *memory = &(nds32->memory);
		uint32_t bak_access_channel = (uint32_t)memory->access_channel;
		if (nds_sys_bus_supported)
			memory->access_channel = NDS_MEMORY_ACC_BUS;
		else {
			LOG_DEBUG("nds_sys_bus_supported not support, switch to CPU mode");
			nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
		}
		uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
		nds32->nds_va_to_pa_off = 1;

		if (target_read_memory(target, addr, 4, 1, (uint8_t *)value) != ERROR_OK) {
			LOG_DEBUG("[ERROR] Unable to read on 0x%" PRIx64, addr);
			return ERROR_FAIL;
		}

		*value =  *value & 0xffffffff;
		LOG_DEBUG("Get component reg 0x%" PRIx64 " data 0x%" PRIx32, addr, *value);

		memory->access_channel = bak_access_channel;
		nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int component_write(struct target *target, enum ndsv5_component_id id, uint64_t addr, uint32_t value)
{
	if (ndsv5_comps[id].addr_type == NDSV5_COMP_ADDR_DMI) {
		RISCV_INFO(r);
		if (!r) {
			LOG_ERROR("riscv_info is NULL!");
			return ERROR_FAIL;
		}

		if (r->dmi_write)
			return r->dmi_write(target, (addr >> 2), value);
	} else {
		struct nds32_v5 *nds32 = target_to_nds32_v5(target);
		struct nds32_v5_memory *memory = &(nds32->memory);
		uint32_t bak_access_channel = (uint32_t)memory->access_channel;
		if (nds_sys_bus_supported)
			memory->access_channel = NDS_MEMORY_ACC_BUS;
		else {
			LOG_DEBUG("nds_sys_bus_supported not support, switch to CPU mode");
			nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
		}
		uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
		nds32->nds_va_to_pa_off = 1;

		LOG_DEBUG("Component set reg: 0x%" PRIx64 ", data: 0x%" PRIx32, addr, value);

		if (target_write_memory(target, addr, 4, 1, (uint8_t *)&value) != ERROR_OK) {
			LOG_DEBUG("[ERROR] Unable to write on 0x%" PRIx64, addr);
			return ERROR_FAIL;
		}

		memory->access_channel = bak_access_channel;
		nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

/* tracer functions */
static uint32_t trace_sel;
static uint32_t selected_encoder(uint32_t offset)
{
	if (ndsv5_comps[NDSV5_COMP_NCETENC].addr_type == NDSV5_COMP_ADDR_DMI)
		return ndsv5_comps[NDSV5_COMP_NCETENC].addr + (trace_sel * (0x2000)) + offset;
	else {
		if (nds_trace_new_bitmap)
			return ndsv5_comps[NDSV5_COMP_NCETENC].addr + (trace_sel * (0x1000)) + offset;
		else
			return ndsv5_comps[NDSV5_COMP_NCETENC].addr + (trace_sel * (0x2000)) + offset;
	}
}

static uint32_t selected_tbuf_sink(struct target *target, uint32_t offset)
{
	static uint32_t sink_offset = 0xFFFFFFFF;

	/* Check TRRAMIMPL once */
	if (sink_offset == 0xFFFFFFFF) {
		LOG_DEBUG("Check Trace RAM Sink");
		uint32_t  tf_info_reg;
		component_read(target, NDSV5_COMP_NCETBUF, &tf_info_reg, DMI_TRRAMIMPL);

		uint8_t trRamCompType = (tf_info_reg >> 8) & 0xF;
		switch (trRamCompType) {
			case 9:
				LOG_DEBUG("Trace RAM Sink");
				sink_offset = DMI_TB_PIBSINKBASE_SHIFT;
				break;
			case 10:
				LOG_DEBUG("Trace PIB Sink");
				sink_offset = 0;
				break;
			default:
				LOG_DEBUG("Unknow Sink Component Type, presume RAM Sink");
				sink_offset = DMI_TB_PIBSINKBASE_SHIFT;
				break;
		};
	}

	return ndsv5_comps[NDSV5_COMP_NCETBUF].addr + sink_offset + offset;
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
	uint32_t  tmux_ctrl_reg;

	LOG_DEBUG("DBG_API:activate TMUX_ITTMUXCTRL %d", trace_sel);
	if (nds_trace_new_bitmap) {
		component_read(target, NDSV5_COMP_NCETMUX, &tmux_ctrl_reg, DMI_ITTMUXCTRL);
		tmux_ctrl_reg |= (0x01 << trace_sel);
		component_write(target, NDSV5_COMP_NCETMUX, DMI_ITTMUXCTRL, tmux_ctrl_reg);
		component_read(target, NDSV5_COMP_NCETMUX, &tmux_ctrl_reg, DMI_ITTMUXCTRL);
	} else {
		component_read(target, NDSV5_COMP_NCETMUX, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
		tmux_ctrl_reg |= (0x01 << trace_sel);
		component_write(target, NDSV5_COMP_NCETMUX, TMUX_ITTMUXCTRL, tmux_ctrl_reg);
		component_read(target, NDSV5_COMP_NCETMUX, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
	}
	LOG_DEBUG("DBG_API: tmux_ctrl_reg 0x%x", tmux_ctrl_reg);

	LOG_DEBUG("DBG_API:activate encoder %d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	te_ctrl_reg |= DMI_TRTECONTROL_teActive;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power up */
	uint32_t timeout_counter = 0;
	const uint32_t timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
		if (te_ctrl_reg & DMI_TRTECONTROL_teActive)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teActive to be set for encoder");

	/* Read initial trace encoder registers */
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	component_read(target, NDSV5_COMP_NCETENC, &te_inst_features_reg, selected_encoder(DMI_TRTEINSTFEATURES));
	component_read(target, NDSV5_COMP_NCETENC, &ts_ctrl_reg, selected_encoder(DMI_TRTSCONTROL));
	component_read(target, NDSV5_COMP_NCETENC, &xtrigger_in_ctrl_reg, selected_encoder(DMI_TRTETRIGEXTINCONT));
	component_read(target, NDSV5_COMP_NCETENC, &xtrigger_out_ctrl_reg, selected_encoder(DMI_TRTETRIGEXTOUTCONT));
	if (nds_trace_new_bitmap)
		LOG_DEBUG("DMI_ATBCONTROL Obsoleted!");
	else
		component_read(target, NDSV5_COMP_NCETENC, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	component_read(target, NDSV5_COMP_NCETENC, &te_info_reg, selected_encoder(DMI_TRTEIMPL));
	if (te_info_reg & DMI_TRTEIMPL_TsCompMode) {
		LOG_DEBUG("DMI_TRTEIMPL_TsCompMode: %d", (te_info_reg & DMI_TRTEIMPL_TsCompMode));
		ndsv5_TsCompMode = 1;
	}

	component_read(target, NDSV5_COMP_NCETBUF, &tf_info_reg, DMI_TRRAMIMPL);
	LOG_DEBUG("DBG_API:te_info_reg = 0x%x, tf_info_reg = 0x%x", te_info_reg, tf_info_reg);
	LOG_DEBUG("DBG_API:te_ctrl_reg = 0x%x, te_inst_features_reg = 0x%x", te_ctrl_reg, te_inst_features_reg);
	LOG_DEBUG("DBG_API:ts_ctrl_reg = 0x%x, xtrigger_in_ctrl_reg = 0x%x", ts_ctrl_reg, xtrigger_in_ctrl_reg);
	LOG_DEBUG("DBG_API:xtrigger_out_ctrl_reg = 0x%x, atb_ctrl_reg = 0x%x", xtrigger_out_ctrl_reg, atb_ctrl_reg);
	return 0;
}

static uint32_t tracer_deactivate_encoder(struct target *target)
{
	uint32_t  te_ctrl_reg;

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	LOG_DEBUG("DBG_API:deactivate encoder%d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	te_ctrl_reg &= ~DMI_TRTECONTROL_teActive;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	uint32_t timeout_counter = 0;
	const uint32_t timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
		if ((te_ctrl_reg & DMI_TRTECONTROL_teActive) == 0)
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

	LOG_DEBUG("DBG_API:enable encoder%d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	te_ctrl_reg |= DMI_TRTECONTROL_teEnable;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	uint32_t timeout_counter = 0;
	const uint32_t timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
		if (te_ctrl_reg & DMI_TRTECONTROL_teEnable)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teEnable to be set for encoder");

	return 0;
}

static uint32_t tracer_disable_encoder(struct target *target)
{
	if (!target->trace_on) {
		LOG_DEBUG("Target has been trun-off tracing, skip!");
		return 0;
	}

	uint32_t  te_ctrl_reg;

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	LOG_DEBUG("DBG_API:disable encoder%d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	te_ctrl_reg &= ~DMI_TRTECONTROL_teEnable;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);

	/* The hardware can take an arbitrarily long time to power down */
	uint32_t timeout_counter = 0;
	const uint32_t timeout_limit	= 50; /* magic number waiting to be tuned */

	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
		if ((te_ctrl_reg & DMI_TRTECONTROL_teEnable) == 0)
			break;
		timeout_counter++;
	}

	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting teEnable to be cleared for encoder");

	target->trace_on = false;
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

	/* New spec 0/3/6 */
	if (te_inst_mode == 7) {
		te_inst_mode = 6;
		call_stack_en = 1;
	}

	LOG_DEBUG("DBG_API:set teInstEnCallStack to %d, teInstStallEn to %d, teInstTrigEn to %d, "
	    "ndsSyncExtra to %d, ndsTracePID to %d, ndsTracePRIV to %d, teInstMode to %d for encoder%d",
	    call_stack_en, stall_en, itrigger_en, trace_sync_extra, trace_context, trace_priv, te_inst_mode, trace_sel);
	LOG_DEBUG("DBG_API:set teInhibitSrc to %d (1 to disable)", inhibit_src);

	component_read(target, NDSV5_COMP_NCETENC, &te_inst_features_reg, selected_encoder(DMI_TRTEINSTFEATURES));
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));

	te_inst_features_reg &= ~(DMI_TRTEINSTFEATURES_teInstNoAddrDiff | DMI_TRTEINSTFEATURES_teInstEnCallStack);
	if (call_stack_en)
		te_inst_features_reg |= (DMI_TRTEINSTFEATURES_teInstEnCallStack);
	if (inst_no_addr_diff)
		te_inst_features_reg |= (DMI_TRTEINSTFEATURES_teInstNoAddrDiff);
	if (nds_trTeInstExtendAddrMSB)
		te_inst_features_reg |= (DMI_TRTEINSTFEATURES_trTeInstExtendAddrMSB);  /* extended MSB to 64-bits */


	te_ctrl_reg &= ~(DMI_TRTECONTROL_teEnable);
	if (inhibit_src)
		te_ctrl_reg |= (DMI_TRTECONTROL_teInhibitSrc);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_teInhibitSrc);
	if (stall_en)
		te_ctrl_reg |= (DMI_TRTECONTROL_teInstStallEn);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_teInstStallEn);
	if (itrigger_en)
		te_ctrl_reg |= (DMI_TRTECONTROL_teInstTrigEn);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_teInstTrigEn);
	if (trace_sync_extra)
		te_ctrl_reg |= (DMI_TRTECONTROL_ndsSyncExtra);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_ndsSyncExtra);
	if (trace_context)
		te_ctrl_reg |= (DMI_TRTECONTROL_ndsTracePID);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_ndsTracePID);
	if (trace_priv)
		te_ctrl_reg |= (DMI_TRTECONTROL_ndsTracePRIV);
	else
		te_ctrl_reg &= ~(DMI_TRTECONTROL_ndsTracePRIV);

	te_ctrl_reg &= ~(DMI_TRTECONTROL_teInstMode_MASK);
	te_ctrl_reg |= ((te_inst_mode << DMI_TRTECONTROL_teInstMode_SHIFT) & DMI_TRTECONTROL_teInstMode_MASK);
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEINSTFEATURES), te_inst_features_reg);
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_set_sync_mode(struct target *target,
	uint32_t  sync_mode,      /* teControl[17:16]: teSyncMode */
	uint32_t  max_interval)   /* teControl[23:20]: teSyncMax */
{
	uint32_t te_ctrl_reg;

	LOG_DEBUG("Set sync mode: %d, max: %d", sync_mode, max_interval);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	te_ctrl_reg &= ~DMI_TRTECONTROL_teSyncMode_MASK;
	te_ctrl_reg &= ~DMI_TRTECONTROL_teSyncMax_MASK;
	te_ctrl_reg |= ((sync_mode << DMI_TRTECONTROL_teSyncMode_SHIFT) & DMI_TRTECONTROL_teSyncMode_MASK);
	te_ctrl_reg |= ((max_interval << DMI_TRTECONTROL_teSyncMax_SHIFT) & DMI_TRTECONTROL_teSyncMax_MASK);
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));
	return 0;
}

static uint32_t tracer_set_filter(struct target *target,
		uint32_t filtermatchinst, uint32_t filtermatchcontext, uint32_t filtermatchmask)
{
	LOG_DEBUG("Setting filter");

	/* Enable filter control */
	uint32_t trTeFilterControl_0;
	component_read(target, NDSV5_COMP_NCETENC, &trTeFilterControl_0, selected_encoder(DMI_TRTEFILTER0CONTROL));
	trTeFilterControl_0 |= 0x1; /* trTeFilterEnable = 1 */
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEFILTER0CONTROL), trTeFilterControl_0);
	component_read(target, NDSV5_COMP_NCETENC, &trTeFilterControl_0, selected_encoder(DMI_TRTEFILTER0CONTROL));
	LOG_DEBUG("trTeFilterControl_0: 0x%x (After enabling)", trTeFilterControl_0);

	if (filtermatchinst)
		trTeFilterControl_0 |= 0x3;     /* trTeFilterEnable = 1, trTeFilterMatchPrivilege = 1 */
	if (filtermatchcontext)
		trTeFilterControl_0 |= 0x10001; /* trTeFilterEnable = 1, nds_trTeFilterMatchContext = 1 */
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEFILTER0CONTROL), trTeFilterControl_0);
	component_read(target, NDSV5_COMP_NCETENC, &trTeFilterControl_0, selected_encoder(DMI_TRTEFILTER0CONTROL));
	LOG_DEBUG("trTeFilterControl_0: 0x%x (After setting)", trTeFilterControl_0);

	/* Set trTeFilteriMatchInst */
	if (filtermatchinst) {
		uint32_t trTeFilterMatchInst_0;
		component_read(target, NDSV5_COMP_NCETENC, &trTeFilterMatchInst_0, selected_encoder(DMI_TRTEFILTER0MATCHINST));
		trTeFilterMatchInst_0 = filtermatchinst;
		component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEFILTER0MATCHINST), trTeFilterMatchInst_0);
		component_read(target, NDSV5_COMP_NCETENC, &trTeFilterMatchInst_0, selected_encoder(DMI_TRTEFILTER0MATCHINST));
		LOG_DEBUG("trTeFilterMatchInst_0: 0x%x", trTeFilterMatchInst_0);
	}

	/* Set trTeFilterMatchValueContext & trTeFilterMatchMaskContext */
	if (filtermatchcontext) {
		uint32_t trTeFilterMatchValueContext;
		component_read(target, NDSV5_COMP_NCETENC, &trTeFilterMatchValueContext, selected_encoder(DMI_TRTEFILTER0MATCHVALUEIMPDEF));
		trTeFilterMatchValueContext = filtermatchcontext;
		component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEFILTER0MATCHVALUEIMPDEF), trTeFilterMatchValueContext);
		component_read(target, NDSV5_COMP_NCETENC, &trTeFilterMatchValueContext, selected_encoder(DMI_TRTEFILTER0MATCHVALUEIMPDEF));
		LOG_DEBUG("trTeFilterMatchValueContext: 0x%x", trTeFilterMatchValueContext);

		uint32_t trTeFilterMatchMaskContext = filtermatchmask;
		component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEFILTER0MATCHMASKIMPDEF), trTeFilterMatchMaskContext);
		component_read(target, NDSV5_COMP_NCETENC, &trTeFilterMatchMaskContext, selected_encoder(DMI_TRTEFILTER0MATCHMASKIMPDEF));
		LOG_DEBUG("trTeFilterMatchMaskContext: 0x%x", trTeFilterMatchMaskContext);
	}

	/* Enable filter 1 in trTeInstFilters */
	uint32_t trTeInstFilters = 0x1;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTEINSTFILTERS), trTeInstFilters);
	component_read(target, NDSV5_COMP_NCETENC, &trTeInstFilters, selected_encoder(DMI_TRTEINSTFILTERS));
	LOG_DEBUG("trTeInstFilters: 0x%x", trTeInstFilters);

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

	component_read(target, NDSV5_COMP_NCETENC, &ts_ctrl_reg, selected_encoder(DMI_TRTSCONTROL));
	ts_ctrl_reg &= ~(DMI_TRTSCONTROL_tsActive|DMI_TRTSCONTROL_tsCount|DMI_TRTSCONTROL_tsDebug);
	if (mode_enable)
		ts_ctrl_reg |= (DMI_TRTSCONTROL_tsActive);
	if (counter_enable)
		ts_ctrl_reg |= (DMI_TRTSCONTROL_tsCount);
	if (stop_on_debug)
		ts_ctrl_reg |= (DMI_TRTSCONTROL_tsDebug);

	ts_ctrl_reg &= ~DMI_TRTSCONTROL_tsType_MASK;
	ts_ctrl_reg |= ((msg_type << DMI_TRTSCONTROL_tsType_SHIFT) & DMI_TRTSCONTROL_tsType_MASK);
	ts_ctrl_reg &= ~DMI_TRTSCONTROL_tsPrescale_MASK;
	ts_ctrl_reg |= ((prescale << DMI_TRTSCONTROL_tsPrescale_SHIFT) & DMI_TRTSCONTROL_tsPrescale_MASK);
	ts_ctrl_reg &= ~DMI_TRTSCONTROL_ndsTimestampSelect_MASK;
	ts_ctrl_reg |=
		((nds_timestam_select << DMI_TRTSCONTROL_ndsTimestampSelect_SHIFT) & DMI_TRTSCONTROL_ndsTimestampSelect_MASK);

	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTSCONTROL), ts_ctrl_reg);
	component_read(target, NDSV5_COMP_NCETENC, &ts_ctrl_reg, selected_encoder(DMI_TRTSCONTROL));
	LOG_DEBUG("DBG_API: after setting %x", ts_ctrl_reg);
	return 0;
}

static uint32_t tracer_enable_itracing(struct target *target)
{
	uint32_t te_ctrl_reg;

	LOG_DEBUG("DBG_API:enable instruction trace for encoder%d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));

	te_ctrl_reg |= DMI_TRTECONTROL_teInstTracing;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_disable_itracing(struct target *target)
{
	uint32_t te_ctrl_reg;

	LOG_DEBUG("DBG_API:disable instruction trace for encoder%d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &te_ctrl_reg, selected_encoder(DMI_TRTECONTROL));

	te_ctrl_reg &= ~DMI_TRTECONTROL_teInstTracing;
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTECONTROL), te_ctrl_reg);
	return 0;
}

static uint32_t tracer_reset_timestamp(struct target *target)
{
	uint32_t ts_ctrl_reg;

	LOG_DEBUG("DBG_API:reset timestamp for encoder %d", trace_sel);
	component_read(target, NDSV5_COMP_NCETENC, &ts_ctrl_reg, selected_encoder(DMI_TRTSCONTROL));
	ts_ctrl_reg |= (DMI_TRTSCONTROL_tsReset);
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_TRTSCONTROL), ts_ctrl_reg);
	return 0;
}

static int tracer_set_atbid(struct target *target, uint32_t atbid)
{
	/* WARNING: For old trace sub-system, we need to setting this!! */
	uint32_t    atb_ctrl_reg;
	if ((atbid == 0) || (atbid >= 0x70)) {
		LOG_DEBUG("DBG_API:ERROR:atbID %d is invalid", atbid);
		return -1;
	}
	LOG_DEBUG("set atbid: %d", atbid);
	component_read(target, NDSV5_COMP_NCETENC, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	atb_ctrl_reg &= ~DMI_ATBCONTROL_atbId_MASK;
	atb_ctrl_reg |= ((atbid << DMI_ATBCONTROL_atbId_SHIFT) & DMI_ATBCONTROL_atbId_MASK);
	component_write(target, NDSV5_COMP_NCETENC, selected_encoder(DMI_ATBCONTROL), atb_ctrl_reg);
	LOG_DEBUG("Set DMI_ATBCONTROL 0x:%x", atb_ctrl_reg);

	component_read(target, NDSV5_COMP_NCETENC, &atb_ctrl_reg, selected_encoder(DMI_ATBCONTROL));
	LOG_DEBUG("Check DMI_ATBCONTROL 0x:%x", atb_ctrl_reg);
	return 0;
}

static uint32_t tracer_activate_tbuf(struct target *target)
{
	uint32_t tf_ctrl_reg;
	uint32_t timeout_limit;
	uint32_t timeout_counter;
	uint32_t ram_sink = 0;

	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		LOG_DEBUG("DBG_API:activate RAM sink");
		ram_sink = DMI_TRRAMCONTROL;
	} else if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB) {
		LOG_DEBUG("DBG_API:active AICE2T sink");

		/* The trace_freq is stalled to point to an array of unsigned integers as follows:
		 * [0]: Trace size
		 * [1]: Stop on wrap (1: enable, 0: disable)
		 */
		unsigned int trace_freq[2];
		trace_freq[0] = ndsv5_trace_ram_size;
		trace_freq[1] = nds_tracer_stop_on_wrap;
		adapter_config_trace(true, 0, 0, (unsigned int *)&trace_freq, 0, NULL);
		LOG_DEBUG("DBG_API:activate PIB sink");
		ram_sink = selected_tbuf_sink(target, DMI_TRPIBCONTROL);
	}

	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
	tf_ctrl_reg |= DMI_TRRAMCONTROL_atbActive;
	component_write(target, NDSV5_COMP_NCETBUF, ram_sink, tf_ctrl_reg);

	timeout_limit   = 50;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
		if (tf_ctrl_reg & DMI_TRRAMCONTROL_atbActive)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfActive to be set");

	return 0;
}

static uint32_t tracer_deactivate_tbuf(struct target *target)
{
	uint32_t tf_ctrl_reg;
	uint32_t timeout_limit;
	uint32_t timeout_counter;

	uint32_t ram_sink = 0;
	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		LOG_DEBUG("DBG_API:deactivate RAM sink");
		ram_sink = DMI_TRRAMCONTROL;
	} else if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB) {
		LOG_DEBUG("DBG_API:deactive AICE2T sink");
		adapter_config_trace(false, 0, 0, NULL, 0, NULL);
		LOG_DEBUG("DBG_API:deactivate PIB sink");
		ram_sink = selected_tbuf_sink(target, DMI_TRPIBCONTROL);
	}


	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
	tf_ctrl_reg &= ~DMI_TRRAMCONTROL_atbActive;
	component_write(target, NDSV5_COMP_NCETBUF, ram_sink, tf_ctrl_reg);

	timeout_limit   = 50;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
		if ((tf_ctrl_reg & DMI_TRRAMCONTROL_atbActive) == 0)
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
	uint32_t teramlimitlow;
	uint64_t nds_trRamLimit;

	LOG_DEBUG("DBG_API:tbuf:enable recording");
	switch (ndsv5_trace_mem_mode) {
		case NDSV5_TRACE_MEM_SRAM:
			/* reset teRamWP before recording */
			/* The trRamWP/trRamRP registers should be initialized to trRamStart */
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPLOW, 0);
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPHIGH, 0);
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPLOW, 0);
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPHIGH, 0);
			/* trRamStart */
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMSTARTLOW, 0);
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMSTARTHIGH, 0);
			/* trRamLimit */
			/* component_read(target, NDSV5_COMP_NCETBUF, &teramlimitlow, DMI_TRRAMLIMITLOW); */
			teramlimitlow = 0xFFC;  /* fixed size for SRAM mode */
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMLIMITLOW, 0xFFC);
			TB_RAM_SIZE = teramlimitlow + 4;
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPLOW, 0);
			timeout_limit   = 100;
			timeout_counter = 0;
			while (timeout_counter < timeout_limit) {
				component_read(target, NDSV5_COMP_NCETBUF, &teWrap, DMI_TRRAMWPLOW);
				if (teWrap == 0)
					break;
				timeout_counter++;
			}
			tracer_tbuf_set_ram_mode(target);
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPLOW, 0x0);
			break;

		case NDSV5_TRACE_MEM_SMEM:
			if (ndsv5_tracer_smem_capability_check(target) != ERROR_OK) {
				LOG_ERROR("Trace SMEM mode not support");
				break;
			}

			tracer_tbuf_set_ram_mode(target);

			/* The trRamWP/trRamRP registers should be initialized to trRamStart */
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPLOW, (nds_trRamStart & 0xFFFFFFFF));
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPHIGH, (nds_trRamStart >> 32));
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPLOW, (nds_trRamStart & 0xFFFFFFFF));
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPHIGH, (nds_trRamStart >> 32));

			/* trRamStart */
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMSTARTLOW, (nds_trRamStart & 0xFFFFFFFF));
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMSTARTHIGH, (nds_trRamStart >> 32));

			/* trRamLimit */
			nds_trRamLimit = nds_trRamStart + ndsv5_trace_ram_size - 1;
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMLIMITLOW, (nds_trRamLimit & 0xFFFFFFFF));
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMLIMITHIGH, (nds_trRamLimit >> 32));

			TB_RAM_SIZE = ndsv5_trace_ram_size;
			break;

		case NDSV5_TRACE_MEM_PIB:
			TB_RAM_SIZE = ndsv5_trace_ram_size * 2; /* Avoid PIB recorded a little bigger than setting */
			break;

		default:
			break;
	};
	LOG_DEBUG("bufsize(etb_wptr) = 0x%x(%d)", TB_RAM_SIZE, TB_RAM_SIZE);
	ndsv5_tracer_buffer_init();

	uint32_t ram_sink = 0;
	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		LOG_DEBUG("DBG_API:enable RAM sink");
		ram_sink = DMI_TRRAMCONTROL;
	} else if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB) {
		LOG_DEBUG("DBG_API:enable PIB sink");
		ram_sink = selected_tbuf_sink(target, DMI_TRPIBCONTROL);
	}

	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
	tf_ctrl_reg |= DMI_TRRAMCONTROL_tfEnable;
	component_write(target, NDSV5_COMP_NCETBUF, ram_sink, tf_ctrl_reg);

	timeout_limit   = 100;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
		if (tf_ctrl_reg & DMI_TRRAMCONTROL_tfEnable)
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

	uint32_t ram_sink = 0;
	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		LOG_DEBUG("DBG_API:disable recording RAM sink");
		ram_sink = DMI_TRRAMCONTROL;
	} else if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB) {
		LOG_DEBUG("DBG_API:disable PIB sink");
		ram_sink = selected_tbuf_sink(target, DMI_TRPIBCONTROL);
	}

	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
	tf_ctrl_reg &= ~DMI_TRRAMCONTROL_tfEnable;
	component_write(target, NDSV5_COMP_NCETBUF, ram_sink, tf_ctrl_reg);

	timeout_limit   = 100;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
		if ((tf_ctrl_reg & DMI_TRRAMCONTROL_tfEnable) == 0)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfEnable to be cleared");

	/* Poll tfControl.tfEmpty in all Trace Funnels. Wait until all are 1 */
	timeout_limit   = 1000;
	timeout_counter = 0;
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, ram_sink);
		if (tf_ctrl_reg & DMI_TRRAMCONTROL_tfEmpty)
			break;
		timeout_counter++;
	}
	if (timeout_counter >= timeout_limit)
		LOG_DEBUG("DBG_API:ERROR:timeout waiting tfEmpty to be set after disabling tbuf");

	return 0;
}

static uint32_t tracer_tbuf_set_trace_format(struct target *target, uint32_t format)
{
	return 0;  /* Should removed this!? */

	uint32_t tf_ctrl_reg;

	LOG_DEBUG("DBG_API:tbuf:recording format is set to %d", format);
	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, DMI_TRRAMCONTROL);
	tf_ctrl_reg &= ~DMI_TRRAMCONTROL_teFormat_MASK;
	tf_ctrl_reg |= ((format << DMI_TRRAMCONTROL_teFormat_SHIFT) & DMI_TRRAMCONTROL_teFormat_MASK);
	component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMCONTROL, tf_ctrl_reg);

	return 0;
}

static uint32_t tracer_tbuf_set_stop_on_wrap(struct target *target, uint32_t mode)
{
	uint32_t tf_ctrl_reg;

	LOG_DEBUG("DBG_API:tbuf:tfStopOnWrap is set to %d", mode);
	component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, DMI_TRRAMCONTROL);
	if (mode)
		tf_ctrl_reg |= DMI_TRRAMCONTROL_tfStopOnWrap;
	else
		tf_ctrl_reg &= ~DMI_TRRAMCONTROL_tfStopOnWrap;
	component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMCONTROL, tf_ctrl_reg);

	return 0;
}

static void tracer_tbuf_set_ram_mode(struct target *target)
{
	uint32_t tf_ctrl_reg;

	LOG_DEBUG("DBG_API:tbuf:trRamModeSMEM is set to %d", ndsv5_trace_mem_mode);

	switch (ndsv5_trace_mem_mode) {
		case NDSV5_TRACE_MEM_SRAM:
			component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, DMI_TRRAMCONTROL);
			tf_ctrl_reg &= ~DMI_TRRAMCONTROL_trRamModeSMEM;
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMCONTROL, tf_ctrl_reg);
			break;
		case NDSV5_TRACE_MEM_SMEM:
			component_read(target, NDSV5_COMP_NCETBUF, &tf_ctrl_reg, DMI_TRRAMCONTROL);
			tf_ctrl_reg |= DMI_TRRAMCONTROL_trRamModeSMEM;
			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMCONTROL, tf_ctrl_reg);
			break;
		default:
			break;

	};
}

static uint32_t tracer_tbuf_get_teWrap(struct target *target)
{
	uint32_t teWrap, ifWrap;

	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		component_read(target, NDSV5_COMP_NCETBUF, &teWrap, DMI_TRRAMWPLOW);
		ifWrap = (teWrap & DMI_TRRAMWPLOW_teWrap);
		return ifWrap;
	} else if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB) {
		size_t nwords = 0;
		adapter_poll_trace(NULL, &nwords);
		LOG_DEBUG("Current nwords: 0x%zx", nwords);
		return ((nwords*4) >= (ndsv5_trace_ram_size-1));
	}
	return 0;
}

static uint32_t tracer_enable_multiplexer(struct target *target)
{
	uint32_t  trFunnel_ctrl_reg, trFunnel_Impl_reg;
	if (nds_tracer_multiplexer == 0) {
		LOG_DEBUG("TMUX not found, skip");
		return 0;
	}

	component_read(target, NDSV5_COMP_NCETMUX, &trFunnel_Impl_reg, DMI_TRFUNNELIMPL);
	LOG_DEBUG("DBG_API:tracer_enable_multiplexer, trFunnelImpl = 0x%x", trFunnel_Impl_reg);


	const uint32_t  timeout_limit = 50; /* magic number waiting to be tuned */
	uint32_t  timeout_counter = 0;

	/* Active Trace Funnel */
	component_read(target, NDSV5_COMP_NCETMUX, &trFunnel_ctrl_reg, DMI_TRFUNNELCONTROL);
	trFunnel_ctrl_reg |= DMI_TRFUNNELCONTROL_trFunnelActive;
	component_write(target, NDSV5_COMP_NCETMUX, DMI_TRFUNNELCONTROL, trFunnel_ctrl_reg);
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETMUX, &trFunnel_ctrl_reg, DMI_TRFUNNELCONTROL);
		if (trFunnel_ctrl_reg & DMI_TRFUNNELCONTROL_trFunnelActive)
			break;
		timeout_counter++;
	}

	/* Enable Trace Funnel */
	trFunnel_ctrl_reg |= DMI_TRFUNNELCONTROL_trFunnelEnable;
	component_write(target, NDSV5_COMP_NCETMUX, DMI_TRFUNNELCONTROL, trFunnel_ctrl_reg);

	/*
	uint32_t  tmux_ctrl_reg;
	component_read(target, NDSV5_COMP_NCETMUX, &tmux_ctrl_reg, TMUX_ITTMUXCTRL);
	LOG_DEBUG("DBG_API: tmux_ctrl_reg 0x%x", tmux_ctrl_reg);
	*/
	return 0;
}

static uint32_t tracer_enable_multiplexer2(struct target *target)
{
	uint32_t  trFunnel_ctrl_reg, trFunnel_Impl_reg;
	const uint32_t  timeout_limit = 50; /* magic number waiting to be tuned */
	uint32_t  timeout_counter = 0;

	/* For Level-2 Trace Multiplexer(TMUX2), check address specified */
	if (ndsv5_comps[NDSV5_COMP_NCETMUX2].addr == APB_TRACEADDRESSUNKNOWN) {
		LOG_DEBUG("Skip Enable Level-2 Trace Multiplexe(TMUX2), Address unspecified");
		return 0;
	}

	/* Check system bus exist */
	if (!nds_sys_bus_supported) {
		LOG_ERROR("System bus non-exist!");
		return -1;
	}

	/* For Level-2 Trace Multiplexer(TMUX2), check DEVARCH.ARCHID[15:0] */
	uint32_t  devarch_reg;
	component_read(target, NDSV5_COMP_NCETMUX2, &devarch_reg, DMI_NCETMUX200_DEVARCH2);
	LOG_DEBUG("TMUX_DEVARCH = 0x%x", devarch_reg);
	if ((devarch_reg & 0xFFFF) == 0x0000) {
		LOG_DEBUG("Skip Enable Level-2 Trace Multiplexe(TMUX2), DEVARCH error");
		return 0;
	}

	/* Read Trace Multiplexer Implementation Register */
	component_read(target, NDSV5_COMP_NCETMUX2, &trFunnel_Impl_reg, DMI_TRFUNNELIMPL2);
	LOG_DEBUG("DBG_API:tracer_enable_multiplexer2(TMUX2), trFunnelImpl = 0x%x", trFunnel_Impl_reg);

	/* Active TMUX2 */
	LOG_DEBUG("Active TMUX2");
	component_read(target, NDSV5_COMP_NCETMUX2, &trFunnel_ctrl_reg, DMI_TRFUNNELCONTROL2);
	trFunnel_ctrl_reg |= DMI_TRFUNNELCONTROL_trFunnelActive;
	component_write(target, NDSV5_COMP_NCETMUX2, DMI_TRFUNNELCONTROL2, trFunnel_ctrl_reg);
	while (timeout_counter < timeout_limit) {
		component_read(target, NDSV5_COMP_NCETMUX2, &trFunnel_ctrl_reg, DMI_TRFUNNELCONTROL2);
		if (trFunnel_ctrl_reg & DMI_TRFUNNELCONTROL_trFunnelActive)
			break;
		timeout_counter++;
	}
	LOG_DEBUG("Active TMUX2 Done");

	/* Enable TMUX2 */
	LOG_DEBUG("Enable TMUX2");
	trFunnel_ctrl_reg |= DMI_TRFUNNELCONTROL_trFunnelEnable;
	component_write(target, NDSV5_COMP_NCETMUX2, DMI_TRFUNNELCONTROL2, trFunnel_ctrl_reg);
	LOG_DEBUG("Enable TMUX2 Done");

	return 0;
}

static int ndsv5_tracer_buffer_init(void)
{
	if (p_etb_buf_start) {
		free(p_etb_buf_start);
		p_etb_buf_start = NULL;
	}
	LOG_DEBUG("Allocate buffer %d bytes", TB_RAM_SIZE*2);
	p_etb_buf_start = (uint32_t *)malloc(TB_RAM_SIZE*2);
	p_etb_buf_end = p_etb_buf_start;
	p_etb_buf_end += (TB_RAM_SIZE >> 2);

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

uint32_t ndsv5_tracer_setting(struct target *target, bool setbuffer)
{
	if (target->trace_on) {
		LOG_DEBUG("Target has been trun-on tracing, skip!");
		return 0;
	}

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	/* Activate trace encoder and set register */
	tracer_activate_encoder(target);

	/* Set teInstMode to 3, ndsTracePRIV to 0, ndsTracePID to 0, ndsSyncExtra to 0,
		teInstTrigEn to 1, teInstStallEn to 0, teInstEnCallStack to 0 */
	tracer_set_itracing_mode(target, 0, nds_teInstNoAddrDiff, nds_teInhibitSrc, 0, 1, 1, 1, 0, nds_trTeInstMode);

	if (nds_tracer_action == CSR_MCONTROL_ACTION_TRACE_OFF)
		tracer_enable_itracing(target);

	/* Set teSyncMode to 1, teSyncMax to 4 */
	tracer_set_sync_mode(target, nds_teSyncMode, nds_trTeSyncMax);

	if (nds_trTeFilteriMatchInst || nds_trTeFilterMatchValueContext) {
		tracer_set_filter(target,
				nds_trTeFilteriMatchInst,
				nds_trTeFilterMatchValueContext,
				nds_trTeFilterMatchMaskContext);
	}

	tracer_reset_timestamp(target);

	if (nds_timestamp_on) {
		uint32_t tsDebug, tsType, tsPrescale, tsSelect;
		if (nds_trTsControl == 0) {
			/* Activate timestamp and enable internal timestamp counter, set tsDebug to 0,
			   tsType to 1(externel), tsPrescale to 0, ndsTimestampSelect to 1(trTsEnable) */
			tsDebug = 0;
			tsType = 1;  /* External */
			tsPrescale = 0;
			tsSelect = 1;
		} else {
			if (nds_trTsControl & DMI_TRTSCONTROL_tsDebug)
				tsDebug = 1;
			else
				tsDebug = 0;
			tsType = (nds_trTsControl & DMI_TRTSCONTROL_tsType_MASK)
				>> DMI_TRTSCONTROL_tsType_SHIFT;
			tsPrescale = (nds_trTsControl & DMI_TRTSCONTROL_tsPrescale_MASK)
				>> DMI_TRTSCONTROL_tsPrescale_SHIFT;
			tsSelect = (nds_trTsControl & DMI_TRTSCONTROL_ndsTimestampSelect_MASK)
				>> DMI_TRTSCONTROL_ndsTimestampSelect_SHIFT;
		}
		tracer_set_timestamp_mode(target, 1, 1, tsDebug, tsType, tsPrescale, tsSelect);
	}

	/* WARNING: For old trace subsystem, we need setting this!! */
	if (!nds_trace_new_bitmap)
		tracer_set_atbid(target, 1);

	if (setbuffer == true) {
		/* Activate trace buffer and set its register */
		tracer_activate_tbuf(target);
		tracer_tbuf_set_trace_format(target, 1);
		if (nds_tracer_stop_on_wrap == 0)
			tracer_tbuf_set_stop_on_wrap(target, 0);
		else
			tracer_tbuf_set_stop_on_wrap(target, 1);

		/* Enable trace buffer for recording */
		tracer_tbuf_enable_recording(target);
	}

	/* Enable trace encoder and wait for trace-on event */
	tracer_enable_encoder(target);

	target->trace_on = true;
	return 0;
}

uint32_t ndsv5_tracer_all_cores_setting(void)
{
	if (nds_tracer_on != 0)
		return 0;

	struct target *target = all_targets;
	uint32_t coreid;
	if (nds_tracer_capability == 0)
		ndsv5_tracer_capability_check(target);

	if (nds_tracer_capability != 1) {
		LOG_DEBUG("TB_DEVARCH != 0x4200");
		return 0;
	}

	for (target = all_targets; target; target = target->next) {
		if (target->trace_on) {
			LOG_DEBUG("Target has been trun-on tracing, skip!");
			continue;
		}

		if (target->smp) {
			struct target_list *tlist;
			foreach_smp_target(tlist, target->smp_targets) {
				struct target *t = tlist->target;
				riscv_info_t *r = riscv_info(t);
				coreid = r->current_hartid;
				LOG_DEBUG("smp-coreid: %d ", coreid);
				if (((0x01 << coreid) & nds_tracer_active_id) != 0) {
					tracer_enable_multiplexer(t);
					tracer_enable_multiplexer2(t);
					ndsv5_tracer_setting(t, true);
				}
			}
		} else {
			RISCV_INFO(r);
			coreid = r->current_hartid;
			LOG_DEBUG("amp-coreid: %d ", coreid);
			if (((0x01 << coreid) & nds_tracer_active_id) != 0) {
				tracer_enable_multiplexer(target);
				tracer_enable_multiplexer2(target);
				ndsv5_tracer_setting(target, true);
			}
		}
	}

	nds_tracer_on = 1;
	return 0;
}

static uint32_t nds_tracer_active_id_current; /* avoid using nds_tracer_active_id */
uint32_t ndsv5_tracer_setting_current(struct target *target)
{
	LOG_DEBUG("nds_tracer_active_id_current: 0x%x", nds_tracer_active_id_current);

	if (nds_tracer_capability == 0)
		ndsv5_tracer_capability_check(target);

	if (nds_tracer_capability != 1) {
		LOG_DEBUG("TB_DEVARCH != 0x4200");
		return 0;
	}

	/* First trace, enable buffer */
	if (!nds_tracer_active_id_current) {
		tracer_enable_multiplexer(target);
		tracer_enable_multiplexer2(target);

		/* Activate trace buffer and set its register */
		tracer_activate_tbuf(target);
		tracer_tbuf_set_trace_format(target, 1);
		if (nds_tracer_stop_on_wrap == 0)
			tracer_tbuf_set_stop_on_wrap(target, 0);
		else
			tracer_tbuf_set_stop_on_wrap(target, 1);

		/* Enable trace buffer for recording */
		tracer_tbuf_enable_recording(target);
	}

	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_set_current_hartid(target, t->coreid);
			uint32_t coreid = t->coreid;
			LOG_DEBUG("smp-coreid: %d ", coreid);
			nds_tracer_active_id_current |= (0x01 << coreid);
			ndsv5_tracer_setting(t, false);
		}
	} else {
		uint32_t coreid = target->coreid;
		LOG_DEBUG("amp-coreid: %d ", coreid);
		nds_tracer_active_id_current |= (0x01 << coreid);
		ndsv5_tracer_setting(target, false);
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

uint32_t tracer_disable_encoder_current(struct target *target)
{
	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_set_current_hartid(t, t->coreid);
			tracer_disable_encoder(t);
		}
	} else {
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

uint32_t tracer_deactivate_encoder_current(struct target *target)
{
	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			riscv_set_current_hartid(t, t->coreid);
			tracer_deactivate_encoder(t);
		}
	} else {
		tracer_deactivate_encoder(target);
	}
	return 0;
}

uint32_t ndsv5_tracer_disable_all(struct target *target)
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

uint32_t ndsv5_tracer_disable_current(struct target *target)
{
	riscv_set_current_hartid(target, target->coreid);

	RISCV_INFO(r);
	tracer_select_hart(r->current_hartid);

	/* Disable instruction tracing */
	tracer_disable_itracing(target);

	/* Disable trace encoder */
	tracer_disable_encoder_current(target);

	/* Deactivate trace encoder and trace buffer */
	tracer_deactivate_encoder_current(target);

	/* Remove from active list */
	if (target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			uint32_t coreid = t->coreid;
			nds_tracer_active_id_current &= ~(0x01 << coreid);
		}
	} else {
		uint32_t coreid = target->coreid;
		nds_tracer_active_id_current &= ~(0x01 << coreid);
	}

	/* Clean buffer if all target free */
	if (!nds_tracer_active_id_current) {
		tracer_deactivate_tbuf(target);
		ndsv5_tracer_buffer_free();
		nds_tracer_on = 0;
	}

	return 0;
}

static int ndsv5_tracer_read_etb(struct target *target)
{
	uint32_t etb_rptr, etb_wptr, etb_data;
	uint32_t fifo_words = 0;
	uint32_t i;
	size_t nwords = 0;

	/* Disable instruction tracing */
	tracer_disable_itracing(target);

	/* Disable trace encoder and trace buffer */
	tracer_disable_encoder_all();
	tracer_tbuf_disable_recording(target);

	nds_tracer_on = 0;

	if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM ||
	    ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SMEM) {
		component_read(target, NDSV5_COMP_NCETBUF, &etb_wptr, DMI_TRRAMWPLOW);
		component_read(target, NDSV5_COMP_NCETBUF, &etb_rptr, DMI_TRRAMRPLOW);
		LOG_DEBUG("1.etb_wptr = 0x%x, etb_rptr = 0x%x", etb_wptr, etb_rptr);

		if (etb_wptr & DMI_TRRAMWPLOW_teWrap) {
			LOG_DEBUG("teWrap");
			etb_wptr &= ~DMI_TRRAMWPLOW_teWrap;
			etb_rptr = etb_wptr;
			fifo_words = TB_RAM_SIZE;
		} else {
			switch (ndsv5_trace_mem_mode) {
				case NDSV5_TRACE_MEM_SRAM:
					etb_rptr = 0;
					fifo_words = etb_wptr;
					break;

				case NDSV5_TRACE_MEM_SMEM:
					fifo_words = etb_wptr - etb_rptr;
					break;

				default:
					break;
			};
		}
		fifo_words >>= 2;
		component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMRPLOW, etb_rptr);
		component_read(target, NDSV5_COMP_NCETBUF, &etb_wptr, DMI_TRRAMWPLOW);
		component_read(target, NDSV5_COMP_NCETBUF, &etb_rptr, DMI_TRRAMRPLOW);
		LOG_DEBUG("2.etb_wptr = 0x%x, etb_rptr = 0x%x", etb_wptr, etb_rptr);

		/* copy pkt from ETB to tmp buffer */
		uint32_t *pcurr_wptr = (uint32_t *)p_etb_wptr;
		pcurr_wptr += fifo_words;
		LOG_DEBUG("p_etb_wptr = 0x%lx, p_etb_buf_end = 0x%lx, fifo_words = 0x%x",
				(unsigned long)p_etb_wptr, (unsigned long)p_etb_buf_end, fifo_words);
	}

	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	uint32_t bak_nds_va_to_pa_off = nds32->nds_va_to_pa_off;
	enum nds_memory_access orig_channel = nds32->memory.access_channel;
	switch (ndsv5_trace_mem_mode) {
		case NDSV5_TRACE_MEM_SRAM:
			for (i = 0; i < fifo_words; i++) {
				component_read(target, NDSV5_COMP_NCETBUF, &etb_data, DMI_TRRAMDATA);
				*p_etb_wptr++ = etb_data;
			}
			component_read(target, NDSV5_COMP_NCETBUF, &etb_wptr, DMI_TRRAMWPLOW);
			component_read(target, NDSV5_COMP_NCETBUF, &etb_rptr, DMI_TRRAMRPLOW);

			LOG_DEBUG("3.etb_wptr = 0x%x, etb_rptr = 0x%x, fifo_words = 0x%x", etb_wptr, etb_rptr, fifo_words);

			component_write(target, NDSV5_COMP_NCETBUF, DMI_TRRAMWPLOW, 0x0);
			break;

		case NDSV5_TRACE_MEM_SMEM:
			nds32->nds_va_to_pa_off = 1;

			if (nds_sys_bus_supported)
				nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;
			else {
				LOG_DEBUG("nds_sys_bus_supported not support, switch to CPU mode");
				nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;
			}
			target_read_buffer(target, etb_rptr, fifo_words * 4, (uint8_t *)p_etb_buf_start);
			p_etb_wptr += fifo_words * 4;
			nds32->nds_va_to_pa_off = bak_nds_va_to_pa_off;
			nds32->memory.access_channel = orig_channel;
			break;

		case NDSV5_TRACE_MEM_PIB:
			jtag_poll_set_enabled(false);
			adapter_poll_trace(NULL, &nwords);
			LOG_DEBUG("Dump from PIB %zx words", nwords);
			p_etb_wptr += nwords * 4;
			adapter_poll_trace((uint8_t *)p_etb_buf_start, &nwords);
			jtag_poll_set_enabled(true);
			break;

		default:
			break;
	};

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

	/* Read satp */
	uint64_t satp = 0;
	if (target->reg_cache->reg_list[GDB_REGNO_CSR0 + CSR_SATP].exist == true) {
		if (riscv_get_register(target, &satp, GDB_REGNO_SATP) != ERROR_OK)
			satp = 0;
	}

	/* Header format:
	 * v1 (legacy): [srcbits][teInstNoAddrDiff][VALEN][version]
	 * v2 (current): [len0][len1][len2][version][TLVs...]
	 */
	struct ndsv5_tracer_header header;
	unsigned char header_buf[64];
	size_t header_bytes = 0, idx;

	header.version = TRACER_VERSION;
	if (nds_teInhibitSrc == 0)
		header.srcbits = 6;
	else
		header.srcbits = 0;
	header.inst_no_addr_diff = (unsigned char)nds_teInstNoAddrDiff;
	header.ts_comp_mode = (unsigned char)ndsv5_TsCompMode;

	unsigned xlen = riscv_xlen(target);
	int mode = get_field(satp, RISCV_SATP_MODE(xlen));
	LOG_DEBUG("satp 0x%" PRIx64 ", xlen: %d, mode: %d", satp, xlen, mode);
	switch (mode) {
	case SATP_MODE_SV32:
		header.valen = 32;
		break;
	case SATP_MODE_SV39:
		header.valen = 39;
		break;
	case SATP_MODE_SV48:
		header.valen = 48;
		break;
	case SATP_MODE_SV57:
		header.valen = 57;
		break;
	case SATP_MODE_SV64:
		header.valen = 64;
		break;
	default:
		header.valen = (unsigned char)xlen;
		break;
	}

	if (ndsv5_tracer_build_header_v2(&header, header_buf, sizeof(header_buf), &header_bytes) != ERROR_OK) {
		LOG_ERROR("Failed to build tracer header");
		fclose(pPacketFile);
		return ERROR_FAIL;
	}

	/* Write parameters into file */
	for (idx = 0; idx < header_bytes; idx++) {
		fputc(header_buf[idx], pPacketFile);
		LOG_DEBUG("header[%lu] = 0x%x", (unsigned long)idx, header_buf[idx]);
	}

	char *pbuf_start = (char *)p_etb_buf_start;
	/* get data from ETB */
	ndsv5_tracer_read_etb(target);

	if (p_etb_wptr != p_etb_buf_start) {
		total_pkt_bytes = (p_etb_wptr - p_etb_buf_start);
		if (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_SRAM)
			total_pkt_bytes <<= 2;
	}
	LOG_DEBUG("p_etb_wptr = 0x%p, total_pkt_bytes = %d(0x%x)",
		p_etb_wptr, total_pkt_bytes, total_pkt_bytes);

	if (total_pkt_bytes) {
		if (0 && (ndsv5_trace_mem_mode == NDSV5_TRACE_MEM_PIB)) {
			/* Strange HW bug!? Skip pkt first word */
			/* ASTv540 update, temporary removed HW workaround */
			fwrite(pbuf_start+4, 1, total_pkt_bytes-4, pPacketFile);
		} else {
			fwrite(pbuf_start, 1, total_pkt_bytes, pPacketFile);
		}
	} else
		LOG_DEBUG("Buffer empty!!");

	fclose(pPacketFile);

	/* copy another ntracer.log */
	char log_filename[256], write_line[32];
	char *pLogName = (char *)&log_filename[0];
	memcpy(&log_filename[0], pFileName, strlen((char *)pFileName));
	memcpy(&log_filename[strlen((char *)pFileName)], ".log", 5);
	pPacketFile = fopen(pLogName, "wb");

	uint32_t value;
	/* Write header into file (word by word). */
	for (idx = 0; idx < header_bytes; idx += sizeof(uint32_t)) {
		memcpy(&value, &header_buf[idx], sizeof(uint32_t));
		sprintf(write_line, "%08x\n", value);
		fwrite(write_line, 1, sizeof(write_line), pPacketFile);
	}

	/* Write packets into file */
	uint32_t i;
	for (i = 0; i < total_pkt_bytes; i += 4) {
		memcpy(&value, pbuf_start + i, sizeof(uint32_t));
		sprintf(write_line, "%08x\n", value);
		fwrite(write_line, 1, sizeof(write_line), pPacketFile);
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
	/*
	 * After April 24, 2024, the RTL design underwent changes,
	 * relocating the address space of CoreSight registers from 0x1E00-0x1FFF to 0xE00 - 0xFFF
	 * for three IPs (NCETENC200, NCETBUF200, and NCETMUX200).
	 * Consequently, the codes for checking registers CIDR0-3 (0x1FF0-0x1FFC -> 0x0FF0-0x0FFC) and
	 * DEVARCH (0x1FBC-> 0x0FBC) need to be modified accordingly.
	 *
	 * The checking procedure needs to be altered as follows:
	 * If the expected data for CIDR0-3 or DEVARCH of encoder0 is obtained in the old address space
	 *     Return OLD_Bitmap
	 * Else
	 *     If all zeros are obtained for CIDR0-3 or DEVARCH of encoder0 in the old address space,
	 *         If the expected data for the CSRs of encoder0 is obtained in the new address space
	 *             Return NEW_bitmap,
	 *         Else
	 *             return ERROR
	 *     Else
	 *         return ERROR
	 *
	 * */

	uint32_t trace_sel_bak = trace_sel;
	uint32_t devarch_reg = 0x0;
	nds_trace_new_bitmap = false;
	trace_sel = 0; /* Examine Encoder#0 only */

	/* Check OLD VERSION bitmap if use DMI bus */
	if (ndsv5_comps[NDSV5_COMP_NCETENC].addr_type == NDSV5_COMP_ADDR_DMI) {
		component_read(target, NDSV5_COMP_NCETENC, &devarch_reg, selected_encoder(DMI_DEVARCH));
		LOG_DEBUG("DMI_DEVARCH = 0x%x", devarch_reg);
	}

	/* Read on NEW bitmap */
	if (devarch_reg == 0x0) {
		nds_trace_new_bitmap = true;
		LOG_DEBUG("Maybe new bitmap");

		component_read(target, NDSV5_COMP_NCETENC, &devarch_reg, selected_encoder(DMI_DEVARCH_NEW));
		LOG_DEBUG("TENC_DEVARCH = 0x%x", devarch_reg);
	}

	/* Restore trace_sel */
	trace_sel = trace_sel_bak;

	if ((devarch_reg & 0xFFFF) != 0x4500) {
		LOG_DEBUG("TENC Failed in detection, abort");
		nds_tracer_capability = 0xFF;
		return ERROR_FAIL;
	} else {
		nds_tracer_capability = 1;

		/* Detect TMUX */
		component_read(target, NDSV5_COMP_NCETMUX, &devarch_reg, DMI_NCETMUX200_DEVARCH);
		LOG_DEBUG("TMUX_DEVARCH = 0x%x", devarch_reg);
		if ((devarch_reg & 0xFFFF) == 0x4D00)
			nds_tracer_multiplexer = 1;
		else
			LOG_DEBUG("Maybe TMUX not exist, abort");

		return ERROR_OK;
	}

	nds_tracer_capability = 0xFF;
	return ERROR_FAIL;
}

int ndsv5_tracer_smem_capability_check(struct target *target)
{
	static int is_supported = -1;

	if (is_supported == -1) {
		uint32_t  tf_info_reg;
		component_read(target, NDSV5_COMP_NCETBUF, &tf_info_reg, DMI_TRRAMIMPL);
		LOG_DEBUG("DMI_TRRAMIMPL: 0x%x", tf_info_reg);

		/* Check trRamHasSMEM */
		if ((tf_info_reg >> 13) & 0x1)
			is_supported = 1;
		else
			is_supported = 0;
	}

	return is_supported ? ERROR_OK : ERROR_FAIL;
}

int ndsv5_tracer_pib_capability_check(struct target *target)
{
	static int is_supported = -1;

	if (ndsv5_mpsse_t2 == (uint32_t) -1 ||
	    ndsv5_mpsse_t2 == 0) {
		LOG_DEBUG("Not AICE-2T, unsuppport!");
		is_supported = 0;
	}

	if (is_supported == -1) {
		uint32_t  tf_info_reg;
		component_read(target, NDSV5_COMP_NCETBUF, &tf_info_reg, DMI_TRRAMIMPL);
		LOG_DEBUG("DMI_TRRAMIMPL: 0x%x", tf_info_reg);

		/* Check trRamCompType */
		uint8_t trRamCompType = (tf_info_reg >> 8) & 0xF;
		if (trRamCompType != 10) {
			/* Maybe offset 0x1000 */
			component_read(target, NDSV5_COMP_NCETBUF, &tf_info_reg, DMI_TRRAMIMPL+DMI_TB_PIBSINKBASE_SHIFT);
			LOG_DEBUG("DMI_TRRAMIMPL(offset): 0x%x", tf_info_reg);
			trRamCompType = (tf_info_reg >> 8) & 0xF;
		}

		if (trRamCompType == 10)
			is_supported = 1;
		else
			is_supported = 0;
	}

	return is_supported ? ERROR_OK : ERROR_FAIL;
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
	unsigned long buffer_size = TB_RAM_SIZE;
	unsigned long read_size = 0, decoded_bytes = 0;
	unsigned int cur_idx = 0, i;
	unsigned char cur_data;
	unsigned char *pPktBuf, *curr_buf;
	char *pOutFileName = (char *)&pkt_decoded_filename[0];
	FILE *pPacketFile = NULL;
	char *p_text = &pkt_decoded_data[0];
	char tmp_text[256];
	char *p_tmp_text = &tmp_text[0];
	int ret_code = 0;
	size_t header_bytes = 0;
	struct ndsv5_tracer_header header;

	memcpy(&pkt_decoded_filename[0], pPktFileName, strlen((char *)pPktFileName));
	memcpy(&pkt_decoded_filename[strlen((char *)pPktFileName)], "-de", 4);
	pPacketFile = fopen(pPktFileName, "rb");
	gpPktDecodedFile = fopen(pOutFileName, "wb");
	pPktBuf = (unsigned char *)malloc(TB_RAM_SIZE);
	if (pPktBuf == NULL) {
		TRACER_DECODE_MSG(("ERROR!! packet buffer !!"));
		return -1;
	}

	curr_buf = pPktBuf;
	if (pPacketFile) {
		char *ret = strstr(pPktFileName, ".log");
		if (ret)
			/* get data from .log file */
			read_size = ndsv5_tracer_read_logfile(curr_buf, pPacketFile);
		else
			/* get data from raw-pkt file */
			read_size = fread(curr_buf, 1, buffer_size, pPacketFile);

		/* Header format:
		 * v1: [srcbits][teInstNoAddrDiff][VALEN][version]
		 * v2: [len0][len1][len2][version][TLVs...]
		 */
		if (ndsv5_tracer_parse_header(curr_buf, read_size, &header, &header_bytes) != ERROR_OK) {
			LOG_ERROR("Failed to parse tracer header");
			ret_code = -1;
			goto out;
		}

		teSrcBits = header.srcbits;
		teInstNoAddrDiff = header.inst_no_addr_diff;
		ndsv5_TsCompMode = header.ts_comp_mode;

		LOG_DEBUG("Tracer header version=%u header_bytes=%lu srcbits=%u teInstNoAddrDiff=%u VALEN=%u TsCompMode=%u",
			header.version, (unsigned long)header_bytes,
			header.srcbits, header.inst_no_addr_diff, header.valen,
			header.ts_comp_mode);

		curr_buf += header_bytes;
		read_size -= (unsigned long)header_bytes;

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
				ret_code = -1;
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

out:
	if (pPacketFile)
		fclose(pPacketFile);
	if (gpPktDecodedFile)
		fclose(gpPktDecodedFile);
	free(pPktBuf);

	return ret_code;
}
