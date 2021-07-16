#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "riscv.h"
#include "debug_defines.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

static void dump_field(const struct scan_field *field);

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans, size_t idle)
{
scans += 4;
#if _NDS_RW_MEMORY_64BITS_
scans += 2;
#endif
struct riscv_batch *out = malloc(sizeof(*out));
memset(out, 0, sizeof(*out));
out->target = target;
out->allocated_scans = scans;
out->used_scans = 0;
out->idle_count = idle;
out->data_out = malloc(sizeof(*out->data_out) * (scans) * sizeof(uint64_t));
out->data_in  = malloc(sizeof(*out->data_in)  * (scans) * sizeof(uint64_t));
out->fields = malloc(sizeof(*out->fields) * (scans));
out->last_scan = RISCV_SCAN_TYPE_INVALID;
out->read_keys = malloc(sizeof(*out->read_keys) * (scans));
out->read_keys_used = 0;
return out;
}

void riscv_batch_free(struct riscv_batch *batch)
{
free(batch->data_in);
free(batch->data_out);
free(batch->fields);
free(batch->read_keys);
free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
#if _NDS_RW_MEMORY_64BITS_
return batch->used_scans > (batch->allocated_scans - 6);
#endif
return batch->used_scans > (batch->allocated_scans - 4);
}

int riscv_batch_run(struct riscv_batch *batch)
{
	if (batch->used_scans == 0) {
		LOG_DEBUG("Ignoring empty batch.");
		return ERROR_OK;
	}

	keep_alive();

	LOG_DEBUG("running a batch of %ld scans", (long)batch->used_scans);
	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);
		if (batch->idle_count > 0)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	LOG_DEBUG("executing queue");
	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("Unable to execute JTAG queue");
#if _NDS_DISABLE_ABORT_
		NDS32_LOG("Unable to execute JTAG queue");
#endif
		return ERROR_FAIL;
	}

#if _NDS_V5_ONLY_
	struct scan_field *field;
	uint64_t in;
	unsigned int in_op;
	for (size_t i = 0; i < batch->used_scans; ++i) {
		field = batch->fields + i;
		if (field->in_value) {
			in = buf_get_u64(field->in_value, 0, field->num_bits);
			in_op = get_field(in, DTM_DMI_OP);
			if (in_op == 3) {
				// if (status == DMI_STATUS_BUSY)
				LOG_DEBUG("in_op = 0x%x, DMI_STATUS_BUSY", in_op);
				batch->used_scans --; // remove riscv_batch_add_nop()
				return ERROR_FAIL;
			}
		}
		dump_field(batch->fields + i);
	}
	return ERROR_OK;
#else
	for (size_t i = 0; i < batch->used_scans; ++i)
		dump_field(batch->fields + i);
	return ERROR_OK;
#endif
}

void riscv_batch_add_dmi_write(struct riscv_batch *batch, unsigned address, uint64_t data)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_write_u64(batch->target, (char *)field->out_value, address, data);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	batch->used_scans++;
}

size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, unsigned address)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_read_u64(batch->target, (char *)field->out_value, address);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	batch->used_scans++;

	/* FIXME We get the read response back on the next scan.  For now I'm
	 * just sticking a NOP in there, but this should be coelesced away. */
#if _NDS_JTAG_SCANS_OPTIMIZE_
	if (nds_jtag_scans_optimize == 0) {
		riscv_batch_add_nop(batch);
		batch->read_keys[batch->read_keys_used] = batch->used_scans - 1;
	} else {
		batch->read_keys[batch->read_keys_used] = batch->used_scans;
	}
	LOG_DEBUG("batch->read_keys_used=0x%x, batch->read_keys[] 0x%x", batch->read_keys_used, batch->read_keys[batch->read_keys_used]);
#else
	riscv_batch_add_nop(batch);
	batch->read_keys[batch->read_keys_used] = batch->used_scans - 1;
#endif
	LOG_DEBUG("read key %u for batch 0x%p is %u (0x%p)",
			(unsigned) batch->read_keys_used, batch, (unsigned) (batch->used_scans - 1),
			batch->data_in + sizeof(uint64_t) * (batch->used_scans + 1));
	return batch->read_keys_used++;
}

uint64_t riscv_batch_get_dmi_read(struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t *base = batch->data_in + 8 * index;
	return base[0] |
		((uint64_t) base[1]) << 8 |
		((uint64_t) base[2]) << 16 |
		((uint64_t) base[3]) << 24 |
		((uint64_t) base[4]) << 32 |
		((uint64_t) base[5]) << 40 |
		((uint64_t) base[6]) << 48 |
		((uint64_t) base[7]) << 56;
}

void riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->out_value);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	batch->used_scans++;
	LOG_DEBUG("  added NOP with in_value=%p", field->in_value);
}

#if _NDS_V5_ONLY_
void dump_field(const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value != NULL);
	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;
	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;
	unsigned int op_address = in_address;

	char in_text[500];
	in_text[0] = 0x0;
	decode_dmi(in_text, in_address, in_data);
	char append_text[500];
	append_text[0] = 0x0;
	if (in_address == DMI_COMMAND) {
		decode_dmi(append_text, DMI_COMMAND_AC_ACCESS_REGISTER, in_data);
	} else if ((in_address >= DMI_PROGBUF0) && (in_address <= DMI_PROGBUF15)) {
		ndsv5_decode_progbuf(append_text, in_data);
	}
	if (field->in_value) {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__,
				"%db %s %08x @%02x -> %s %08x @%02x %s:%s %s",
				field->num_bits,
				op_string[out_op], out_data, out_address,
				status_string[in_op], in_data, in_address, dmi_reg_string[op_address], in_text, append_text);
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__, "%db %s %08x @%02x -> ?",
				field->num_bits, op_string[out_op], out_data, out_address);
	}
}

#else
void dump_field(const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value != NULL);
	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
		unsigned int in_op = get_field(in, DTM_DMI_OP);
		unsigned int in_data = get_field(in, DTM_DMI_DATA);
		unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__,
				"%db %s %08x @%02x -> %s %08x @%02x [0x%p -> 0x%p]",
				field->num_bits,
				op_string[out_op], out_data, out_address,
				status_string[in_op], in_data, in_address,
				field->out_value, field->in_value);
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__, "%db %s %08x @%02x -> ?",
				field->num_bits, op_string[out_op], out_data, out_address);
	}
}
#endif
