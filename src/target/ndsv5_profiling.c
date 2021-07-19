/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "log.h"
#include "binarybuffer.h"
#include "riscv/riscv.h"
#include "riscv/ndsv5.h"
#include "register.h"

#define NDS32_PROFILE_SAMPLES_PER_S   100/*(2)*/
#define NDS32_PROFILE_PROBE_MS        (1000/NDS32_PROFILE_SAMPLES_PER_S)

uint32_t nds_bak_polling_period;
extern int polling_period;

typedef unsigned char UNIT[2];  /* unit of profiling */
static uint32_t gmon_big_endian;

static void writeData(FILE *f, const void *data, size_t len)
{
	size_t written = fwrite(data, 1, len, f);
	if (written != len)
		LOG_ERROR("failed to write %zu bytes: %s", len, strerror(errno));
}

static void writeLong(FILE *f, int l)
{
	int i;
	char c;

	if (gmon_big_endian == 1) {
		for (i = 3; i >= 0; i--) {
			c = (l >> (i*8))&0xff;
			writeData(f, &c, 1);
		}
	} else {
		for (i = 0; i < 4; i++) {
			c = (l >> (i*8))&0xff;
			writeData(f, &c, 1);
		}
	}
}

static void writeString(FILE *f, char *s)
{
	writeData(f, s, strlen(s));
}

static int write_gmon_0(struct nds32_v5 *nds32)
{
	/* figure out bucket size */
	uint64_t addr_min;
	uint64_t addr_max;
	if (nds32->prof_with_range) {
		addr_min = nds32->prof_addr_min;
		addr_max = nds32->prof_addr_max;
	} else {
		LOG_ERROR("Profile must have address range!");
		addr_min = 0;
		addr_max = 1u << 30; /* 1G */
	}

	uint32_t addressSpace = (uint32_t)(addr_max - addr_min);
	assert(addressSpace >= 2);
	/* FIXME: What is the reasonable number of buckets?
	 * The profiling result will be more accurate if there are enough buckets. */
	uint32_t numBuckets = (uint32_t) (addressSpace >> 1);

	if (numBuckets > 0x200000)
		numBuckets = 0x200000;

	LOG_DEBUG("addressSpace:0x%x, numBuckets:0x%x ", addressSpace, numBuckets);
	nds32->prof_num_buckets = (uint32_t)numBuckets;
	nds32->prof_total_samples = 0;
	/* TODO: not malloc every time */
	int *buckets = malloc(sizeof(int) * numBuckets);
	if (buckets == NULL)
		return ERROR_FAIL;

	memset(buckets, 0, sizeof(int) * numBuckets);

	if (nds32->prof_buckets)
		free(nds32->prof_buckets);

	nds32->prof_buckets = (uint32_t *) buckets;

	return ERROR_OK;
}

static void write_gmon_1(struct nds32_v5 *nds32, uint32_t num_samples)
{
	if (NULL == nds32->prof_buckets) {
		assert(0);
		return;
	}
	uint64_t b = nds32->prof_num_buckets;
	uint64_t c = nds32->prof_addr_max - nds32->prof_addr_min;

	for (uint32_t i = 0; i < num_samples; i++) {
		uint64_t address = nds32->prof_samples[i];
		/*
		LOG_INFO("address: 0x%lx, min: 0x%lx, max: 0x%lx",
			(long unsigned int)address,
			(long unsigned int)nds32->prof_addr_min,
			(long unsigned int)nds32->prof_addr_max);
		*/
		if ((address < nds32->prof_addr_min) || (nds32->prof_addr_max <= address))
			continue;

		uint64_t a = (address - nds32->prof_addr_min);
		uint64_t index_t = (a * b) / c; /* danger!!!! int32 overflows */
		LOG_DEBUG("a: 0x%lx, b: 0x%lx, c: 0x%lx, index_t: 0x%lx",
			(long unsigned int)a, (long unsigned int)b, (long unsigned int)c, (long unsigned int)index_t);
		if (index_t < nds32->prof_num_buckets)
			nds32->prof_buckets[index_t]++;
	}

	nds32->prof_total_samples += num_samples;
}

extern char *log_output_path;
static void write_gmon_2(struct nds32_v5 *nds32)
{
	char filename[2048];
	/* gmon.out output path depend on log file path */
	LOG_INFO("log_output_path: %s", log_output_path);
	memset(filename, 0, sizeof(filename));

	char *c = strstr(log_output_path, "iceman_debug0.log");
	if (c)
		*c = '\0';

	strncpy(filename, log_output_path, strlen(log_output_path));

	struct target *target = nds32->target;
	uint32_t coreid = target->coreid;
	LOG_DEBUG("coreid: %d", coreid);
	if (coreid == 0)
		strncat(filename, "gmon.out", 8);
	else {
		char name_tmp[32];
		memset(name_tmp, 0, sizeof(name_tmp));
		sprintf(name_tmp, "gmon_core%02d.out", coreid);
		strncat(filename, name_tmp, strlen(name_tmp));
	}
	LOG_INFO("filename: %s", filename);

	FILE *f = fopen(filename, "wb");
	if (f == NULL)
		return;

	writeString(f, "gmon");
	writeLong(f, 0x00000001); /* Version */
	writeLong(f, 0); /* padding */
	writeLong(f, 0); /* padding */
	writeLong(f, 0); /* padding */

	uint8_t zero = 0;  /* GMON_TAG_TIME_HIST */
	writeData(f, &zero, 1);

	/* append binary memory gmon.out &profile_hist_hdr ((char*)&profile_hist_hdr + sizeof(struct gmon_hist_hdr)) */
	if (nds32->gmon_64_bit == 0) {
		writeLong(f, nds32->prof_addr_min); /* low_pc */
		writeLong(f, nds32->prof_addr_max); /* high_pc */
	} else {
		writeLong(f, nds32->prof_addr_min & 0xffffffff); /* low_pc */
		writeLong(f, nds32->prof_addr_min >> 32);        /* low_pc */
		writeLong(f, nds32->prof_addr_max & 0xffffffff); /* high_pc */
		writeLong(f, nds32->prof_addr_max >> 32);        /* high_pc */
	}

	writeLong(f, nds32->prof_num_buckets); /* # of buckets */
	writeLong(f, NDS32_PROFILE_SAMPLES_PER_S);
	writeString(f, "seconds");
	for (size_t i = 0; i < (15-strlen("seconds")); i++)
		writeData(f, &zero, 1);
	writeString(f, "s");

	/* append binary memory gmon.out profile_hist_data (profile_hist_data + profile_hist_hdr.hist_size) */
	char *data = malloc(2 * nds32->prof_num_buckets);
	if (data != NULL) {
		for (uint32_t i = 0; i < nds32->prof_num_buckets; i++) {
			int val;
			val = nds32->prof_buckets[i];
			/*
			LOG_DEBUG("prof_buckets[%d]:0x%x", i, nds32->prof_buckets[i]);
			*/
			if (val > 65535)
				val = 65535;
			if (gmon_big_endian == 1) {
				data[i * 2 + 1] = val&0xff;
				data[i * 2] = (val >> 8) & 0xff;
			} else {
				data[i * 2] = val&0xff;
				data[i * 2 + 1] = (val >> 8) & 0xff;
			}
		}
		writeData(f, data, nds32->prof_num_buckets * 2);
		free(data);
	}

	free(nds32->prof_buckets);
	nds32->prof_buckets = NULL;

	fclose(f);

	LOG_INFO("Wrote gmom.out! prof_total_samples = %d", nds32->prof_total_samples);
}

#if _NDS_MEM_Q_ACCESS_
extern int ndsv5_probe_pc_quick_access(struct target *target, uint64_t *pc_value);
#endif

static int ndsv5_profile_probe_pc(void *priv)
{
	struct target *target = priv;
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	assert(nds32);
	NDS_INFO("probe_pc...");
#if _NDS_MEM_Q_ACCESS_
	if (riscv_debug_buffer_size(target) >= 6) {
		LOG_DEBUG("quick access probe_pc");
		uint64_t reg_pc_value = 0;
		if (ndsv5_probe_pc_quick_access(target, &reg_pc_value) != ERROR_OK) {
			LOG_DEBUG("probe_pc_quick FAIL");
			return ERROR_FAIL;
		}
		NDS_INFO("reg_pc_value: 0x%lx", (long unsigned int)reg_pc_value);
		nds32->prof_samples[nds32->prof_num_samples] = reg_pc_value;
		nds32->prof_num_samples++;
		if (nds32->prof_num_samples >= NDS32_MAX_PROFILE_SAMPLES) {
			LOG_ERROR("prof_num_samples overflow: %x", nds32->prof_num_samples);
			nds32->prof_num_samples--;
		}
		return ERROR_OK;
	}
#endif
	if (nds32->gdb_run_mode_acting == false)
		return ERROR_OK;

	nds32->gdb_run_mode_acting = false;
	/* get PC value */
	struct target_type *tt = get_target_type(target);
	ndsv5_without_announce = 1;
	if (tt->halt(target) != ERROR_OK)
		LOG_ERROR("tt->halt() ERROR");

	/* when polling target halted, do NOT announce gdb */
	ndsv5_poll_wo_announce(target);
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	reg_pc->type->get(reg_pc);
	uint64_t reg_value_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	nds32->prof_samples[nds32->prof_num_samples] = reg_value_pc;
	nds32->prof_num_samples++;
	if (nds32->prof_num_samples >= NDS32_MAX_PROFILE_SAMPLES) {
		LOG_ERROR("prof_num_samples overflow: %x", nds32->prof_num_samples);
		nds32->prof_num_samples--;
	}

	/* current pc, addr = 0, do not handle breakpoints, not debugging */
	int retval = tt->resume(target, 1, 0, 0, 0);
	nds32->gdb_run_mode_acting = true;
	ndsv5_without_announce = 0;
	NDS_INFO("reg_value_pc: 0x%lx", (long unsigned int)reg_value_pc);

	/* reset timer interval */
	target_unregister_timer_callback(ndsv5_profile_probe_pc, target);
	target_register_timer_callback(ndsv5_profile_probe_pc,
				NDS32_PROFILE_PROBE_MS, 1, target);
	return retval;
}

int ndsv5_profile_state(struct target *target)
{
	/* LOG_DEBUG("=== %s ===", __func__); */
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	assert(nds32);

	if ((nds32->prof_num_samples >= nds32->prof_sample_threshold) ||
		(target->state == TARGET_HALTED)) {
		write_gmon_1(nds32, nds32->prof_num_samples); /* merge gmon data */
		nds32->prof_num_samples = 0;
	}
	return ERROR_OK;
}

int ndsv5_profile_post(struct target *target)
{
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	assert(nds32);

	LOG_DEBUG("target->state = %d"
		", nds32->hit_syscall = %d"
		", nds32->active_syscall_id = %x"
		"\n",
		target->state,
		nds32->hit_syscall,
		(uint32_t)nds32->active_syscall_id
		);

	nds32->gdb_run_mode_acting = false;
	/* disable probe */
	target_unregister_timer_callback(ndsv5_profile_probe_pc, target);
	polling_period = nds_bak_polling_period;

	/*
	if (((TARGET_HALTED == target->state) && !nds32->hit_syscall) ||
	    (nds32->active_syscall_id == NDS32_SYSCALL_EXIT) ||
	    (nds32->active_syscall_id == NDS32_VIRTUAL_EXIT)) {
	*/
	if (((target->state == TARGET_HALTED) && (!nds32->hit_syscall)) ||
	    (nds32->active_syscall_id == NDS_EBREAK_EXIT)) {
		nds32->is_in_profiling = false;
		write_gmon_1(nds32, nds32->prof_num_samples);
		write_gmon_2(nds32);
	}
	return ERROR_OK;
}

int ndsv5_profile_init(struct target *target)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	assert(nds32);

	if (nds32->is_in_profiling) {
		if (nds32->gdb_run_mode_acting == false) {
			target_register_timer_callback(ndsv5_profile_probe_pc,
				NDS32_PROFILE_PROBE_MS, 1, target);
			nds32->gdb_run_mode_acting = true;
		}
	} else {
		/* Reset profiling buffers */
		write_gmon_0(nds32);

		/*
		nds32->prof_num_request = NDS32_MAX_PROFILE_ITERATIONS;
		*/
		nds32->prof_num_samples = 0;

		nds32->prof_sample_threshold = (NDS32_MAX_PROFILE_SAMPLES >> 2);
		LOG_DEBUG("nds32->prof_sample_threshold = %d\n", nds32->prof_sample_threshold);

		/* To achieve 50 samples per second */
		target_register_timer_callback(ndsv5_profile_probe_pc,
			NDS32_PROFILE_PROBE_MS, 1, target);
		nds32->is_in_profiling = true;
		nds32->gdb_run_mode_acting = true;
	}

	/* Reduce polling_period in server_loop() for profiling-probe pc per-10ms */
	nds_bak_polling_period = polling_period;
	polling_period = 0;
	return ERROR_OK;
}

