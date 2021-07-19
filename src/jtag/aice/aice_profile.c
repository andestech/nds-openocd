/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include <target/target.h>
#include "aice_usb.h"
#include "aice_apis.h"
#include <target/nds32.h>
#include <target/nds32_disassembler.h>
#include <target/target_request.h>
#include <target/nds32_tracer.h>

#define DTR_INDEX (0)
#define DTR_BUF (R0)

typedef unsigned char UNIT[2];  /* unit of profiling */

#ifndef max
#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; })
#endif

#ifndef min
#define min(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a < _b ? _a : _b; })
#endif

static int write_gmon_0(struct nds32 *nds32);
static void write_gmon_1(struct nds32 *nds32, uint32_t num_samples);
static void write_gmon_2(struct nds32 *nds32);
static int aice_profile_probe_entry(struct target *target, struct aice_profiling_info *profiling_info);
static uint32_t gmon_big_endian = 0;

uint32_t nds32_pwr_sample_rate;
uint32_t nds32_pwr_sample_mode;
char *p_nds32_pwr_sample_data = NULL;
uint32_t nds32_bak_polling_period;
extern int polling_period;

int pwr_set_curmon_disen(void);
int pwr_set_curmon_en(void);
uint32_t pwr_get_curmon_data(void);
uint32_t pwr_get_curmon_status(void);
int pwr_set_curmon_mode(uint32_t sample_mode);
int pwr_set_curmon_rate(uint32_t sample_rate);
int nds32_pwr_check(struct target *target);

static int fill_profiling_batch_commands_v2(struct target *target, uint32_t reg_no)
{
	uint32_t dim_instructions[4];
	uint32_t coreid = target_to_coreid(target);
	aice_set_command_mode(AICE_COMMAND_MODE_BATCH);

	/* step 1-0 assume/expect DBGER.DEX is clear */
	/* step 1-1 halt */
	if (aice_write_misc(target, NDS_EDM_MISC_EDM_CMDR, 0) != ERROR_OK)
		return ERROR_FAIL;
	/* step 1-2 expect DBGER.DEX is set */

	/* sample register */
	if (NDS32_REG_TYPE_GPR == nds32_reg_type(reg_no)) {
		//TODO: feed me!
		assert(0);
		/* general registers */
		dim_instructions[0] = MTSR_DTR(reg_no);
		dim_instructions[1] = DSB;
		dim_instructions[2] = NOP;
		dim_instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_SPR == nds32_reg_type(reg_no)) {
		//TODO: feed me!
		assert(0);
		/* user special registers */
		dim_instructions[0] = MFUSR_G0(DTR_BUF, nds32_reg_sr_index(reg_no));
		dim_instructions[1] = MTSR_DTR(DTR_BUF);
		dim_instructions[2] = DSB;
		dim_instructions[3] = BEQ_MINUS_12;
	} else { /* system registers */
		/* step 2-0 backup $R0 => $DTR, read $REGISTER => $R0 via DIM */
		dim_instructions[0] = INST_MTSR(DTR_BUF, DTR_INDEX);                    // $DTR_BUF => $DTR
		dim_instructions[1] = 0x6402ac02;//INST_MFSR(nds32_reg_sr_index(reg_no), DTR_INDEX); // $REG => $DTR_BUF
		dim_instructions[2] = DSB;
		dim_instructions[3] = BEQ_MINUS_12;
//		for (int _i=0;_i<4;++_i)
//			LOG_INFO("dim[%d] = 0x%08x", _i, dim_instructions[_i]);
		/* step 2-1 execute DIM */
		if (aice_write_dim(target, dim_instructions, 4) != ERROR_OK)
			return ERROR_FAIL;
		/* step 3 read $DTR ($R0) to buffer */
		aice_port->api->pnds32->read_dtr_to_buffer(coreid, AICE_BATCH_DATA_BUFFER_0);
		/* step 4 write $DTR ($R0) from buffer */
		aice_port->api->pnds32->write_dtr_from_buffer(coreid, AICE_BATCH_DATA_BUFFER_0);
		/* step 5 clear DBGER.DEX before IRET */
		if (aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_DEX) != ERROR_OK)
			return ERROR_FAIL;
		/* step 6-0 move $REGISTER => $DTR, restore $R0 <= $DTR via DIM */
		dim_instructions[0] = INST_MTSR(DTR_BUF, DTR_INDEX);
		dim_instructions[1] = INST_MFSR(DTR_BUF, DTR_INDEX);
		dim_instructions[2] = DSB;
		dim_instructions[3] = IRET;
//		for (int _i=0;_i<4;++_i)
//			LOG_INFO("dim[%d] = 0x%08x", _i, dim_instructions[_i]);
	}
	/* step 6-1 execute DIM */
	if (aice_write_dim(target, dim_instructions, 4) != ERROR_OK)
		return ERROR_FAIL;

	/* step 7 save sampled $REGISTER to data buffer */
	aice_port->api->pnds32->read_dtr_to_buffer(coreid, AICE_BATCH_DATA_BUFFER_1);

	//aice_port->api->pnds32->set_command_mode(AICE_COMMAND_MODE_NORMAL);

	/* use BATCH_BUFFER_WRITE to fill command-batch-buffer */
	if (aice_port->api->pnds32->batch_buffer_write(AICE_BATCH_COMMAND_BUFFER_0) != ERROR_OK)
		return ERROR_FAIL;

	usb_out_packets_buffer_length = 0;
	usb_in_packets_buffer_length = 0;

	return ERROR_OK;
}

static int aice_usb_profiling(struct target *target, uint32_t interval, uint32_t iteration,
		uint32_t reg_no, uint32_t *samples, uint32_t *num_samples)
{
	struct nds32 *nds32 = target_to_nds32(target);

	//TODO: add command query canProfile
	assert(nds32->profiling_support);

	int retval = ERROR_OK;
	*num_samples = 0;

	/* initialize DIM size */
	if (aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DIM_SIZE, 4) != ERROR_OK)
		return ERROR_FAIL;

	/* Use AICE_BATCH_DATA_BUFFER_0 to read/write $DTR.
	 * Set it to circular buffer */
	if (aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL, 0xC0000) != ERROR_OK)
		return ERROR_FAIL;

	fill_profiling_batch_commands_v2(target, reg_no);

	const uint32_t sample_threshold = min((NDS32_MAX_PROFILE_SAMPLES >> 2), (aice_batch_data_buf1_size >> 1));
	LOG_OUTPUT("sample_threshold = %d\n", sample_threshold);

	uint32_t total_samples = 0;
	while (total_samples < iteration) {
		uint32_t iteration_left = iteration - total_samples;
		uint32_t sub_iteration = min(NDS32_MAX_PROFILE_ITERATIONS, iteration_left);

		// update iteration register
		uint32_t reg_iteration = (interval << 16) | sub_iteration;
		if (aice_write_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION,
					reg_iteration) != ERROR_OK) {
			retval = ERROR_FAIL;
			goto end_profiling;
		}

		// initialize batch data buffer #1 to store samples ($PC)
		if (aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL,
					0x40000) != ERROR_OK) {
			retval = ERROR_FAIL;
			goto end_profiling;
		}

		ASSERTOK(aice_run_target(target));

		/* enable BATCH command */
		if (aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL,
					0x80000000) != ERROR_OK) {
			aice_halt_target(target);
			retval = ERROR_FAIL;
			goto end_profiling;
		}

#if 0
		/* wait a while (AICE bug, workaround) */
		alive_sleep(sub_iteration);
#endif

		// read samples and check status
		uint32_t is_break = false;
		uint32_t is_error = false;
		uint32_t is_done = false;
		uint32_t sub_total_samples = 0;
		while (!(is_done || is_error || is_break)) {
			uint32_t batch_status;
			uint32_t batch_iteration = 0;

			aice_read_ctrl(AICE_READ_CTRL_BATCH_STATUS, &batch_status);

			if ((0x1f & batch_status) || nds32->gdb_run_mode_halt) { // batch event or Ctrl+C
				if (0x10 & batch_status) { //DBGI_EX
					is_break = true;
				} else if (0x1 & batch_status) { //COMPLETE
					is_done = true;
				} else if (0xe & batch_status) { //ERROR
					is_error = true;
				} else {
					is_break = true;
					//TODO: read iteration before batch command stop
					//      because it will be reset when stop
					ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &batch_iteration));
					//stop batch command
					ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0));
				}

				ASSERTOK(aice_halt_target(target)); //to backup registers
			}

			// get samples from batch data buffer
			if (0 == batch_iteration)
				ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &batch_iteration));
			uint32_t current_iterations = sub_iteration - (NDS32_MAX_PROFILE_ITERATIONS & batch_iteration);
			uint32_t samples_to_read = current_iterations - sub_total_samples;
			if ((samples_to_read >= sample_threshold) || is_done || is_error || is_break) {
				ASSERTOK(aice_port->api->pnds32->batch_buffer_read(AICE_BATCH_DATA_BUFFER_1,
					(unsigned char*)(samples + total_samples), samples_to_read));
				sub_total_samples += samples_to_read;
				total_samples += samples_to_read;
			}

			keep_alive();
		}

		//TODO: ASSERT(target is halt)

		/* check if breakpoint hit */
		uint32_t reason;
		get_debug_reason(target, &reason);
		if (is_break) { //DBGI_EX
			#if 0
			if ((NDS_EDMSW_DETYPE_BREAK == reason) || (NDS_EDMSW_DETYPE_BREAK16 == reason)) {
				//software break (instruction)
				int regnum = nds32->register_map(nds32, PC);
				uint32_t value_pc;
				ASSERTOK(aice_read_reg(target, regnum, &value_pc));
				uint32_t opcode;
				ASSERTOK(nds32_read_opcode(nds32, value_pc, &opcode));
				// if hit 'break 0x7FFF' => exit
				if (opcode == VIRTUAL_IO_EXIT) {
					target->is_program_exit = 1;
					LOG_USER("Inferior exit");
				} else {
					LOG_USER("Inferior break");
				}
				// make gdb server send notification to front-end
			} else if (NDS_EDMSW_DETYPE_INSTBP == reason) {
				//hardware break
				LOG_USER("Inferior break");
			}
			#else
			LOG_USER("Inferior break");
			#endif

			//TODO: any better idea? (magic)
			//to cause target state change on next poll, and then
			//reply to gdb front-end
			target->state = TARGET_RUNNING;
		}

		if (is_error || is_break)
			break;
	}

end_profiling:
	*num_samples = total_samples;
	LOG_OUTPUT("total_samples = %d\n", total_samples);

	//disable batch command
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0));

	return retval;
}

static int aice_usb_profiling_init(struct target* target)
{
	LOG_DEBUG("=== %s ===", __func__);

	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);

	if (nds32->is_in_profiling) {
		//blank
	} else {
		//reset profiling buffers
		write_gmon_0(nds32);

		nds32->prof_num_request = NDS32_MAX_PROFILE_ITERATIONS;
		nds32->prof_num_samples = 0;

		nds32->prof_sample_threshold = min((NDS32_MAX_PROFILE_SAMPLES >> 2), (aice_batch_data_buf1_size >> 1));
		LOG_OUTPUT("nds32->prof_sample_threshold = %d\n", nds32->prof_sample_threshold);

		uint32_t interval = 10; //to achieve 100 samples per second
		uint32_t reg_no = nds32->register_map(nds32, PC);

		// init DIM size
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DIM_SIZE, 4));
		fill_profiling_batch_commands_v2(target, reg_no);
		// set number of iterations
		uint32_t val_iteration;
		val_iteration = interval << 16 | nds32->prof_num_request;
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, val_iteration));

		nds32->is_in_profiling = true;
	}

	// Use AICE_BATCH_DATA_BUFFER_0 to read/write $DTR.
	// Set it to circular buffer
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL, 0xC0000));

	// init AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL to store $PC
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL, 0x40000));

	return ERROR_OK;
}

static int aice_usb_profiling_post_run(struct target* target)
{
	LOG_DEBUG("=== %s ===", __func__);

	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);
	nds32->gdb_run_mode_acting = true;

	// enable BATCH command
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0x80000000));

	// wait a while (AICE bug, workaround)
	//alive_sleep(100); //ms

	return ERROR_OK;
}

static int aice_usb_profile_post(struct target *target)
{
	int rz = ERROR_OK;

	struct nds32 *nds32 = target_to_nds32(target);

	LOG_DEBUG("target->state = %d"
		", nds32->hit_syscall = %d"
		", nds32->active_syscall_id = %x"
		"\n",
		target->state,
		nds32->hit_syscall,
		nds32->active_syscall_id
		);

	if (((TARGET_HALTED == target->state) && !nds32->hit_syscall) ||
		//target->is_program_exit) {
		(nds32->active_syscall_id == NDS32_SYSCALL_EXIT) ||
		(nds32->active_syscall_id == NDS32_VIRTUAL_EXIT)) {
		nds32->gdb_run_mode_acting = false;
		nds32->is_in_profiling = false;
		write_gmon_2(nds32);

		//disable batch command (again)
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0));
	}

	return rz;
}

static int aice_usb_state_profile(struct target *target, enum aice_target_state_s *state)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t batch_status;
	uint32_t this_iteration;

	*state = AICE_TARGET_RUNNING;

	// check status
	int is_complete = false;
	int is_break = false;
	int is_error = false;
	do {
		ASSERTOK(aice_read_ctrl(AICE_READ_CTRL_BATCH_STATUS, &batch_status));
		LOG_DEBUG("BATCH_STATUS = 0x%08x", batch_status);

		uint32_t batch_iteration = 0;
		if ((0x1f & batch_status) || nds32->gdb_run_mode_halt) { // batch event or Ctrl+C
			if (0x10 & batch_status) { //DBGI_EX
				is_break = true;
			} else if (0x1 & batch_status) { //COMPLETE
				is_complete = true;
			} else if (0xe & batch_status) { //ERROR
				is_error = true;
			} else {
				//TODO: read iteration before batch command stop
				//      because it will be reset when stop
				ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &batch_iteration));
				//stop batch command
				ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0));
			}

			ASSERTOK(aice_halt_target(target)); //to backup registers
			*state = AICE_TARGET_HALTED;
		}

		// get samples from batch data buffer
		if (0 == batch_iteration)
			ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &batch_iteration));
		uint32_t iter_count = (NDS32_MAX_PROFILE_ITERATIONS & batch_iteration);
		this_iteration = nds32->prof_num_request - iter_count;
		LOG_DEBUG("BATCH_ITERATION = 0x%08x (0x%08x)", batch_iteration, this_iteration);
/*
		if (this_iteration == NDS32_MAX_PROFILE_ITERATIONS) {
			LOG_DEBUG("this_iteration = %d, prof_total_samples = %d", this_iteration, nds32->prof_total_samples);
			this_iteration -= nds32->prof_total_samples;
		}
		else
			this_iteration -= (nds32->prof_total_samples % NDS32_MAX_PROFILE_ITERATIONS);
*/
		this_iteration -= nds32->prof_total_samples;
		if (0xffff0000 & this_iteration) {
			LOG_INFO("BATCH_ITERATION = 0x%08x (0x%08x)", batch_iteration, this_iteration);
			LOG_INFO("prof_total_samples = %d", nds32->prof_total_samples);
			LOG_INFO("nds32->prof_num_request = %d", nds32->prof_num_request);
			assert(0);
		}

		if ((this_iteration >= nds32->prof_sample_threshold) || (AICE_TARGET_HALTED == *state)) {
			//read samples now
			LOG_DEBUG("this_iteration = %d", this_iteration);
			if (0 < this_iteration) {
				ASSERTOK(aice_port->api->pnds32->batch_buffer_read(AICE_BATCH_DATA_BUFFER_1,
					(unsigned char*)(nds32->prof_samples), this_iteration));

				write_gmon_1(nds32, this_iteration); //merge gmon data
			}
		} else {
			keep_alive();
		}

		if (is_break) {
			//could be a syscall, deferred resolving to upper frame
		} else if (is_complete) {
			//continue profiling
			//ASSERTOK(aice_run_target(target));
			//*state = AICE_TARGET_RUNNING;
		} else if (is_error) {
			//re-run again
			ASSERTOK(aice_run_target(target)); //continue profiling
			*state = AICE_TARGET_RUNNING;
		}
	} while(false);

	return ERROR_OK;
}

#ifdef __MINGW32__
static LARGE_INTEGER
getFILETIMEoffset(void)
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

static int
clock_gettime(int X, struct timeval *tv)
{
    LARGE_INTEGER           t;
    FILETIME            f;
    double                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized)
    {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter)
        {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        }
        else
        {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else
    {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = microseconds;
    tv->tv_sec = t.QuadPart / 1000000;
    tv->tv_usec = t.QuadPart % 1000000;
    return (0);
}
#endif


#ifdef __MINGW32__

#define BILLION 1000000L;
#define CLOCK_ID 0

#define TIME_DEFINE() struct timeval time_start, time_stop;

#define TIME_START() do{ \
		if(clock_gettime(CLOCK_ID, &time_start) == -1) { \
			perror("clock_gettime"); \
			exit(EXIT_FAILURE); \
		} \
	}while(0)

#define TIME_STOP() do{ \
		if(clock_gettime(CLOCK_ID, &time_stop) == -1) { \
			perror("clock_gettime"); \
			exit(EXIT_FAILURE); \
		} \
	}while(0)

#define TIME_ACC() do{ \
	time_total += (time_stop.tv_sec - time_start.tv_sec) \
		+ (double)(time_stop.tv_usec - time_start.tv_usec) / BILLION; \
	}while(0)

#else

#define BILLION 1000000000L;
//#define CLOCK_ID CLOCK_PROCESS_CPUTIME_ID //seems to exclude IO time
#define CLOCK_ID CLOCK_REALTIME

#define TIME_DEFINE() struct timespec time_start, time_stop;

#define TIME_START() do{ \
		if(clock_gettime(CLOCK_ID, &time_start) == -1) { \
			perror("clock_gettime"); \
			exit(EXIT_FAILURE); \
		} \
	}while(0)

#define TIME_STOP() do{ \
		if(clock_gettime(CLOCK_ID, &time_stop) == -1) { \
			perror("clock_gettime"); \
			exit(EXIT_FAILURE); \
		} \
	}while(0)

#define TIME_ACC() do{ \
	time_total += (time_stop.tv_sec - time_start.tv_sec) \
		+ (double)(time_stop.tv_nsec - time_start.tv_nsec) / BILLION; \
	}while(0)

#endif

static int aice_usb_profiling_bench(struct target *target, struct command_invocation *cmd)
{
	// parse arguments
	const uint32_t ITERATION_MAX = 2048 / 4;
	const uint32_t D4_SAMPLES = 1024;
	const uint32_t D4_ITERATIONS = 256;
	const uint32_t D4_INTERVAL = 10;
	const uint32_t D4_DUMP_SAMPLES = 0;
	const uint32_t D4_IS_KEEP_RUNNING = 0;
	uint32_t cfg_samples = D4_SAMPLES;
	uint32_t cfg_iterations = D4_ITERATIONS;
	uint32_t cfg_interval = D4_INTERVAL;
	uint32_t cfg_dump_samples = D4_DUMP_SAMPLES;
	uint32_t cfg_is_keep_running = D4_IS_KEEP_RUNNING;
	if (1 < CMD_ARGC) {
		unsigned argc = CMD_ARGC - 1;
		const char **argv = CMD_ARGV + 1;
		if (0 < argc) {
			cfg_interval = strtoul(argv[0], NULL, 0);
			cfg_interval = min((1u << 16)-1, cfg_interval);
		}
		if (1 < argc) {
			cfg_iterations = strtoul(argv[1], NULL, 0);
			cfg_iterations = min(ITERATION_MAX, cfg_iterations);
		}
		if (2 < argc) {
			cfg_samples = strtoul(argv[2], NULL, 0);
			cfg_samples = min(1u << 20, cfg_samples);
		}
		if (3 < argc) {
			cfg_dump_samples = strtoul(argv[3], NULL, 0);
		}
		if (4 < argc) {
			cfg_is_keep_running = strtoul(argv[4], NULL, 0);
		}
		LOG_USER("cfg_interval     = %u", cfg_interval);
		LOG_USER("cfg_iterations   = %u", cfg_iterations);
		LOG_USER("cfg_samples      = %u", cfg_samples);
		LOG_USER("cfg_dump_samples = %u", cfg_dump_samples);
		LOG_USER("cfg_is_keep_running = %u", cfg_is_keep_running);
	}

	//assume target is halt by now
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32->profiling_support);
	int pc_reg_num = nds32->register_map(nds32, PC);

	//init DIM size
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DIM_SIZE, 4));

	//AICE_BATCH_DATA_BUFFER_0 for $DTR buffer
	ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL, 0xC0000));

	fill_profiling_batch_commands_v2(target, pc_reg_num);

	uint32_t samples[ITERATION_MAX];
	int retval = ERROR_OK;
	uint32_t total_samples = 0;
	uint32_t iteration;
	uint32_t reg_batch_status;
	uint32_t reason;
	uint32_t reg_pc;
	double time_total = 0;
	int is_running = 0;
	int is_halt = 0;
	TIME_DEFINE();
	if (cfg_is_keep_running) {
		TIME_START();
	}
	while (cfg_samples > total_samples) {
		iteration = cfg_samples - total_samples;
		iteration = min(cfg_iterations, iteration);

		//set number of iterations
		uint32_t reg_batch_iteration;
		reg_batch_iteration = (cfg_interval << 16) | iteration;
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, reg_batch_iteration));

#if 0
		ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &reg_batch_iteration));
		LOG_USER("reg_batch_iteration = 0x%08x", reg_batch_iteration);
#endif

		//init/reset AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL to store $PC
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL, 0x40000));

		if ((0 == cfg_is_keep_running) || (0 == is_running)) {
			//LOG_INFO("aice_run_target()");
			ASSERTOK(aice_run_target(target));
			is_running = 1;
			is_halt = 0;
		}

		// start to count time here (right before BATCH enable)
		if (!cfg_is_keep_running) {
			//printf("TIME_START()\n");
			TIME_START();
		}

		//enable BATCH command
		ASSERTOK(aice_write_ctrl(AICE_WRITE_CTRL_BATCH_CTRL, 0x80000000));

		//wait a while (AICE bug, workaround)
		if (0)
			alive_sleep(iteration);

		//poll status
		while (1) {
			ASSERTOK(aice_read_ctrl(AICE_READ_CTRL_BATCH_STATUS, &reg_batch_status));

			if (0x1 & reg_batch_status) { //COMPLETE
				if (0 == cfg_is_keep_running) {
					ASSERTOK(aice_halt_target(target));
					is_halt = 1;
					TIME_STOP();
				}
				break;
			} else if (0x10 & reg_batch_status) { //DBGI_EX
				get_debug_reason(target, &reason);
				//forward target to DBGI
				if (NDS_EDMSW_DETYPE_DBINT != reason) {
					uint32_t instructions[4] = {NOP, NOP, NOP, IRET };
					ASSERTOK(aice_execute_dim(target, instructions, 4));
				}
				is_halt = 1;
				retval = ERROR_FAIL;
				TIME_STOP();
				break;
			} else if (reg_batch_status & 0xE) {
				//stop counting time (right after halt)
				TIME_STOP();
				retval = ERROR_FAIL;
				break;
			}

			if (0)
				keep_alive();
		}

		//printf("TIME_STOP()\n");
		//TODO: ASSERT(target is halt)

		// get samples from batch data buffer
		if (ERROR_OK == retval) {
#if 0
			//stop time here (right before read samples)
			if (0 == cfg_is_keep_running) {
				TIME_STOP();
			}
#endif
			ASSERTOK(aice_read_ctrl(AICE_WRITE_CTRL_BATCH_ITERATION, &reg_batch_iteration));
			uint32_t iter_count = iteration - (((1u<<16)-1) & reg_batch_iteration);
			uint32_t read_count = 0;
			while (read_count < iter_count) {
				const uint32_t MAX_LEN_BATCH_BUF_READ = 256;
				uint32_t count = min(MAX_LEN_BATCH_BUF_READ, iter_count - read_count);
				ASSERTOK(aice_port->api->pnds32->batch_buffer_read(AICE_BATCH_DATA_BUFFER_1,
					(unsigned char *)&samples[read_count], count));
				read_count += count;
			}

			if (0 == cfg_is_keep_running) {
				TIME_ACC();
				LOG_DEBUG("time_total = %lf", time_total);
			}

			if (cfg_dump_samples) {
				uint32_t* dump_start = samples;
				//uint32_t* dump_end = samples + iter_count;
				uint32_t* dump_end = samples + min(cfg_dump_samples, iter_count);
				uint32_t last = *dump_start + 1;
				int count = 0;
				int idx = 0;
				while (dump_start != dump_end) {
					if (last == *dump_start) {
						++count;
					} else {
						if (count) {
							LOG_USER("  repeated %d times", count);
							count = 0;
						}
						LOG_USER("PC[%d]=0x%08x", idx, *dump_start);
						last = *dump_start;
					}
					++idx;
					++dump_start;
				}
				if (count) {
					LOG_USER("  repeated %d times", count);
					count = 0;
				}
			}
		} else {
			//TIME_ACC();
			LOG_USER("*** Error: reg_batch_status = 0x%08x", reg_batch_status);
			assert(0);
			break;
		}

		// check if breakpoint hit
		if (0x10 & reg_batch_status) { //DBGI_EX
			if ((NDS_EDMSW_DETYPE_BREAK == reason) || (NDS_EDMSW_DETYPE_BREAK16 == reason)) {
				//software break (instruction)
				ASSERTOK(aice_read_reg(target, pc_reg_num, &reg_pc));
				uint32_t opcode;
				ASSERTOK(nds32_read_opcode(nds32, reg_pc, &opcode));
				struct nds32_instruction instruction;
				ASSERTOK(nds32_evaluate_opcode(nds32, reg_pc, &instruction));
				// if hit 'break 0x7FFF' => exit
				if ((instruction.info.opc_6 == 0x32) &&
					(instruction.info.sub_opc == 0xA) &&
					(instruction.info.imm == 0x7FFF)) {
					LOG_USER("Inferior exit");
				} else {
					LOG_USER("Inferior break");
				}
			} else if (NDS_EDMSW_DETYPE_INSTBP == reason) {
				//hardware break
				LOG_USER("Inferior break");
			} else {
				//other debug exceptions
				LOG_USER("encounter a debug exception which is not handled!");
			}
		}

		total_samples += iteration;

		if (0x1 != (0x1f & reg_batch_status)) //COMPLETE
			break;

		if (0 == cfg_is_keep_running) {
			LOG_DEBUG("sampling ...");
		}
	}

	if (cfg_is_keep_running) {
		if (ERROR_OK == retval) {
			TIME_STOP();
		}
		TIME_ACC();
	}

	if (total_samples) {
		//calculate results
		LOG_USER("=== bench result ===");
//		LOG_USER("CLOCKS_PER_SEC = %lu", CLOCKS_PER_SEC);
		LOG_USER("cfg_interval        = %u", cfg_interval);
		LOG_USER("cfg_iterations      = %u", cfg_iterations);
		LOG_USER("cfg_samples         = %u", cfg_samples);
		LOG_USER("cfg_dump_samples    = %u", cfg_dump_samples);
		LOG_USER("cfg_is_keep_running = %u", cfg_is_keep_running);
		LOG_USER("total_samples       = %u", total_samples);
		LOG_USER("time_total          = %lf", time_total);
		LOG_USER("--- PCs/Second      = %.3lf ---", ((double)total_samples) / time_total);
	}

	//ensure target is halt
	if (0 == is_halt) {
		//LOG_INFO("aice_halt_target()");
		ASSERTOK(aice_halt_target(target));
	}

	return retval;
}

int aice_usb_profile_entry(struct target *target, struct aice_profiling_info *profiling_info)
{
	struct nds32 *nds32 = target_to_nds32(target);
	if (nds32->data_endian == TARGET_BIG_ENDIAN) {
		gmon_big_endian = 1;
		LOG_DEBUG("BIG_ENDIAN");
	} else {
		gmon_big_endian = 0;
		LOG_DEBUG("LITTLE_ENDIAN");
	}

	if (nds32->edm.support_probe == true) {
		return aice_profile_probe_entry(target, profiling_info);
	}

	if (profiling_info->profiling_type == AICE_PROFILE_MODE_NORMAL) {
		return aice_usb_profiling(target, profiling_info->interval, profiling_info->iteration,
			profiling_info->reg_no, profiling_info->psamples, profiling_info->pnum_samples);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_INIT) {
		return aice_usb_profiling_init(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_POSTRUN) {
		return aice_usb_profiling_post_run(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_POST) {
		return aice_usb_profile_post(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_STATE) {
		return aice_usb_state_profile(target, profiling_info->pstate);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_BENCH) {
		return aice_usb_profiling_bench(target, profiling_info->bench_cmd);
	}

	return ERROR_FAIL;
}

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
	}
	else {
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

static int write_gmon_0(struct nds32 *nds32)
{
	/* figure out bucket size */
	uint32_t addr_min;
	uint32_t addr_max;
	if (nds32->prof_with_range) {
		addr_min = nds32->prof_addr_min;
		addr_max = nds32->prof_addr_max;
	} else {
		LOG_ERROR("Profile must have address range!");
		addr_min = 0;
		addr_max = 1u << 30; //1G
	}

	int addressSpace = addr_max - addr_min;
	assert(addressSpace >= 2);

	/* FIXME: What is the reasonable number of buckets?
	 * The profiling result will be more accurate if there are enough buckets. */
	static const uint32_t maxBuckets = 2 * 1024 * 1024; /* maximum buckets. */
	uint32_t numBuckets = addressSpace / sizeof(UNIT);  /* 2-bytes unit */
	if (numBuckets > maxBuckets)
		numBuckets = maxBuckets;
	nds32->prof_num_buckets = numBuckets;
	nds32->prof_total_samples = 0;
	//TODO: not malloc every time
	int *buckets = malloc(sizeof(int) * numBuckets);
	if (buckets == NULL) {
		return ERROR_FAIL;
	}
	memset(buckets, 0, sizeof(int) * numBuckets);

	if(nds32->prof_buckets) {
		free(nds32->prof_buckets);
	}
	nds32->prof_buckets = (uint32_t*) buckets;

	return ERROR_OK;
}

static void write_gmon_1(struct nds32 *nds32, uint32_t num_samples)
{
	if(NULL == nds32->prof_buckets) {
		assert(0);
		return;
	}

	long long b = nds32->prof_num_buckets;
	long long c = nds32->prof_addr_max - nds32->prof_addr_min;
	for (uint32_t i = 0; i < num_samples; i++) {
		uint32_t address = nds32->prof_samples[i];

		#if 0
		if (gmon_big_endian == 1) {
			address = (((nds32->prof_samples[i] >> 24) & 0x00FF) |
				((nds32->prof_samples[i] >> 8) & 0xFF00) |
				((nds32->prof_samples[i] & 0xFF00) << 8) |
				((nds32->prof_samples[i] & 0x00FF) << 24) );
		}
		LOG_DEBUG("address:0x%x, prof_samples[i]:0x%x", address, nds32->prof_samples[i]);
		#endif

		if ((address < nds32->prof_addr_min) || (nds32->prof_addr_max <= address))
			continue;

		long long a = address - nds32->prof_addr_min;
		int index_t = (a * b) / c; /* danger!!!! int32 overflows */
		nds32->prof_buckets[index_t]++;
	}

	nds32->prof_total_samples += num_samples;
}
extern char* log_output_path;
static void write_gmon_2(struct nds32 *nds32)
{
	char filename[2048];
	/* gmon.out output path depend on log file path */
	LOG_INFO("log_output_path: %s", log_output_path);
	memset(filename, 0, sizeof(filename));

	char *c = strstr(log_output_path, "iceman_debug0.log");
	if( c ) {
		*c = '\0';
	}
	strncpy(filename, log_output_path, strlen(log_output_path));
	uint32_t coreid = target_to_coreid(nds32->target);
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
	writeLong(f, nds32->prof_addr_min); /* low_pc */
	writeLong(f, nds32->prof_addr_max); /* high_pc */
	writeLong(f, nds32->prof_num_buckets); /* # of buckets */
	writeLong(f, 100); /* KLUDGE! We lie, ca. 100Hz best case. */
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
			//LOG_DEBUG("prof_buckets[i]:0x%x", nds32->prof_buckets[i]);
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

int aice_profiling(struct target *target, struct aice_profiling_info *profiling_info)
{
	if (aice_port->api->profiling == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}
	return aice_port->api->profiling(target, profiling_info);
}

int aice_profile_state(struct target *target, enum aice_target_state_s *state)
{
	struct aice_profiling_info profiling_info;
	profiling_info.profiling_type = AICE_PROFILE_MODE_STATE;
	profiling_info.pstate = state;
	return aice_profiling(target, &profiling_info);
}

int aice_profile_post(struct target *target)
{
	struct aice_profiling_info profiling_info;
	profiling_info.profiling_type = AICE_PROFILE_MODE_POST;
	return aice_profiling(target, &profiling_info);
}

int aice_profile_bench(struct target *target, struct command_invocation *cmd)
{
	struct aice_profiling_info profiling_info;
	profiling_info.profiling_type = AICE_PROFILE_MODE_BENCH;
	profiling_info.bench_cmd = cmd;
	return aice_profiling(target, &profiling_info);
}

static int aice_profile_probe_pc(void *priv)
{
	struct target *target = priv;
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);

	if (nds32->gdb_run_mode_acting == false)
		return ERROR_OK;

	uint32_t value_probe=0;
	int result = aice_read_misc(target, NDS_EDM_MISC_EDM_PROBE, &value_probe);
	LOG_DEBUG("probe_pc: %x", value_probe);

	if ((value_probe & 0x01) == 0) {
		nds32->prof_samples[nds32->prof_num_samples] = value_probe;
		nds32->prof_num_samples ++;
		if (nds32->prof_num_samples >= NDS32_MAX_PROFILE_SAMPLES) {
			LOG_ERROR("prof_num_samples overflow: %x", nds32->prof_num_samples);
			nds32->prof_num_samples --;
		}
	}
	//uint32_t request=0;
	//result = target_request(target, request);
	return result;
}

static int aice_profile_probe_state(struct target* target, enum aice_target_state_s *pstate)
{
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);
	//aice_profile_probe_pc(target);

	if (nds32->gdb_run_mode_halt) {
		ASSERTOK(aice_halt_target(target)); //to backup registers
		//*state = AICE_TARGET_HALTED;
	}

	if (aice_state(target, pstate) != ERROR_OK)
			return ERROR_FAIL;

LOG_DEBUG("num=%d, threshold=%d, state=%x", nds32->prof_num_samples, nds32->prof_sample_threshold, *pstate);
	if ((nds32->prof_num_samples >= nds32->prof_sample_threshold) ||
		(*pstate == AICE_TARGET_HALTED)) {
		write_gmon_1(nds32, nds32->prof_num_samples); //merge gmon data
		nds32->prof_num_samples = 0;
	}
	return ERROR_OK;
}

static int aice_profile_probe_init(struct target* target)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);

	if (nds32->is_in_profiling) {
		//blank
	} else {
		//reset profiling buffers
		write_gmon_0(nds32);

		//nds32->prof_num_request = NDS32_MAX_PROFILE_ITERATIONS;
		nds32->prof_num_samples = 0;

		nds32->prof_sample_threshold = (NDS32_MAX_PROFILE_SAMPLES >> 2);
		LOG_DEBUG("nds32->prof_sample_threshold = %d\n", nds32->prof_sample_threshold);

		//to achieve 100 samples per second
		target_register_timer_callback(aice_profile_probe_pc, 10, 1, target);
		nds32->is_in_profiling = true;
	}
	// reduce polling_period in server_loop() for profiling-probe pc per-10ms
	nds32_bak_polling_period = polling_period;
	polling_period = 0;
	return ERROR_OK;
}

static int aice_profile_probe_post(struct target *target)
{
	int rz = ERROR_OK;

	struct nds32 *nds32 = target_to_nds32(target);

	LOG_DEBUG("target->state = %d"
		", nds32->hit_syscall = %d"
		", nds32->active_syscall_id = %x"
		"\n",
		target->state,
		nds32->hit_syscall,
		nds32->active_syscall_id
		);

	if (((TARGET_HALTED == target->state) && !nds32->hit_syscall) ||
		(nds32->active_syscall_id == NDS32_SYSCALL_EXIT) ||
		(nds32->active_syscall_id == NDS32_VIRTUAL_EXIT)) {
		nds32->gdb_run_mode_acting = false;
		nds32->is_in_profiling = false;
		write_gmon_1(nds32, nds32->prof_num_samples);
		write_gmon_2(nds32);
		//disable probe
		target_unregister_timer_callback(aice_profile_probe_pc, target);
		polling_period = nds32_bak_polling_period;
	}
	return rz;
}

static int aice_profile_probe_post_run(struct target* target)
{
	LOG_DEBUG("=== %s ===", __func__);

	struct nds32 *nds32 = target_to_nds32(target);
	assert(nds32);
	nds32->gdb_run_mode_acting = true;
	return ERROR_OK;
}

static int aice_profile_probe_entry(struct target *target, struct aice_profiling_info *profiling_info)
{
	if (profiling_info->profiling_type == AICE_PROFILE_MODE_NORMAL) {
		return aice_usb_profiling(target, profiling_info->interval, profiling_info->iteration,
			profiling_info->reg_no, profiling_info->psamples, profiling_info->pnum_samples);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_INIT) {
		return aice_profile_probe_init(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_POSTRUN) {
		return aice_profile_probe_post_run(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_POST) {
		return aice_profile_probe_post(target);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_STATE) {
		return aice_profile_probe_state(target, profiling_info->pstate);
	}
	else if (profiling_info->profiling_type == AICE_PROFILE_MODE_BENCH) {
		return aice_usb_profiling_bench(target, profiling_info->bench_cmd);
	}
	return ERROR_FAIL;
}

#if IS_CYGWIN == 1
#include <windows.h>

static LARGE_INTEGER
getFILETIMEoffset(void)
{
	SYSTEMTIME s;
	FILETIME f;
	LARGE_INTEGER t;

	s.wYear = 1970;
	s.wMonth = 1;
	s.wDay = 1;
	s.wHour = 0;
	s.wMinute = 0;
	s.wSecond = 0;
	s.wMilliseconds = 0;
	SystemTimeToFileTime(&s, &f);
	t.QuadPart = f.dwHighDateTime;
	t.QuadPart <<= 32;
	t.QuadPart |= f.dwLowDateTime;
	return (t);
}

int gettimeofday(struct timeval *tv, void *tz)
{
	LARGE_INTEGER  t;
	FILETIME  f;
	double                  microseconds;
	static LARGE_INTEGER    offset;
	static double           frequencyToMicroseconds;
	static int              initialized = 0;
	static BOOL             usePerformanceCounter = 0;

	if (!initialized) {
		LARGE_INTEGER performanceFrequency;
		initialized = 1;
		usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
		if (usePerformanceCounter) {
			QueryPerformanceCounter(&offset);
			frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
		}
		else {
			offset = getFILETIMEoffset();
			frequencyToMicroseconds = 10.;
		}
	}
	if (usePerformanceCounter)
		QueryPerformanceCounter(&t);
	else {
		GetSystemTimeAsFileTime(&f);
		t.QuadPart = f.dwHighDateTime;
		t.QuadPart <<= 32;
		t.QuadPart |= f.dwLowDateTime;
	}

	t.QuadPart -= offset.QuadPart;
	microseconds = (double)t.QuadPart / frequencyToMicroseconds;
	t.QuadPart = microseconds;
	tv->tv_sec = t.QuadPart / 1000000;
	tv->tv_usec = t.QuadPart % 1000000;
	return (0);
}
#endif

char *dump_pwr_path = NULL;
void nds32_pwr_write_file(struct nds32 *nds32)
{
	char filename[2048];
	// depend on log file path
	LOG_INFO("log_output_path: %s", log_output_path);
	memset(filename, 0, sizeof(filename));

	char *c = strstr(log_output_path, "iceman_debug0.log");
	if( c ) {
		*c = '\0';
	}
	strncpy(filename, log_output_path, strlen(log_output_path));
	if (dump_pwr_path == NULL)
		strncat(filename, "sample.pwr", 10);
	else
		strncat(filename, dump_pwr_path, strlen(dump_pwr_path));
	LOG_INFO("dump filename: %s", filename);
	FILE *f = fopen(filename, "wb");
	if (f == NULL)
		return;
	writeString(f, "ANDS");
	writeLong(f, nds32_pwr_sample_mode);   /* pwr_sample_mode */
	writeLong(f, 0xFFFFFFFF);              /* padding */
	writeLong(f, 0xFFFFFFFF);              /* padding */
	writeLong(f, 0xFFFFFFFF);              /* padding */
	writeLong(f, nds32_pwr_sample_rate);   /* pwr_sample_rate */
	writeLong(f, nds32->pwr_num_samples); /* pwr_total_samples */

	if (p_nds32_pwr_sample_data) {
		writeData(f, p_nds32_pwr_sample_data, nds32->pwr_num_samples * 4);
		//free(p_nds32_pwr_sample_data);
		//p_nds32_pwr_sample_data = NULL;
		nds32->pwr_num_samples = 0;
	}
	writeString(f, "END-");
	fclose(f);
	LOG_USER("FINISH_DUMP_PWR_%s", filename);
}

static int nds32_pwr_sampling(void *priv)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct target *target = priv;
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->gdb_pwr_mode_acting == false)
		return ERROR_OK;

	nds32->gdb_pwr_mode_acting = false;
	/* get PWR value */
	if ((p_nds32_pwr_sample_data) &&
		(nds32->pwr_num_samples < nds32->pwr_sample_threshold)) {
		uint32_t *p_sample_word = (uint32_t *)&p_nds32_pwr_sample_data[nds32->pwr_num_samples * 4];
		//*p_sample_word = nds32->pwr_num_samples;  // for testing
		//nds32->pwr_num_samples ++;
		uint32_t curmon_status, curmon_data;
		while(1) {
			curmon_status = pwr_get_curmon_status();
			if (curmon_status & XMEM_CURMON_STATUS_EMPTY)
				break;
			curmon_data = pwr_get_curmon_data();
			*p_sample_word++ = curmon_data;
			LOG_DEBUG("curmon_data: 0x%x", curmon_data);
			nds32->pwr_num_samples ++;
		}
	}
	nds32->gdb_pwr_mode_acting = true;
	return ERROR_OK;
}

int nds32_pwr_init(struct target* target)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!nds32->is_in_pwring) {
		//reset profiling buffers
		//write_gmon_0(nds32);

		//nds32->prof_num_request = NDS32_MAX_PROFILE_ITERATIONS;
		nds32->pwr_num_samples = 0;
		nds32->pwr_sample_threshold = 0x8000;
		if (p_nds32_pwr_sample_data)
			free(p_nds32_pwr_sample_data);
		p_nds32_pwr_sample_data = malloc(nds32->pwr_sample_threshold * 4);

		pwr_set_curmon_disen();
		if (nds32_pwr_sample_rate == 0)
			nds32_pwr_sample_rate = 1;
		LOG_DEBUG("nds32_pwr_sample_rate = %d\n", nds32_pwr_sample_rate);
		pwr_set_curmon_rate(nds32_pwr_sample_rate);
		LOG_DEBUG("nds32_pwr_sample_mode = %d\n", nds32_pwr_sample_mode);
		pwr_set_curmon_mode(nds32_pwr_sample_mode);
		pwr_set_curmon_en();

		// to achieve 50 samples per second
		target_register_timer_callback(nds32_pwr_sampling, 1000/nds32_pwr_sample_rate, 1, target);
		nds32->is_in_pwring = true;
		nds32->gdb_pwr_mode_acting = true;
		LOG_DEBUG("nds32->gdb_pwr_mode_acting = true");
	}
	return ERROR_OK;
}

int nds32_pwr_post(struct target *target)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32 *nds32 = target_to_nds32(target);

	LOG_DEBUG("target->state = 0x%x, nds32->hit_syscall = 0x%x", target->state, nds32->hit_syscall);
	if ( (target->state == TARGET_HALTED) && (!nds32->hit_syscall) ) {
		//(nds32->active_syscall_id == NDS_EBREAK_EXIT) ) {
		//disable probe
		target_unregister_timer_callback(nds32_pwr_sampling, target);
		nds32->gdb_pwr_mode_acting = false;
		nds32->is_in_pwring = false;
		pwr_set_curmon_disen();
		//nds32_pwr_write_file(nds32);
	}
	return ERROR_OK;
}

int nds32_pwr_state(struct target* target, enum aice_target_state_s *pstate)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->pwr_num_samples >= nds32->pwr_sample_threshold) {
		// stop sampling
		LOG_DEBUG("stop sampling");
		pwr_set_curmon_disen();
	}
	if (aice_state(target, pstate) != ERROR_OK)
			return ERROR_FAIL;
	return ERROR_OK;
}

extern int aice_xwrite_word(uint32_t addr, uint32_t value);
extern int aice_xread_word(uint32_t lo_addr);

int pwr_set_curmon_disen(void) {
	aice_xwrite_word(XMEM_CURMON_CTRL, 0);
	return ERROR_OK;
}

int pwr_set_curmon_en(void) {
	aice_xwrite_word(XMEM_CURMON_CTRL, 1);
	return ERROR_OK;
}

uint32_t pwr_get_curmon_data(void) {
	uint32_t curmon_data = (uint32_t)aice_xread_word(XMEM_CURMON_RDATA);
	#if 1
	return curmon_data;
	#else
	uint32_t ret_data = curmon_data;

	ret_data = (curmon_data & 0xFF000000) >> 24;
	ret_data |= (curmon_data & 0x00FF0000) >> 8;
	ret_data |= (curmon_data & 0x0000FF00) << 8;
	ret_data |= (curmon_data & 0x000000FF) << 24;
	return ret_data;
	#endif
}

uint32_t pwr_get_curmon_status(void) {
	uint32_t curmon_status = (uint32_t)aice_xread_word(XMEM_CURMON_STATUS);
	uint32_t ret_status = 0;

	ret_status = (curmon_status & 0xFF000000) >> 24;
	ret_status |= (curmon_status & 0x00FF0000) >> 8;
	ret_status |= (curmon_status & 0x0000FF00) << 8;
	ret_status |= (curmon_status & 0x000000FF) << 24;
	return ret_status;
}

int pwr_set_curmon_mode(uint32_t sample_mode) {
	aice_xwrite_word(XMEM_CURMON_MODE, sample_mode|XMEM_CURMON_MODE_OVERWRITE);
	return ERROR_OK;
}

int pwr_set_curmon_rate(uint32_t sample_rate) {
	aice_xwrite_word(XMEM_CURMON_RATE, sample_rate);
	return ERROR_OK;
}

int nds32_pwr_check(struct target *target)
{
	// check ICE_CONFIG.XRW, Indicate ICE-Box supports XREAD and XWRITE
	if ((aice_ice_config & (0x01 << 5)) == 0)
		return ERROR_FAIL;

	uint32_t curmon_id = 0;
	curmon_id = aice_xread_word(XMEM_CURMON_ID);
	LOG_DEBUG("curmon_id = 0x%x", curmon_id);
	if (curmon_id != NDS32_CURMONID) {
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

#if 0
function read_single_end_v1()
	set_curmon_disen()	
	set_curmon_cfg(2)		
	set_curmon_rate(1)
	set_curmon_en()			
	
	local voltage_rdata = get_curmon_data(XMEM_CURMON_RDATA)
	--printf("voltage rdata = 0x%x => %.2f mV\n", voltage_rdata, voltage_rdata/1000)
	--local ltc2990_rdata = get_curmon_data(XMEM_CURMON_LTC2990)
	--printf("ltc2990_rdata = 0x%x => expect vol = %.2f uV \n", ltc2990_rdata, ltc2990_rdata*305.18)
	return voltage_rdata	-- uV
end

function read_current()
	set_curmon_disen()		
	set_curmon_mode(1)		
	set_curmon_rate(1)
	set_curmon_en()
	
	local current_rdata = get_curmon_data(XMEM_CURMON_RDATA)
	--printf("current rdata = 0x%x => %.2f uA\n", current_rdata, current_rdata)
	--local ltc2990_rdata = get_curmon_data(XMEM_CURMON_LTC2990)
	--printf("ltc2990_rdata = 0x%x => expect vol = %.2f uV \n", ltc2990_rdata, ltc2990_rdata*19.42)
	return current_rdata	-- uA
end

function read_power()
	set_curmon_disen()
	set_curmon_mode(0)		
	set_curmon_rate(1)
	set_curmon_en()	
	
	local power_rdata = get_curmon_data(XMEM_CURMON_RDATA)
	--printf("power rdata = 0x%x => %.2f nW\n", power_rdata, power_rdata*1000)
	--local ltc2990_rdata = get_curmon_data(XMEM_CURMON_LTC2990)
	--printf("ltc2990_rdata = 0x%x => expect diffvol = %.2f uV \n", ltc2990_rdata, ltc2990_rdata*19.42)
	return power_rdata	-- nW
	
end

function read_power_pkt()
	local len = 200;
	set_curmon_disen()
	set_curmon_mode(0)		
	set_curmon_rate(100)
	set_curmon_en()	

	-- Get 200 rdata with sample rate = 100 => need 2sec
	aice.sleep(2000)		-- wait for 2sec
	local rdata = aice_xread_fixed(rdata_addr, len)
	
	for i=1, len do
		--printf("rdata[%3d] = %.2f\n", i, rdata[i])
		if (rdata[i] == 0) then
			error(sprintf("curmon sample rate works unexpected"));
		end
	end
	printf(" >> [CURMON] sample rate test PASS!!\n")
end
-- --------------------------------------------
-- Main function
-- --------------------------------------------
function ait_curmon_sample()
	if (not is_aice2() or not is_aice2_t()) then
		return "SKIP", "This test case only support AICE2 and AICE2-T"
	end

	local idcode = get_curmon_data(XMEM_CURMON_IDCODE)
	--printf("idcode = %x\n", idcode)
	if (not eq(idcode, 0x1001363d)) then
		return "SKIP", "No current monitor supported"
	end
	
	local power = 0
	local voltage = 0
	local current = 0

	
	printf("Start power conversion...\n")
	power   = read_power()
	printf("Start current conversion...\n")
	current = read_current()
	printf("Start voltage conversion...\n")
	voltage = read_single_end_v1()
	printf("Start sample rate testing...\n")
	read_power_pkt()
	
	printf("Target Power/Voltage/Current results:\n")
	printf(" >> current = %.2f uA\n", current)
	printf(" >> voltage = %.2f uV \n", voltage)
	local ex_power = (voltage/1000) * current
	printf(" >> expect power = %.2f nW\n", ex_power)
	printf(" >> read power   = %.2f nW\n", power)

	--if ((power==0) or (voltage==0) or (current==0)) then
	--	error(sprintf("power/current/voltage value is 0, please check the target is connected!\n"));
	--end

	-- power checking
	if (ex_power/power > 1.01) or (ex_power/power < 0.99) then
		error(sprintf("power read out is unexpected: read power = %.2f, expect power = %.2f (nW)\n", power, ex_power));
	else 
		printf("read power is as expected\n")
	end

	
end
test_suite[ait_no] = ait_curmon_sample
test_name[ait_no] = "ait_curmon_sample"
ait_no = ait_no + 1

#endif