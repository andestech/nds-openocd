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
#ifndef _AICE_APIS_H_
#define _AICE_APIS_H_

/* Constants for AICE command WRITE_CTRL:JTAG_PIN_CONTROL */
#define AICE_JTAG_PIN_CONTROL_SRST		0x01
#define AICE_JTAG_PIN_CONTROL_TRST		0x02
#define AICE_JTAG_PIN_CONTROL_STOP		0x04
#define AICE_JTAG_PIN_CONTROL_RESTART	0x08

#define BACKUP_EDM_CTRL    0

struct target* coreid_to_target(uint32_t coreid);
uint32_t target_to_coreid(struct target* target);

struct cache_info {
	uint32_t set;
	uint32_t way;
	uint32_t line_size;

	uint32_t log2_set;
	uint32_t log2_line_size;
};

struct aice_nds32_info {
	uint32_t edm_version;
	uint32_t edm_cfg;
	char *edm_passcode;
	//uint32_t r0_backup;
	//uint32_t r1_backup;
	uint32_t host_dtr_backup;
	uint32_t target_dtr_backup;
	uint32_t edmsw_backup;
	uint32_t edm_ctl_backup;
	bool debug_under_dex_on;
	bool dex_use_psw_on;
	bool host_dtr_valid;
	bool target_dtr_valid;
	enum aice_target_state_s core_state;
	bool cache_init;
	struct cache_info icache;
	struct cache_info dcache;
};

enum cache_t{ICACHE, DCACHE};

//--- aice_apis.c ----------------------------------------------------------
extern struct aice_nds32_info core_info[];
extern uint32_t aice_ice_config;
extern uint32_t aice_hardware_version;
extern uint32_t aice_firmware_version;
extern uint32_t aice_fpga_version;
extern uint32_t aice_batch_data_buf1_size;
extern uint32_t aice_packet_append_size;

int aice_reg_set_ace_access_op(struct target *target);
int aice_write_ctrl(uint32_t address, uint32_t WriteData);
int aice_read_ctrl(uint32_t address, uint32_t *pReadData);
int aice_read_edmsr(uint32_t target_id, uint32_t address, uint32_t *pReadData);
int aice_write_edmsr(uint32_t target_id, uint32_t address, uint32_t WriteData);

int aice_write_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
int aice_read_edm_by_coreid(uint32_t coreid, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
int aice_write_edm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
int aice_read_edm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
int aice_write_dtr(struct target *target, uint32_t val);
int aice_read_dtr(struct target *target, uint32_t *pReadData);
int aice_write_misc(struct target *target, uint32_t address, uint32_t val);
int aice_read_misc(struct target *target, uint32_t address, uint32_t *pReadData);
int aice_write_dim(struct target *target, uint32_t *pWriteData, uint32_t num_of_words);
int aice_do_execute(struct target *target);

int aice_check_dbger(struct target *target, uint32_t expect_status);
int aice_check_edmctl(struct target *target, uint32_t expect_status);
int aice_execute_dim(struct target *target, uint32_t *insts, uint32_t n_inst);
int aice_read_reg(struct target *target, uint32_t num, uint32_t *val);
int aice_write_reg(struct target *target, uint32_t num, uint32_t val);

int aice_init_edm_registers(struct target *target, bool clear_dex_use_psw);
int aice_backup_tmp_registers(struct target *target);
int aice_restore_tmp_registers(struct target *target);
int aice_backup_edm_registers(struct target *target);
int aice_restore_edm_registers(struct target *target);
int aice_core_init(uint32_t coreid);
int aice_edm_init(uint32_t coreid);
bool is_v2_edm(uint32_t coreid);
int aice_program_edm(struct target *target, char *command_sequence);
int aice_get_info(void);
int aice_edm_config(struct target *target);
int aice_set_command_mode(uint32_t command_mode);
uint32_t aice_get_command_mode(void);
int aice_monitor_command(struct target *target, uint32_t nCmd, char **p_command, int *p_len, char **p_ret_data);
int aice_packbuffer_read(struct target *target, unsigned char *pReadData, unsigned int num_of_bytes);
int get_debug_reason(struct target *target, uint32_t *reason);
int aice_set_edm_passcode(struct target *target, char *edm_passcode);
int aice_diagnosis(struct target *target);
int aice_set_jtag_clock(uint32_t a_clock);
int aice_idcode(uint32_t *idcode, uint8_t *num_of_idcode);

int aice_write_debug_reg(struct target *target, uint32_t addr, uint32_t val);
int aice_read_debug_reg(struct target *target, uint32_t addr, uint32_t *val);
int aice_read_register(struct target *target, uint32_t num, uint32_t *val);
int aice_read_acr(struct target *target, uint32_t num, char *val);
int aice_write_register(struct target *target, uint32_t num, uint32_t val);
int aice_write_acr(struct target *target, uint32_t num, char *val);
int aice_read_reg_64(struct target *target, uint32_t num, uint64_t *val);
int aice_write_reg_64(struct target *target, uint32_t num, uint64_t val);

int aice_reset_target(struct target *target, uint32_t reset_cmd);
int aice_halt_target(struct target *target);
int aice_step(struct target *target);
int aice_state(struct target *target, enum aice_target_state_s *state);
int aice_run_target(struct target *target);
int aice_open_device(struct aice_port_param_s *param);
int aice_close_device(void);
int aice_reset_device(void);
int aice_issue_srst(struct target *target);
int aice_issue_restart(struct target *target);
int aice_issue_reset_hold(struct target *target);
int aice_issue_reset_hold_multi(struct target *target);
int aice_usb_set_custom_srst_script(const char *script);
int aice_usb_set_custom_trst_script(const char *script);
int aice_usb_set_custom_restart_script(const char *script);
int aice_set_dis_DEH_SEL(uint32_t if_disable);
int aice_xread(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr);
int aice_xwrite(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr);
//--- aice_memory.c ----------------------------------------------------------
int aice_read_mem_unit(struct target *target, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer);
int aice_write_mem_unit(struct target *target, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer);
int aice_read_mem_bulk(struct target *target, uint32_t addr, uint32_t length, uint8_t *buffer);
int aice_write_mem_bulk(struct target *target, uint32_t addr, uint32_t length, uint8_t *buffer);
int aice_acc_memory_mode(struct target *target, enum nds_memory_select mem_select);

//--- aice_cache.c ----------------------------------------------------------
int aice_read_tlb(struct target *target, uint32_t virtual_address, uint32_t *physical_address);
int aice_cache_ctl(struct target *target, uint32_t subtype, uint32_t address);
int aice_dump_tlb(struct target *target, char *filename);
int aice_dump_tlb_va(struct target *target, uint32_t va);
int aice_dump_cache(struct target *target, unsigned int cache_type, const char* filename); 
int aice_dump_cache_va(struct target *target, unsigned int cache_type, uint32_t va);

//--- aice_profile.c ----------------------------------------------------------
int aice_profiling(struct target *target, struct aice_profiling_info *profiling_info);
int aice_profile_state(struct target *target, enum aice_target_state_s *state);
int aice_profile_post(struct target *target);
int aice_profile_bench(struct target *target, struct command_invocation *cmd);
int nds32_pwr_init(struct target *target);
int nds32_pwr_post(struct target *target);
int nds32_pwr_state(struct target *target, enum aice_target_state_s *pstate);
extern uint32_t nds32_pwr_sample_rate;
extern uint32_t nds32_pwr_sample_mode;
//--- aice_print_info.c ----------------------------------------------------------
void aice_print_info(unsigned int pipe_cmmd, unsigned int address,
	unsigned int *pInfoData, unsigned int target_id, unsigned int jdp_id);

int aice_write_sdm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
int aice_read_sdm(struct target *target, uint32_t JDPInst, uint32_t address, uint32_t *pEDMData, uint32_t num_of_words);
#endif
