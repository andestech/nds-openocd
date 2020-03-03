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

#include <jtag/interface.h>
#include <jtag/commands.h>
#include <transport/transport.h>
#include <target/target.h>
#include <jtag/aice/aice_transport.h>
#include "aice_usb.h"
#include "aice_port.h"
#include "aice_apis.h"
#include <target/nds32_log.h>

#define AICE_KHZ_TO_SPEED_MAP_SIZE  17
#define AICE_KHZ_USE_DEFAULT        16
static int aice_khz_to_speed_map[AICE_KHZ_TO_SPEED_MAP_SIZE] = {
    30000,
    15000,
    7500,
    3750,
    1875,
    937,
    468,
    234,
    48000,
    24000,
    12000,
    6000,
    3000,
    1500,
    750,
    375,
    0,
};

static struct aice_port_param_s param;
uint32_t aice_no_crst_detect = 0;
uint32_t aice_no_force_V3_EDM = 0;
static uint32_t aice_num_of_nds32_core = 0;
static uint32_t aice_num_of_id_codes_on_chain = 0;
// set default value = 16, if there is no command 'port_config' in .cfg file
unsigned int nds32_userdef_num_of_ports = AICE_MAX_NUM_CORE;
extern uint32_t force_edm_v3;

static uint32_t *p_aice_target_id_codes;
uint32_t aice_usb_pack_command = 1;
char *nds32_OpenOCD_version = (char *)PACKAGE_STRING;
/***************************************************************************/
/* External interface implementation */
static uint32_t aice_target_id_codes[AICE_MAX_NUM_CORE];
unsigned int aice_num_of_target_id_codes;
extern const char *burner_port;
unsigned int diagnosis_memory = 0;
unsigned int diagnosis_address = 0;
unsigned int aice_do_diagnosis = 0;
extern const char *aice_clk_string[];
extern int nds32_callback_event_handler(struct target *target,
		enum target_event event, void *priv);
/***************************************************************************/
struct vid_pid_s vid_pid_array[AICE_MAX_VID_NUM];
int vid_pid_array_top = -1;
/***************************************************************************/
int abs_chain_table[AICE_MAX_NUM_CORE] = {-1};
unsigned int aice_efreq_value = 0;


/* AICE operations */
const char *target_cfg_name_str;
#define FILENAME_INPUT     "target/nds32.cfg.tpl"
#define FILENAME_OUTPUT    "target/nds32v3_0.cfg"
#define NDS32_CPUTAPID     0x1000063d
#define LINE_BUFFER_SIZE   2048
#define NDS32_UPDATE_TARGET_ARCH  "    target create $_TARGETNAME nds32_v3 -endian little " \
                                  "-chain-position $_TARGETNAME -coreid $i -variant $_ACE_CONF\n"

static int aice_update_target_cfg(uint32_t num_cores, uint32_t *p_coreid)
{
	uint32_t i, num_nds32_cores = 0;
	char line_buffer[LINE_BUFFER_SIZE];
	FILE *pSrcFile = NULL;
	FILE *pDstFile = NULL;

	pSrcFile = fopen(FILENAME_INPUT, "rb");
	pDstFile = fopen(target_cfg_name_str, "wb");
	if ((pSrcFile == NULL) || (pDstFile == NULL)) {
		LOG_DEBUG(NDS32_MSG_TARGET_CFG_FAIL);
		return ERROR_FAIL;
	}

	while (fgets(line_buffer, LINE_BUFFER_SIZE, pSrcFile) != NULL){
		if (strstr(line_buffer, "set number_of_core") != NULL) {
			for (i = 0; i < num_cores; i++) {
				if (*p_coreid == NDS32_CPUTAPID) {
					fprintf(pDstFile, "jtag newtap $_CHIPNAME cpu%d -expected-id 0x%x\n",
							num_nds32_cores, *p_coreid++);
					abs_chain_table[num_nds32_cores] = i;
					num_nds32_cores ++;
				}
				else {
					fprintf(pDstFile, "jtag newtap vendor cpu%d -expected-id 0x%x\n",
							i, *p_coreid++);
				}
			}
			fputs("\n", pDstFile);
			if (num_nds32_cores > nds32_userdef_num_of_ports)
				num_nds32_cores = nds32_userdef_num_of_ports;
			fprintf(pDstFile, "set number_of_core %02d\n", num_nds32_cores);
		}
		else if (strstr(line_buffer, "<_TARGET_ARCH>") != NULL) {
			fputs(NDS32_UPDATE_TARGET_ARCH, pDstFile);
		}
		else if (strstr(line_buffer, "jtag newtap") != NULL) {
			// do NOT copy string
		}
		else
			fputs(line_buffer, pDstFile);
	}
	if (pSrcFile)
		fclose(pSrcFile);
	if (pDstFile)
		fclose(pDstFile);

	aice_num_of_nds32_core = num_nds32_cores;
	return ERROR_OK; 
}

static int aice_update_tap_info(void)
{
	LOG_DEBUG("=== %s ===", __func__);
	struct target *target;

	for (target = all_targets; target; target = target->next) {
		target->tap->idcode = p_aice_target_id_codes[target->tap->abs_chain_position];
	}
	return ERROR_OK;
}

int aice_scan_jtag_chain(void)
{
	if (p_aice_target_id_codes == NULL) {
		LOG_DEBUG("The 1st idcode NOT yet !!");
		return ERROR_FAIL;
	}

	LOG_DEBUG("=== %s ===", __func__);
	uint8_t num_of_idcode = 0;

	int res = aice_port->api->idcode(p_aice_target_id_codes, &num_of_idcode);
	if (res != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_SCAN);
		exit(-1);
		return res;
	}

	for (uint32_t i = 0; i < num_of_idcode; i++) {
		LOG_DEBUG("id_codes[%d] = 0x%x", i, p_aice_target_id_codes[i]);
	}

	return aice_update_tap_info();
}
/*
void aice_check_update_idcodes(void)
{
    int corr_idcodes = 0;

    for( unsigned int i = 0; i < aice_num_of_target_id_codes; i++ ) {
        if (aice_target_id_codes[i] == 0x1000063d) {
            // remove incorrect idcode
            aice_target_id_codes[corr_idcodes] = aice_target_id_codes[i];

            // build up abs_chain_table
            abs_chain_table[corr_idcodes] = i;

            // update corr_idcodes
            corr_idcodes++;
        }
    }

    aice_num_of_target_id_codes = corr_idcodes;
}
*/
int aice_scan_id_codes(uint32_t *idcode, uint8_t *num_of_idcode)
{
	int res;
	uint32_t coreid;

	res = aice_idcode(idcode, num_of_idcode);
	if (res != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_SCAN);
		exit(-1);
		return res;
	}

	p_aice_target_id_codes = idcode;
	aice_num_of_id_codes_on_chain = *num_of_idcode;
	aice_num_of_nds32_core = *num_of_idcode;
	for (coreid = 0; coreid < aice_num_of_nds32_core; coreid++) {
			aice_core_init(coreid);
	}

	// update target.cfg
	aice_update_target_cfg(aice_num_of_id_codes_on_chain, p_aice_target_id_codes);

	if (aice_num_of_nds32_core == 1) {
		NDS32_LOG(NDS32_MSG_TARGET_NUM_ONE);
	}
	else {
		NDS32_LOG(NDS32_MSG_TARGET_NUMS, aice_num_of_nds32_core);
	}

	LOG_DEBUG("After IDcodes Check:");
	for( coreid = 0; coreid < aice_num_of_nds32_core; coreid++ ) {
		LOG_DEBUG("core#%d, abs_coreid=#%d, id=0x%x", coreid, abs_chain_table[coreid],
				p_aice_target_id_codes[abs_chain_table[coreid]]);
	}

	return ERROR_OK;
}

int aice_init_targets(void)
{
	struct target *target;
	struct aice_port_s *aice;

	LOG_DEBUG("aice_init_targets");

	for (target = all_targets; target; target = target->next) {
		target->tap->idcode = aice_target_id_codes[target->tap->abs_chain_position];

		unsigned ii, limit = target->tap->expected_ids_cnt;
		int found = 0;

		for (ii = 0; ii < limit; ii++) {
			uint32_t expected = target->tap->expected_ids[ii];

			/* treat "-expected-id 0" as a "don't-warn" wildcard */
			if (!expected || (target->tap->idcode == expected)) {
				found = 1;
				break;
			}
		}

		if (found == 0) {
			LOG_ERROR
				("aice_init_targets: target not found: idcode: %" PRIx32,
				 target->tap->idcode);
			return ERROR_FAIL;
		}

		aice = calloc(1, sizeof(struct aice_port_s));
		aice->port = aice_port;
		aice->coreid = target->tap->abs_chain_position;
		/* ========================= */
		/* = for testing =========== */
		/* ========================= */
		//aice->coreid = 0;
		/* ========================= */

		target->tap->priv = aice;
		target->tap->hasidcode = 1;
	}

	if (aice_do_diagnosis) {
		target = all_targets;
		aice_diagnosis(target);
	}

	return ERROR_OK;
}

extern unsigned int aice_max_retry_times;
/***************************************************************************/
/* End of External interface implementation */

/* initial aice
 * 1. open usb
 * 2. get/show version number
 * 3. reset
 */
static int aice_init(void)
{
	time_t seconds;

	seconds = time (NULL);
	LOG_DEBUG("log time: %s", ctime (&seconds));
	LOG_DEBUG("%s (%s)", param.device_desc, nds32_OpenOCD_version);
	if (aice_open_device(&param) != ERROR_OK) {
		return ERROR_JTAG_INIT_FAILED;
	}
	if (aice_scan_id_codes(aice_target_id_codes, (uint8_t *)&aice_num_of_target_id_codes) != ERROR_OK)
		return ERROR_FAIL;
	LOG_INFO("AICE JTAG Interface ready");
	return ERROR_OK;
}

/* cleanup aice resource
 * close usb
 */
static int aice_quit(void)
{
	aice_close_device();
	return ERROR_OK;
}

static int aice_execute_reset(struct jtag_command *cmd)
{
	static int last_trst;
	int retval = ERROR_OK;

	LOG_DEBUG_IO("reset trst: %i", cmd->cmd.reset->trst);

	if (cmd->cmd.reset->trst != last_trst) {
		if (cmd->cmd.reset->trst)
			//retval = aice_port->api->reset();
			retval = aice_reset_device();

		last_trst = cmd->cmd.reset->trst;
	}

	return retval;
}

static int aice_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type) {
		case JTAG_RESET:
			retval = aice_execute_reset(cmd);
			break;
		default:
			retval = ERROR_OK;
			break;
	}
	return retval;
}

/* aice has no need to implement jtag execution model
*/
static int aice_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int retval;

	retval = ERROR_OK;

	while (cmd) {
		if (aice_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

		cmd = cmd->next;
	}

	return retval;
}

/* set jtag frequency(base frequency/frequency divider) to your jtag adapter */
static int aice_speed(int speed)
{
	char tmp_str[100] = {0};
	int retval = aice_set_jtag_clock(speed);

	if( aice_efreq_value != 0 ) {
		if( (unsigned int)(aice_efreq_value/1000/1000) != 0 )
			sprintf(tmp_str, "%2.3lf MHz", (double)aice_efreq_value/1000/1000);
		else if( (unsigned int)(aice_efreq_value/1000) != 0 )
			sprintf(tmp_str, "%2.3lf KHz", (double)aice_efreq_value/1000);
		else
			sprintf(tmp_str, "%u Hz", aice_efreq_value);


		if (retval == ERROR_OK) {
			NDS32_LOG(NDS32_MSG_JTAG_FREQ_OK, tmp_str);
		}
		else {
			NDS32_LOG(NDS32_MSG_JTAG_FREQ_FAIL, tmp_str);
			return ERROR_FAIL;
		}

		return ERROR_OK;
	}



	if (retval == ERROR_OK) {
		NDS32_LOG(NDS32_MSG_JTAG_FREQ_OK, aice_clk_string[jtag_clock]);
	}
	else {
		NDS32_LOG(NDS32_MSG_JTAG_FREQ_FAIL, aice_clk_string[speed]);
		return ERROR_FAIL;
	}
	return retval;
}

/* convert jtag adapter frequency(base frequency/frequency divider) to
 * human readable KHz value */
static int aice_speed_div(int speed, int *khz)
{
	if( aice_efreq_value != 0 ) {
		*khz = aice_efreq_value/1000;
		return ERROR_OK;
	}

	*khz = aice_khz_to_speed_map[speed];

	return ERROR_OK;
}

/* convert human readable KHz value to jtag adapter frequency
 * (base frequency/frequency divider) */
static int aice_khz(int khz, int *jtag_speed)
{
	int i;
	for (i = 0 ; i < AICE_KHZ_TO_SPEED_MAP_SIZE ; i++) {
		if (khz == aice_khz_to_speed_map[i]) {
			if (i == AICE_KHZ_USE_DEFAULT)
				*jtag_speed = i;
			else if (8 <= i)
				*jtag_speed = i | AICE_TCK_CONTROL_TCK3048;
			else
				*jtag_speed = i;
			break;
		}
	}

	if (i == AICE_KHZ_TO_SPEED_MAP_SIZE) {
		// Support extended TCK frequency range
		if( aice_efreq_value != 0 ) {
			*jtag_speed = 0;
			return ERROR_OK;
		}

		LOG_INFO("No support the jtag clock: %d", khz);
		LOG_INFO("Supported jtag clocks are:");

		for (i = 0 ; i < AICE_KHZ_TO_SPEED_MAP_SIZE ; i++)
			LOG_INFO("* %d", aice_khz_to_speed_map[i]);

		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/***************************************************************************/
/* Command handlers */
COMMAND_HANDLER(aice_handle_aice_info_command)
{
	LOG_DEBUG("aice_handle_aice_info_command");

	command_print(CMD, "Description: %s", param.device_desc);
	command_print(CMD, "Serial number: %s", param.serial);
	if (strncmp(aice_port->name, "aice_pipe", 9) == 0)
		command_print(CMD, "Adapter: %s", param.adapter_name);

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_port_command)
{
	LOG_DEBUG("aice_handle_aice_port_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Need exactly one argument to 'aice port'");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (const struct aice_port *l = aice_port_get_list(); l->name; l++) {
		if (strcmp(l->name, CMD_ARGV[0]) == 0) {
			aice_port = l;
			return ERROR_OK;
		}
	}

	LOG_ERROR("No AICE port '%s' found", CMD_ARGV[0]);
	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_desc_command)
{
	LOG_DEBUG("aice_handle_aice_desc_command");

	if (CMD_ARGC == 1)
		param.device_desc = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice desc <description>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_serial_command)
{
	LOG_DEBUG("aice_handle_aice_serial_command");

	if (CMD_ARGC == 1)
		param.serial = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice serial <serial-number>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_vid_pid_command)
{
	LOG_DEBUG("aice_handle_aice_vid_pid_command");

	if (CMD_ARGC != 2) {
		LOG_WARNING("ignoring extra IDs in aice vid_pid (maximum is 1 pair)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

    vid_pid_array_top++;

    if( vid_pid_array_top >= AICE_MAX_VID_NUM ) {
        LOG_ERROR("vid_pid array over AICE_MAX_VID_NUM error, ignore to add new vid_pid config!");
		return ERROR_FAIL;
    }

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vid_pid_array[vid_pid_array_top].vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], vid_pid_array[vid_pid_array_top].pid);

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_adapter_command)
{
	LOG_DEBUG("aice_handle_aice_adapter_command");

	if (CMD_ARGC == 1)
		param.adapter_name = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice adapter <adapter-name>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_port_config_command)
{
	LOG_DEBUG("aice_handle_aice_port_config_command");

	if (CMD_ARGC > 1) {
		burner_port = strdup(CMD_ARGV[0]);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], nds32_userdef_num_of_ports);
		target_cfg_name_str = strdup(CMD_ARGV[2]);
	}
	else
		LOG_ERROR("expected more argument to aice port_config <burner_port> <port_nums> <target_cfg_name>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_retry_times_command)
{
	LOG_DEBUG("aice_handle_aice_retry_times_command");

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], aice_max_retry_times);
	else
		LOG_ERROR("expected exactly one argument to aice retry_times <num_of_retry>");

	return ERROR_OK;
}
unsigned int aice_set_clk_first=16;
COMMAND_HANDLER(aice_handle_aice_count_to_check_dbger_command)
{
	char c;
	char *count_to_check_dbger;

	LOG_DEBUG("aice_handle_aice_count_to_check_dbger_command");
	if (CMD_ARGC != 0) {
		count_to_check_dbger = strdup(CMD_ARGV[0]);
		sscanf(count_to_check_dbger, " %u%c", &aice_count_to_check_dbger, &c);
		if (c == 's' || c == 'S')
			aice_count_to_check_dbger *= 1000;
		if (aice_count_to_check_dbger > 5000)
			aice_set_usb_timeout = aice_count_to_check_dbger;
		if (CMD_ARGC == 2)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], aice_set_clk_first);
		LOG_DEBUG("aice_set_usb_timeout=%d, aice_set_clk_first=%d", aice_set_usb_timeout, aice_set_clk_first);
	}
	else
		LOG_ERROR("expected exactly one argument to aice count_to_check_dbger "
				"<count_of_checking>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_no_crst_detect_command)
{
	LOG_DEBUG("aice_handle_aice_no_crst_detect_command");

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], aice_no_crst_detect);
	else
		LOG_ERROR("expected exactly one argument to aice no_crst_detect");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_force_edm_v3_command)
{
	LOG_DEBUG("aice_handle_force_edm_v3_command");

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], force_edm_v3);
	else
		LOG_ERROR("expected exactly one argument to force_edm_v3");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_dis_deh_sel_command)
{
	LOG_DEBUG("aice_handle_aice_dis_deh_sel_command");

	if (CMD_ARGC == 1) {
		uint32_t if_disable = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], if_disable);
		aice_set_dis_DEH_SEL(if_disable);
	}
	else
		LOG_ERROR("expected exactly one argument to aice no_crst_detect");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_custom_srst_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_srst_script_command");

	if (CMD_ARGC > 0) {
		aice_usb_set_custom_srst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_custom_trst_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_trst_script_command");

	if (CMD_ARGC > 0) {
		aice_usb_set_custom_trst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_custom_restart_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_restart_script_command");

	if (CMD_ARGC > 0) {
		aice_usb_set_custom_restart_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

extern char *custom_initial_script;
COMMAND_HANDLER(aice_handle_aice_custom_initial_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_initial_script_command");

	if (CMD_ARGC > 0) {
		custom_initial_script = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_reset_command)
{
	LOG_DEBUG("aice_handle_aice_reset_command");

	return aice_port->api->reset();
}

COMMAND_HANDLER(aice_handle_aice_diagnosis_command)
{
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], diagnosis_memory);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], diagnosis_address);
	LOG_DEBUG("aice_handle_aice_diagnosis_command %x, %x", diagnosis_memory, diagnosis_address);
	aice_do_diagnosis = 1;
	return ERROR_OK;
}

uint32_t nds32_reset_halt_as_init = 0;
COMMAND_HANDLER(aice_handle_aice_reset_halt_as_init_command)
{
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nds32_reset_halt_as_init);
	return ERROR_OK;
}

uint32_t nds32_reset_aice_as_startup = 0;
COMMAND_HANDLER(aice_handle_reset_aice_as_startup_command)
{
	nds32_reset_aice_as_startup = 1;
	return ERROR_OK;
}

extern uint32_t DIMBR_D4;
COMMAND_HANDLER(aice_handle_edm_dimb)
{
    if (CMD_ARGC > 0) {
    	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], DIMBR_D4);
        LOG_DEBUG("change DIMBR to 0x%x", DIMBR_D4);
        return ERROR_OK;
    }

    return ERROR_FAIL;
}

extern uint32_t l2c_pa_base;
COMMAND_HANDLER(aice_handle_l2c_base)
{
    if (CMD_ARGC > 0) {
    	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], l2c_pa_base);
        LOG_DEBUG("change L2C Base to 0x%x", l2c_pa_base);
        return ERROR_OK;
    }

    return ERROR_FAIL;
}



extern unsigned long nds32_chk_tracer_support;
extern int burner_debug_mode;
extern void nds32_reg_set_cop_reg_nums(uint32_t cop_id, uint32_t cop_reg_nums, uint32_t if_ena);
extern unsigned int log_usb_packets;

COMMAND_HANDLER(aice_handle_misc_config)
{
	uint32_t cop_reg_nums = 0, if_ena = 0, reg_id = 0;

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "tracer_disable") == 0) {
			LOG_DEBUG("tracer_disable");
#if NDS32_TRACER_SUPPORT
			nds32_chk_tracer_support = 0xFF;
#endif
		} else if (strcmp(CMD_ARGV[0], "usb_pack_disable") == 0) {
			LOG_DEBUG("usb_pack_disable");
			aice_usb_pack_command = 0;
		} else if (strcmp(CMD_ARGV[0], "usb_pack_level") == 0) {
			uint32_t usb_pack_level = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], usb_pack_level);
			LOG_DEBUG("usb_pack_level %d", usb_pack_level);
			if (aice_usb_pack_command != 0) {
				if (usb_pack_level == 2)
					aice_usb_pack_command = 2;
				else if (usb_pack_level == 1)
					aice_usb_pack_command = 1;
				else
					aice_usb_pack_command = 0;
			}
		} else if (strcmp(CMD_ARGV[0], "usb_log_enable") == 0) {
			LOG_DEBUG("usb_log_enable");
			uint32_t log_usb_ena = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], log_usb_ena);
			if (log_usb_ena == 1)
				log_usb_packets = 1;
			else
				log_usb_packets = 0;
		} else if (strcmp(CMD_ARGV[0], "burner_debug_enable") == 0) {
			LOG_DEBUG("burner_debug_enable");
#if NDS32_BURNER_SERVER_SUPPORT
			burner_debug_mode = 1;
#endif
		} else if (strcmp(CMD_ARGV[0], "cop") == 0) {
			if (CMD_ARGC < 4)
				return ERROR_FAIL;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_id);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], cop_reg_nums);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], if_ena);
			LOG_DEBUG("cop%d, cop_reg_nums=%d, if_ena=%d", reg_id, cop_reg_nums, if_ena);
			nds32_reg_set_cop_reg_nums(reg_id, cop_reg_nums, if_ena);
		} else if (strcmp(CMD_ARGV[0], "usb_timeout") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], aice_set_usb_timeout);
		}
	}
	return ERROR_OK;
}

extern uint32_t aice_default_use_sdm, aice_current_use_sdm;
extern uint32_t aice_sdm_support_ena;
extern int aice_sdm_direct_select(uint32_t coreid);
extern int aice_sdm_write_misc(uint32_t address, uint32_t val);
extern int aice_sdm_scan_chain(uint32_t *p_idcode, uint8_t *num_of_idcode);
extern int aice_sdm_daisy_chain(void);
COMMAND_HANDLER(aice_handle_sdm)
{
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "enable") == 0) {
			aice_sdm_support_ena = 1;
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			aice_sdm_support_ena = 0;
		} else if (strcmp(CMD_ARGV[0], "use_sdm") == 0) {
			aice_default_use_sdm = 1;
			aice_sdm_support_ena = 1;
		} else if (strcmp(CMD_ARGV[0], "type_daisy_chain") == 0) {
			aice_sdm_daisy_chain();
		} else if (strcmp(CMD_ARGV[0], "type_direct") == 0) {
			aice_current_use_sdm = 1;
		} else if (strcmp(CMD_ARGV[0], "scan_chain") == 0) {
			uint32_t i, idcode[16];
			uint8_t num_of_idcode = 0;
			for (i=0; i<16; i++)
				idcode[i] = 0;
			aice_sdm_scan_chain(&idcode[0], &num_of_idcode);
		} else if (strcmp(CMD_ARGV[0], "direct_select") == 0) {
			aice_current_use_sdm = 1;
			uint32_t select_core = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], select_core);
			aice_sdm_direct_select(select_core);
		} else if (strcmp(CMD_ARGV[0], "write_misc") == 0) {
			uint32_t write_misc_addr = 0, write_misc_data = 0;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], write_misc_addr);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], write_misc_data);
			aice_sdm_write_misc(write_misc_addr, write_misc_data);
		}
	}
	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_efreq_hz)
{
    if (CMD_ARGC == 1) {
    	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], aice_efreq_value);
        LOG_DEBUG("Set extended TCK frequency %d", aice_efreq_value);
        return ERROR_OK;
    }

    return ERROR_FAIL;
}
static const struct command_registration aice_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &aice_handle_aice_info_command,
		.mode = COMMAND_EXEC,
		.help = "show aice info",
		.usage = "aice info",
	},
	{
		.name = "port",
		.handler = &aice_handle_aice_port_command,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the AICE",
		.usage = "aice port ['aice_pipe'|'aice_usb']",
	},
	{
		.name = "desc",
		.handler = &aice_handle_aice_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the aice device description",
		.usage = "aice desc [desciption string]",
	},
	{
		.name = "serial",
		.handler = &aice_handle_aice_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the AICE device",
		.usage = "aice serial [serial string]",
	},
	{
		.name = "vid_pid",
		.handler = &aice_handle_aice_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor and product ID of the AICE device",
		.usage = "aice vid_pid (vid pid)*",
	},
	{
		.name = "adapter",
		.handler = &aice_handle_aice_adapter_command,
		.mode = COMMAND_CONFIG,
		.help = "set the file name of adapter",
		.usage = "aice adapter [adapter name]",
	},
	{
		.name = "port_config",
		.handler = &aice_handle_aice_port_config_command,
		.mode = COMMAND_CONFIG,
		.help = "set the port config",
		.usage = "aice port_config [burner_port] [gdb portnums] [target_cfg_name]",
	},
	{
		.name = "retry_times",
		.handler = &aice_handle_aice_retry_times_command,
		.mode = COMMAND_ANY,
		.help = "set retry times as AICE timeout",
		.usage = "aice retry_times num_of_retry",
	},
	{
		.name = "count_to_check_dbger",
		.handler = &aice_handle_aice_count_to_check_dbger_command,
		.mode = COMMAND_ANY,
		.help = "set retry times(counts or seconds) as checking $DBGER status",
		.usage = "aice count_to_check_dbger count_of_checking",
	},
	{
		.name = "no_crst_detect",
		.handler = &aice_handle_aice_no_crst_detect_command,
		.mode = COMMAND_ANY,
		.help = "No CRST detection in debug session",
		.usage = "aice no_crst_detect 0",
	},
	{
		.name = "force_edm_v3",
		.handler = &aice_handle_force_edm_v3_command,
		.mode = COMMAND_ANY,
		.help = "disable force_edm_v3",
		.usage = "aice force_edm_v3 0",
	},
	{
		.name = "dis_deh_sel",
		.handler = &aice_handle_aice_dis_deh_sel_command,
		.mode = COMMAND_ANY,
		.help = "disable DEH_SEL when exiting debug session",
		.usage = "aice dis_deh_sel 1",
	},
	{
		.name = "custom_srst_script",
		.handler = &aice_handle_aice_custom_srst_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_srst_script script_file_name",
		.help = "set custom srst script",
	},
	{
		.name = "custom_trst_script",
		.handler = &aice_handle_aice_custom_trst_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_trst_script script_file_name",
		.help = "set custom trst script",
	},
	{
		.name = "custom_restart_script",
		.handler = &aice_handle_aice_custom_restart_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_restart_script script_file_name",
		.help = "set custom restart script",
	},
	{
		.name = "custom_initial_script",
		.handler = &aice_handle_aice_custom_initial_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_initial_script script_file_name",
		.help = "set custom initial script",
	},
	{
		.name = "reset",
		.handler = &aice_handle_aice_reset_command,
		.mode = COMMAND_EXEC,
		.usage = "aice reset",
		.help = "reset AICE",
	},
	{
		.name = "diagnosis",
		.handler = &aice_handle_aice_diagnosis_command,
		.mode = COMMAND_CONFIG,
		.usage = "aice diagnosis memory",
		.help = "AICE diagnosis",
	},
	{
		.name = "reset_halt_as_init",
		.handler = &aice_handle_aice_reset_halt_as_init_command,
		.mode = COMMAND_ANY,
		.usage = "aice reset_halt_as_init",
		.help = "AICE reset_halt as init",
	},
	{
		.name = "reset_aice_as_startup",
		.handler = &aice_handle_reset_aice_as_startup_command,
		.mode = COMMAND_CONFIG,
		.usage = "aice reset_aice_as_startup",
		.help = "AICE reset_aice_as_startup",
	},
	{
		.name = "edm_dimb",
		.handler = &aice_handle_edm_dimb,
		.mode = COMMAND_ANY,
		.usage = "aice edm_dimb <Debug Instruction Memory Base>",
		.help = "set Debug Instruction Memory Base register value.",
	},
	{
		.name = "misc_config",
		.handler = &aice_handle_misc_config,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "others config",
	},
	{
		.name = "sdm",
		.handler = &aice_handle_sdm,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "",
	},
	{
		.name = "l2c_base",
		.handler = &aice_handle_l2c_base,
		.mode = COMMAND_ANY,
		.usage = "aice l2c_base <L2C Base>",
		.help = "set L2C Base addess.",
	},
	{
		.name = "efreq_hz",
		.handler = &aice_handle_efreq_hz,
		.mode = COMMAND_CONFIG,
		.usage = "aice efreq_hz <clock Hz>",
		.help = "set extendend TCK frequency.",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration aice_command_handlers[] = {
	{
		.name = "aice",
		.mode = COMMAND_ANY,
		.help = "perform aice management",
		.usage = "aice [subcommand]",
		.chain = aice_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
/***************************************************************************/
/* End of Command handlers */

struct jtag_interface aice_interface = {
	.name = "aice",
	.commands = aice_command_handlers,
	.transports = aice_transports,
	.init = aice_init,
	.quit = aice_quit,
	.execute_queue = aice_execute_queue,
	.speed = aice_speed,		/* set interface speed */
	.speed_div = aice_speed_div,	/* return readable value */
	.khz = aice_khz,		/* convert khz to interface speed value */
};
