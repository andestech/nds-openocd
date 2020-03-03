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

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include "aice_port.h"
#include "aice_pipe.h"
#include "aice_usb.h"
#include "aice_apis.h"
#include <target/nds32_log.h>

#define AICE_PIPE_MAXLINE 4096

static int aice_pipe_parent_init(struct aice_port_param_s *param);
static int aice_pipe_reset(void);


#ifdef _WIN32
PROCESS_INFORMATION proc_info;

HANDLE aice_pipe_output[2];
HANDLE aice_pipe_input[2];
/***************************************************************************/
static int aice_pipe_write(const void *buffer, int count)
{
	BOOL success;
	DWORD written;

	success = WriteFile(aice_pipe_output[1], buffer, count, &written, NULL);
	if (!success) {
		LOG_ERROR("(WIN32) write to pipe failed, error code: 0x%08lx", GetLastError());
		return -1;
	}

	return written;
}

static int aice_pipe_read(void *buffer, int count)
{
	BOOL success;
	DWORD has_read;

	success = ReadFile(aice_pipe_input[0], buffer, count, &has_read, NULL);
	if (!success || (has_read == 0)) {
		LOG_ERROR("(WIN32) read from pipe failed, error code: 0x%08lx", GetLastError());
		return -1;
	}

	return has_read;
}

static int aice_pipe_child_init(struct aice_port_param_s *param)
{
	STARTUPINFO start_info;
	BOOL success;

	ZeroMemory(&proc_info, sizeof(PROCESS_INFORMATION));
	ZeroMemory(&start_info, sizeof(STARTUPINFO));
	start_info.cb = sizeof(STARTUPINFO);
	start_info.hStdError = aice_pipe_input[1];
	start_info.hStdOutput = aice_pipe_input[1];
	start_info.hStdInput = aice_pipe_output[0];
	start_info.dwFlags |= STARTF_USESTDHANDLES;

	success = CreateProcess(NULL,
			param->adapter_name,
			NULL,
			NULL,
			TRUE,
			0,
			NULL,
			NULL,
			&start_info,
			&proc_info);

	if (!success) {
		LOG_ERROR("Create new process failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_pipe_open_pipe(struct aice_port_param_s *param)
{
	SECURITY_ATTRIBUTES attribute;

	attribute.nLength = sizeof(SECURITY_ATTRIBUTES);
	attribute.bInheritHandle = TRUE;
	attribute.lpSecurityDescriptor = NULL;

	if (!CreatePipe(&aice_pipe_output[0], &aice_pipe_output[1],
				&attribute, AICE_PIPE_MAXLINE)) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}
	if (!CreatePipe(&aice_pipe_input[0], &aice_pipe_input[1],
				&attribute, AICE_PIPE_MAXLINE)) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}

	/* do not inherit aice_pipe_output[1] & aice_pipe_input[0] to child process */
	if (!SetHandleInformation(aice_pipe_output[1], HANDLE_FLAG_INHERIT, 0))
		return ERROR_FAIL;
	if (!SetHandleInformation(aice_pipe_input[0], HANDLE_FLAG_INHERIT, 0))
		return ERROR_FAIL;

	aice_pipe_child_init(param);

	aice_pipe_parent_init(param);

	return ERROR_OK;
}

#else

int aice_pipe_output[2];
int aice_pipe_input[2];

static int aice_pipe_write(const void *buffer, int count)
{
	if (write(aice_pipe_output[1], buffer, count) != count) {
		LOG_ERROR("write to pipe failed");
		return -1;
	}

	return count;
}

static int aice_pipe_read(void *buffer, int count)
{
	int n;
	long long then, cur;

	then = timeval_ms();

	while (1) {
		n = read(aice_pipe_input[0], buffer, count);

		if ((n == -1) && (errno == EAGAIN)) {
			cur = timeval_ms();
			if (cur - then > 500)
				keep_alive();
			continue;
		} else if (n > 0)
			break;
		else {
			LOG_ERROR("read from pipe failed");
			break;
		}
	}

	return n;
}

static int aice_pipe_child_init(struct aice_port_param_s *param)
{
	close(aice_pipe_output[1]);
	close(aice_pipe_input[0]);

	if (aice_pipe_output[0] != STDIN_FILENO) {
		if (dup2(aice_pipe_output[0], STDIN_FILENO) != STDIN_FILENO) {
			LOG_ERROR("Map aice_pipe to STDIN failed");
			return ERROR_FAIL;
		}
		close(aice_pipe_output[0]);
	}

	if (aice_pipe_input[1] != STDOUT_FILENO) {
		if (dup2(aice_pipe_input[1], STDOUT_FILENO) != STDOUT_FILENO) {
			LOG_ERROR("Map aice_pipe to STDOUT failed");
			return ERROR_FAIL;
		}
		close(aice_pipe_input[1]);
	}

	if (execl(param->adapter_name, param->adapter_name, (char *)0) < 0) {
		LOG_ERROR("Execute aice_pipe failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void sig_pipe(int signo)
{
	exit(1);
}

static int aice_pipe_open_pipe(struct aice_port_param_s *param)
{
	pid_t pid;

	if (signal(SIGPIPE, sig_pipe) == SIG_ERR) {
		LOG_ERROR("Register SIGPIPE handler failed");
		return ERROR_FAIL;
	}

	if (pipe(aice_pipe_output) < 0 || pipe(aice_pipe_input) < 0) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}

	pid = fork();
	if (pid < 0) {
		LOG_ERROR("Fork new process failed");
		return ERROR_FAIL;
	} else if (pid == 0) {
		/* child-process */
		if (aice_pipe_child_init(param) != ERROR_OK) {
			LOG_ERROR("AICE_PIPE child process initial error");
			return ERROR_FAIL;
		}
	}
	/* parent-process */
	if (aice_pipe_parent_init(param) != ERROR_OK) {
		LOG_ERROR("AICE_PIPE parent process initial error");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
#endif

static int aice_pipe_parent_init(struct aice_port_param_s *param)
{
	char line[AICE_PIPE_MAXLINE] = {0};
	char command[AICE_PIPE_MAXLINE];
    int get_value = 0;

    #ifndef _WIN32
    close(aice_pipe_output[0]);
    close(aice_pipe_input[1]);

    /* set read end of pipe as non-blocking */
    get_value = fcntl(aice_pipe_input[0], F_GETFL);
    LOG_DEBUG("Before fnctl 0x%x", get_value);

    if (fcntl(aice_pipe_input[0], F_SETFL, get_value & ~O_NONBLOCK))
        return ERROR_FAIL;

    get_value = fcntl(aice_pipe_input[0], F_GETFL);
    LOG_DEBUG("After fnctl 0x%x", get_value);
    #endif // _WIN32

	/* send open to adapter */
	command[0] = AICE_OPEN;

	if (aice_pipe_write(command, 1) < 0) {
		LOG_ERROR("write failed\n");
		return ERROR_FAIL;
	}

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0) {
		LOG_ERROR("read failed\n");
		return ERROR_FAIL;
	}

	if (line[0] != AICE_OK) {
		NDS32_LOG(NDS32_ERRMSG_USB_OPEN_NOVID);
		exit(-1);
		return ERROR_FAIL;
	}

	int len = get_u32(line+1);
	line[5+len] = '\0';
	//LOG_DEBUG("DES_STRING: len=%d, %s", len, line+5);

	NDS32_LOG(NDS32_MSG_SHOW_AICE, line+5);
	return ERROR_OK;
}

static int aice_pipe_open_device(struct aice_port_param_s *param)
{
	if( ERROR_OK != aice_pipe_open_pipe(param) ) {
		LOG_ERROR("AICE_PIPE open error");
		return ERROR_FAIL;
	}

	if (ERROR_FAIL == aice_get_info()) {
		LOG_ERROR("Cannot get AICE info!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_pipe_close(void)
{
	LOG_DEBUG( "AICE pipe close" );

	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_CLOSE;

	if (aice_pipe_write(command, 1) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] != AICE_OK)
		return ERROR_FAIL;

#ifdef _WIN32
	WaitForSingleObject(proc_info.hProcess, INFINITE);
	CloseHandle(proc_info.hProcess);
	CloseHandle(proc_info.hThread);
#endif

	return ERROR_OK;
}

static int aice_pipe_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	LOG_DEBUG( "AICE pipe read idcode" );

	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_IDCODE;

	if (aice_pipe_write(command, 1) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*num_of_idcode = line[0];

	if ((*num_of_idcode == 0) || (*num_of_idcode > AICE_MAX_NUM_CORE)) {
		LOG_ERROR("The ice chain over 16 targets");
		*num_of_idcode = 0;
		return ERROR_FAIL;
	}
	for (int i = 0; i < line[0]; i++)
		idcode[i] = get_u32(line + i * 4 + 1);

	return ERROR_OK;
}

static int aice_pipe_set_jtag_clock(uint32_t a_clock)
{
	LOG_DEBUG( "AICE pipe set jtag clock: %d", a_clock );

	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_SET_JTAG_CLOCK;
	set_u32(command + 1, a_clock);

	if (aice_pipe_write(command, 5) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK) {
		jtag_clock = get_u32(line+1);
		LOG_DEBUG( "AICE pipe jtag clock return: %d", jtag_clock );
		return ERROR_OK;
	}
	else {
		LOG_ERROR( "AICE pipe set jtag clock failed!!" );
		return ERROR_FAIL;
	}
}

static int aice_pipe_reset(void)
{
	LOG_DEBUG( "AICE pipe reset" );

	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_RESET;

	if (aice_pipe_write(command, 1) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] != AICE_OK) {
		LOG_ERROR( "AICE pipe reset failed!!" );
		return ERROR_FAIL;
	}
	return ERROR_OK;
}



static int handle_receive_cmd(char *buffer)
{
    char str[AICE_PIPE_MAXLINE];
    int length = 0;

    switch(buffer[0]) {
        case 1: // Print string
            length = get_u32( buffer+1 );
            memset(str, 0, AICE_PIPE_MAXLINE);
            memcpy(str, buffer+1+4, length);
            
            LOG_DEBUG("Len=%d, Recv=%s", length, str);
            fprintf(stdout, "%s\n", str);
            fflush(stdout);
            return ERROR_OK;
        default:
            LOG_ERROR("Unknown Received CMD!!");
            return ERROR_FAIL;
    };

    return ERROR_OK;
}


static int aice_pipe_read_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	//LOG_DEBUG("AICE Read EDM PIPE: target_id=0x%08X, cmd=0x%02X, addr=0x%08X", target_id, JDPInst, address);

	command[0] = AICE_READ_EDM;
	command[1] = JDPInst;
	set_u32( command+2, target_id );
	set_u32( command+6, address );
	set_u32( command+10, num_of_words );

	if (aice_pipe_write(command, 14) < 0)
		return ERROR_FAIL;

    memset(line, 0, AICE_PIPE_MAXLINE);
	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK ) {
		//LOG_DEBUG( "data_recv:" );
		for( uint32_t i = 0; i < num_of_words; i++ ) {
			EDMData[i] = get_u32( line+1+i*4 );
			//LOG_DEBUG( "0x%08X", EDMData[i] );
		}
		aice_print_info(AICE_READ_EDM, address, (unsigned int *)EDMData, target_id, JDPInst);

        if( line[1+num_of_words*4] != 0 )
            return handle_receive_cmd(line+1+num_of_words*4);
	}
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_write_edm( uint32_t target_id, uint8_t JDPInst, uint32_t address, uint32_t *EDMData, uint32_t num_of_words )
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	//LOG_DEBUG("AICE Write EDM PIPE: target_id=0x%08X, cmd=0x%02X, addr=0x%08X", target_id, JDPInst, address);
	aice_print_info(AICE_WRITE_EDM, address, (unsigned int *)EDMData, target_id, JDPInst);

	command[0] = AICE_WRITE_EDM;
	command[1] = JDPInst;
	set_u32( command+2, target_id );
	set_u32( command+6, address );
	set_u32( command+10, num_of_words );

	//LOG_DEBUG( "data_send:" );
	for( uint32_t i = 0; i < num_of_words; i++ ) {
		set_u32( command+14+i*4, EDMData[i] );
		//LOG_DEBUG( "0x%08X", EDMData[i] );
	}

	if (aice_pipe_write(command, 14+num_of_words*4) < 0)
		return ERROR_FAIL;

    memset(line, 0, AICE_PIPE_MAXLINE);
	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;


    if( line[0] == AICE_OK ) {
        if( line[1] != 0 )    // Skip 1byte(STATUS)
            return handle_receive_cmd(line+1);
    }
    else
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_write_ctrl(uint32_t address, uint32_t WriteData)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	//LOG_DEBUG("AICE Write ctrl PIPE: addr=0x%08X, data=0x%08X", address, WriteData);
	unsigned int ShowInfo = WriteData;
	aice_print_info(AICE_WRITE_CTRL, address, (unsigned int *)&ShowInfo, 0, 0);

	command[0] = AICE_WRITE_CTRL;
	set_u32( command+1, address );
	set_u32( command+5, WriteData );

	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

    memset(line, 0, AICE_PIPE_MAXLINE);
	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

    if( line[0] == AICE_OK ) {
        if( line[1] != 0 )    // Skip 1byte(STATUS)
            return handle_receive_cmd(line+1);
    }
    else 
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_read_ctrl(uint32_t address, uint32_t *pReadData)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	//LOG_DEBUG("AICE Read ctrl PIPE: addr=0x%08X", address);

	command[0] = AICE_READ_CTRL;
	set_u32( command+1, address );

	if (aice_pipe_write(command, 5) < 0)
		return ERROR_FAIL;

    memset(line, 0, AICE_PIPE_MAXLINE);
	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK ) {
		*pReadData = get_u32( line+1 );
		aice_print_info(AICE_READ_CTRL, address, (unsigned int *)pReadData, 0, 0);

        if( line[1+4] != 0 )    // Skip 1byte(STATUS) & 4bytes(DATA) 
            return handle_receive_cmd(line+1+4);
	}
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_execute_custom_script(struct target *target, const char *script)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	memset( command, 0, AICE_PIPE_MAXLINE );
	command[0] = AICE_CUSTOM_SCRIPT;
	strncpy( command+1, script, strlen(script) );

	if (aice_pipe_write(command, 1+strlen(script)+1) < 0)  ///< length[ CMD, filename, '\0' ]
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] != AICE_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_monitor_command( uint32_t nCmd, char **command, int *len, char **ret_data )
{
	LOG_DEBUG("aice_pipe_monitor_command");

	char line[AICE_PIPE_MAXLINE];
	char *send_command = (char *) malloc(AICE_PIPE_MAXLINE*sizeof(char));
	memset( send_command, 0, AICE_PIPE_MAXLINE );

	send_command[0] = AICE_CUSTOM_MONITOR_CMD;

	uint32_t i;
	int now_length = 5;
	for( i = 0; i < nCmd; i++ ) {
		LOG_DEBUG("\t%d of CMD: len=%d, %s", i, len[i], command[i]);

		//strncpy( send_command+now_length, command[i], strlen(command[i]) );
		memcpy(send_command+now_length, command[i], len[i]);
		now_length += len[i];
		send_command[now_length] = ' ';
		now_length++;
	}
	set_u32(send_command+1, now_length-6); //remove first 5 bytes and last 1 byte,
	                                       // 1st byte->AICE_CUSTOM_MONITOR_CMD,
	                                       // 2~5 byte->length,
	                                       // last byte->space

	if (aice_pipe_write(send_command, now_length-1) < 0) {
		free(send_command);
		return ERROR_FAIL;
	}
	free(send_command);

    memset(line, 0, AICE_PIPE_MAXLINE);
	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] != AICE_OK)
		return ERROR_FAIL;

	int ret_size = get_u32(line+1);
	LOG_DEBUG("Monitor command return length: %d bytes", ret_size);

	*ret_data = (char *)malloc( (4+ret_size)*sizeof(char) );
	if( !ret_data ) {
		LOG_ERROR("AICE Monitore Command allocate ret_data failed!!");
		return ERROR_FAIL;
	}
	memcpy(*ret_data, line+1, 4+ret_size);   // LEN=4-Bytes, DATA=szie-Bytes

    if( line[1+4+ret_size] != 0 )
        return handle_receive_cmd(line+1+4+ret_size);

	return ERROR_OK;
}

static int aice_pipe_set_command_mode(enum aice_command_mode command_mode)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	LOG_DEBUG("AICE set_command_mode PIPE: command=0x%08X", command_mode);

	command[0] = AICE_SET_CMD_MODE;
	set_u32( command+1, command_mode );

	if (aice_pipe_write(command, 4) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_dtr_to_buffer(uint32_t target_id, uint32_t buffer_idx)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	LOG_DEBUG("AICE read_dtr_to_buffer PIPE: buffer_idx=0x%08X", buffer_idx);

	command[0] = AICE_READ_DTR_TO_BUFFER;
	set_u32( command+1, target_id );
	set_u32( command+5, buffer_idx );

	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_write_dtr_from_buffer(uint32_t target_id, uint32_t buffer_idx)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	LOG_DEBUG("AICE write_dtr_from_buffer PIPE: buffer_idx=0x%08X", buffer_idx);

	command[0] = AICE_WRITE_DTR_FROM_BUFFER;
	set_u32( command+1, target_id );
	set_u32( command+5, buffer_idx );

	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_batch_buffer_write(uint32_t buf_index)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	LOG_DEBUG("AICE batch_buffer_write PIPE: buffer_idx=0x%08X", buf_index);

	command[0] = AICE_BATCH_BUFFER_WRITE;
	set_u32( command+1, buf_index );

	if (aice_pipe_write(command, 5) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_batch_buffer_read(uint32_t buf_index, unsigned char *pReadData, uint32_t num_of_words)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];
	unsigned char *plineBuf = (unsigned char *)(line+1);

	LOG_DEBUG("AICE batch_buffer_read PIPE: buffer_idx=0x%08X, num_of_words=0x%08X", buf_index, num_of_words);

	command[0] = AICE_BATCH_BUFFER_READ;
	set_u32( command+1, buf_index );
	set_u32( command+5, num_of_words );

	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;
	for( uint32_t i = 0; i < num_of_words*4; i++ ) {
		*pReadData++ = *plineBuf++;
	}
	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_pack_buffer_read(unsigned char *pReadData, uint32_t num_of_bytes)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];
	unsigned char *plineBuf = (unsigned char *)(line+1);

	LOG_DEBUG("AICE pack_buffer_read PIPE: num_of_bytes=0x%08X", num_of_bytes);

	command[0] = AICE_PACK_BUFFER_READ;
	set_u32( command+1, num_of_bytes );

	if (aice_pipe_write(command, 5) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;
	for( uint32_t i = 0; i < num_of_bytes; i++ ) {
		*pReadData++ = *plineBuf++;
	}
	if( line[0] == AICE_OK )
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

int aice_pipe_icemem_xwrite(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pSetData, uint32_t num_of_words, uint32_t attr)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];
	uint32_t write_words;

	LOG_DEBUG("AICE XWrite PIPE: lo_addr=0x%08X, num_of_words=0x%08X, attr=0x%08X",
	  lo_addr, num_of_words, attr);

	while(num_of_words) {		
		// The max pipe length is 4096 bytes
		if (num_of_words > 1000)
			write_words = 1000;
		else
			write_words = num_of_words;

		command[0] = AICE_XWRITE;
		set_u32( command+2, lo_addr );
		set_u32( command+6, hi_addr );
		set_u32( command+10, write_words );
		set_u32( command+14, attr );
	
		//LOG_DEBUG( "data_send:" );
		for( uint32_t i = 0; i < write_words; i++ ) {
			set_u32( command+18+i*4, pSetData[i] );
		}
	
		if (aice_pipe_write(command, 18+num_of_words*4) < 0)
			return ERROR_FAIL;
	
		if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
			return ERROR_FAIL;
	
		if( line[0] != AICE_OK )
			return ERROR_FAIL;

		num_of_words -= write_words;
		pSetData += write_words;
		if (attr != 0x02) { // attr=0x02, fixed address
			lo_addr += (write_words << 2);
		}
	}
	return ERROR_OK;
}

int aice_pipe_icemem_xread(uint32_t lo_addr, uint32_t hi_addr,
  uint32_t *pGetData, uint32_t num_of_words, uint32_t attr)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];
	uint32_t read_words;

	LOG_DEBUG("AICE XRead PIPE: lo_addr=0x%08X, num_of_words=0x%08X, attr=0x%08X",
	  lo_addr, num_of_words, attr);

	while(num_of_words) {		
		// The max pipe length is 4096 bytes
		if (num_of_words > 1000)
			read_words = 1000;
		else
			read_words = num_of_words;

		command[0] = AICE_XREAD;
		set_u32( command+2, lo_addr );
		set_u32( command+6, hi_addr );
		set_u32( command+10, read_words );
		set_u32( command+14, attr );
	
		if (aice_pipe_write(command, 18) < 0)
			return ERROR_FAIL;
	
		if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
			return ERROR_FAIL;
	
		if( line[0] == AICE_OK ) {
			for( uint32_t i = 0; i < read_words; i++ ) {
				pGetData[i] = get_u32( line+1+i*4 );
				//LOG_DEBUG( "0x%08X", pGetData[i] );
			}
		}
		else
			return ERROR_FAIL;

		num_of_words -= read_words;
		pGetData += read_words;
		if (attr != 0x02) { // attr=0x02, fixed address
			lo_addr += (read_words << 2);
		}
	}
	return ERROR_OK;
}

struct aice_nds32_api_s aice_nds32_pipe = {
	/** */
	.write_ctrl = aice_pipe_write_ctrl,
	/** */
	.read_ctrl = aice_pipe_read_ctrl,
	/** AICE read_dtr */
	.read_dtr_to_buffer = aice_pipe_read_dtr_to_buffer,
	/** AICE write_dtr */
	.write_dtr_from_buffer = aice_pipe_write_dtr_from_buffer,
	/** AICE batch_buffer_write */
	.batch_buffer_write = aice_pipe_batch_buffer_write,
	/** AICE batch_buffer_read */
	.batch_buffer_read = aice_pipe_batch_buffer_read,
	/** */
	.execute_custom_script = aice_pipe_execute_custom_script,
	/** */
	.set_command_mode = aice_pipe_set_command_mode,
	/** AICE pack_buffer_read */
	.pack_buffer_read = aice_pipe_pack_buffer_read,
	/** */
	.monitor_command = aice_pipe_monitor_command,
	/** */
	.xwrite = aice_pipe_icemem_xwrite,
	/** */
	.xread = aice_pipe_icemem_xread,
};

/** */
struct aice_port_api_s aice_pipe_api = {
	/** */
	.open = aice_pipe_open_device,
	/** */
	.close = aice_pipe_close,
	/** */
	.reset = aice_pipe_reset,
	/** */
	.idcode = aice_pipe_idcode,
	/** */
	.set_jtag_clock = aice_pipe_set_jtag_clock,
	/** */
	.assert_srst = aice_usb_assert_srst,
	/** */
	.state = aice_get_state,
	/** */
	.read_edm = aice_pipe_read_edm,
	/** */
	.write_edm = aice_pipe_write_edm,
	/** */
	.profiling = aice_usb_profile_entry,
	/** */
	.diagnosis = aice_usb_do_diagnosis,
	/** */
	.pnds32 = &aice_nds32_pipe,
};
