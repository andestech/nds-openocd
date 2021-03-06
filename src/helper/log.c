/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "command.h"
#include "time_support.h"

#include <stdarg.h>

#ifdef _DEBUG_FREE_SPACE_
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#else
#error "malloc.h is required to use --enable-malloc-logging"
#endif
#endif

int debug_level = -1;

#if _NDS32_ONLY_
int nds_bak_debug_level = -1;
void nds_dump_detail_debug_info(uint32_t);
char *p_nds_bak_debug_buffer_cur;
char *p_nds_bak_debug_buffer_start;
char *p_nds_bak_debug_buffer_end;
uint32_t nds_bak_debug_file_nums = 16;
uint32_t nds_bak_debug_buffer_overwrite;
char *log_output_path;
char *nds_workspace_folder;
#endif /* _NDS32_ONLY_ */

static FILE *log_output;
static struct log_callback *log_callbacks;

static int64_t last_time;
static int64_t current_time;

static int64_t start;

static const char * const log_strings[6] = {
	"User : ",
	"Error: ",
	"Warn : ",	/* want a space after each colon, all same width, colons aligned */
	"Info : ",
	"Debug: ",
	"Debug: "
};

static int count;

/* forward the log to the listeners */
static void log_forward(const char *file, unsigned line, const char *function, const char *string)
{
	struct log_callback *cb, *next;
	cb = log_callbacks;
	/* DANGER!!!! the log callback can remove itself!!!! */
	while (cb) {
		next = cb->next;
		cb->fn(cb->priv, file, line, function, string);
		cb = next;
	}
}

/* The log_puts() serves two somewhat different goals:
 *
 * - logging
 * - feeding low-level info to the user in GDB or Telnet
 *
 * The latter dictates that strings without newline are not logged, lest there
 * will be *MANY log lines when sending one char at the time(e.g.
 * target_request.c).
 *
 */
static void log_puts(enum log_levels level,
	const char *file,
	int line,
	const char *function,
	const char *string)
{
	char *f;
	if (level == LOG_LVL_OUTPUT) {
		/* do not prepend any headers, just print out what we were given and return */
		fputs(string, log_output);
		fflush(log_output);
		return;
	}

	f = strrchr(file, '/');
	if (f != NULL)
		file = f + 1;

	if (strlen(string) > 0) {

#if _NDS32_ONLY_
		char tmp_line[1024];
		char *pline_string = (char *)&tmp_line[0];
		/* print with count and time information */
		int64_t t = timeval_ms() - start;
		int	str_cnt = 0;
		if (strlen(string) > 800) {
			str_cnt = sprintf(pline_string, "%s%d %" PRId64 " %s:%d %s()"
					": skipping...\n", log_strings[level + 1], count, t, file, line, function);
		} else {
			str_cnt = sprintf(pline_string, "%s%d %" PRId64 " %s:%d %s()"
					": %s", log_strings[level + 1], count, t, file, line, function,
					string);
		}
		/* backup debug message into string buffer */
		if (p_nds_bak_debug_buffer_cur) {
			char *pbak_string = (char *)p_nds_bak_debug_buffer_cur;
			if ((pbak_string + str_cnt) >= (p_nds_bak_debug_buffer_end-2)) {
				p_nds_bak_debug_buffer_cur = p_nds_bak_debug_buffer_start;
				pbak_string = p_nds_bak_debug_buffer_cur;
				nds_bak_debug_buffer_overwrite = 1;
			}
			strncpy(pbak_string, pline_string, str_cnt);
			p_nds_bak_debug_buffer_cur += str_cnt;
		}

		if ((level == LOG_LVL_DEBUG) && (debug_level == LOG_LVL_INFO)) {
			/* do NOT output */
		} else if (debug_level >= LOG_LVL_ERROR) {
			fputs(pline_string, log_output);
			if (level == LOG_LVL_ERROR)
				nds_dump_detail_debug_info(0);
#else /* _NDS32_ONLY_ */
		if (debug_level >= LOG_LVL_DEBUG) {
			/* print with count and time information */
			int64_t t = timeval_ms() - start;
#ifdef _DEBUG_FREE_SPACE_
			struct mallinfo info;
			info = mallinfo();
#endif
			fprintf(log_output, "%s%d %" PRId64 " %s:%d %s()"
#ifdef _DEBUG_FREE_SPACE_
				" %d"
#endif
				": %s", log_strings[level + 1], count, t, file, line, function,
#ifdef _DEBUG_FREE_SPACE_
				info.fordblks,
#endif
				string);

#endif /* _NDS32_ONLY_ */
		} else {
			/* if we are using gdb through pipes then we do not want any output
			 * to the pipe otherwise we get repeated strings */
			fprintf(log_output, "%s%s",
				(level > LOG_LVL_USER) ? log_strings[level + 1] : "", string);
		}
	} else {
		/* Empty strings are sent to log callbacks to keep e.g. gdbserver alive, here we do
		 *nothing. */
	}

	fflush(log_output);

	/* Never forward LOG_LVL_DEBUG, too verbose and they can be found in the log if need be */
	if (level <= LOG_LVL_INFO)
		log_forward(file, line, function, string);
}

void log_printf(enum log_levels level,
	const char *file,
	unsigned line,
	const char *function,
	const char *format,
	...)
{
	char *string;
	va_list ap;

	count++;

#if _NDS32_ONLY_
	if ((level > debug_level) && (debug_level != LOG_LVL_INFO))
		return;
#else /* _NDS32_ONLY_ */
	if (level > debug_level)
		return;
#endif /* _NDS32_ONLY_ */

	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL) {
		log_puts(level, file, line, function, string);
		free(string);
	}

	va_end(ap);
}

void log_vprintf_lf(enum log_levels level, const char *file, unsigned line,
		const char *function, const char *format, va_list args)
{
	char *tmp;

	count++;

#if _NDS32_ONLY_
	if ((level > debug_level) && (debug_level != LOG_LVL_INFO))
		return;
#else /* _NDS32_ONLY_ */
	if (level > debug_level)
		return;
#endif /* _NDS32_ONLY_ */

	tmp = alloc_vprintf(format, args);

	if (!tmp)
		return;

	/*
	 * Note: alloc_vprintf() guarantees that the buffer is at least one
	 * character longer.
	 */
	strcat(tmp, "\n");
	log_puts(level, file, line, function, tmp);
	free(tmp);
}

void log_printf_lf(enum log_levels level,
	const char *file,
	unsigned line,
	const char *function,
	const char *format,
	...)
{
	va_list ap;

	va_start(ap, format);
	log_vprintf_lf(level, file, line, function, format, ap);
	va_end(ap);
}

COMMAND_HANDLER(handle_debug_level_command)
{
	if (CMD_ARGC == 1) {
		int new_level;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], new_level);
		if ((new_level > LOG_LVL_DEBUG_IO) || (new_level < LOG_LVL_SILENT)) {
			LOG_ERROR("level must be between %d and %d", LOG_LVL_SILENT, LOG_LVL_DEBUG_IO);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		debug_level = new_level;
	} else if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "debug_level: %i", debug_level);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_log_output_command)
{
	if (CMD_ARGC == 1) {
		FILE *file = fopen(CMD_ARGV[0], "w");
		if (file == NULL) {
			LOG_ERROR("failed to open output log '%s'", CMD_ARGV[0]);
			return ERROR_FAIL;
		}
		if (log_output != stderr && log_output != NULL) {
			/* Close previous log file, if it was open and wasn't stderr. */
			fclose(log_output);
		}
		log_output = file;


#if _NDS32_ONLY_
		log_output_path = strdup(CMD_ARGV[0]);
		fputs(log_output_path, log_output);
		fputs("\n", log_output);
		fflush(log_output);


		char name_tmp[2048] = {0};
		strncpy(name_tmp, log_output_path, strlen(log_output_path));
		char *c = strstr(name_tmp, "iceman_debug0.log");
		if (c)
			*c = '\0';
		nds_workspace_folder = strdup(name_tmp);
#endif /* _NDS32_ONLY_ */
	}

	return ERROR_OK;
}

static const struct command_registration log_command_handlers[] = {
	{
		.name = "log_output",
		.handler = handle_log_output_command,
		.mode = COMMAND_ANY,
		.help = "redirect logging to a file (default: stderr)",
		.usage = "file_name",
	},
	{
		.name = "debug_level",
		.handler = handle_debug_level_command,
		.mode = COMMAND_ANY,
		.help = "Sets the verbosity level of debugging output. "
			"0 shows errors only; 1 adds warnings; "
			"2 (default) adds other info; 3 adds debugging; "
			"4 adds extra verbose debugging.",
		.usage = "number",
	},
	COMMAND_REGISTRATION_DONE
};

int log_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, log_command_handlers);
}

void log_init(void)
{
	/* set defaults for daemon configuration,
	 * if not set by cmdline or cfgfile */
	if (debug_level == -1)
		debug_level = LOG_LVL_INFO;

	char *debug_env = getenv("OPENOCD_DEBUG_LEVEL");
	if (NULL != debug_env) {
		int value;
		int retval = parse_int(debug_env, &value);
		if (ERROR_OK == retval &&
				debug_level >= LOG_LVL_SILENT &&
				debug_level <= LOG_LVL_DEBUG_IO)
				debug_level = value;
	}

	if (log_output == NULL)
		log_output = stderr;

	start = last_time = timeval_ms();
}

int set_log_output(struct command_context *cmd_ctx, FILE *output)
{
	log_output = output;
	return ERROR_OK;
}

/* add/remove log callback handler */
int log_add_callback(log_callback_fn fn, void *priv)
{
	struct log_callback *cb;

	/* prevent the same callback to be registered more than once, just for sure */
	for (cb = log_callbacks; cb; cb = cb->next) {
		if (cb->fn == fn && cb->priv == priv)
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* alloc memory, it is safe just to return in case of an error, no need for the caller to
	 *check this */
	cb = malloc(sizeof(struct log_callback));
	if (cb == NULL)
		return ERROR_BUF_TOO_SMALL;

	/* add item to the beginning of the linked list */
	cb->fn = fn;
	cb->priv = priv;
	cb->next = log_callbacks;
	log_callbacks = cb;

	return ERROR_OK;
}

int log_remove_callback(log_callback_fn fn, void *priv)
{
	struct log_callback *cb, **p;

	for (p = &log_callbacks; (cb = *p); p = &(*p)->next) {
		if (cb->fn == fn && cb->priv == priv) {
			*p = cb->next;
			free(cb);
			return ERROR_OK;
		}
	}

	/* no such item */
	return ERROR_COMMAND_SYNTAX_ERROR;
}

/* return allocated string w/printf() result */
char *alloc_vprintf(const char *fmt, va_list ap)
{
	va_list ap_copy;
	int len;
	char *string;

	/* determine the length of the buffer needed */
	va_copy(ap_copy, ap);
	len = vsnprintf(NULL, 0, fmt, ap_copy);
	va_end(ap_copy);

	/* allocate and make room for terminating zero. */
	/* FIXME: The old version always allocated at least one byte extra and
	 * other code depend on that. They should be probably be fixed, but for
	 * now reserve the extra byte. */
	string = malloc(len + 2);
	if (string == NULL)
		return NULL;

	/* do the real work */
	vsnprintf(string, len + 1, fmt, ap);

	return string;
}

char *alloc_printf(const char *format, ...)
{
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	va_end(ap);
	return string;
}

/* Code must return to the server loop before 1000ms has returned or invoke
 * this function.
 *
 * The GDB connection will time out if it spends >2000ms and you'll get nasty
 * error messages from GDB:
 *
 * Ignoring packet error, continuing...
 * Reply contains invalid hex digit 116
 *
 * While it is possible use "set remotetimeout" to more than the default 2000ms
 * in GDB, OpenOCD guarantees that it sends keep-alive packages on the
 * GDB protocol and it is a bug in OpenOCD not to either return to the server
 * loop or invoke keep_alive() every 1000ms.
 *
 * This function will send a keep alive packet if >500ms has passed since last time
 * it was invoked.
 *
 * Note that this function can be invoked often, so it needs to be relatively
 * fast when invoked more often than every 500ms.
 *
 */
void keep_alive()
{
	current_time = timeval_ms();
	if (current_time-last_time > 1000) {
		extern int gdb_actual_connections;

		if (gdb_actual_connections)
			LOG_WARNING("keep_alive() was not invoked in the "
				"1000ms timelimit. GDB alive packet not "
				"sent! (%" PRId64 "). Workaround: increase "
				"\"set remotetimeout\" in GDB",
				current_time-last_time);
		else
			LOG_DEBUG("keep_alive() was not invoked in the "
				"1000ms timelimit (%" PRId64 "). This may cause "
				"trouble with GDB connections.",
				current_time-last_time);
	}
	if (current_time-last_time > 500) {
		/* this will keep the GDB connection alive */
		LOG_USER_N("%s", "");

		/* DANGER!!!! do not add code to invoke e.g. target event processing,
		 * jim timer processing, etc. it can cause infinite recursion +
		 * jim event callbacks need to happen at a well defined time,
		 * not anywhere keep_alive() is invoked.
		 *
		 * These functions should be invoked at a well defined spot in server.c
		 */

		last_time = current_time;
	}
}

/* reset keep alive timer without sending message */
void kept_alive()
{
	current_time = timeval_ms();
	last_time = current_time;
}

/* if we sleep for extended periods of time, we must invoke keep_alive() intermittantly */
void alive_sleep(uint64_t ms)
{
	uint64_t napTime = 10;
	for (uint64_t i = 0; i < ms; i += napTime) {
		uint64_t sleep_a_bit = ms - i;
		if (sleep_a_bit > napTime)
			sleep_a_bit = napTime;

		usleep(sleep_a_bit * 1000);
		keep_alive();

		#if _NDS32_ONLY_
		extern int server_get_shutdown(void);
		/* NDS32: ctrl-c detect */
		if (server_get_shutdown() == 1) {
			exit(-1);
			return;
		}
		#endif /* _NDS32_ONLY_ */
	}
}

void busy_sleep(uint64_t ms)
{
	uint64_t then = timeval_ms();
	while (timeval_ms() - then < ms) {
		/*
		 * busy wait
		 */
	}
}

#if _NDS32_ONLY_
FILE *get_log_output(void)
{
	return log_output;
}

void nds_dump_detail_debug_info(uint32_t if_final)
{
	static uint32_t nds_bak_debug_buf_cur_id;

	FILE *pDebugBakFile = NULL;
	char *pbak_string;
	uint32_t copy_cnt;

	if ((p_nds_bak_debug_buffer_start == NULL) ||
	    (p_nds_bak_debug_buffer_cur == p_nds_bak_debug_buffer_start))
		return;

	char filename[2048];
	char name_tmp[32];
	memset(name_tmp, 0, sizeof(name_tmp));
	if (if_final) {
		strcpy(name_tmp, "iceman_detail_last.log");
	} else {
		if (nds_bak_debug_buf_cur_id >= (nds_bak_debug_file_nums-1))
			return;
		sprintf(name_tmp, "iceman_detail%02d.log", nds_bak_debug_buf_cur_id);
		nds_bak_debug_buf_cur_id++;
	}
	/* depend on log file path */
	memset(filename, 0, sizeof(filename));
	if (log_output_path) {
		char *c = strstr(log_output_path, "iceman_debug0.log");
		if (c)
			*c = '\0';

		strncpy(filename, log_output_path, strlen(log_output_path));
	}
	strncat(filename, name_tmp, strlen(name_tmp));

	char name_delete[2048];
	memset(name_tmp, 0, sizeof(name_tmp));

	if (nds_bak_debug_buf_cur_id <= 1) {
		/* delete all iceman_detailxx.log before */
		for (uint32_t i = 0; i < nds_bak_debug_file_nums; i++) {
			memset(name_delete, 0, sizeof(name_delete));
			sprintf(name_tmp, "iceman_detail%02d.log", i);
			if (log_output_path)
				strncpy(name_delete, log_output_path, strlen(log_output_path));

			strncat(name_delete, name_tmp, strlen(name_tmp));
			/*NDS_INFO("remove: %s", name_delete);*/
			remove(name_delete);
		}
		memset(name_delete, 0, sizeof(name_delete));
		if (log_output_path)
			strncpy(name_delete, log_output_path, strlen(log_output_path));
		strcpy(name_tmp, "iceman_detail_last.log");
		strncat(name_delete, name_tmp, strlen(name_tmp));
		remove(name_delete);
	}
	/*
	strncat(filename, "detail.log", 10);
	NDS_INFO("nds_dump_detail: %s", filename);
	*/

	pDebugBakFile = fopen(filename, "wb");
	/* copy bak_debug_buffer to output file */
	if (nds_bak_debug_buffer_overwrite == 1) {
		pbak_string = (char *)p_nds_bak_debug_buffer_cur;
		copy_cnt = p_nds_bak_debug_buffer_end - p_nds_bak_debug_buffer_cur;
		if (copy_cnt)
			fwrite((char *)pbak_string, 1, copy_cnt, pDebugBakFile);
		/*NDS_INFO("write %d bytes", copy_cnt);*/
	}
	pbak_string = (char *)p_nds_bak_debug_buffer_start;
	copy_cnt = p_nds_bak_debug_buffer_cur - p_nds_bak_debug_buffer_start;
	if (copy_cnt)
		fwrite((char *)pbak_string, 1, copy_cnt, pDebugBakFile);
	/*NDS_INFO("write %d bytes", copy_cnt);*/
	fclose(pDebugBakFile);

	/* reset backup info buffer */
	nds_bak_debug_buffer_overwrite = 0;
	p_nds_bak_debug_buffer_cur = p_nds_bak_debug_buffer_start;
	/*NDS_INFO("finish");*/
}
#endif /* _NDS32_ONLY_ */

