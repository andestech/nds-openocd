/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef NDS32_LOG_H
#define NDS32_LOG_H

extern int server_get_shutdown(void);
/* To achieve C99 printf compatibility in MinGW, gnu_printf should be
 * used for __attribute__((format( ... ))), with GCC v4.4 or later
 */
#if (defined(IS_MINGW) && (((__GNUC__ << 16) + __GNUC_MINOR__) >= 0x00040004))
#define PRINTF_ATTRIBUTE_FORMAT gnu_printf
#else
#define PRINTF_ATTRIBUTE_FORMAT printf
#endif

#if _NDS32_ONLY_
#define NDS32_LOG(expr ...) \
do { \
	log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fprintf(stdout, "\n"); \
	fflush(stdout); \
	if (server_get_shutdown() == 1) { \
		exit(-1); \
	} \
} while (0)

#define NDS32_LOG_ERROR(expr ...) \
do { \
	log_printf_lf(LOG_LVL_ERROR, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fprintf(stdout, "\n"); \
	fflush(stdout); \
	if (server_get_shutdown() == 1) { \
		exit(-1); \
	} \
} while (0)

#else
#define NDS32_LOG(expr ...) \
do { \
	log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fprintf(stdout, "\n"); \
	fflush(stdout); \
} while (0)

#define NDS32_LOG_ERROR(expr ...) \
do { \
	log_printf_lf(LOG_LVL_ERROR, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fprintf(stdout, "\n"); \
	fflush(stdout); \
} while (0)

#endif

#define NDS32_LOG_LF(expr ...) \
do { \
	log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fflush(stdout); \
} while (0)

#define NDS32_LOG_R(expr ...) \
do { \
	log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, __func__, expr); \
	fprintf(stdout, expr); \
	fprintf(stdout, "\r"); \
	fflush(stdout); \
} while (0)



/* show message */
#define NDS32_MSG_HW_RESET_HOLD         "hardware reset-and-hold success"
#define NDS32_MSG_HW_RESET_HOLD_ID         "hardware reset-and-hold success on [%s] hart %d"
#define NDS32_MSG_SW_RESET_HOLD         "software reset-and-hold success"
#define NDS32_MSG_SHOW_AICE_VERSION     "%s %s v%d.%d.%d"
#define NDS32_MSG_SHOW_AICE             "%s"
#define NDS32_MSG_SHOW_AICE_VENDOR      "3rd-party ICE-box: ice_ver1 = 0x%08x, ice_ver2 = 0x%08x, ice_ver3 = 0x%08x"
#define NDS32_MSG_JTAG_FREQ_OK          "JTAG frequency %s"
#define NDS32_MSG_JTAG_FREQ_FAIL        "JTAG frequency %s cannot be correctly set."
#define NDS32_MSG_TARGET_NUM_ONE        "There is 1 core in target"
#define NDS32_MSG_TARGET_NUMS           "There are %d cores in target"
#define NDS32_MSG_TARGET_EDMVER         "Core #%u: EDM version 0x%x"
#define NDS32_MSG_TARGET_CFG_FAIL       "aice_update_target_cfg open fail"
/* show error message */
#define NDS32_ERRMSG_USB_VIDPID         "<-- There is no vid_pid in config files -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_USB_OPEN           "<-- Can not open usb (vid=0x%x, pid=0x%x) -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_USB_OPEN_NOVID     "<-- Can not open usb -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_AICE_DISCONNECT    "<-- AICE ERROR! AICE is unplugged. -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_AICE_RESET_BOX     "<-- Failed to initialize AICE box! -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_EDM_FAIL      "<-- EDM access ERROR! -->"
#define NDS32_ERRMSG_SRST_FAIL     "<-- aice_issue_srst ERROR! -->"
#define NDS32_ERRMSG_RESTART_FAIL  "<-- aice_issue_restart ERROR! -->"
#define NDS32_ERRMSG_TARGET_SCAN   "<-- scan target error -->\nNo target specified\n" \
                                   "<-- TARGET ERROR! Failed to identify AndesCore " \
                                   "JTAG Manufacture ID in the JTAG scan chain. " \
                                   "Failed to access EDM registers. -->" \
                                   "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_TARGET_RESET       "<-- TARGET WARNING! The debug target has been reset. -->"
#define NDS32_ERRMSG_TARGET_EXIT_DEBUG  "<-- TARGET WARNING! The debug target exited " \
                                        "the debug mode unexpectedly. -->"
#define NDS32_ERRMSG_TARGET_RESET_HOLD  "<-- SW reset-and-hold failed. -->"
#define NDS32_ERRMSG_TARGET_HW_RESET_HOLD  "<-- HW reset-and-hold failed. -->"
#define NDS32_ERRMSG_TARGET_MAX_INTLVL  "<-- TARGET ERROR! Reaching the max interrupt stack level. -->"
#define NDS32_ERRMSG_TARGET_MAX_INTLVL2 "<-- TARGET ERROR! Reaching the max interrupt stack level; " \
                                        "CPU is stalled at 0x%08x for debugging. -->"
#define NDS32_ERRMSG_TARGET_MAX_INTLVL3 "<-- TARGET ERROR! Reaching the max interrupt stack level %d. -->"

#define NDS32_ERRMSG_TARGET_HW_BREAK_WATCH     "<-- TARGET STATUS: Inserted number of " \
                                               "hardware breakpoint: %d, hardware watchpoints: %d. -->"
#define NDS32_ERRMSG_TARGET_HW_WATCH           "<-- TARGET STATUS: Inserted number of " \
                                               "hardware watchpoint: %d. -->"
#define NDS32_ERRMSG_TARGET_MAX_HW_BREAKPOINT  "<-- TARGET WARNING! Too many " \
                                               "hardware breakpoints/watchpoints are inserted. " \
                                               "The number of available hardware " \
                                               "breakpoints/watchpoints is %d. -->"
#define NDS32_ERRMSG_TARGET_GLOBAL_STOP        "<-- TARGET WARNING! The number of " \
                                               "watchpoints exceeds the hardware " \
                                               "resources. Stop at every load/store " \
                                               "instruction to check for watchpoint matches. -->"

#define NDS32_ERRMSG_TARGET_DISCONNECT  "<-- TARGET ERROR! Target is disconnected with AICE. -->" \
                                        "\n<-- ICEman exit... -->"
#define NDS32_ERRMSG_TARGET_ILL_ACCESS  "<-- TARGET ERROR! Insufficient security privilege " \
                                        "to execute the debug operations. -->"
#define NDS32_ERRMSG_TARGET_EXEC_DIM    "<-- TARGET ERROR! Debug operations do not finish properly: " \
                                        "0x%08x 0x%08x 0x%08x 0x%08x. -->"
#define NDS32_ERRMSG_TARGET_READ_DTR    "<-- TARGET ERROR! The debug target failed to update " \
                                        "the DTR register. -->"
#define NDS32_ERRMSG_TARGET_WRITE_DTR   "<-- TARGET ERROR! AICE failed to write to the DTR register. -->"
#define NDS32_ERRMSG_TARGET_EXCEPTION   "TARGET WARNING! Exception is detected and suppressed."
#define NDS32_ERRMSG_TARGET_DBGI        "<-- TARGET ERROR! Unable to stop the debug target through DBGI. -->"

#endif	/* NDS32_LOG_H */
