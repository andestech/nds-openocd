/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
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
#ifndef __NDS32_EDM_H__
#define __NDS32_EDM_H__

/**
 * @file
 * This is the interface to the Embedded Debug Module for Andes cores.
 */

/* EDM misc registers */
enum nds_edm_misc_reg {
	NDS_EDM_MISC_DIMIR = 0x0,
	NDS_EDM_MISC_SBAR,
	NDS_EDM_MISC_EDM_CMDR,
	NDS_EDM_MISC_DBGER,
	NDS_EDM_MISC_ACC_CTL,
	NDS_EDM_MISC_EDM_PROBE,
	NDS_EDM_MISC_GEN_PORT0,
	NDS_EDM_MISC_GEN_PORT1,
};

/* EDM system registers */
/* Address Encoding of EDM System Register and DIMIR */
enum nds_edm_system_reg {
	NDS_EDM_SR_BPC0 = 0x00,
	NDS_EDM_SR_BPC1,
	NDS_EDM_SR_BPC2,
	NDS_EDM_SR_BPC3,
	NDS_EDM_SR_BPC4,
	NDS_EDM_SR_BPC5,
	NDS_EDM_SR_BPC6,
	NDS_EDM_SR_BPC7,
	NDS_EDM_SR_BPA0 = 0x08,
	NDS_EDM_SR_BPA1,
	NDS_EDM_SR_BPA2,
	NDS_EDM_SR_BPA3,
	NDS_EDM_SR_BPA4,
	NDS_EDM_SR_BPA5,
	NDS_EDM_SR_BPA6,
	NDS_EDM_SR_BPA7,
	NDS_EDM_SR_BPAM0 = 0x10,
	NDS_EDM_SR_BPAM1,
	NDS_EDM_SR_BPAM2,
	NDS_EDM_SR_BPAM3,
	NDS_EDM_SR_BPAM4,
	NDS_EDM_SR_BPAM5,
	NDS_EDM_SR_BPAM6,
	NDS_EDM_SR_BPAM7,
	NDS_EDM_SR_BPV0 = 0x18,
	NDS_EDM_SR_BPV1,
	NDS_EDM_SR_BPV2,
	NDS_EDM_SR_BPV3,
	NDS_EDM_SR_BPV4,
	NDS_EDM_SR_BPV5,
	NDS_EDM_SR_BPV6,
	NDS_EDM_SR_BPV7,
	NDS_EDM_SR_BPCID0 = 0x20,
	NDS_EDM_SR_BPCID1,
	NDS_EDM_SR_BPCID2,
	NDS_EDM_SR_BPCID3,
	NDS_EDM_SR_BPCID4,
	NDS_EDM_SR_BPCID5,
	NDS_EDM_SR_BPCID6,
	NDS_EDM_SR_BPCID7,
	NDS_EDM_SR_EDM_CFG = 0x28,
	NDS_EDM_SR_EDMSW = 0x30,
	NDS_EDM_SR_EDM_CTL = 0x38,
	NDS_EDM_SR_EDM_DTR = 0x40,
	NDS_EDM_SR_BPMTV = 0x48,
	NDS_EDM_SR_DIMBR = 0x50,
	NDS_EDM_SR_TECR0 = 0x70,
	NDS_EDM_SR_TECR1 = 0x71,
};

enum nds_memory_access {
	NDS_MEMORY_ACC_BUS = 0,
	NDS_MEMORY_ACC_CPU,
};

enum nds_memory_select {
	NDS_MEMORY_SELECT_AUTO = 0,
	NDS_MEMORY_SELECT_MEM = 1,
	NDS_MEMORY_SELECT_ILM = 2,
	NDS_MEMORY_SELECT_DLM = 3,
};

#define NDS_DBGER_DEX          (0x01 << 0)
#define NDS_DBGER_DPED         (0x01 << 1)
#define NDS_DBGER_CRST         (0x01 << 2)
#define NDS_DBGER_AT_MAX       (0x01 << 3)
#define NDS_DBGER_ILL_SEC_ACC  (0x01 << 4)

// DBGER[26]: GLOBAL_STALL, Indicates that the processor is stalled by the global_stall signal.
#define NDS_DBGER_GLOBAL_STALL (0x01 << 26)
// DBGER[27]: MEM_PEND, Indicates that the processor has pending memory accesses.
#define NDS_DBGER_MEM_PEND     (0x01 << 27)
// DBGER[28]: HDBG, Indicates that the processor is halted in the host debug mode.
#define NDS_DBGER_HDBG         (0x01 << 28)
// DBGER[29]: STANDBY, Indicates that the processor is in the standby mode.
#define NDS_DBGER_STANDBY      (0x01 << 29)

#define NDS_DBGER_ALL_SUPRS_EX (0x01 << 30)
#define NDS_DBGER_RESACC       (0x01 << 31)
#define NDS_DBGER_CLEAR_ALL    (0x1F)
#define NDS_DBGER_CLEAR_DEX    (0x01)

#define NDS_EDMSW_DETYPE_SHIFT (12)

#define NDS_EDMSW_WDV		(1 << 0)
#define NDS_EDMSW_RDV		(1 << 1)
#define NDS_EDMSW_DETYPE    (0xf << NDS_EDMSW_DETYPE_SHIFT)

#define NDS_EDMSW_DETYPE_BREAK      (0)
#define NDS_EDMSW_DETYPE_BREAK16    (1)
#define NDS_EDMSW_DETYPE_INSTBP     (2)
#define NDS_EDMSW_DETYPE_DAWP       (3)
#define NDS_EDMSW_DETYPE_DVWP       (4)
#define NDS_EDMSW_DETYPE_DVWPI      (5)
#define NDS_EDMSW_DETYPE_DBINT      (6)
#define NDS_EDMSW_DETYPE_HWSIGSTEP  (7)
#define NDS_EDMSW_DETYPE_DAWPN      (8)
#define NDS_EDMSW_DETYPE_DVWPN      (9)
#define NDS_EDMSW_DETYPE_LSINSTGSTP (10)

#define NDS_EDMCTL_DBGIM	(1 << 0)
#define NDS_EDMCTL_DBGACKM	(1 << 1)
#define NDS_EDMCTL_LDBGIM	(1 << 2)
#define NDS_EDMCTL_LDBGACKM	(1 << 3)
#define NDS_EDMCTL_LDSTOP	(1 << 4)
#define NDS_EDMCTL_STSTOP	(1 << 5)
#define NDS_EDMCTL_EDM_MODE	(1 << 6)
#define NDS_EDMCTL_MAX_STOP	(1 << 29)
#define NDS_EDMCTL_DEX_USE_PSW	(1 << 30)
#define NDS_EDMCTL_DEH_SEL	(1 << 31)

/* SDM Misc. Registers */
#define NDS_SDM_TAPID          0x1000163d
#define NDS_SDM_TAPID_DUMMY    0x0bad0bad

#define NDS_SDM_SELECT_MASK             0xC0000000
#define NDS_SDM_SELECT_SDM              0x80000000
#define NDS_SDM_SELECT_DIRECT           0x40000000
#define NDS_SDM_SELECT_DAISY_CHAIN      0x00000000

#define NDS_SDM_MISC_SDM_CFG            0
#define NDS_SDM_MISC_SBAR               1
#define NDS_SDM_MISC_PROCESSOR_CTL      2
#define NDS_SDM_MISC_PROCESSOR_STATUS   3
#define NDS_SDM_MISC_SDM_CTL            4

#define NDS_SDM_MISC_SDM_CFG_SDM_VER_MASK   (0xFFFF0000)
#define NDS_SDM_MISC_SDM_CFG_SDM_VER_BIT    16
#define NDS_SDM_MISC_SDM_CFG_SDM_PARALLEL_DBG_BUS    (0x1 << 15)
#define NDS_SDM_MISC_SDM_CFG_SDM_PROCESSOR_NUM_MASK  (0x0F)

#define NDS_SDM_MISC_PROCESSOR_CTL_TRST              (0x1 << 0)
#define NDS_SDM_MISC_PROCESSOR_CTL_POWERUP_REQ       (0x1 << 1)

#define NDS_SDM_MISC_PROCESSOR_STATUS_EVER_DISABLED  (0x1 << 0)
#define NDS_SDM_MISC_PROCESSOR_STATUS_POWERUP_ACK    (0x1 << 1)

#define NDS_SDM_MISC_SDM_CTL_PARALLEL_DBG_BUS_RESET  (0x80000000)
#define NDS_SDM_MISC_SDM_CTL_MEM_SEL_MASK            (0x00000007)


#endif /* __NDS32_EDM_H__ */
