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

#ifndef __NDS32_TRACER_H__
#define __NDS32_TRACER_H__

#define NDS32_ETBID    0x1000363D
#define NDS32_ETMID    0x1000263D
#define NDS32_TPIUID   0x1000463D
#define NDS32_CURMONID 0x1001363D

//#define ETM_BASE       0x0000
//#define ETB_BASE       0x1000

/* ETB/TPIU Control Registers */
#define NDS_ETB_ID           (0x00)  // 0x0	ID	0x1000363D	RO
#define NDS_ETB_CFG1         (0x04)  // 0x4	CFG1	Revision, depth	RO
#define NDS_ETB_CFG2         (0x08)  // 0x8	CFG2	Reserved	RO
#define NDS_ETB_STATUS       (0x10)  // 0x10	STATUS	Enabled, resumed, wrap	RO
#define NDS_ETB_CTL          (0x14)  // 0x14	CTL	Enable, pause	RW
#define NDS_ETB_MODE         (0x18)  // 0x18	MODE
#define NDS_ETB_CAPTURE_CNT  (0x1C)  // 0x1C	CAPTURE_CNT	The number of words, which are captured after a trigger. (count[N:2])	RW
#define NDS_ETB_RPTR         (0x30)  // 0x30	RPTR	Read point[31:2]	RW
#define NDS_ETB_RDATA        (0x34)  // 0x34	RDATA	Read data[31:0]	RO
#define NDS_ETB_WPTR         (0x38)  // 0x38	WPTR	Write pointer.	RW
#define NDS_ETB_PATTERN      (0x40)  // 0x40	PATTERN	Analysis pattern.	WO

/* ETB CFG1 */
#define ETB_CFG1_REVISION_SHIFT   16    // REVISION	31,16	0x1_00_0 (v1.0.0)	RO	0x1000
#define ETB_CFG1_REVISION_MASK    0xFFFF
#define ETB_CFG1_DEPTH_MASK       0x1F  // DEPTH	4,0	0: 4B
                                        //            1: 8B
                                        //            ...
                                        //            30: 4GB
                                        //            31: Reserved	RO	IM

/* ETB STATUS */
#define ETB_STATUS_WRAP      (0x01 << 2)  // WRAP	2	Indicate the buffer wraps around	RO	0
#define ETB_STATUS_PAUSED    (0x01 << 1)  // PAUSED	1	Indicate capture is paused	RO	1
#define ETB_STATUS_ENABLED   (0x01 << 0)  // ENABLED	0	Indicate ETB is enabled	RO	0

/* ETB CTL */
#define ETB_CTL_PAUSE        (0x01 << 1)  // PAUSE	1	0: resume trace capture
                                          //          1: pause trace capture	WO
#define ETB_CTL_ENABLE       (0x01 << 0)  // ENABLE	0	0: disabled ETB
                                          //          1: enabled ETB	WO

/* ETB MODE */
#define ETB_MODE_OVERWRITE   (0x01 << 0)  // 0 ETB is paused when trace buffer wraps around.
                                          // 1 ETB is not paused when trace buffer wraps around. 

/* ETM Control Registers */
#define NDS_ETM_ID           (0x00)	// 0x0	IDCODE	0x1000263D	RO
#define NDS_ETM_CFG1         (0x04)	// 0x4	CFG1  Revision, multi-source support, number of range comparators, RAS depth	RO
#define NDS_ETM_CFG2         (0x08)	// 0x8	CFG2  Reserved	RO
#define NDS_ETM_SRCID        (0x0C)	// 0xC	SRCID	Source ID of the ETM trace (CoreID[3:0])	RO
#define NDS_ETM_STATUS       (0x10)	// 0x10	STATUS	Enabled, triggered, empty	RO
#define NDS_ETM_CTL          (0x14)	// 0x14	CTL	Enable	WO
#define NDS_ETM_MODE         (0x18)	// 0x18	MODE	Trace format, stall, dbgi	RW*
#define NDS_ETM_SYNC_CTL     (0x1C)	// 0x1C	SYNC_CTL	Period, mode	RW*
#define NDS_ETM_TRIGGER_CTL  (0x20)	// 0x20	TRIGGER_CTL	Logical combination of events	RW*
#define NDS_ETM_FILTER_CTL   (0x24)	// 0x24	FILTER_CTL	Logical combination of events	RW*
#define NDS_ETM_RANGE0_BEG   (0x80)	// 0x80	RANGE0_BEG[31:0]		RW*
#define NDS_ETM_RANGE0_END   (0x88)	// 0x88	RANGE0_END[31:0]		RW*
#define NDS_ETM_RANGE1_BEG   (0x90)	// 0x90	RANGE1_BEG[31:0]		RW*
#define NDS_ETM_RANGE1_END   (0x98)	// 0x98	RANGE1_END[31:0]		RW*

/* ETM CFG1 */
#define ETM_CFG1_REVISION_SHIFT   16    // Revision	31,16	0x1_00_0 (v1.0.0)	RO	IM
#define ETM_CFG1_REVISION_MASK    0xFFFF
#define ETM_CFG1_RANGE_NUM_SHIFT  4     // RANGE_NUM	6,4	1-8 range comparators	RO	IM
#define ETM_CFG1_RANGE_NUM_MASK   0x07
#define ETM_CFG1_RAS_DEPTH_SHIFT  1     // RAS_DEPTH	3,1	0x0 = not support
#define ETM_CFG1_RAS_DEPTH_MASK   0x07  // 0x1-0x7 = 1, 2, 4, 8, 16, 32, 64	RO	IM
#define ETM_CFG1_MULTI_SOURCE     0x01  // MULTI_SOURCE	0	1 = support multiple sources(frame, sync)	RO	IM

/* ETM STATUS */
#define ETM_STATUS_ENABLED         (0x01 << 0)  // ENABLED 1 (0) Indicates ETM is enabled. RO 0
#define ETM_STATUS_TRIGGERED       (0x01 << 1)  // TRIGGERED 1 (1) Indicates ETM is triggered. RO 0
#define ETM_STATUS_EMPTY           (0x01 << 2)  // EMPTY 1 (2) Indicates ETM FIFO is empty. RO 0

/* ETM CTL */
#define ETM_CTL_DISABLE            (0x00 << 0)  // Enable   0   0: Disable
#define ETM_CTL_ENABLE             (0x01 << 0)  //          1: Enable	WO	0
/* ETM MODE */
#define ETM_MODE_CYCLE_ACCURATE    (0x01 << 0)  // CYCLE_ACCURATE	0	1: generate cycle count packet	RW	0
#define ETM_MODE_DIRECT_TARGET     (0x01 << 1)  // DIRECT_TARGET	1	1: Generate target address of direct branches	RW	0
#define ETM_MODE_CID               (0x01 << 2)  // CID	2	1: generate CID	RW	0
#define ETM_MODE_FULL_ACTION_MASK  (0x03 << 16) // FULL_ACTION	17,16	Action when FIFO is almost full
#define ETM_MODE_FULL_ACTION_NONE  (0x00 << 16) //    0x0: no action
#define ETM_MODE_FULL_ACTION_STALL (0x01 << 16) //    0x1: stall processor
#define ETM_MODE_FULL_ACTION_DBGI  (0x02 << 16) //    0x2: issue DBGI

/* ETM SYNC_CTL */
#define ETM_SYNC_CTL_PERIOD_MASK   (0x03 << 0)  // PERIOD	1, 0	The period to insert a SYNC0/SYNC1 packet.
#define ETM_SYNC_CTL_PERIOD_256B   (0x00 << 0)  // 0: 256B
#define ETM_SYNC_CTL_PERIOD_512B   (0x01 << 0)  // 1: 512B
#define ETM_SYNC_CTL_PERIOD_1KB    (0x02 << 0)  // 2: 1KB
#define ETM_SYNC_CTL_PERIOD_2KB    (0x03 << 0)  // 3: 2KB	RW	2

/* ETM TRIGGER EVENT ID */
#define ETM_TRIGGER_EVENT_A_TRUE      (0x00)  // 0x00	Always True
#define ETM_TRIGGER_EVENT_A_EXTERNAL  (0x08)  // 0x08-0x0F	External Input 0 ~ 7 (from other modules)
#define ETM_TRIGGER_EVENT_A_EDM       (0x10)  // 0x10-0x17	EDM Trigger Event 0 ~ 7 (only 2 is implemented)
#define ETM_TRIGGER_EVENT_A_RANGE     (0x18)  // 0x18-0x1F	Range 0 ~ 7 (to be described latter)

#define ETM_TRIGGER_EVENT_B_TRUE      (0x00 << 8)  // 0x00	Always True
#define ETM_TRIGGER_EVENT_B_EXTERNAL  (0x08 << 8)  // 0x08-0x0F	External Input 0 ~ 7 (from other modules)
#define ETM_TRIGGER_EVENT_B_EDM       (0x10 << 8)  // 0x10-0x17	EDM Trigger Event 0 ~ 7 (only 2 is implemented)
#define ETM_TRIGGER_EVENT_B_RANGE     (0x18 << 8)  // 0x18-0x1F	Range 0 ~ 7 (to be described latter)

/* ETM TRIGGER COMBINATION ID */
#define ETM_TRIGGER_A_OR_B          (0x00 << 24)  // 0	A || B
#define ETM_TRIGGER_A_AND_B         (0x01 << 24)  // 1	A && B
#define ETM_TRIGGER_A_XOR_B         (0x02 << 24)  // 2	A ^ B
#define ETM_TRIGGER_A_AND_NOT_B     (0x03 << 24)  // 3	A && !B
#define ETM_TRIGGER_NOT_A_AND_NOT_B (0x04 << 24)  // 4	!A && !B
#define ETM_TRIGGER_NOT_A_OR_NOT_B  (0x05 << 24)  // 5	!A || !B
#define ETM_TRIGGER_NOT_A_XOR_B     (0x06 << 24)  // 6	!A ^ B
#define ETM_TRIGGER_NOT_A_OR_B      (0x07 << 24)  // 7	!A || B

/* ICE-Box Peripheral Memory Map */
#define JTAGCTL_BASE   0x0000   // JTAG Controller (JTAGCTL)
#define TDRX_BASE      0x1000   // Trace Data Receiver (TDRX)
#define OTB_BASE       0x2000   // Off-Chip Trace Buffer Control Registers
#define CURMON_BASE    0x3000   // Current Monitor (CURMON)

#define XMEM_TDRX_ID          (TDRX_BASE+0x00) // 0x00 IDCODE, Tracer IDCODE Register (0x1001163d)
#define XMEM_TDRX_CTL         (TDRX_BASE+0x04) // 0x04 CTL, Tracer Control
#define XMEM_TDRX_STATUS      (TDRX_BASE+0x08) // 0x08 STATUS, Tracer Status
#define XMEM_TDRX_ERRST       (TDRX_BASE+0x0C) // 0x0C ERRST, Tracer Error Status

#define XMEM_OTB_ID           (OTB_BASE+0x00)	 // 0x00 IDCODE, OTB IDCODE register (0x1001263D)
#define XMEM_OTB_CFG1         (OTB_BASE+0x04)	 // 0x04 CFG1, OTB Configuration register
#define XMEM_OTB_STATUS       (OTB_BASE+0x10)	 // 0x10 STATUS, OTB Status register
#define XMEM_OTB_CTL          (OTB_BASE+0x14)	 // 0x14 CTL, OTB Control register
#define XMEM_OTB_RPTR         (OTB_BASE+0x30)	 // 0x30 RPTR, OTB Read Pointer register
#define XMEM_OTB_DATA         (OTB_BASE+0x34)	 // 0x34 RDATA, OTB Read Data register
#define XMEM_OTB_WPTR         (OTB_BASE+0x38)	 // 0x38 WPTR, OTB Write Pointer register

#define XMEM_CURMON_ID        (CURMON_BASE+0x00)
#define XMEM_CURMON_CFG1      (CURMON_BASE+0x04)
#define XMEM_CURMON_STATUS    (CURMON_BASE+0x10)
#define XMEM_CURMON_CTRL      (CURMON_BASE+0x14)
#define XMEM_CURMON_MODE      (CURMON_BASE+0x18)
#define XMEM_CURMON_RATE      (CURMON_BASE+0x1C)
#define XMEM_CURMON_RPTR      (CURMON_BASE+0x30)
#define XMEM_CURMON_RDATA     (CURMON_BASE+0x34)

#define XMEM_CURMON_STATUS_ENABLED   (0x01 << 0)
#define XMEM_CURMON_STATUS_EMPTY     (0x01 << 1)
#define XMEM_CURMON_STATUS_FULL      (0x01 << 2)
#define XMEM_CURMON_STATUS_OVERWRITE (0x01 << 3)

#define XMEM_CURMON_MODE_POWER      0
#define XMEM_CURMON_MODE_CURRENT    1
#define XMEM_CURMON_MODE_VOLTAGE    2
#define XMEM_CURMON_MODE_OVERWRITE  0x80000000

#endif /* __NDS32_TRACER_H__ */
