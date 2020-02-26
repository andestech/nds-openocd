/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2019 Andes Technology, Ya-Ting Lin <yating@andestech.com>
 * Copyright (C) 2019 Hellosun Wu <wujiheng.tw@gmail.com>
 */

#ifndef __NDSV5_ENCODING_H_
#define __NDSV5_ENCODING_H_

/* AndeStar V5 Machine Mode - 9.2. Configuration Control & Status Registers */
#define CSR_MICM_CFG		0xFC0
#define CSR_MDCM_CFG		0xFC1
#define CSR_MMSC_CFG		0xFC2
#define CSR_MMSC_CFG2		0xFC3


/* AndeStar V5 Machine Mode - 9.3. Memory and Miscellaneous CSRs */
#define CSR_MILMB		0x7C0
#define CSR_MDLMB		0x7C1
#define CSR_MECC_CODE		0x7C2
#define CSR_MNVEC		0x7C3
#define CSR_MPFT_CTL		0x7C5
#define CSR_MCACHE_CTL		0x7CA
#define CSR_MMISC_CTL		0x7D0
#define CSR_MCCTLBEGINADDR	0x7CB
#define CSR_MCCTLCOMMAND	0x7CC
#define CSR_MCCTLDATA		0x7CD
#define CSR_MPPIB		0x7F0
#define CSR_MFIOB		0x7F1


/* AndeStar V5 Machine Mode - 9.4. Trap related CSRs */
#define CSR_MXSTATUS		0x7C4
#define CSR_MDCAUSE		0x7C9
#define CSR_MSLIDELEG		0x7D5
#define CSR_MSAVESTATUS		0x7D6
#define CSR_MSAVEEPC1		0x7D7
#define CSR_MSAVECAUSE1		0x7D8
#define CSR_MSAVEEPC2		0x7D9
#define CSR_MSAVECAUSE2		0x7DA
#define CSR_MSAVEDCAUSE1	0x7DB
#define CSR_MSAVEDCAUSE2	0x7DC


/* AndeStar V5 Machine Mode - 9.5. Hardware Stack Protection and Recording CSRs */
#define CSR_MHSP_CTL		0x7C6
#define CSR_MSP_BOUND		0x7C7
#define CSR_MSP_BASE		0x7C8


/* AndeStar V5 Machine Mode - 9.6. Counter Related CSRs */
#define CSR_MCOUNTERWEN		0x7CE
#define CSR_MCOUNTERMASK_M	0x7D1
#define CSR_MCOUNTERMASK_S	0x7D2
#define CSR_MCOUNTERMASK_U	0x7D3
#define CSR_MCOUNTERINTEN	0x7CF
#define CSR_MCOUNTEROVF		0x7D4


/* AndeStar V5 Machine Mode - 9.7. Enhanced CLIC Related CSRs */
/* bug17785 comment9 :remove other csrs */
#define CSR_MIRQ_ENTRY		0x7EC
/*
#define CSR_MINTSEL_JAL		0x7ED
#define CSR_PUSHMCAUSE		0x7EE
#define CSR_PUSHMEPC		0x7EF
#define CSR_PUSHMXSTATUS	0x7EB
*/

/* AndeStar V5 Machine Mode - 9.8. Physical Memory Attribute Unit CSRs */
#define CSR_PMACFG0		0xBC0
#define CSR_PMACFG1		0xBC1
#define CSR_PMACFG2		0xBC2
#define CSR_PMACFG3		0xBC3
#define CSR_PMAADDR0		0xBD0
#define CSR_PMAADDR1		0xBD1
#define CSR_PMAADDR2		0xBD2
#define CSR_PMAADDR3		0xBD3
#define CSR_PMAADDR4		0xBD4
#define CSR_PMAADDR5		0xBD5
#define CSR_PMAADDR6		0xBD6
#define CSR_PMAADDR7		0xBD7
#define CSR_PMAADDR8		0xBD8
#define CSR_PMAADDR9		0xBD9
#define CSR_PMAADDR10		0xBDA
#define CSR_PMAADDR11		0xBDB
#define CSR_PMAADDR12		0xBDC
#define CSR_PMAADDR13		0xBDD
#define CSR_PMAADDR14		0xBDE
#define CSR_PMAADDR15		0xBDF


/* 10. AndeStar V5 Debug Mode Control and Status Registers */
#define CSR_DEXC2DBG		0x7E0
#define CSR_DDCAUSE		0x7E1


/* AndeStar V5 Supervisor Mode - 12.1. Trap related CSRs */
#define CSR_SLIE		0x9C4
#define CSR_SLIP		0x9C5
#define CSR_SDCAUSE		0x9C9


/* AndeStar V5 Supervisor Mode - 12.2. Control related CSRs */
#define CSR_SCCTLDATA		0x9CD
#define CSR_SMISC_CTL		0x9D0


/* AndeStar V5 Supervisor Mode - 12.3. Counter related CSRs */
#define CSR_SCOUNTERINTEN	0x9CF
#define CSR_SCOUNTERMASK_M	0x9D1
#define CSR_SCOUNTERMASK_S	0x9D2
#define CSR_SCOUNTERMASK_U	0x9D3
#define CSR_SCOUNTEROVF		0x9D4
#define CSR_SHPMEVENT3		0x9E3
#define CSR_SHPMEVENT4		0x9E4
#define CSR_SHPMEVENT5		0x9E5
#define CSR_SHPMEVENT6		0x9E6
#define CSR_SCOUNTINHIBIT	0x9E0


/* 13. AndeStar V5 User Mode Control and Status Registers */
#define CSR_UITB		0x800
#define CSR_UCODE		0x801
#define CSR_UDCAUSE		0x809
#define CSR_UCCTLBEGINADDR	0x80B
#define CSR_UCCTLCOMMAND	0x80C
#define CSR_WFE			0x810
#define CSR_SLEEPVALUE		0x811
#define CSR_TXEVT		0x812


/* RISC-V standard machine mode CSRs */
/* WARNNING: This field may be replaced by official(github/riscv) */
/* Counter related CSRs */
#define CSR_MCOUNTINHIBIT	0x320

/* (Removed) Other CSRs */
/*
#define CSR_MRANDSEQ		0x7FC
#define CSR_MRANDSEQH		0x7FD
#define CSR_MRANDSTATE		0x7FE
#define CSR_MRANDSTATEH		0x7FF
#define CSR_CYCLE		0xC00
#define CSR_TIME		0xC01
#define CSR_INSTRET		0xC02
#define CSR_HPMCOUNTER3		0xC03
#define CSR_CYCLEH		0xC80
#define CSR_TIMEH		0xC81
#define CSR_INSTRETH		0xC82
#define CSR_HPMCOUNTER3H	0xC83
*/

/* Sync 20220511: Removed by upstream */
#define CSR_USTATUS 0x0
#define CSR_UIE 0x4
#define CSR_UTVEC 0x5
#define CSR_USCRATCH 0x40
#define CSR_UEPC 0x41
#define CSR_UCAUSE 0x42
#define CSR_UTVAL 0x43
#define CSR_UIP 0x44


#endif /* __NDSV5_ENCODING_H_ */










#ifdef DECLARE_CSR
DECLARE_CSR(micm_cfg, CSR_MICM_CFG)
DECLARE_CSR(mdcm_cfg, CSR_MDCM_CFG)
DECLARE_CSR(mmsc_cfg, CSR_MMSC_CFG)
DECLARE_CSR(mmsc_cfg2, CSR_MMSC_CFG2)

DECLARE_CSR(milmb, CSR_MILMB)
DECLARE_CSR(mdlmb, CSR_MDLMB)
DECLARE_CSR(mecc_code, CSR_MECC_CODE)
DECLARE_CSR(mnvec, CSR_MNVEC)
DECLARE_CSR(mpft_ctl, CSR_MPFT_CTL)
DECLARE_CSR(mcache_ctl, CSR_MCACHE_CTL)
DECLARE_CSR(mmisc_ctl, CSR_MMISC_CTL)
DECLARE_CSR(mcctlbeginaddr, CSR_MCCTLBEGINADDR)
DECLARE_CSR(mcctlcommand, CSR_MCCTLCOMMAND)
DECLARE_CSR(mcctldata, CSR_MCCTLDATA)
DECLARE_CSR(mppib, CSR_MPPIB)
DECLARE_CSR(mfiob, CSR_MFIOB)

DECLARE_CSR(mxstatus, CSR_MXSTATUS)
DECLARE_CSR(mdcause, CSR_MDCAUSE)
DECLARE_CSR(mslideleg, CSR_MSLIDELEG)
DECLARE_CSR(msavestatus, CSR_MSAVESTATUS)
DECLARE_CSR(msaveepc1, CSR_MSAVEEPC1)
DECLARE_CSR(msavecause1, CSR_MSAVECAUSE1)
DECLARE_CSR(msaveepc2, CSR_MSAVEEPC2)
DECLARE_CSR(msavecause2, CSR_MSAVECAUSE2)
DECLARE_CSR(msavedcause1, CSR_MSAVEDCAUSE1)
DECLARE_CSR(msavedcause2, CSR_MSAVEDCAUSE2)

DECLARE_CSR(mhsp_ctl, CSR_MHSP_CTL)
DECLARE_CSR(msp_bound, CSR_MSP_BOUND)
DECLARE_CSR(msp_base, CSR_MSP_BASE)

DECLARE_CSR(mcounterwen, CSR_MCOUNTERWEN)
DECLARE_CSR(mcountermask_m, CSR_MCOUNTERMASK_M)
DECLARE_CSR(mcountermask_s, CSR_MCOUNTERMASK_S)
DECLARE_CSR(mcountermask_u, CSR_MCOUNTERMASK_U)
DECLARE_CSR(mcounterinten, CSR_MCOUNTERINTEN)
DECLARE_CSR(mcounterovf, CSR_MCOUNTEROVF)

DECLARE_CSR(mirq_entry, CSR_MIRQ_ENTRY)
/*
DECLARE_CSR(mintsel_jal, CSR_MINTSEL_JAL)
DECLARE_CSR(pushmcause, CSR_PUSHMCAUSE)
DECLARE_CSR(pushmepc, CSR_PUSHMEPC)
DECLARE_CSR(pushmxstatus, CSR_PUSHMXSTATUS)
*/

DECLARE_CSR(pmacfg0, CSR_PMACFG0)
DECLARE_CSR(pmacfg1, CSR_PMACFG1)
DECLARE_CSR(pmacfg2, CSR_PMACFG2)
DECLARE_CSR(pmacfg3, CSR_PMACFG3)
DECLARE_CSR(pmaaddr0, CSR_PMAADDR0)
DECLARE_CSR(pmaaddr1, CSR_PMAADDR1)
DECLARE_CSR(pmaaddr2, CSR_PMAADDR2)
DECLARE_CSR(pmaaddr3, CSR_PMAADDR3)
DECLARE_CSR(pmaaddr4, CSR_PMAADDR4)
DECLARE_CSR(pmaaddr5, CSR_PMAADDR5)
DECLARE_CSR(pmaaddr6, CSR_PMAADDR6)
DECLARE_CSR(pmaaddr7, CSR_PMAADDR7)
DECLARE_CSR(pmaaddr8, CSR_PMAADDR8)
DECLARE_CSR(pmaaddr9, CSR_PMAADDR9)
DECLARE_CSR(pmaaddr10, CSR_PMAADDR10)
DECLARE_CSR(pmaaddr11, CSR_PMAADDR11)
DECLARE_CSR(pmaaddr12, CSR_PMAADDR12)
DECLARE_CSR(pmaaddr13, CSR_PMAADDR13)
DECLARE_CSR(pmaaddr14, CSR_PMAADDR14)
DECLARE_CSR(pmaaddr15, CSR_PMAADDR15)

DECLARE_CSR(dexc2dbg, CSR_DEXC2DBG)
DECLARE_CSR(ddcause, CSR_DEXC2DBG)

DECLARE_CSR(slie, CSR_SLIE)
DECLARE_CSR(slip, CSR_SLIP)
DECLARE_CSR(sdcause, CSR_SDCAUSE)

DECLARE_CSR(scctldata, CSR_SCCTLDATA)
DECLARE_CSR(smisc_ctl, CSR_SMISC_CTL)

DECLARE_CSR(scounterinten, CSR_SCOUNTERINTEN)
DECLARE_CSR(scountermask_m, CSR_SCOUNTERMASK_M)
DECLARE_CSR(scountermask_s, CSR_SCOUNTERMASK_S)
DECLARE_CSR(scountermask_u, CSR_SCOUNTERMASK_U)
DECLARE_CSR(scounterovf, CSR_SCOUNTEROVF)
DECLARE_CSR(shpmevent3, CSR_SHPMEVENT3)
DECLARE_CSR(shpmevent4, CSR_SHPMEVENT4)
DECLARE_CSR(shpmevent5, CSR_SHPMEVENT5)
DECLARE_CSR(shpmevent6, CSR_SHPMEVENT6)
DECLARE_CSR(scountinhibit, CSR_SCOUNTINHIBIT)

DECLARE_CSR(uitb, CSR_UITB)
DECLARE_CSR(ucode, CSR_UCODE)
DECLARE_CSR(udcause, CSR_UDCAUSE)
DECLARE_CSR(ucctlbeginaddr, CSR_UCCTLBEGINADDR)
DECLARE_CSR(ucctlcommand, CSR_UCCTLCOMMAND)
DECLARE_CSR(wfe, CSR_WFE)
DECLARE_CSR(sleepvalue, CSR_SLEEPVALUE)
DECLARE_CSR(txevt, CSR_TXEVT)

DECLARE_CSR(mcountinhibit, CSR_MCOUNTINHIBIT)

/* Sync 20220511: Removed by upstream */
DECLARE_CSR(ustatus, CSR_USTATUS)
DECLARE_CSR(uie, CSR_UIE)
DECLARE_CSR(utvec, CSR_UTVEC)
DECLARE_CSR(uscratch, CSR_USCRATCH)
DECLARE_CSR(uepc, CSR_UEPC)
DECLARE_CSR(ucause, CSR_UCAUSE)
DECLARE_CSR(utval, CSR_UTVAL)
DECLARE_CSR(uip, CSR_UIP)


#endif /* DECLARE_CSR */

