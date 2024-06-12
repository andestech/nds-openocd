/******************************************************************************/
/* Nexus Trace Support                                                        */
/******************************************************************************/
#if _NDS_V5_ONLY_


/*****************************
 * DMI_NCETENC200
 *****************************/
#define DMI_TRTECONTROL      (0x000) /* Trace Encoder/Funnel Control Register */
#define	DMI_TRTEIMPL         (0x004) /* Trace Encoder/Funnel Implementation Information */
#define	DMI_TRTEINSTFEATURES (0x008) /* Extra Instruction Trace Encoder Features */
#define	DMI_TRTEINSTFILTERS  (0x00C) /* Determine which filters qualify an instruction trace */

/* Timestamp Register */
#define	DMI_TRTSCONTROL      (0x040) /* Timestamp Control Register */

/* External Trigger Register Interface */
#define	DMI_TRTETRIGEXTINCONT  (0x054) /* External Trigger Input Control Register */
#define	DMI_TRTETRIGEXTOUTCONT (0x058) /* External Trigger Output Control Register */

/* Trace Filter Control */
#define	DMI_TRTEFILTER0CONTROL          (0x400) /* Filter Control Register */
#define	DMI_TRTEFILTER0MATCHINST        (0x404) /* Filter Match Control Register 0 */
#define	DMI_TRTEFILTER0MATCHVALUEIMPDEF (0x410) /* Filter Match Control Register 1 */
#define	DMI_TRTEFILTER0MATCHMASKIMPDEF  (0x414) /* Filter Match Control Register 2 */

/* CoreSight Registers */
#define	DMI_DEVARCH_NEW      (0xFBC) /* Device Architecture Register */

/* (Obsolete) Miscellaneous Registers */
#define	DMI_DEVARCH          (0x1FBC) /* Device Architecture Register */
#define	DMI_ATBCONTROL       (0x1000) /* Control Registers for ATB Trace Sink */



/*****************************
 * DMI_NCETBUF200_RAMSINK
 *****************************/
#define DMI_TB_PIBSINKBASE_SHIFT (0x1000)
/* Function Mode Registers */
#define DMI_TRRAMCONTROL   (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x000)) /* Trace Buffer Control Register */
#define DMI_TRRAMIMPL      (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x004)) /* Trace Buffer Control Register */
#define DMI_TRRAMSTARTLOW  (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x010)) /* Trace Buffer Start Register */
#define DMI_TRRAMSTARTHIGH (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x014)) /* Trace Buffer Start High Bits Register */
#define DMI_TRRAMLIMITLOW  (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x018)) /* Trace Buffer Limit Register */
#define DMI_TRRAMLIMITHIGH (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x01C)) /* Trace Buffer Limit High Bits Register */
#define DMI_TRRAMWPLOW     (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x020)) /* Trace Buffer Write Pointer Register */
#define DMI_TRRAMWPHIGH    (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x024)) /* Trace Buffer Write Pointer High Bits Register */
#define DMI_TRRAMRPLOW     (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x028)) /* Trace Buffer Read Pointer Register */
#define DMI_TRRAMRPHIGH    (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x02C)) /* Trace Buffer Read Pointer High Bits Register */
#define DMI_TRRAMDATA      (ndsv5_comps[NDSV5_COMP_NCETBUF].addr + (0x040)) /* Trace Buffer Data Register */

/*****************************
 * DMI_NCETBUF200_PIBSINK
 *****************************/
#define DMI_TRPIBCONTROL (0x000 >> 2) /* Trace PIB Sink Control Register */


/*****************************
 * DMI_NCETMUX200
 *****************************/
/* Function Mode Registers */
#define DMI_TRFUNNELCONTROL (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0x000)) /* trFunnelControl, Control Register */
#define DMI_TRFUNNELIMPL    (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0x004)) /* trFunnelImpl, Implementation register */

/* CoreSight Integration Registers */
#define DMI_ITTMUXCTRL (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0xEFC)) /* Integration Test Control Register */

/* CoreSight Registers */
#define DMI_NCETMUX200_DEVARCH (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0xFBC)) /* DEVARCH, Device Architecture Register */

/* (Obsolete)?! */
#define TMUX_ITTMUXCTRL (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0x1EFC)) /* Integration Test Control Register */
#define TMUX_DEVARCH    (ndsv5_comps[NDSV5_COMP_NCETMUX].addr + (0x1FBC)) /* DEVARCH, Device Architecture Register */



/******************************************************************************************************************/
/* trFunnelActive Primary enable for trace funnel
When 0, the Trace Funnel may have clocks gated off or be powered down */
#define DMI_TRFUNNELCONTROL_trFunnelActive    0x01
/* trFunnelEnable Trace Funnel enabled */
#define DMI_TRFUNNELCONTROL_trFunnelEnable    0x02

/* Trace Encoder/Funnel Control Register */
#define DMI_TRTECONTROL_teSink_MASK      (0x0f << 28)
#define DMI_TRTECONTROL_teFormat_MASK    (0x07 << 24)
#define DMI_TRTECONTROL_teSyncMax_SHIFT  20
#define DMI_TRTECONTROL_teSyncMax_MASK   (0x0f << DMI_TRTECONTROL_teSyncMax_SHIFT)
#define DMI_TRTECONTROL_teSyncMode_SHIFT 16
#define DMI_TRTECONTROL_teSyncMode_MASK  (0x03 << DMI_TRTECONTROL_teSyncMode_SHIFT)

#define DMI_TRTECONTROL_teInhibitSrc     (0x01 << 15)
#define DMI_TRTECONTROL_teInstStallEn    (0x01 << 13)
#define DMI_TRTECONTROL_teInstStallOrOverflow (0x01 << 12)

#define DMI_TRTECONTROL_teInstTrigEn     (0x01 << 11)
#define DMI_TRTECONTROL_ndsSyncExtra     (0x01 << 9)  /* ndsTraceOnDebug ? */
#define DMI_TRTECONTROL_ndsTracePID      (0x01 << 9)
#define DMI_TRTECONTROL_ndsTracePRIV     (0x01 << 7)
#define DMI_TRTECONTROL_teInstMode_SHIFT 4
#define DMI_TRTECONTROL_teInstMode_MASK  (0x07 << DMI_TRTECONTROL_teInstMode_SHIFT)
#define DMI_TRTECONTROL_teEmpty          (0x01 << 3)
#define DMI_TRTECONTROL_teInstTracing    (0x01 << 2)
#define DMI_TRTECONTROL_teEnable         (0x01 << 1)
#define DMI_TRTECONTROL_teActive         (0x01 << 0)

/* teInstFeatures, Extra Instruction Trace Encoder Features */
#define	DMI_TRTEINSTFEATURES_trTeInstExtendAddrMSB      (0x01 << 10)
#define	DMI_TRTEINSTFEATURES_teInstEnJumpTargetCache    (0x01 << 5)
#define	DMI_TRTEINSTFEATURES_teInstEnBranchPrediction   (0x01 << 4)
#define	DMI_TRTEINSTFEATURES_teInstEnCallStack          (0x01 << 3)
#define	DMI_TRTEINSTFEATURES_teInstEnSequentialJump     (0x01 << 2)
#define	DMI_TRTEINSTFEATURES_teInstNoTrapAddr           (0x01 << 1)
#define	DMI_TRTEINSTFEATURES_teInstNoAddrDiff           (0x01 << 0)

/* tsControl,  Timestamp Control Register */
#define	DMI_TRTSCONTROL_tsWidth_SHIFT             24
#define	DMI_TRTSCONTROL_tsWidth_MASK              (0xff << DMI_TRTSCONTROL_tsWidth_SHIFT)
#define	DMI_TRTSCONTROL_ndsTimestampSelect_SHIFT  15
#define	DMI_TRTSCONTROL_ndsTimestampSelect_MASK   (0x01 << DMI_TRTSCONTROL_ndsTimestampSelect_SHIFT)
#define	DMI_TRTSCONTROL_tsPrescale_SHIFT          8
#define	DMI_TRTSCONTROL_tsPrescale_MASK           (0x03 << DMI_TRTSCONTROL_tsPrescale_SHIFT)
#define	DMI_TRTSCONTROL_tsType_SHIFT              4
#define	DMI_TRTSCONTROL_tsType_MASK               (0x07 << DMI_TRTSCONTROL_tsType_SHIFT)
#define	DMI_TRTSCONTROL_tsDebug                   (0x01 << 3)
#define	DMI_TRTSCONTROL_tsReset                   (0x01 << 2)
#define	DMI_TRTSCONTROL_tsCount                   (0x01 << 1)
#define	DMI_TRTSCONTROL_tsActive                  (0x01 << 0)

/* atbControl, Control Registers for ATB Trace Sink */
#define	DMI_ATBCONTROL_atbId_SHIFT     8
#define	DMI_ATBCONTROL_atbId_MASK      (0x7f << DMI_ATBCONTROL_atbId_SHIFT)
#define	DMI_ATBCONTROL_atbEmpty        (0x01 << 3)
#define	DMI_ATBCONTROL_atbEnable       (0x01 << 1)
#define	DMI_ATBCONTROL_atbActive       (0x01 << 0)

/* tfControl, Trace Buffer Control Register */
#define	DMI_TRRAMCONTROL_tfStopOnWrap  (0x01 << 8)
#define	DMI_TRRAMCONTROL_trRamModeSMEM (0x01 << 4)
#define	DMI_TRRAMCONTROL_tfEmpty       (0x01 << 3)
#define	DMI_TRRAMCONTROL_tfEnable      (0x01 << 1)
#define	DMI_TRRAMCONTROL_atbActive     (0x01 << 0)

#define	DMI_TRRAMCONTROL_tfSink_SHIFT     28
#define	DMI_TRRAMCONTROL_tfSink_MASK      (0x0f << DMI_TRRAMCONTROL_tfSink_SHIFT)
#define	DMI_TRRAMCONTROL_teFormat_SHIFT   24
#define	DMI_TRRAMCONTROL_teFormat_MASK    (0x07 << DMI_TRRAMCONTROL_teFormat_SHIFT)

/* Trace Buffer Write Pointer Register */
#define	DMI_TRRAMWPLOW_teWrap             (0x01 << 0)
#define	DMI_TRRAMWPLOW_teRamWp_MASK       (0xFFFFFFFC)


/* 6-MDO + 2-MSEO bits */
/* MSEO[1:0] Description */
#define MSEO_MASK       0x03 /* The start of a message, fixed-length fields,
				or in the middle of a variable-length fields;
				All messages start with fixed-length fields (6-bit TCODE)
				and these fields are packed together into bit [7:2] without any special marks. */
#define MSEO_START      0x00 /* The end of a variable-length field;
				variable-length fields are padded to align with MDO-boundaries. */
#define MSEO_END_FIELD  0x01
#define MSEO_END_MSG    0x03 /* 0x03 The end of message. */

/* TCODE,  Message Type (6-bits) */
enum NEXUS_TRACE_TCODE {
	TCODE_OwnershipTrace         =  2,
	TCODE_DirectBranch           =  3,
	TCODE_IndirectBranch         =  4,
	TCODE_Error                  =  8,
	TCODE_ProgramTraceSync       =  9,
	TCODE_DirectBranchSync       = 11,
	TCODE_IndirectBranchSync     = 12,
	TCODE_ResourceFull           = 27,
	TCODE_IndirectBranchHist     = 28,
	TCODE_IndirectBranchHistSync = 29,
	TCODE_ProgramCorrelation     = 33,
};

#endif /* _NDS_V5_ONLY_ */

