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
#include <target/target.h>
#include <target/nds32.h>
#include <target/nds32_log.h>
#include "aice_usb.h"
#include "aice_apis.h"
#include "aice_usb_command.h"

extern uint32_t nds32_reset_aice_as_startup;
extern struct vid_pid_s vid_pid_array[AICE_MAX_VID_NUM];
extern int vid_pid_array_top;
//extern unsigned int aice_num_of_target_id_codes;
extern char *pdescp_Manufacturer;
extern char *pdescp_Product;
extern unsigned int descp_bcdDevice;
int aice_efreq_support = 0;

/***************************************************************************/

char AndesName[] = {"Andes"};
#define AICE_PROD_MAX    5
char *pAICEName[AICE_PROD_MAX] = {
    "AICE",
    "AICE-MCU",
    "AICE-MINI",
    "AICE2-T",
    "AICE2",
};


/***************************************************************************/
const char *aice_clk_string[] = {
	"30 MHz",
	"15 MHz",
	"7.5 MHz",
	"3.75 MHz",
	"1.875 MHz",
	"937.5 KHz",
	"468.75 KHz",
	"234.375 KHz",
	"48 MHz",
	"24 MHz",
	"12 MHz",
	"6 MHz",
	"3 MHz",
	"1.5 MHz",
	"750 KHz",
	"375 KHz",
	""
};

static int aice_usb_open_device(struct aice_port_param_s *param)
{
	int retval = ERROR_FAIL;

	for( int i = 0; i <= vid_pid_array_top; i++ ) {
		param->vid = vid_pid_array[i].vid;
		param->pid = vid_pid_array[i].pid;

		retval = aice_usb_open(param->vid, param->pid);

		if( retval == ERROR_OK )
			break;
	}

	if( retval != ERROR_OK ) {
		NDS32_LOG(NDS32_ERRMSG_USB_OPEN, param->vid, param->pid);
		return ERROR_FAIL;
	}

	//TODO: find better place to do following stuff
	// EDM no need to reset, but to set V3 mode (when first connected, and after each target reset)
	// (EDM) V3 mode affect interrupt stack behavior
	// DIM BASE, DEH_SEL, ...
	if (nds32_reset_aice_as_startup == 1) {
		aice_reset_aice_as_startup();
	}

	if (ERROR_FAIL == aice_get_info()) {
		LOG_ERROR("Cannot get AICE info!");
		return ERROR_FAIL;
	}

	if ((aice_ice_config & (0x01 << 30)) == 0) {
		aice_set_write_pins_support(0);
	}
	else {
		aice_set_write_pins_support(1);
	}

	if ((aice_ice_config & (0x01 << 4)) == 0 )
		aice_efreq_support = 0;
	else
		aice_efreq_support = 1;


	if ((param->vid == 0x1CFC) && (param->pid == 0x0000)) {
		char *vid_str, *pid_str;
		unsigned int vid_idx, pid_idx;

		vid_idx = (aice_hardware_version & 0xFF000000) >> 24;
		pid_idx = (aice_hardware_version & 0x00FF0000) >> 16;
		vid_str = (char *)&AndesName[0];
		pid_str = (char *)pAICEName[pid_idx];

		if ((vid_idx == 0) && (pid_idx < AICE_PROD_MAX)) {
			NDS32_LOG(NDS32_MSG_SHOW_AICE_VERSION,
					vid_str,
					pid_str,
					(aice_hardware_version & 0xFFFF),
					aice_firmware_version,
					aice_fpga_version
				 );
		}
		else if (vid_idx == 0) {
			NDS32_LOG("Andes ICE: ice_ver1 = 0x%08x, ice_ver2 = 0x%08x, ice_ver3 = 0x%08x",
				aice_hardware_version, aice_firmware_version, aice_fpga_version);
		}
		else {
			NDS32_LOG(NDS32_MSG_SHOW_AICE_VENDOR,
				aice_hardware_version, aice_firmware_version, aice_fpga_version);
		}
	} else if ((param->vid == 0x1CFC) && (param->pid == 0x0001)) {
		char *vid_str = (char *)&AndesName[0];
		char *pid_str = "AICE-MINI+";
		NDS32_LOG(NDS32_MSG_SHOW_AICE_VERSION,
				vid_str,
				pid_str,
				(aice_hardware_version & 0xFFFF),
				aice_firmware_version,
				aice_fpga_version
			 );
	} else {
		// show get_descriptor_string
		NDS32_LOG("%s %s bcdDevice=0x%x", pdescp_Manufacturer, pdescp_Product, descp_bcdDevice);
	}
	return ERROR_OK;
}

static int aice_usb_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	int retval;
	unsigned char length = 0;

	retval = aice_scan_chain(idcode, &length);
	if ((retval != ERROR_OK) || (length == 0xFF)) {
		LOG_ERROR("No target connected");
		return ERROR_FAIL;
	} else if (length == AICE_MAX_NUM_CORE) {
		LOG_ERROR("The ice chain over 16 targets");
	} else {
		*num_of_idcode = (length + 1);
	}
	return retval;
}

int aice_usb_assert_srst(struct target *target, enum aice_srst_type_s srst)
{
	LOG_DEBUG("srst = 0x%x", (uint32_t)srst);
	if ((AICE_SRST != srst) && (AICE_RESET_HOLD != srst))
		return ERROR_FAIL;

	/* clear DBGER */
	if (aice_write_misc(target, NDS_EDM_MISC_DBGER,
				NDS_DBGER_CLEAR_ALL) != ERROR_OK)
		return ERROR_FAIL;

	int result = ERROR_OK;
	if (AICE_SRST == srst)
		result = aice_issue_srst(target);
	else {
		uint32_t tap_count = jtag_tap_count();
		LOG_DEBUG("tap_count = 0x%x", (uint32_t)tap_count);
		if (tap_count == 1)
			result = aice_issue_reset_hold(target);
		else
			result = aice_issue_reset_hold_multi(target);
	}

	/* Clear DBGER.CRST after reset to avoid 'core-reset checking' errors.
	 * assert_srst is user-intentional reset behavior, so we could
	 * clear DBGER.CRST safely.
	 */
//	for (target = all_targets; target; target = target->next) {
		if (aice_write_misc(target,
				NDS_EDM_MISC_DBGER, NDS_DBGER_CRST) != ERROR_OK)
		return ERROR_FAIL;
//	}
	return result;
}

int aice_get_state(void)
{
	uint32_t ice_state = 0xFF;

	int result = aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &ice_state);
	if (result != ERROR_OK) {
		NDS32_LOG(NDS32_ERRMSG_AICE_DISCONNECT);
		exit(-1);
		return ERROR_FAIL;
	}
	else if ((ice_state & 0x20) == 0) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

//=====================================================================================
int do_diagnosis_check_core(struct target *target)
{
    /// TODO HERE
	uint32_t coreid = target_to_coreid(target);
	//struct nds32 *nds32 = target_to_nds32(target);
	//struct nds32_memory *memory = &(nds32->memory);
    int i;
    uint32_t cpu_ver_value = 0;     // CR0
    uint32_t icm_cfg_value = 0;     // CR1
    uint32_t dcm_cfg_value = 0;     // CR2
    uint32_t mmu_cfg_value = 0;     // CR3
    uint32_t msc_cfg_value = 0;     // CR4
    uint32_t core_id_value = 0;     // CR5
    uint32_t edm_cfg_value = 0;     // DR40
    uint32_t ivb_value = 0;         // IR3
    uint32_t ice_config_value = 0;  // ICE-Box control register 0x4

    uint32_t fucop_exist_value = 0; // CR6
    uint32_t fucop_ctl_vaule = 0;   // FUCOP_CTL
    uint32_t fpcfg_value = 0;       // FPU ISA: FPCFG
    uint32_t freg = 0;
    char *fpu_type = NULL;
    char *freg_str = NULL;
    
    char *cpuid = NULL;
    uint32_t basev = 0;
    
    uint32_t audio = 0;
    
    uint32_t de = 0;
    uint32_t drde = 0;

    uint32_t line_size = 0;
    uint32_t way = 0;
    uint32_t sets = 0;
    uint32_t lock = 0;

    uint32_t ilmb_value;            // MR6
    uint32_t dlmb_value;            // MR7

    uint32_t lm_size = 0;
    uint32_t lm_capacity = 0;

    uint32_t mmps = 0;
    uint32_t mmpv = 0;
    uint32_t fatb = 0; 
    uint32_t fatbsz = 0;
    uint32_t tbw = 0;
    uint32_t tbs = 0;
    uint32_t ep8min4 = 0;
    uint32_t epsz = 0;
    uint32_t tblck = 0;
    uint32_t hptwk = 0; 
    uint32_t entries = 0;
    uint32_t page_size = 0;

    uint32_t dma_cfg_value = 0;     // DMAR0
    uint32_t nchannel = 0;

    uint32_t bc = 0;
    uint32_t dimu = 0;
    uint32_t dalm = 0;
    uint32_t mcd = 0;

    uint32_t nivic = 0;
    uint32_t ivic_ver = 0;

    aice_read_reg(target, CR0, &cpu_ver_value);
    aice_read_reg(target, CR1, &icm_cfg_value);
    aice_read_reg(target, CR2, &dcm_cfg_value);
    aice_read_reg(target, CR3, &mmu_cfg_value);
    aice_read_reg(target, CR4, &msc_cfg_value);
    aice_read_reg(target, CR5, &core_id_value);
    aice_read_reg(target, IR3, &ivb_value);

    aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, (uint32_t*)&edm_cfg_value);
    aice_read_ctrl(AICE_READ_CTRL_GET_JTAG_PIN_STATUS, &ice_config_value);
    
    switch( (cpu_ver_value&0xFF000000) >> 24 ) {
        case 0x06:
            cpuid = "N6";
            break;

        case 0x07:
            cpuid = "N7";
            break;

        case 0x08:
            cpuid = "N8";
            break;

        case 0x09:
            cpuid = "N903";
            break;

        case 0x0A:
            cpuid = "N1003/N1033";
            break;

        case 0x0C:
            cpuid = "N12";
            break;

        case 0x0D:
            cpuid = "N13";
            break;

        case 0x0F:
            cpuid = "N15";
            break;

        case 0x18:
            cpuid = "N820";
            break;

        case 0x19:
            cpuid = "N968";
            break;

        case 0x1A:
            cpuid = "N1008/N1068";
            break;

        case 0x28:
            cpuid = "E830";
            break;

        case 0xDA:
            cpuid = "D10";
            break;

        case 0xDF:
            cpuid = "D15";
            break;

        case 0xE8:
            cpuid = "E8";
            break;

        case 0xF8:
            cpuid = "S8";
            break;

        default:
            cpuid = "UNKNOWN";
            break;
    };
    //printf("\n");
    printf("CPU_VER=%08x(%s Rev:0x%02x) EDM_CFG=%08x\n", cpu_ver_value, cpuid, ((cpu_ver_value&0x00FF0000)>>16), edm_cfg_value);

    ice_config_value &= 0x3;
    if( ice_config_value == 0x1 ) 
        printf("2 wire JTAG (Basic Mode)\n");
    else if( ice_config_value == 0x3 )
        printf("2 wire JTAG (Fast Mode)\n");
    else
        printf("5 wire JTAG\n");
    printf("\n");

    basev = (msc_cfg_value&0xe000) >> 13;
   
    if( msc_cfg_value&0x100000 )
        printf("(v%dm)", basev+1);
    else
        printf("(v%d)", basev+1);

    if( msc_cfg_value&0xe00000 )
        printf(" (shadow_sp)");

    if( msc_cfg_value&0x400 )
        printf(" (gpr16)");
    else
        printf(" (gpr32)");
    
    if( msc_cfg_value&0x1000 )
        printf(" (intl2)");
    else
        printf(" (intl3)");

    if( msc_cfg_value&0x1000000 )
        printf(" (ex9)");
    
    if( cpu_ver_value&0x2 )
        printf(" (a16)");

    if( cpu_ver_value&0x1 )
        printf(" (pex)");

    if( cpu_ver_value&0x4 )
        printf(" (pex2)");

    if( msc_cfg_value&0x80000 )
        printf(" (ifc)");

    if( cpu_ver_value&0x10 )
        printf(" (str)");

    
    // MSC_CFG[6] is recycled by V3 architecture
    if( basev >= 2 ||  msc_cfg_value&0x40 )
        printf(" (mac)");

    // MSC_CFG[5] is recycled by V3 architecture
    if( basev >= 2 ||  msc_cfg_value&0x20 )
        printf(" (div)");

    if( msc_cfg_value&0x10000 )
        printf(" (no d0/d1)");
    printf("\n");


    /********************************************/
    // TODO:FPU
    if( cpu_ver_value&0x8 ) {
        printf("NDS_ISA_COP_SUPPORT");
        aice_read_reg(target, CR6, &fucop_exist_value);

        if( fucop_exist_value&0x1) {
            if( fucop_exist_value&0x80000000 ) {
                aice_read_reg(target, FUCPR, &fucop_ctl_vaule);

                // Enable FPU
                if( (fucop_ctl_vaule&0x1) == 0x0 ) {
                    aice_write_reg(target, FUCPR, fucop_ctl_vaule | 0x1);
                }

                aice_read_reg(target, FPCFG, &fpcfg_value);

                // Disable FPU, by restore FUCOP_CTL
                if( (fucop_ctl_vaule&0x1) == 0x0 ) {
                    aice_write_reg(target, FUCPR, fucop_ctl_vaule);
                }

                freg = (fpcfg_value&0xc) >> 2;

                if( fpcfg_value&0x2 ) {
                    fpu_type = "fpu";
                    if (freg == 0)
                        freg_str = "8s4d";
                    else if (freg == 1)
                        freg_str = "16s8d";
                    else if (freg == 2)
                        freg_str = "32s16d";
                    else
                        freg_str = "32s32d";
                } else {
                    fpu_type = "sfp";
                    if (freg == 0)
                        freg_str = "8s";
                    else if (freg == 1)
                        freg_str = "16s";
                    else
                        freg_str = "32s";
                }

                printf(" (%s %s)", fpu_type, freg_str);
            } else {
                printf(" (cop0)");
            }
        }

        if( fucop_exist_value&0x2 )
            printf(" (cop1)");

        if( fucop_exist_value&0x4 )
            printf(" (cop2)");

        if( fucop_exist_value&0x8 )
            printf(" (cop3)");

        printf("\n");
    }


    if( msc_cfg_value&0x180 ) {
        printf("NDS_ISA_ADSP_SUPPORT");
        audio = (msc_cfg_value&0x180) >> 7;

        if( audio == 1 )
            printf(" (32-bit only)");
        else if( audio == 2 )
            printf(" (24-bit only)");
        else
            printf(" (mixed 32/24-bit)");

        printf("\n");
    }


    if( msc_cfg_value&0x800 ) 
        printf("NDS_ADDR_WIDTH_24 (16MB)");
    else
        printf("NDS_ADDR_WIDTH_32 (4GB)");
    de   = mmu_cfg_value&0x4000000  >> 26;
    drde = mmu_cfg_value&0x80000000 >> 31;

    if( de )
        printf(" (big endian)");
    else
        printf(" (little endian)");

    if( de != drde ) {
        if( drde ) 
            printf(" (device: big endian)");
        else
            printf(" (device: little endian)");
    }
    printf("\n");


    if( icm_cfg_value&0x1c0 ) {
        printf("NDS_ICACHE_SUPPORT");

        line_size = (icm_cfg_value&0x1c0) >> 6;
        way       = (icm_cfg_value&0x38)  >> 3;
        sets      = (icm_cfg_value&0x7)   >> 0;
        lock      = (icm_cfg_value&0x200) >> 9;
        
        line_size = 1 << (line_size+2);
        way       = way + 1;
        sets      = 1 << (sets+6);

        printf(" (%dKB)", (line_size * sets * way)/1024);

        if( way > 1 )
            printf(" (%dways)", way);
        else
            printf(" (direct-mapped)");
        printf(" (line size=%dB)", line_size);

        if( lock )
            printf(" (lock)");

        printf("\n");
    } else {
        printf("// no icache\n");
    }


    if( dcm_cfg_value&0x1c0 ) {
        printf("NDS_DCACHE_SUPPORT");

        line_size = (dcm_cfg_value&0x1c0) >> 6;
        way       = (dcm_cfg_value&0x38)  >> 3;
        sets      = (dcm_cfg_value&0x7)   >> 0;
        lock      = (dcm_cfg_value&0x200) >> 9;
        
        line_size = 1 << (line_size+2);
        way       = way + 1;
        sets      = 1 << (sets+6);

        printf(" (%dKB)", (line_size * sets * way)/1024);

        if( way > 1 )
            printf(" (%dways)", way);
        else
            printf(" (direct-mapped)");
        printf(" (line size=%dB)", line_size);

        if( lock )
            printf(" (lock)");

        printf("\n");
    } else {
        printf("// no dcache\n");
    }


    if( icm_cfg_value&0x1c00 ) {
        printf("NDS_ILM_SUPPORT");
        aice_read_reg(target, MR6, &ilmb_value);

        lm_size = (ilmb_value&0x1e) >> 1;

        if( lm_size == 9 || lm_size == 10 )
            lm_capacity = 1 << (lm_size-9);
        else if( lm_size == 11 || lm_size == 12 )
            lm_capacity = 1 << lm_size;
        else if( lm_size == 15 )
            lm_capacity = 0;
        else
            lm_capacity = 1 << (lm_size+2);

        if( lm_capacity >= 1024 )
            printf(" (%dMB)", lm_capacity / 1024);
        else
            printf(" (%dKB)", lm_capacity);

        if( ilmb_value&0x1 )
            printf(" (boot from ilm@0x%08x)", (ilmb_value&0xfffffc00));

        printf("\n");
    } else {
        printf("// no ilm\n");
    }


    if( dcm_cfg_value&0x1c00 ) {
        printf("NDS_DLM_SUPPORT");
        aice_read_reg(target, MR7, &dlmb_value);

        lm_size = (dlmb_value&0x1e) >> 1;

        if( lm_size == 9 || lm_size == 10 )
            lm_capacity = 1 << (lm_size-9);
        else if( lm_size == 11 || lm_size == 12 )
            lm_capacity = 1 << lm_size;
        else if( lm_size == 15 )
            lm_capacity = 0;
        else
            lm_capacity = 1 << (lm_size+2);

        if( lm_capacity >= 1024 )
            printf(" (%dMB)", lm_capacity / 1024);
        else
            printf(" (%dKB)", lm_capacity);

        printf("\n");
    } else {
        printf("// no dlm\n");
    }


    mmps = mmu_cfg_value&0x3;
    if( mmps == 1 ) {
        mmpv = (mmu_cfg_value&0x7c) >> 2;

        if( mmpv&0x10 )
            printf("NDS_SMPU_SUPPORT");
        else
            printf("NDS_MPU_SUPPORT");

        printf(" (ver=0x%x)\n", mmpv);
    } else if( mmps == 2 ) {
        printf("NDS_MMU_SUPPORT");
        mmpv    = (mmu_cfg_value & 0x78)      >> 2;
        fatb    = (mmu_cfg_value & 0x8)       >> 7;
        fatbsz  = (mmu_cfg_value & 0x7f00)    >> 8; // overloaded field.
        tbw     = (mmu_cfg_value & 0x700)     >> 8;
        tbs     = (mmu_cfg_value & 0x3800)    >> 11;
        ep8min4 = (mmu_cfg_value & 0x8000)    >> 15;
        epsz    = (mmu_cfg_value & 0xff0000)  >> 16;
        tblck   = (mmu_cfg_value & 0x1000000) >> 24;
        hptwk   = (mmu_cfg_value & 0x2000000) >> 25;

        printf(" (version%d)", mmpv+1);
        if( hptwk ) 
            printf(" (hptwk)");

        if( tblck )
            printf(" (locking)");

        if( fatb == 1 ) 
            printf(" (%d entries) (fully-associative)", fatbsz);
        else {
            entries = (1 << (tbs+2)) * (tbw+1);
            printf(" (%d entries) (%dways)", entries, tbw+1);
        }

        if( ep8min4 )
            printf(" (8KB page)");

        for( i = 0; i < 7; i++ ) {
            if( epsz&(1 << i) ) {
                page_size = 1 << (2*i+4);

                if( page_size < 1024 )
                    printf(" (%dKB page)", page_size);
                else
                    printf(" (%dMB page)", page_size/1024);
            }
        }

        printf("\n");
    } else {
        printf("// no mmu/mpu\n");
    }

    if( msc_cfg_value&0x2 ) {
        printf("NDS_LDMA_SUPPORT");

        aice_read_reg(target, DMAR0, &dma_cfg_value);
        nchannel = (dma_cfg_value&0x3) + 1;

        printf(" (version=0x%04x)", (dma_cfg_value >> 16));
        printf(" (%d channels)", nchannel);

        if( dma_cfg_value&0x10 )
            printf(" (2D dma)");

        if( dma_cfg_value&0x8 )
            printf(" (unaligned external address)");
        
        printf("\n");
    }


    if( msc_cfg_value&0x4 )
        printf("NDS_PERFORMANCE_MONITOR_SUPPORT\n");


    if( msc_cfg_value&0x8 )
        printf("NDS_HSMP_SUPPORT\n");


    if( msc_cfg_value&0x10 )
        printf("NDS_EPT_SUPPORT\n");


    if( msc_cfg_value&0x200 )
        printf("NDS_L2CC_SUPPORT\n");


    if( msc_cfg_value&0x1 ) {
        printf("NDS_EDM_SUPPORT");

        bc   = edm_cfg_value&0x7;
        dimu = edm_cfg_value&0x8;
        dalm = edm_cfg_value&0x10;
        mcd  = edm_cfg_value&0x20;

        printf(" (ver=0x%04x)", (edm_cfg_value >> 16));
        printf(" (%d breakpoints)", bc + 1);

        if( dimu ) 
            printf(" (DIM)"); // we'll always have DIM

        if( dalm )
            printf(" (direct local memory access)");
        else
            printf(" (NO direct local memory access)");
        
        if( mcd )
            printf(" (multicore)");

        printf("\n");    
    }


    printf("interrupts:");
    nivic    = (ivb_value&0xf)    >> 1;
    ivic_ver = (ivb_value&0x1800) >> 11;
    
    switch( nivic ) {
        case 0:
            printf(" #ivic=6");
            break;

        case 1:
            printf(" #ivic=2");
            break;

        case 2:
            printf(" #ivic=10");
            break;

        case 3:
            printf(" #ivic=16");
            break;

        case 4:
            printf(" #ivic=24");
            break;

        case 5:
            printf(" #ivic=32");
            break;

        default:
            printf(" (#ivic=unknown");
            break;
    };

    if( ivic_ver == 0 )
        printf("; level only;");
    else
        printf("; level/edge;");

    if( ivb_value&0x1 )
        printf(" programmable priority");
    else
        printf(" fixed priority");

    if( ivb_value&0x2000 )
        printf(" (default=evic_mode)");
    else
        printf(" (default=ivic_mode)");

    printf("\n");

    fflush(stdout);
    return ERROR_OK;
}

enum{
		CONFIRM_USB_CONNECTION = 0,
		CONFIRM_AICE_VERSIONS,
		CONFIRM_JTAG_FREQUENCY_DETECTED,
		CONFIRM_SET_JTAG_FREQUENCY,
		CONFIRM_JTAG_CONNECTIVITY,
		CONFIRM_JTAG_DOMAIN,
		CONFIRM_TRST_WORKING,
		CONFIRM_SRST_NOT_AFFECT_JTAG,
		CONFIRM_SELECT_CORE,
		CONFIRM_RESET_HOLD,
		CONFIRM_DIM_AND_CPU_DOMAIN,
		CONFIRM_MEMORY_ON_BUS,
		CONFIRM_MEMORY_ON_CPU,
		CONFIRM_END
};

const char *confirm_messages[]={
	"check USB connectivity ...",
	"check AICE versions ...",
	"check the detected JTAG frequency ...",
	"check changing the JTAG frequency ...",
	"check JTAG connectivity ...",
	"check that JTAG domain is operational ...",
	"check that TRST resets the JTAG domain ...",
	"check that SRST does not resets JTAG domain ...",
	"check selecting core ...",
	"check reset-and-debug ...",
	"check that DIM and CPU domain are operational ...",
	"check accessing memory through BUS ...",
	"check accessing memory through CPU ..."
};

unsigned int confirm_result[CONFIRM_END];
const char *confirm_result_msg[]={
	"[PASS]",
	"[FAIL]",
	"[NON-SUPPORTED]",
};

extern int nds32_select_memory_mode(struct target *target, uint32_t address,
		uint32_t length, uint32_t *end_address);

static const int NDS32_LM_SIZE_TABLE[16] = {
	4 * 1024,
	8 * 1024,
	16 * 1024,
	32 * 1024,
	64 * 1024,
	128 * 1024,
	256 * 1024,
	512 * 1024,
	1024 * 1024,
	1 * 1024,
	2 * 1024,
};

uint32_t diagnosis_before_mr6 = 0, diagnosis_before_mr7 = 0;
static int do_diagnosis_update_lm_info(struct target *target)
{
	uint32_t coreid = target_to_coreid(target);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	uint32_t value_cr1 = 0, value_cr2 = 0;
	uint32_t value_mr6 = 0, value_mr7 = 0;
	uint32_t size_index;
	/* get EDM version */
	uint32_t value_edmcfg, edm_version;
	aice_read_edmsr(coreid, NDS_EDM_SR_EDM_CFG, &value_edmcfg);
	edm_version = (value_edmcfg >> 16) & 0xFFFF;
	if ((edm_version & 0x1000) || (0x60 <= edm_version))
		nds32->edm.access_control = true;
	else
		nds32->edm.access_control = false;

	if ((value_edmcfg >> 4) & 0x1)
		nds32->edm.direct_access_local_memory = true;
	else
		nds32->edm.direct_access_local_memory = false;

	if (edm_version <= 0x20)
		nds32->edm.direct_access_local_memory = false;

	aice_read_reg(target, CR1, &value_cr1);
	aice_read_reg(target, CR2, &value_cr2);
	memory->ilm_base = (value_cr1 >> 10) & 0x7;
	memory->ilm_align_ver = (value_cr1 >> 13) & 0x3;
	memory->dlm_base = (value_cr2 >> 10) & 0x7;
	memory->dlm_align_ver = (value_cr2 >> 13) & 0x3;

	if ((value_cr1 >> 10) & 0x7) { /* ILMB exists */
		aice_read_reg(target, MR6, &value_mr6);
		diagnosis_before_mr6 = value_mr6;
		value_mr6 |= 0x01;
		aice_write_reg(target, MR6, value_mr6);
		size_index = (value_mr6 >> 1) & 0xF;
		memory->ilm_size = NDS32_LM_SIZE_TABLE[size_index];

		memory->ilm_enable = true;
		if (memory->ilm_align_ver == 0) { /* 1MB aligned */
			memory->ilm_start = value_mr6 & 0xFFF00000;
			memory->ilm_end = memory->ilm_start + memory->ilm_size;
		} else if (memory->ilm_align_ver == 1) { /* aligned to local memory size */
			memory->ilm_start = value_mr6 & 0xFFFFFC00;
			memory->ilm_end = memory->ilm_start + memory->ilm_size;
		} else {
			memory->ilm_start = -1;
			memory->ilm_end = -1;
		}
	}
	if ((value_cr2 >> 10) & 0x7) { /* DLMB exists */
		aice_read_reg(target, MR7, &value_mr7);
		diagnosis_before_mr7 = value_mr7;
		value_mr7 |= 0x01;
		aice_write_reg(target, MR7, value_mr7);
		size_index = (value_mr7 >> 1) & 0xF;
		memory->dlm_size = NDS32_LM_SIZE_TABLE[size_index];

		memory->dlm_enable = true;
		if (memory->dlm_align_ver == 0) { /* 1MB aligned */
			memory->dlm_start = value_mr7 & 0xFFF00000;
			memory->dlm_end = memory->dlm_start + memory->dlm_size;
		} else if (memory->dlm_align_ver == 1) { /* aligned to local memory size */
			memory->dlm_start = value_mr7 & 0xFFFFFC00;
			memory->dlm_end = memory->dlm_start + memory->dlm_size;
		} else {
			memory->dlm_start = -1;
			memory->dlm_end = -1;
		}
	}
	LOG_DEBUG("diagnosis_update_lm_info: value_edmcfg=%x", value_edmcfg);
	LOG_DEBUG("diagnosis_update_lm_info: MR6=%x, MR7=%x", value_mr6, value_mr7);
	return ERROR_OK;
}

extern unsigned int diagnosis_memory;
extern unsigned int diagnosis_address;

int aice_usb_do_diagnosis(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	//struct aice_port_param_s aice_param;
	//struct aice_port_param_s *param = (struct aice_port_param_s *)&aice_param;
	unsigned int id_codes[AICE_MAX_NUM_CORE];
	uint32_t end_address, i = 0;
	//uint16_t vid = 0x1CFC;
	//uint16_t pid = 0x0;
	int last_fail_item, item;
	uint32_t value, backup_value, test_value;
	uint32_t selected_core;
	uint32_t hardware_version=0, firmware_version=0, fpga_version=0;
	uint32_t scan_clock;
	uint8_t test_memory_value=0, memory_value=0, backup_memory_value;
	uint8_t total_num_of_core = 0;
	int read_hold_success = 0;

	nds32_reg_init(nds32);

	printf("********************\n");
	printf("Diagnostic Report\n");
	printf("********************\n");

	for (i=0; i<CONFIRM_END; i++)
		confirm_result[i] = 1;
	/* 1. confirm USB connection */
	last_fail_item = CONFIRM_USB_CONNECTION;
	//if (ERROR_OK != aice_usb_open(vid, pid))
	//	goto report;
	if (ERROR_OK != aice_get_info())
		goto report;
	confirm_result[last_fail_item] = 0;
	/* clear timeout status */
	if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	/* clear NO_DBGI_PIN */
	uint32_t pin_status=0;
	if (aice_read_ctrl(AICE_READ_CTRL_GET_JTAG_PIN_STATUS, &pin_status) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status & (~0x02)) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	/* get hardware and firmware version */
	last_fail_item = CONFIRM_AICE_VERSIONS;
	if (aice_read_ctrl(AICE_READ_CTRL_GET_HARDWARE_VERSION, &hardware_version) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FIRMWARE_VERSION, &firmware_version) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FPGA_VERSION, &fpga_version) != ERROR_OK)
		goto diagnosis_JTAG_frequency_detected;

	confirm_result[last_fail_item] = 0;

diagnosis_JTAG_frequency_detected:
	/* 2. Report JTAG frequency detected */
	last_fail_item = CONFIRM_JTAG_FREQUENCY_DETECTED;
	if (aice_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL, AICE_TCK_CONTROL_TCK_SCAN) != ERROR_OK)
		goto diagnosis_set_JTAG_frequency;
	if (aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &scan_clock) != ERROR_OK)
		goto diagnosis_set_JTAG_frequency;

	if ((scan_clock & 0x20) == 0) {
		NDS32_LOG(NDS32_ERRMSG_TARGET_DISCONNECT);
		exit(-1);
	}

	scan_clock &= 0x0F;
	// If scan-freq = 48MHz, use 24MHz by default
	if (scan_clock == 8)
		scan_clock = 9;
	printf("JTAG frequency %s, %d\n", aice_clk_string[scan_clock], scan_clock);
	confirm_result[last_fail_item] = 0;

diagnosis_set_JTAG_frequency:
	scan_clock = jtag_clock;
	/* 3. [Optional] set user specified JTAG frequency */
	last_fail_item = CONFIRM_SET_JTAG_FREQUENCY;

	printf("set JTAG frequency %s ...\n", aice_clk_string[scan_clock + 1]);
	if (ERROR_OK != aice_set_jtag_clock(scan_clock + 1))
		goto diagnosis_JTAG_connect;
	printf("set JTAG frequency %s ...\n", aice_clk_string[scan_clock]);
	if (ERROR_OK != aice_set_jtag_clock(scan_clock))
		goto diagnosis_JTAG_connect;

	printf("ice_state = %08x\n", scan_clock);
	confirm_result[last_fail_item] = 0;

diagnosis_JTAG_connect:
	/* 4. Report JTAG scan chain (confirm JTAG connectivity) */
	last_fail_item = CONFIRM_JTAG_CONNECTIVITY;
	if (ERROR_OK != aice_port->api->idcode(&id_codes[0], &total_num_of_core))
		goto diagnosis_JTAG_domain;
	printf("There %s %u %s in target\n", total_num_of_core > 1 ? "are":"is", total_num_of_core, total_num_of_core > 1 ? "cores":"core");
	confirm_result[last_fail_item] = 0;

diagnosis_JTAG_domain:
	for(i = 0; i < total_num_of_core; i++) {
		/* get EDM version */
		uint32_t value_edmcfg, edm_version;
		aice_read_edmsr(i, NDS_EDM_SR_EDM_CFG, &value_edmcfg);
		edm_version = (value_edmcfg >> 16) & 0xFFFF;

		if( edm_version == 0x0 )
			continue;

		selected_core = i;
		target = coreid_to_target(i);
		// mark for AICE v1.6.6
		//aice_select_target(i,id_codes[i]);

		/* 5. Read/write SBAR: confirm JTAG domain working */
		last_fail_item = CONFIRM_JTAG_DOMAIN;
		test_value = rand() & 0xFFFFFD;

		aice_read_misc(target, NDS_EDM_MISC_SBAR, &backup_value);
		aice_write_misc(target, NDS_EDM_MISC_SBAR, test_value);
		aice_read_misc(target, NDS_EDM_MISC_SBAR, &value);
		//if (value != test_value)
		//	goto report;
		if(value == test_value)
			confirm_result[last_fail_item] = 0;
		/* 5.a */
		if (edm_version < 0x51) { //|| (cpu_version == 0x0c1c)) {
			confirm_result[CONFIRM_TRST_WORKING] = 2;
			confirm_result[CONFIRM_SRST_NOT_AFFECT_JTAG] = 2;
			goto diagnosis_JTAG_domain_skip_SRST;
		}

		last_fail_item = CONFIRM_TRST_WORKING;
		aice_write_misc(target, NDS_EDM_MISC_SBAR, 0);
		aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL, AICE_JTAG_PIN_CONTROL_TRST);
		aice_read_misc(target, NDS_EDM_MISC_SBAR, &value);
		//if(value != 0x1)
		//	goto report;
		if(value == 0x1)
			confirm_result[last_fail_item] = 0;
		/* 5.b */
		last_fail_item = CONFIRM_SRST_NOT_AFFECT_JTAG;
		aice_write_misc(target, NDS_EDM_MISC_SBAR, 0);
		//aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL, AICE_JTAG_PIN_CONTROL_SRST);
		aice_issue_srst(target);
		aice_read_misc(target, NDS_EDM_MISC_SBAR, &value);
		//if(value != 0x0)
		//	goto report;
		if(value == 0x0)
			confirm_result[last_fail_item] = 0;
		/* restore SBAR */
		aice_write_misc(target, NDS_EDM_MISC_SBAR, backup_value);
diagnosis_JTAG_domain_skip_SRST:
		/* 6. Select first Andes core based on scan chain */
		last_fail_item = CONFIRM_SELECT_CORE;
		aice_core_init(selected_core);
		//if (ERROR_OK != aice_edm_init(selected_core))
		//	goto report;
		if (ERROR_OK == aice_edm_init(selected_core))
			confirm_result[last_fail_item] = 0;
		printf("Core #%u: EDM version 0x%x\n", selected_core, core_info[selected_core].edm_version);

		/* 7. Restart target (debug and reset)*/
		last_fail_item = CONFIRM_RESET_HOLD;
		/* clear DBGER */
		if (aice_write_misc(target, NDS_EDM_MISC_DBGER, NDS_DBGER_CLEAR_ALL) != ERROR_OK)
			goto report;

		if (ERROR_OK == aice_issue_reset_hold(target)) {
			confirm_result[last_fail_item] = 0;
			read_hold_success = 1;
		}
		/* 8. Test read/write R0: confirm DIM and CPU domain operational */
		last_fail_item = CONFIRM_DIM_AND_CPU_DOMAIN;
		test_value = rand();
		aice_read_reg(target, R0, &backup_value);
		aice_write_reg(target, R0, test_value);
		aice_read_reg(target, R0, &value);
		//if (value != test_value)
		//	goto report;
		aice_write_reg(target, R0, backup_value);
		if (value == test_value)
			confirm_result[last_fail_item] = 0;

		if(diagnosis_memory) {
			do_diagnosis_update_lm_info(target);
			/* 9. Test direct memory access (bus view) confirm BUS working */
			last_fail_item = CONFIRM_MEMORY_ON_BUS;
			test_memory_value = rand();
			memory->access_channel = NDS_MEMORY_ACC_BUS;

			nds32_select_memory_mode(target, diagnosis_address, 1, &end_address);
			aice_read_mem_unit(target, diagnosis_address, 1, 1, &backup_memory_value);
			aice_write_mem_unit(target, diagnosis_address, 1, 1, &test_memory_value);
			aice_read_mem_unit(target, diagnosis_address, 1, 1, &memory_value);
			//if (memory_value != test_memory_value)
			//	goto report;
			if (memory_value == test_memory_value)
				confirm_result[last_fail_item] = 0;
			aice_write_mem_unit(target, diagnosis_address, 1, 1, &backup_memory_value);

			/* 10. Test memory access through DIM (CPU view) reconfirm CPU working */
			last_fail_item = CONFIRM_MEMORY_ON_CPU;
			test_memory_value = rand();
			memory->access_channel = NDS_MEMORY_ACC_CPU;

			nds32_select_memory_mode(target, diagnosis_address, 1, &end_address);
			aice_read_mem_unit(target, diagnosis_address, 1, 1, &backup_memory_value);
			aice_write_mem_unit(target, diagnosis_address, 1, 1, &test_memory_value);
			aice_read_mem_unit(target, diagnosis_address, 1, 1, &memory_value);
			//if (memory_value != test_memory_value)
			//	goto report;
			if (memory_value == test_memory_value)
				confirm_result[last_fail_item] = 0;
			aice_write_mem_unit(target, diagnosis_address, 1, 1, &backup_memory_value);
			if (diagnosis_before_mr6 != 0)
				aice_write_reg(target, MR6, diagnosis_before_mr6);
			if (diagnosis_before_mr7 != 0)
				aice_write_reg(target, MR7, diagnosis_before_mr7);
		}
		//last_fail_item = CONFIRM_END;

	report:
		printf("********************\n");
		for (item = CONFIRM_USB_CONNECTION; item <= last_fail_item; item++) {
			//if(!diagnosis_memory && (item == CONFIRM_MEMORY_ON_BUS || item == CONFIRM_MEMORY_ON_CPU))
			//	continue;
			printf("%s %s\n", confirm_result_msg[confirm_result[item]], confirm_messages[item]);
		}
		printf("********************\n");


		if( read_hold_success == 1 ) {
			printf("********************\n");
			printf("Check Bitmap Report\n");
			printf("********************\n");
			do_diagnosis_check_core(target);
			printf("(Report Done)\n********************\n\n\n");
		}

		fflush(stdout);
	}

	return ERROR_OK;
}

static int aice_usb_monitor_command( uint32_t nCmd, char **command, int *len, char **ret_data )
{
    LOG_DEBUG("aice_usb_monitor_command");

    uint32_t i;
    for( i = 0; i < nCmd; i++ ) {
        LOG_DEBUG("\t%d of CMD: len=%d, %s", i, len[i], command[i]);
    }

    *ret_data = (char*)malloc(4*sizeof(char));

    // LEN = 0
    ret_data[0] = ret_data[1] = ret_data[2] = ret_data[3] = 0;

    return ERROR_OK;
}

struct aice_nds32_api_s aice_nds32_usb = {
	/** */
	.write_ctrl = aice_usb_write_ctrl,
	/** */
	.read_ctrl = aice_usb_read_ctrl,
	/** AICE read_dtr */
	.read_dtr_to_buffer = aice_read_dtr_to_buffer,
	/** AICE write_dtr */
	.write_dtr_from_buffer = aice_write_dtr_from_buffer,
	/** AICE batch_buffer_write */
	.batch_buffer_write = aice_batch_buffer_write,
	/** AICE batch_buffer_read */
	.batch_buffer_read = aice_batch_buffer_read,
	/** */
	.execute_custom_script = aice_usb_execute_custom_script,
	/** */
	.set_command_mode = aice_usb_set_command_mode,
	/** AICE pack_buffer_read */
	.pack_buffer_read = aice_pack_buffer_read,
	/** */
	.monitor_command = aice_usb_monitor_command,
	/** */
	.xwrite = aice_icemem_xwrite,
	/** */
	.xread = aice_icemem_xread,
};

/** */
struct aice_port_api_s aice_usb_api = {
	/** */
	.open = aice_usb_open_device,
	/** */
	.close = aice_usb_close,
	/** */
	.reset = aice_usb_reset_box,
	/** */
	.idcode = aice_usb_idcode,
	/** */
	.set_jtag_clock = aice_usb_set_clock,
	/** */
	.assert_srst = aice_usb_assert_srst,
	/** */
	.state = aice_get_state,
	/** */
	.read_edm = aice_usb_read_edm,
	/** */
	.write_edm = aice_usb_write_edm,
	/** */
	.profiling = aice_usb_profile_entry,
	/** */
	.diagnosis = aice_usb_do_diagnosis,
	/** */
	.pnds32 = &aice_nds32_usb,
};
