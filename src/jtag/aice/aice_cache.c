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

#include <target/target.h>
#include "aice_port.h"
#include "aice_apis.h"
#include "aice_jdp.h"
#include <target/nds32_log.h>

uint32_t backup_tlb_vpn_ = 0;
uint32_t backup_tlb_data_ = 0;
uint32_t backup_tlb_misc_ = 0;

#define MSC_CFG_L2C_MASK (0x00000200)


static void aice_save_tlb_regs(struct target *target, uint32_t memory_protection)
{
	aice_read_reg(target, MR2, &backup_tlb_vpn_);
	aice_read_reg(target, MR3, &backup_tlb_data_);
	/* SW-workaround, avoid generating exception
		Do NOT write mr3(TLB_DATA).M = 0
		Using of Reserved values (0, 4, 6) will generate Reserved PTE Attribute exception (Data). */
	if (backup_tlb_data_ == 0)
		backup_tlb_data_ = 0x00000006;
	if (memory_protection == 2)
		aice_read_reg(target, MR4, &backup_tlb_misc_);
}

static void aice_restore_tlb_regs(struct target *target, uint32_t memory_protection)
{
	aice_write_reg(target, MR2, backup_tlb_vpn_);
	aice_write_reg(target, MR3, backup_tlb_data_);
	if (memory_protection == 2)
		aice_write_reg(target, MR4, backup_tlb_misc_);
}
/*
uint32_t vrmpu_addr_mask[24] = {
	0xFFFFFC00,
	0xFFFFF800,
	0xFFFFF000,
	0xFFFFE000,
	0xFFFFC000,
	0xFFFF8000,
	0xFFFF0000,
	0xFFFE0000,
	0xFFFC0000,
	0xFFF80000,
	0xFFF00000,
	0xFFE00000,
	0xFFC00000,
	0xFF800000,
	0xFF000000,
	0xFE000000,
	0xFC000000,
	0xF8000000,
	0xF0000000,
	0xE0000000,
	0xC0000000,
	0x80000000,
	0x00000000,
	0x00000000,
};
*/
static int aice_read_tlb_mpu(struct target *target, uint32_t virtual_address,
		uint32_t *physical_address)
{
	LOG_DEBUG("aice_read_tlb_mpu, virtual address: 0x%08x", virtual_address);
	uint32_t instructions[4];
	uint32_t value_mr3 = 0;
	uint32_t virtual_offset;
	uint32_t physical_page_number;
	// for MPU
	//uint32_t way = 1;
	uint32_t set = 8;
	uint32_t si;
	//psz = 4096;
	si = (virtual_address >> 29) % set;

	if (aice_write_dtr(target, si << 29) != ERROR_OK)
		return ERROR_FAIL;

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_READ(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);
	//uint32_t value_mr2 = 0;
	//aice_read_reg(target, MR2, &value_mr2);// TLB_VPN
	aice_read_reg(target, MR3, &value_mr3);  // TLB_DATA
	if ((value_mr3 & 0x01) == 0) { // invalid
		*physical_address = virtual_address;
		return ERROR_OK;
	}
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);

	if ((mmu_config->memory_protection_version == 3) &&
		(mmu_config->fully_associative_tlb)) {
		// VRMPU
		LOG_DEBUG("VRMPU");
		*physical_address = virtual_address;
		return ERROR_OK;
		/*
		// to check idr1.VRMPU == 1 (disable)
		// uint32_t idr1_value = 0;
		// nds32_get_mapped_reg(nds32, IDR1, &idr1_value);
		// if (idr1_value & 0x1000)
		//   virtual_offset = virtual_address & 0x1fffffff;
		//   physical_page_number = value_mr3 & 0xFFFFF000;
		uint32_t addr_size = (value_mr3 & 0xFFFFFE00);
		uint32_t i, chk_bit, mask_idx;
		for (i=9; i<32; i++) {
			chk_bit = (0x01 << i);
			if ((addr_size & chk_bit) == 0)
				break;
		}
		mask_idx = (i - 9);
		virtual_offset = virtual_address & 0x1fffffff;
		physical_page_number = (addr_size & vrmpu_addr_mask[mask_idx]);
		*/
	} else if (mmu_config->memory_protection_version == 1) {
		virtual_offset = virtual_address & 0x1fffffff;
		physical_page_number = value_mr3 & 0xFFFFF000;
	} else {
		LOG_DEBUG("NOT Indexed Protection Unit");
		*physical_address = virtual_address;
		return ERROR_OK;
	}

	*physical_address = physical_page_number + virtual_offset;
	LOG_DEBUG("physical_address: 0x%08x", *physical_address);
	return ERROR_OK;
}

static int aice_read_tlb_mmu(struct target *target, uint32_t virtual_address,
		uint32_t *physical_address)
{
	LOG_DEBUG("aice_read_tlb_mmu, virtual address: 0x%08x", virtual_address);

	uint32_t instructions[4];
	uint32_t probe_result;
	uint32_t value_mr3;
	uint32_t value_mr4;
	uint32_t access_page_size;
	uint32_t virtual_offset;
	uint32_t physical_page_number;

	if (aice_write_dtr(target, virtual_address) != ERROR_OK)
		return ERROR_FAIL;

	/* probe TLB first */
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_PROBE(R1, R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	aice_read_reg(target, R1, &probe_result);

	if (probe_result & 0x80000000)
		return ERROR_FAIL;

	/* read TLB entry */
	if (aice_write_dtr(target, probe_result & 0x7FF) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_READ(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	aice_read_reg(target, MR3, &value_mr3);
	aice_read_reg(target, MR4, &value_mr4);

	access_page_size = value_mr4 & 0xF;
	if (0 == access_page_size) { /* 4K page */
		virtual_offset = virtual_address & 0x00000FFF;
		physical_page_number = value_mr3 & 0xFFFFF000;
	} else if (1 == access_page_size) { /* 8K page */
		virtual_offset = virtual_address & 0x00001FFF;
		physical_page_number = value_mr3 & 0xFFFFE000;
	} else if (5 == access_page_size) { /* 1M page */
		virtual_offset = virtual_address & 0x000FFFFF;
		physical_page_number = value_mr3 & 0xFFF00000;
	} else {
		return ERROR_FAIL;
	}

	*physical_address = physical_page_number | virtual_offset;

	return ERROR_OK;
}

int aice_read_tlb(struct target *target, uint32_t virtual_address, uint32_t *physical_address)
{
	int result;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	LOG_DEBUG("aice_read_tlb, memory_protection = %08x, memory_protection_version = %08x", mmu_config->memory_protection, mmu_config->memory_protection_version);

	if (mmu_config->memory_protection == 1) {
		aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_read_tlb_mpu(target, virtual_address, physical_address);
		aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else if (mmu_config->memory_protection == 2) {
		aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_read_tlb_mmu(target, virtual_address, physical_address);
		aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else {
		*physical_address = virtual_address;
		return ERROR_OK;
	}
}

//#define TLB_NUM 128
#define TLB_NUM 2048
#define MMU_CFG_offTBW      8   /* TLB ways(non-associative) TBS */
#define MMU_CFG_offTBS      11  /* TLB sets per way(non-associative) TBS */
#define MMU_CFG_mskTBW      ( 0x7UL << MMU_CFG_offTBW )
#define MMU_CFG_mskTBS      ( 0x7UL << MMU_CFG_offTBS )
unsigned int tlb_misc[TLB_NUM], tlb_vpn[TLB_NUM], tlb_data[TLB_NUM];

static int aice_dump_tlb_mpu(struct target *target, char *filename) 
{
    LOG_DEBUG("aice_dump_tlb_mpu");

    uint32_t instructions[4];
	uint32_t set = 8;   // for MPU
    uint32_t i;

    FILE *pFile; 
    pFile = fopen( filename, "w" );
    if( NULL == pFile ) {
        LOG_ERROR("Error!! Can't open file to write");
        return ERROR_FAIL;
    }

    for( i = 0; i < set; i++ ) {
        if (aice_write_dtr(target, i << 29) != ERROR_OK)
            return ERROR_FAIL;

        instructions[0] = MFSR_DTR(R0);
        instructions[1] = TLBOP_TARGET_READ(R0);
        instructions[2] = DSB;
        instructions[3] = BEQ_MINUS_12;
        aice_execute_dim(target, instructions, 4);

        aice_read_reg(target, MR2, &tlb_vpn[i]);    // TLB_VPN
        aice_read_reg(target, MR3, &tlb_data[i]);   // TLB_DATA

        LOG_DEBUG("idx:0x%08x, VPN:%08x, DATA:%08x\n", i, tlb_vpn[i], tlb_data[i] );
        fprintf(pFile, "idx:0x%08x, VPN:%08x, DATA:%08x\n", i, tlb_vpn[i], tlb_data[i] );

        keep_alive();
    }

    fclose(pFile);

    return ERROR_OK;
}

static int aice_dump_tlb_mmu(struct target *target, char *filename) 
{
    LOG_DEBUG( "aice_dump_tlb_mmu" );

    FILE *pFile; 
    pFile = fopen( filename, "w" );
    if( NULL == pFile ) {
        LOG_ERROR("Error!! Can't open file to write");
        return ERROR_FAIL;
    }

    uint32_t mmu_cfg;
	uint32_t instructions[4];
    uint32_t tlb_num;
    uint32_t i;

    /* WARRNING!!! NO FATBSZ Support */
    aice_read_reg(target, CR3, &mmu_cfg);
    tlb_num = (((mmu_cfg & MMU_CFG_mskTBW) >> MMU_CFG_offTBW) + 1) * 
              (1 << (((mmu_cfg & MMU_CFG_mskTBS) >> MMU_CFG_offTBS) + 2)); 

    LOG_DEBUG("Total %d entry in TLB", tlb_num);

    for( i = 0; i < tlb_num; i++ ) {
        /* read TLB entry */
        if (aice_write_dtr(target, i & 0x7FF) != ERROR_OK)
            return ERROR_FAIL;

        instructions[0] = MFSR_DTR(R0);
        instructions[1] = TLBOP_TARGET_READ(R0);
        instructions[2] = DSB;
        instructions[3] = BEQ_MINUS_12;
        aice_execute_dim(target, instructions, 4);

        aice_read_reg(target, MR2, &tlb_vpn[i]);    // TLB_VPN
        aice_read_reg(target, MR3, &tlb_data[i]);   // TLB_DATA
        aice_read_reg(target, MR4, &tlb_misc[i]);   // TLB_MISC

        LOG_DEBUG("idx:0x%08x, VPN:%08x, MISC:%08x, DATA:%08x\n",
                   i, tlb_vpn[i], tlb_misc[i], tlb_data[i] );
        fprintf(pFile, "idx:0x%08x, VPN:%08x, MISC:%08x, DATA:%08x\n",
                        i, tlb_vpn[i], tlb_misc[i], tlb_data[i] );
        keep_alive();
    }

    fclose(pFile);

    return ERROR_OK;
}

int aice_dump_tlb(struct target *target, char *filename) 
{
	int result;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	LOG_DEBUG("aice_dump_tlb, filename: %s, memory_protection = %08x, memory_protection_version = %08x", 
               filename, mmu_config->memory_protection, mmu_config->memory_protection_version);

	if (mmu_config->memory_protection == 1) {
		//aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_dump_tlb_mpu(target, filename);
		//aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else if (mmu_config->memory_protection == 2) {
		//aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_dump_tlb_mmu(target, filename);
		//aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else {
        LOG_ERROR( "Can't found memory portection unit, no support dump TLB ");
		return ERROR_OK;
	}
}

static int aice_dump_tlb_va_mpu(struct target *target, uint32_t va)
{
    LOG_DEBUG("aice_dump_tlb_va_mpu, virtual address: 0x%08x", va);
    
    uint32_t pa;
    uint32_t si;
    uint32_t set = 8;   // for MPU
    uint32_t value_mr2 = 0;
    uint32_t value_mr3 = 0;

    int retval = aice_read_tlb_mpu(target, va, &pa);
    if( retval == ERROR_OK ) {
        si = (va >> 29) % set;

        aice_read_reg(target, MR2, &value_mr2); // TLB_VPN
        aice_read_reg(target, MR3, &value_mr3); // TLB_DATA

        NDS32_LOG_LF("idx:0x%02x, VPN:%08x, DATA:%08x\n", si, value_mr2, value_mr3);
        LOG_INFO("idx:0x%02x, VPN:%08x, DATA:%08x", si, value_mr2, value_mr3);
        return ERROR_OK;
    } else {
        LOG_ERROR("Unable to dump tlb by VA");
        return ERROR_FAIL;
    }

    return ERROR_OK;
}

static int aice_dump_tlb_va_mmu(struct target *target, uint32_t va)
{
	LOG_DEBUG("aice_dump_tlb_va_mmu, virtual address: 0x%08x",va);

	uint32_t instructions[4];
	uint32_t probe_result;
	uint32_t value_mr2;
	uint32_t value_mr3;
	uint32_t value_mr4;

	if (aice_write_dtr(target, va) != ERROR_OK)
		return ERROR_FAIL;

	/* probe TLB first */
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_PROBE(R1, R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	aice_read_reg(target, R1, &probe_result);

	if (probe_result & 0x80000000) {
        LOG_INFO("Can't found VA in TLB");
        NDS32_LOG_LF("Can't found VA in TLB\n");
		return ERROR_FAIL;
    }

	/* read TLB entry */
	if (aice_write_dtr(target, probe_result & 0x7FF) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_READ(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(target, instructions, 4);

	aice_read_reg(target, MR2, &value_mr2);
	aice_read_reg(target, MR3, &value_mr3);
	aice_read_reg(target, MR4, &value_mr4);


    LOG_INFO("idx:0x%08x, VPN:%08x, MISC:%08x, DATA:%08x",
               (probe_result&0x7FF), value_mr2, value_mr3, value_mr4 );
    NDS32_LOG_LF("idx:0x%08x, VPN:%08x, MISC:%08x, DATA:%08x\n",
               (probe_result&0x7FF), value_mr2, value_mr3, value_mr4 );

    return ERROR_OK;
}


int aice_dump_tlb_va(struct target *target, uint32_t va) 
{
	int result;
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	LOG_DEBUG("aice_dump_tlb_va, va = %08x, memory_protection = %08x, memory_protection_version = %08x", 
               va, mmu_config->memory_protection, mmu_config->memory_protection_version);

	if (mmu_config->memory_protection == 1) {
		aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_dump_tlb_va_mpu(target, va);
		aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else if (mmu_config->memory_protection == 2) {
		aice_save_tlb_regs(target, mmu_config->memory_protection);
		result = aice_dump_tlb_va_mmu(target, va);
		aice_restore_tlb_regs(target, mmu_config->memory_protection);
		return result;
	}
	else {
        LOG_ERROR( "Can't found memory portection unit, no support dump TLB ");
		return ERROR_OK;
	}

}


/*****************************************************************************/

static int aice_init_cache(struct target *target)
{
	LOG_DEBUG("aice_init_cache");

	uint32_t value_cr1=0;
	uint32_t value_cr2=0;

	aice_read_reg(target, CR1, &value_cr1);
	aice_read_reg(target, CR2, &value_cr2);

	uint32_t coreid = target_to_coreid(target);
	struct cache_info *icache = &core_info[coreid].icache;

	icache->set = value_cr1 & 0x7;
	icache->log2_set = icache->set + 6;
	icache->set = 64 << icache->set;
	icache->way = ((value_cr1 >> 3) & 0x7) + 1;
	icache->line_size = (value_cr1 >> 6) & 0x7;
	if (icache->line_size != 0) {
		icache->log2_line_size = icache->line_size + 2;
		icache->line_size = 8 << (icache->line_size - 1);
	} else {
		icache->log2_line_size = 0;
	}

	LOG_DEBUG("\ticache set: %d, way: %d, line size: %d, "
			"log2(set): %d, log2(line_size): %d",
			icache->set, icache->way, icache->line_size,
			icache->log2_set, icache->log2_line_size);

	struct cache_info *dcache = &core_info[coreid].dcache;

	dcache->set = value_cr2 & 0x7;
	dcache->log2_set = dcache->set + 6;
	dcache->set = 64 << dcache->set;
	dcache->way = ((value_cr2 >> 3) & 0x7) + 1;
	dcache->line_size = (value_cr2 >> 6) & 0x7;
	if (dcache->line_size != 0) {
		dcache->log2_line_size = dcache->line_size + 2;
		dcache->line_size = 8 << (dcache->line_size - 1);
	} else {
		dcache->log2_line_size = 0;
	}

	LOG_DEBUG("\tdcache set: %d, way: %d, line size: %d, "
			"log2(set): %d, log2(line_size): %d",
			dcache->set, dcache->way, dcache->line_size,
			dcache->log2_set, dcache->log2_line_size);

	core_info[coreid].cache_init = true;

	return ERROR_OK;
}

static int aice_dcache_inval_all(struct target *target)
{
	LOG_DEBUG("aice_dcache_inval_all");

	uint32_t instructions[4];
	uint32_t coreid = target_to_coreid(target);
	struct cache_info *dcache = &core_info[coreid].dcache;
	uint32_t max_index = ((dcache->way - 1) << (dcache->log2_set + dcache->log2_line_size)) |
		((dcache->set-1) << dcache->log2_line_size);
	uint32_t value_cr4;

	LOG_DEBUG("MAX index: 0x%x", max_index);
	LOG_DEBUG("Decreasing size(line_size): %d", dcache->line_size);

	// Move cache index to R0
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, max_index))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Move decreasing to R1
	instructions[0] = MFSR_DTR(R1);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, dcache->line_size))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Inval all
	instructions[0] = L1D_IX_INVAL(R0);     // INVAL R0
	instructions[1] = SUB(R0, R1);          // R0 = R0 - R1
	instructions[2] = BGEZ_MINUS_8;         // if(R0 >= 0) branch -8
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// DSB
	instructions[0] = DSB;
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Check L2C
	aice_read_reg(target, CR4, &value_cr4);
	if( value_cr4 & MSC_CFG_L2C_MASK ) {
		// Assume CCTL Support L1D_INVALALL
		instructions[0] = L1D_INVALLALL;
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;

		if (ERROR_OK != aice_execute_dim(target, instructions, 4))
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_dcache_va_inval(struct target *target, uint32_t address)
{
	LOG_DEBUG("aice_dcache_va_inval");

	uint32_t instructions[4];
	uint32_t value_cr4;

	// Check L2C
	aice_read_reg(target, CR4, &value_cr4);

	if (aice_write_dtr(target, address) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);

	if( value_cr4 & MSC_CFG_L2C_MASK )
		instructions[1] = L1D_VA_INVAL_ALVL(R0);
	else
		instructions[1] = L1D_VA_INVAL(R0);

	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(target, instructions, 4);
}


///TODO: FiXME, Do not use this value
#define L2CC_PA_BASE		0xE0500000
uint32_t l2c_pa_base = (uint32_t)-1;

// NCEL2C100 Register offset
#define L2_CCTL_CMD		0x60
#define L2_CCTL_STATUS		0x64

// CCTL_ALL_CMD
#define CCTL_ALL_CMD            0x10
#define CCTL_CMD_L2_IX_WB       0x2

/******************************************************************************
 * L2_CCTL_STATUS_Mn (The L2CCTL command working status for Master n)
  *****************************************************************************/
#define L2_CCTL_STATUS_offCMD_COMP      31
#define L2_CCTL_STATUS_mskCMD_COMP      (0x1 << L2_CCTL_STATUS_offCMD_COMP)

static void aice_l2c_cmd_rdy(struct target *target)
{
	uint32_t cctl_status_value=0;

	do {
		aice_read_mem_unit(target, (l2c_pa_base+L2_CCTL_STATUS), 4, 1, (uint8_t *)&cctl_status_value);
	} while( (cctl_status_value&L2_CCTL_STATUS_mskCMD_COMP) == 0 );
}

static void aice_l2c_cmd(struct target *target, uint8_t cmd)
{
	LOG_DEBUG("L2C cmd: 0x%x", cmd);

	uint32_t value = 0;

	value |= cmd;
	aice_write_mem_unit(target, (l2c_pa_base+L2_CCTL_STATUS), 4, 1, (uint8_t *)&value);
}

static int aice_dcache_wb_all(struct target *target)
{
	LOG_DEBUG("aice_dcache_wb_all");

	uint32_t instructions[4];
	uint32_t coreid = target_to_coreid(target);
	struct cache_info *dcache = &core_info[coreid].dcache;
	uint32_t max_index = ((dcache->way - 1) << (dcache->log2_set + dcache->log2_line_size)) |
		((dcache->set-1) << dcache->log2_line_size);
	uint32_t value_cr4;

	LOG_DEBUG("MAX index: 0x%x", max_index);
	LOG_DEBUG("Decreasing size(line_size): %d", dcache->line_size);

	// Move cache index to R0
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, max_index))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Move decreasing to R1
	instructions[0] = MFSR_DTR(R1);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, dcache->line_size))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// WB all
	instructions[0] = L1D_IX_WB(R0);        // WB R0
	instructions[1] = SUB(R0, R1);          // R0 = R0 - R1
	instructions[2] = BGEZ_MINUS_8;         // if(R0 >= 0) branch -8
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// DSB
	instructions[0] = DSB;
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Check L2C
	aice_read_reg(target, CR4, &value_cr4);
	if( (value_cr4 & MSC_CFG_L2C_MASK) ||
	    (l2c_pa_base != (uint32_t)-1)     ) {
		/// Bug-15198
		/* Section 1: Writeback whole L1 cache */
		// Assume CCTL Support L1D_WBALL
		instructions[0] = L1D_WBALL_ALVL;
		instructions[1] = MSYNC_ALL;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;

		if (ERROR_OK != aice_execute_dim(target, instructions, 4))
			return ERROR_FAIL;

		/// Bug-15262, Add l2c base, use default to backward compatible
		if( l2c_pa_base == (uint32_t)-1 )
			l2c_pa_base = L2CC_PA_BASE;


		/* Section 2: Writeback whole L2 cache */
		//change to bus mode for read physical l2cc register address(mmu test)
		struct nds32 *nds32 = target_to_nds32(target);
		enum nds_memory_access access_channel = nds32->memory.access_channel;
		nds32->memory.access_channel = NDS_MEMORY_ACC_BUS;

		aice_l2c_cmd_rdy(target);
		aice_l2c_cmd(target, (CCTL_ALL_CMD | CCTL_CMD_L2_IX_WB));
		aice_l2c_cmd_rdy(target);

		//restore channel mode
		nds32->memory.access_channel = access_channel;
	}

	return ERROR_OK;
}

static int aice_dcache_va_wb(struct target *target, uint32_t address)
{
	LOG_DEBUG("aice_dcache_va_wb");

	uint32_t instructions[4];
	uint32_t value_cr4;

	// Chech CR4
	aice_read_reg(target, CR4, &value_cr4);

	if (aice_write_dtr(target, address) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);

	if( value_cr4 & MSC_CFG_L2C_MASK )
		instructions[1] = L1D_VA_WB_ALVL(R0);
	else
		instructions[1] = L1D_VA_WB(R0);

	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(target, instructions, 4);
}

static int aice_icache_inval_all(struct target *target)
{
	LOG_DEBUG("aice_icache_inval_all");

	uint32_t instructions[4];
	uint32_t coreid = target_to_coreid(target);
	struct cache_info *icache = &core_info[coreid].icache;
	uint32_t max_index = ((icache->way - 1) << (icache->log2_set + icache->log2_line_size)) |
		((icache->set-1) << icache->log2_line_size);
	uint32_t value_cr4;

	LOG_DEBUG("MAX index: 0x%x", max_index);
	LOG_DEBUG("Decreasing size(line_size): %d", icache->line_size);

	// Move cache index to R0
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, max_index))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Move decreasing to R1
	instructions[0] = MFSR_DTR(R1);
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_write_dtr(target, icache->line_size))
		return ERROR_FAIL;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Inval all
	instructions[0] = L1I_IX_INVAL(R0);     // INVAL R0
	instructions[1] = SUB(R0, R1);          // R0 = R0 - R1
	instructions[2] = BGEZ_MINUS_8;         // if(R0 >= 0) branch -8
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// ISB
	instructions[0] = ISB;
	instructions[1] = NOP;
	instructions[2] = NOP;
	instructions[3] = BEQ_MINUS_12;

	if (ERROR_OK != aice_execute_dim(target, instructions, 4))
		return ERROR_FAIL;

	// Check L2C
	aice_read_reg(target, CR4, &value_cr4);
	if( value_cr4 & MSC_CFG_L2C_MASK ) {
		// Assume CCTL Support L1D_INVALALL, L2 Cache doesn't split I/D-Cache
		instructions[0] = L1D_INVALLALL;
		instructions[1] = ISB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;

		if (ERROR_OK != aice_execute_dim(target, instructions, 4))
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_icache_va_inval(struct target *target, uint32_t address)
{
	LOG_DEBUG("aice_icache_va_inval");

	uint32_t instructions[4];
	uint32_t value_cr4;

	// Check L2C
	aice_read_reg(target, CR4, &value_cr4);

	if (aice_write_dtr(target, address) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);

	if( value_cr4 & MSC_CFG_L2C_MASK )
		instructions[1] = L1I_VA_INVAL_ALVL(R0);
	else
		instructions[1] = L1I_VA_INVAL(R0);

	instructions[2] = ISB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(target, instructions, 4);
}

static int aice_icache_ix_inval(struct target *target, uint32_t address)
{
	LOG_DEBUG("aice_icache_ix_inval, 0x%x", address);
	uint32_t instructions[4];
	uint32_t coreid = target->tap->abs_chain_position;
	int result;

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1I_IX_INVAL(R0);
	instructions[2] = ISB;
	instructions[3] = BEQ_MINUS_12;

	struct cache_info *icache = &core_info[coreid].icache;
	uint32_t ways = icache->way;
	uint32_t way_offset = 0x01 << (icache->log2_set + icache->log2_line_size);
	address &= (way_offset - 1);

	for ( uint32_t way = 0; way < ways; way++ ) {
		LOG_DEBUG("icache_ix_inval, way = 0x%x index = 0x%x", way, address);
		if (aice_write_dtr(target, address) != ERROR_OK)
			return ERROR_FAIL;
		result = aice_execute_dim(target, instructions, 4);
		if (result != ERROR_OK) {
			LOG_DEBUG("aice_icache_ix_inval 0x%x FAIL", address);
			return ERROR_FAIL;
		}
		address += way_offset;
	}
	return ERROR_OK;
}

static int aice_isync(struct target *target, uint32_t address)
{
	LOG_DEBUG("aice_isync");
	uint32_t instructions[4];

	if (aice_write_dtr(target, address) != ERROR_OK)
		return ERROR_FAIL;
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = ISYNC(R0);
	instructions[2] = ISB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(target, instructions, 4);
}

int aice_cache_ctl(struct target *target, uint32_t subtype, uint32_t address)
{
	LOG_DEBUG("aice_cache_ctl");

	int result;
	uint32_t coreid = target_to_coreid(target);

	if (core_info[coreid].cache_init == false)
		aice_init_cache(target);

	switch (subtype) {
		case AICE_CACHE_CTL_L1D_INVALALL:
			result = aice_dcache_inval_all(target);
			break;
		case AICE_CACHE_CTL_L1D_VA_INVAL:
			result = aice_dcache_va_inval(target, address);
			break;
		case AICE_CACHE_CTL_L1D_WBALL:
			result = aice_dcache_wb_all(target);
			break;
		case AICE_CACHE_CTL_L1D_VA_WB:
			result = aice_dcache_va_wb(target, address);
			break;
		case AICE_CACHE_CTL_L1I_INVALALL:
			result = aice_icache_inval_all(target);
			break;
		case AICE_CACHE_CTL_L1I_VA_INVAL:
			result = aice_icache_va_inval(target, address);
			break;
		case AICE_CACHE_CTL_L1I_IX_INVAL:
			result = aice_icache_ix_inval(target, address);
			break;
		case AICE_CACHE_CTL_LOOPCACHE_ISYNC:
			result = aice_isync(target, address);
			break;
		default:
			result = ERROR_FAIL;
			break;
	}

	return result;
}

struct cache_element {
    uint32_t pa;
    uint32_t wd;
    uint32_t cacheline[32];
    uint8_t dirty;
    uint8_t valid;
    uint8_t lock;
};

/* This is the number of cache entries. User should change it if necessary. */
#define CACHE_SET_NUM 0x1000
#define CACHE_WAY_NUM 0x8
struct cache_element ce[CACHE_SET_NUM][CACHE_WAY_NUM];
#define CCTL_mskDIRTY 0x400000
#define CCTL_offDIRTY 22
#define CCTL_mskVALID 0x2
#define CCTL_offVALID 1
#define CCTL_mskLOCK 0x1
#define CCTL_offLOCK 0
#define CCTL_mskTAG 0x3ffffc
#define CCTL_offTAG 0x2
#define NDS_PAGE_SHIFT      12          // for PAGE_SIZE 4KB
#define NDS_PAGE_SIZE       (1UL << NDS_PAGE_SHIFT)

int aice_dump_cache(struct target *target, unsigned int cache_type, const char* filename) 
{
	LOG_DEBUG( "Dump Cache" );

	uint32_t coreid = target_to_coreid(target);
	FILE *pFile; 
	uint32_t idx, way, ra, tag, i;
	uint32_t sets, ways, line_size, set_bits, line_bits, way_offset;
	uint32_t cache_index;
	uint32_t instructions_rtag[4];
	uint32_t instructions_rwd[4];
	struct cache_info *cache;
	uint32_t total_cache;
	uint32_t now_cache = 0;

	pFile = fopen( filename, "w" );
	if( NULL == pFile ) {
		LOG_ERROR("Error!! Can't open file to write");
		return ERROR_FAIL;
	}

	if (core_info[coreid].cache_init == false)
		aice_init_cache(target);

	if( cache_type == ICACHE ) 
		cache = &core_info[coreid].icache;
	else if ( cache_type == DCACHE )
		cache = &core_info[coreid].dcache;
	else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	ways = cache->way;
	sets = cache->set;
	set_bits = cache->log2_set;
	line_bits = cache->log2_line_size;
	line_size = cache->line_size;
	way_offset = set_bits + line_bits;

	LOG_DEBUG( "Way:%d, Set:%d, Line Size:%d", ways, sets, line_size); 

	total_cache = ways * sets * (line_size/4);
	LOG_DEBUG( "Total cache:%d", total_cache); 

	// Construct RTAG command
	instructions_rtag[0] = MFSR_DTR(R0);
	if( cache_type == ICACHE )
		instructions_rtag[1] = L1I_IX_RTAG(R0);
	else
		instructions_rtag[1] = L1D_IX_RTAG(R0);
	instructions_rtag[2] = DSB;
	instructions_rtag[3] = BEQ_MINUS_12;

	// Construct RWD command
	instructions_rwd[0] = MFSR_DTR(R0);
	if( cache_type == ICACHE )
		instructions_rwd[1] = L1I_IX_RWD(R0);
	else
		instructions_rwd[1] = L1D_IX_RWD(R0);
	instructions_rwd[2] = DSB;
	instructions_rwd[3] = BEQ_MINUS_12;


	fprintf(pFile, "dump %s\n", cache_type ? "DCACHE":"ICACHE");
	fprintf(pFile, "%8s %4s %4s %1s %1s %1s %8s %8s %8s %8s %8s %8s %8s %8s\n", "ADDRESS", "SET", "WAY", "V", "D", "L", "00", "04", "08", "0C", "10", "14", "18", "1C");
	for ( idx = 0; idx < sets; idx++ ) {
		for ( way = 0; way < ways; way++ ) {

			LOG_DEBUG("Now set:%d, way:%d", idx, way);

			// Read Tag first
			ra = (way << way_offset) | (idx << line_bits);
			if (ERROR_OK != aice_write_dtr(target, ra))
				return ERROR_FAIL;
			keep_alive();

			if (ERROR_OK != aice_execute_dim(target, instructions_rtag, 4))
				return ERROR_FAIL;
			keep_alive();

			aice_read_reg(target, R0, &tag);
			keep_alive();
			LOG_DEBUG("TAG VALUE: 0x%x", tag);
			ce[idx][way].dirty = (uint8_t)((tag & CCTL_mskDIRTY) >> CCTL_offDIRTY);
			ce[idx][way].valid = (uint8_t)((tag & CCTL_mskVALID) >> CCTL_offVALID);
			ce[idx][way].lock = (uint8_t)((tag & CCTL_mskLOCK) >> CCTL_offLOCK);
			ce[idx][way].pa = (tag & CCTL_mskTAG) >> CCTL_offTAG << NDS_PAGE_SHIFT;

			for( i = 0; i < line_size/4; i++ ) {
				cache_index = (ra | i << 2);

				if (ERROR_OK != aice_write_dtr(target, cache_index))
					return ERROR_FAIL;
				keep_alive();

				if (ERROR_OK != aice_execute_dim(target, instructions_rwd, 4))
					return ERROR_FAIL;
				keep_alive();

				aice_read_reg(target, R0, &ce[idx][way].cacheline[i]);
				keep_alive();
			}

			now_cache += line_size/4;
			NDS32_LOG_R("Dump Progressing...%d%%", ((now_cache*100)/total_cache));

			fprintf(pFile, "%08lx %04x %04x %01x %01x %01x ", 
					ce[idx][way].pa + ((idx * line_size) % NDS_PAGE_SIZE), 
					idx, 
					way, 
					ce[idx][way].valid, 
					ce[idx][way].dirty, 
					ce[idx][way].lock);
			for(i = 0; i < line_size/4; i++) {
				fprintf(pFile, "%08x ", ce[idx][way].cacheline[i]);
			}
			fprintf(pFile, "\n");
			keep_alive();
		}
	}
	NDS32_LOG("\nDump Finish!!");
	LOG_DEBUG("\nDump Finish!!");

	//LOG_DEBUG("dump %s\n", cache_type ? "DCACHE":"ICACHE");
	//LOG_DEBUG("%8s %4s %4s %1s %1s %1s %8s %8s %8s %8s %8s %8s %8s %8s\n", "ADDRESS", "SET", "WAY", "V", "D", "L", "00", "04", "08", "0C", "10", "14", "18", "1C");
	//for(idx = 0; idx < sets ; idx++) {
	//    for(way = 0; way < ways; way++) {
	//        LOG_DEBUG("%08lx %04x %04x %01x %01x %01x", 
	//                (ce[idx][way].pa + ((idx * line_size) % NDS_PAGE_SIZE)),
	//                idx,
	//                way,
	//                ce[idx][way].valid,
	//                ce[idx][way].dirty,
	//                ce[idx][way].lock); 
	//        for(i = 0; i < line_size/4; i++) {
	//                LOG_DEBUG("%08x ", ce[idx][way].cacheline[i]);
	//        }
	//    }
	//}

	//fprintf(pFile, "dump %s\n", cache_type ? "DCACHE":"ICACHE");
	//fprintf(pFile, "%8s %4s %4s %1s %1s %1s %8s %8s %8s %8s %8s %8s %8s %8s\n", "ADDRESS", "SET", "WAY", "V", "D", "L", "00", "04", "08", "0C", "10", "14", "18", "1C");
	//for(idx = 0; idx < sets ; idx++) {
	//    for(way = 0; way < ways; way++) {
	//        fprintf(pFile, "%08lx %04x %04x %01x %01x %01x ", 
	//                ce[idx][way].pa + ((idx * line_size) % NDS_PAGE_SIZE), 
	//                idx, 
	//                way, 
	//                ce[idx][way].valid, 
	//                ce[idx][way].dirty, 
	//                ce[idx][way].lock);
	//        for(i = 0; i < line_size/4; i++) {
	//              fprintf(pFile, "%08x ", ce[idx][way].cacheline[i]);
	//        }
	//        fprintf(pFile, "\n");
	//    }
	//}

	fclose(pFile);
	keep_alive();
	return ERROR_OK;
}

static uint32_t va2idx(struct target *target, uint32_t va, unsigned int cache_type, uint32_t *way_offset)
{
	struct cache_info *cache;
	uint32_t coreid = target_to_coreid(target);
	uint32_t set_bits, line_bits;
	uint32_t idx;

	if( cache_type == ICACHE ) 
		cache = &core_info[coreid].icache;
	else if ( cache_type == DCACHE )
		cache = &core_info[coreid].dcache;

	set_bits = cache->log2_set;
	line_bits = cache->log2_line_size;
	*way_offset = set_bits + line_bits;

	idx = (va & (((1 << set_bits) - 1) << line_bits));
	return idx;
}

extern int nds32_walk_page_table(struct nds32 *nds32, const target_addr_t virtual_address,
		target_addr_t *physical_address);
int aice_dump_cache_va(struct target *target, unsigned int cache_type, uint32_t va)
{
	LOG_DEBUG( "Dump Cache" );

	uint32_t coreid = target_to_coreid(target);
	uint32_t idx, way, ra, tag, i;
	uint32_t ways, line_size, line_bits, way_offset;
	uint32_t cache_index;
	uint32_t instructions_rtag[4];
	uint32_t instructions_rwd[4];
	struct cache_info *cache;
	struct cache_element ces[8];    // maximum DCM_CFG.DWAY is 8

	if (core_info[coreid].cache_init == false)
		aice_init_cache(target);

	if( cache_type == ICACHE ) 
		cache = &core_info[coreid].icache;
	else if ( cache_type == DCACHE )
		cache = &core_info[coreid].dcache;
	else {
		LOG_ERROR("%s not supported cache_type:%x", __func__, cache_type);
		return ERROR_FAIL;
	}

	ways = cache->way;
	line_size = cache->line_size;
	line_bits = cache->log2_line_size;

	// Construct RTAG command
	instructions_rtag[0] = MFSR_DTR(R0);
	if( cache_type == ICACHE )
		instructions_rtag[1] = L1I_IX_RTAG(R0);
	else
		instructions_rtag[1] = L1D_IX_RTAG(R0);
	instructions_rtag[2] = DSB;
	instructions_rtag[3] = BEQ_MINUS_12;

	// Construct RWD command
	instructions_rwd[0] = MFSR_DTR(R0);
	if( cache_type == ICACHE )
		instructions_rwd[1] = L1I_IX_RWD(R0);
	else
		instructions_rwd[1] = L1D_IX_RWD(R0);
	instructions_rwd[2] = DSB;
	instructions_rwd[3] = BEQ_MINUS_12;

	uint32_t pa = va; //check_addr, check_way = ways;
	//nds32_virtual_to_physical(target, va, (target_addr_t *)&pa);
	struct nds32 *nds32 = target_to_nds32(target);
	if (nds32->memory.address_translation == true && nds32->memory.va_to_pa_off == 0) {
		if (ERROR_OK != aice_read_tlb(target, va, &pa)) {
			nds32_walk_page_table(nds32, va, (target_addr_t *)&pa);
		}
	}
	LOG_DEBUG("physical_address:0x%x", pa);
	//if cpu_id_family > 13 and is dcache use pa to index, otherwise use va to index
	if (nds32->cpu_version.cpu_id_family > 0x0d && cache_type == DCACHE) {
		idx = va2idx(target, pa, cache_type, &way_offset);
	} else {
		idx = va2idx(target, va, cache_type, &way_offset);
	}
	//check_addr = (pa >> NDS_PAGE_SHIFT) << NDS_PAGE_SHIFT;

	for( way = 0; way < ways; way++ ) {
		// Read Tag first
		ra = (way << way_offset) | idx;
		if (ERROR_OK != aice_write_dtr(target, ra))
			return ERROR_FAIL;

		if (ERROR_OK != aice_execute_dim(target, instructions_rtag, 4))
			return ERROR_FAIL;


		aice_read_reg(target, R0, &tag);
		LOG_DEBUG("TAG VALUE: 0x%x", tag);
		ces[way].dirty = (uint8_t)((tag & CCTL_mskDIRTY) >> CCTL_offDIRTY);
		ces[way].valid = (uint8_t)((tag & CCTL_mskVALID) >> CCTL_offVALID);
		ces[way].lock = (uint8_t)((tag & CCTL_mskLOCK) >> CCTL_offLOCK);
		ces[way].pa = (tag & CCTL_mskTAG) >> CCTL_offTAG << NDS_PAGE_SHIFT;
		//if (check_way == ways && ces[way].pa == check_addr)
		//	check_way = way;

		for( i = 0; i < line_size/4; i++ ) {
			cache_index = (ra | i << 2);

			if (ERROR_OK != aice_write_dtr(target, cache_index))
				return ERROR_FAIL;

			if (ERROR_OK != aice_execute_dim(target, instructions_rwd, 4))
				return ERROR_FAIL;

			aice_read_reg(target, R0, &ces[way].cacheline[i]);
		}

		keep_alive();
	}

	NDS32_LOG_LF("dump %s\n", cache_type ? "DCACHE":"ICACHE");
	NDS32_LOG_LF("%8s %4s %4s %1s %1s %1s %8s %8s %8s %8s %8s %8s %8s %8s\n", "ADDRESS", "SET", "WAY", "V", "D", "L", "00", "04", "08", "0C", "10", "14", "18", "1C");
	for(way = 0; way < ways; way++) {
	//	if (way == check_way)
	//		NDS32_LOG_LF("*");
		NDS32_LOG_LF("%08lx %04x %04x %01x %01x %01x ", 
				//(ces.pa + ((idx * line_size) % NDS_PAGE_SIZE)),
				(ces[way].pa + (((idx >> line_bits) * line_size) % NDS_PAGE_SIZE)),
				(idx >> line_bits),
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock);
		for(i = 0; i < line_size/4; i++) {
			NDS32_LOG_LF("%08x ", ces[way].cacheline[i]);
		}
		NDS32_LOG_LF("\n");
	}

	LOG_INFO("dump %s", cache_type ? "DCACHE":"ICACHE");
	LOG_INFO("%8s %4s %4s %1s %1s %1s %8s %8s %8s %8s %8s %8s %8s %8s", "ADDRESS", "SET", "WAY", "V", "D", "L", "00", "04", "08", "0C", "10", "14", "18", "1C");
	for(way = 0; way < ways; way++) {
	//	if (way == check_way)
	//		LOG_INFO("*");
		LOG_INFO("%08lx %04x %04x %01x %01x %01x ", 
				//(ces.pa + ((idx * line_size) % NDS_PAGE_SIZE)),
				(ces[way].pa + (((idx >> line_bits) * line_size) % NDS_PAGE_SIZE)),
				(idx >> line_bits),
				way,
				ces[way].valid,
				ces[way].dirty,
				ces[way].lock); 
		for(i = 0; i < line_size/4; i++) {
			LOG_INFO("%08x ", ces[way].cacheline[i]);
		}
	}   

	return ERROR_OK;
}

