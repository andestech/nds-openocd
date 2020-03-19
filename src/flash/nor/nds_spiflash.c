/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Megan Wachs <megan@sifive.com> from the original stmsmi.c *
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

/* The Freedom E SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Freedom E platforms.
 *
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus. Writes are only possible in this mode.
 * - HW mode: Memory content is directly
 *   accessible in CPU memory space. CPU can read, write and execute memory
 *   content. */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the controller
 * must have "HW mode" enabled.
 * 1) The command "reset init" has to initialize the controller and put
 *    it in HW mode (this is actually the default out of reset for Freedom E systems).
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "log.h"
#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/nds32_log.h"
#include "target/riscv/encoding.h"
#include "target/riscv/ndsv5_encoding.h"
#include "target/riscv/program.h"
#include "target/riscv/ndsv5.h"

#define STEP_LOCK         1
#define STEP_EXIT         2
#define STEP_UNLOCK       3
#define STEP_TX           4
#define STEP_INIT         6
#define STEP_ERASE        8
#define STEP_RX           9
#define STEP_NOP          0xff

#define NDS_AS_DATA_AREA   0x40000
#define NDS_AS_NUMS        2048

#define NDSSPI_DEV_MAME         "nds-design"
#define NDSSPI_DEV_ID	        0x003525c2
#define NDSSPI_DEV_TOTALSIZE	0x100000
#define NDSSPI_DEV_SECTORSIZE	0x1000
#define NDSSPI_DEV_PAGESIZE	0x100
#define NDSSPI_EBREAK_OFFSET    0x12                 /* v5 algorithm_wa->address + 0x12 => exit_point */

#define NDSSPI_SIZE_PER_DOT     0x2000               /* IDE use 8K/dot */
#define NDSSPI_DOT              "\nSPI_DOT\n"
#define NDSSPI_READ_8K          "\nSPI_READ_8K\n"
#define NDSSPI_WRITE_8K         "\nSPI_WRITE_8K\n"
#define NDSSPI_READ_START       "\nSPI_READ_START\n"
#define NDSSPI_WRITE_START      "\nSPI_WRITE_START\n"
#define NDSSPI_READ_FINISH      "\nSPI_READ_FINISH\n"
#define NDSSPI_WRITE_FINISH     "\nSPI_WRITE_FINISH\n"
#define NDSSPI_INIT_FAIL        "\nSPI_INIT_FAIL\n"

#define TGT_BURN_MAGIC "ANDS"
#define TGT_BURN_VER   0x100
bool rv32e;
bool tgt_burn_new_version;
struct algorithm_header {
	char no_used[4];
	char magic[8];
	uint32_t version;
	uint64_t start_addr;
	uint64_t max_size;
	uint64_t cmd_addr;
	uint64_t cmd_size;
	uint64_t ebreak_addr;
} *algo_hdr;

uint32_t ndsspi_cur_write_bytes;
uint32_t ndsspi_write_bytes_per_dot = NDSSPI_SIZE_PER_DOT;

struct custom_flash_device_info {
	uint32_t device_id;
	uint32_t pagesize;
	uint32_t sectorsize;
	uint32_t size_in_bytes;
	uint32_t device_addr;
} nds_flash_info;

unsigned int nds_spi_rx;
uint32_t custom_flash_device_info_offset;

struct algorithm_steps {
	uint32_t size;
	uint32_t used;
	uint8_t **steps;
};

struct algorithm_steps *nds_as;
struct working_area *nds_algorithm_wa;
struct working_area *nds_data_wa;

static uint8_t *p_algorithm_bin;
bool algorithm_bin_read;
uint64_t g_value_milmb = 1;
uint64_t nds_algorithm_ebreak_offset;
int ndsspi_data_clean(struct flash_bank *bank);
int count_writesize_perdot(uint32_t image_size, uint32_t page_size);
uint64_t nds_algorithm_addr;
uint64_t nds_data_addr;
uint64_t nds_data_size;
uint32_t nds_tgt_version;
struct algorithm_steps *ndsspi_as_new(uint32_t size);
struct algorithm_steps *ndsspi_as_delete(struct algorithm_steps *as);
int ndsspi_steps_execute(struct algorithm_steps *as, struct flash_bank *bank);
int ndsspi_algorithm_apis_init(struct flash_bank *bank);
void ndsspi_as_add_rx(struct algorithm_steps *as, uint32_t offset, uint32_t count);
void ndsspi_as_add_tx(struct algorithm_steps *as, uint32_t offset, uint32_t count, const uint8_t *data);
void ndsspi_as_add_erase(struct algorithm_steps *as, uint32_t sector_1st, uint32_t sector_last);
void ndsspi_as_add_init(struct algorithm_steps *as);
void ndsspi_as_add_onlycmd(struct algorithm_steps *as, int cmd);

struct ndsspi_flash_bank {
	int probed;
	struct flash_device *dev;
};
extern int nds_targetburn_targetnum;
extern int nds_targetburn_corenum;



/* restore stauts at the end of target_burn */
static int restore_status(struct flash_bank *bank)
{
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	if (strcmp(target_type_name(target), "nds32_v3") == 0)
		return ERROR_OK;

	/* FOR v5 : RESTORE ILM*/
	if ((g_value_milmb & 0x1) == 0) {
		LOG_DEBUG("Restore ILM");
		struct reg *reg_milmb = ndsv5_get_reg_by_CSR(target, CSR_MILMB);
		if (reg_milmb == NULL) {
			LOG_DEBUG("get reg_milmb ERROR");
			return ERROR_FAIL;
		}
		ndsv5_set_register_value(reg_milmb, g_value_milmb);
		g_value_milmb = 1;
	}

	/* for v5 : disable EBREAKM/S/U*/
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	nds32->target_burn_attached = false;
	struct reg *reg_dcsr = ndsv5_get_reg_by_CSR(target, CSR_DCSR);
	if (reg_dcsr == NULL) {
		LOG_DEBUG("get reg_dcsr ERROR");
		return ERROR_FAIL;
	}
	uint64_t dcsr_value = ndsv5_get_register_value(reg_dcsr);
	ndsv5_set_register_value(reg_dcsr, (dcsr_value & (~0xb000)));

	return ERROR_OK;
}

static int change_status(struct flash_bank *bank)
{
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	if (strcmp(target_type_name(target), "nds32_v3") == 0)
		return ERROR_OK;

	/* FOR AE350 : ENABLE ILM*/
	struct reg *reg_micm_cfg = ndsv5_get_reg_by_CSR(target, CSR_MICM_CFG);
	if (ndsv5_get_register_value(reg_micm_cfg) & 0x1000) {
		LOG_DEBUG("Enabling ILM");
		struct reg *reg_milmb = ndsv5_get_reg_by_CSR(target, CSR_MILMB);
		if (reg_milmb == NULL) {
			LOG_DEBUG("get reg_milmb ERROR");
			return ERROR_FAIL;
		}
		g_value_milmb = ndsv5_get_register_value(reg_milmb);
		if ((g_value_milmb & 0x1) == 0)
			ndsv5_set_register_value(reg_milmb, (g_value_milmb | 0x1));
	}

	/* for v5 : enable EBREAKM/S/U*/
	struct nds32_v5 *nds32 = target_to_nds32_v5(target);
	nds32->target_burn_attached = true;
	struct reg *reg_dcsr = ndsv5_get_reg_by_CSR(target, CSR_DCSR);
	if (reg_dcsr == NULL) {
		LOG_DEBUG("get reg_dcsr ERROR");
		return ERROR_FAIL;
	}
	uint64_t dcsr_value = ndsv5_get_register_value(reg_dcsr);
	ndsv5_set_register_value(reg_dcsr, (dcsr_value | 0xb000));

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(ndsspi_flash_bank_command)
{
	LOG_DEBUG("%s, base=" TARGET_ADDR_FMT ", size=0x%08x", bank->name, bank->base, bank->size);
	struct ndsspi_flash_bank *ndsspi_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ndsspi_info = malloc(sizeof(struct ndsspi_flash_bank));
	if (ndsspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = ndsspi_info;
	ndsspi_info->probed = 0;

	return ERROR_OK;
}

static int ndsspi_probe(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;
	struct flash_sector *sectors;

	if (ndsspi_info->probed)
		free(bank->sectors);
	ndsspi_info->probed = 0;

	if (ndsspi_algorithm_apis_init(bank) != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("Found flash device \'%s\' (ID 0x%08x), baseaddr = " TARGET_ADDR_FMT,
			ndsspi_info->dev->name, ndsspi_info->dev->device_id, bank->base);

	/* Set correct size value */
	bank->size = ndsspi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		ndsspi_info->dev->size_in_bytes / ndsspi_info->dev->sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_USER_N("\nalloc sectors size error\n");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * ndsspi_info->dev->sectorsize;
		sectors[sector].size = ndsspi_info->dev->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	ndsspi_info->probed = 1;

	/* target_burn_frontend no need recall ndsspi_probe from ndsspi_auto_probe */
	if (tgt_burn_new_version && nds_tgt_version > TGT_BURN_VER)
		algorithm_bin_read = false;
	LOG_USER_N("\nprobe success\n");
	return ERROR_OK;
}

static int ndsspi_auto_probe(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;

	/* if no restart ICEman and repeat target burn, probed = 0 */
	if (algorithm_bin_read) {
		if (ndsspi_info->probed) {
			free(bank->sectors);
			ndsspi_info->probed = 0;
		}
		algorithm_bin_read = false;
	}

	if (ndsspi_info->probed)
		return ERROR_OK;
	return ndsspi_probe(bank);
}

static int ndsspi_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int ndsspi_erase_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	/* Nothing to do. */
	return ERROR_OK;
}

static int ndsspi_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	LOG_DEBUG("%s", __func__);
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;

	if (!(ndsspi_info->probed)) {
		snprintf(buf, buf_size,
				"\nNDSSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nNDSSPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			ndsspi_info->dev->name, ndsspi_info->dev->device_id);

	return ERROR_OK;
}

static int ndsspi_protect(struct flash_bank *bank, int set,
		int first, int last)
{
	LOG_DEBUG("%s", __func__);
	/* default is_protected = 0*/
	int retval = ERROR_OK;
	if (set == 1) {
		/* do lock after write finish need change_status and new nds_as */
		change_status(bank);
		if (nds_as == NULL)
			nds_as = ndsspi_as_new(2);
		ndsspi_as_add_onlycmd(nds_as, STEP_LOCK);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval != ERROR_OK) {
			ndsspi_data_clean(bank);
			LOG_USER_N("\nlock flash error\n");
		}
		/* delete nds_as and restore_status */
		if (nds_as != NULL)
			nds_as = ndsspi_as_delete(nds_as);
		restore_status(bank);
	} else if (set == 0) {
		/* do unlock */
		ndsspi_as_add_onlycmd(nds_as, STEP_UNLOCK);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval != ERROR_OK) {
			ndsspi_data_clean(bank);
			LOG_USER_N("\nunlock flash error\n");
		}
	}

	return retval;
}

static int ndsspi_erase(struct flash_bank *bank, int first, int last)
{
	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;

	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(ndsspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
		if (bank->sectors[sector].is_erased == 1) {
			LOG_DEBUG("Flash sector %d erased", sector);
			if (sector == first)
				first++;
			if (first > last)
				return ERROR_OK;
		}
	}

	/* algorithm mode */
	if ((nds_data_wa != NULL && !tgt_burn_new_version) || (tgt_burn_new_version && nds_data_wa == NULL)) {
		ndsspi_as_add_erase(nds_as, first, last);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval != ERROR_OK)
			ndsspi_data_clean(bank);
		for (sector = first; sector <= last; sector++)
			bank->sectors[sector].is_erased = 1;
		return retval;
	} else {
		LOG_ERROR("NOT target burn");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int ndsspi_write_sector_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	LOG_DEBUG("offset=0x%x, len=0x%x", offset, len);
	int retval = 0;

	/* algorithm mode */
	if ((nds_data_wa != NULL && !tgt_burn_new_version) || (tgt_burn_new_version && nds_data_wa == NULL)) {
		ndsspi_as_add_tx(nds_as, offset, len, buffer);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval == ERROR_OK) {
			if (tgt_burn_new_version && (nds_tgt_version > TGT_BURN_VER))
				LOG_USER_N(NDSSPI_DOT);
			else
				LOG_USER_N(NDSSPI_WRITE_8K);
		} else
			ndsspi_data_clean(bank);
	} else {
		LOG_ERROR("NOT target burn");
		return ERROR_FAIL;
	}
	return retval;
}

static int ndsspi_read_buffer(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count, bool print_dot)
{
	int retval;
	uint32_t cur_size;
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;

	while (count) {
		if (count >= ndsspi_write_bytes_per_dot)
			cur_size = ndsspi_write_bytes_per_dot;
		else
			cur_size = count;

		retval = target_read_buffer(target, offset + bank->base, cur_size, buffer);
		if (retval != ERROR_OK)
			return ERROR_FAIL;
		offset += cur_size;
		buffer += cur_size;
		count -= cur_size;
		if (print_dot) {
			if (tgt_burn_new_version && (nds_tgt_version > TGT_BURN_VER))
				LOG_USER_N(NDSSPI_DOT);
			else
				LOG_USER_N(NDSSPI_READ_8K);
		}
	}
	return ERROR_OK;
}

void ndsspi_read_rx(struct target *target, uint32_t rx_count, uint8_t *rx_data)
{
	LOG_DEBUG("ndsspi_read_rx");
	uint32_t size_head = 5, rx_size;
	uint8_t get_rx_size[2];
	while (rx_count > 0) {
		target_read_buffer(target, (nds_data_addr + size_head), 2, &get_rx_size[0]);
		rx_size = get_rx_size[0];
		rx_size |= (get_rx_size[1] << 8);

		target_read_buffer(target, (nds_data_addr + size_head + 2), rx_size, rx_data);

		size_head += (7 + rx_size);
		rx_data += rx_size;
		rx_count -= rx_size;
	}
}

static int ndsspi_rx_buffer(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;
	uint32_t cur_count;
	int retval = ERROR_OK;
	int sector;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		printf("Target not halted");
		fflush(stdout);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > ndsspi_info->dev->size_in_bytes) {
		LOG_WARNING("Read past end of flash. Extra data discarded.");
		count = ndsspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			printf("Flash sector %d protected", sector);
			fflush(stdout);
			return ERROR_FAIL;
		}
	}

	while (count > 0) {
		if (count >= ndsspi_write_bytes_per_dot)
			cur_count = ndsspi_write_bytes_per_dot;
		else
			cur_count = count;

		ndsspi_as_add_rx(nds_as, offset, cur_count);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval == ERROR_OK) {
			if (tgt_burn_new_version && (nds_tgt_version > TGT_BURN_VER))
				LOG_USER_N(NDSSPI_DOT);
			else
				LOG_USER_N(NDSSPI_READ_8K);
		} else
			ndsspi_data_clean(bank);

		ndsspi_read_rx(target, cur_count, buffer);

		count -= cur_count;
		offset += cur_count;
		buffer += cur_count;
	}

	return retval;
}

static int ndsspi_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;

	change_status(bank);

	LOG_DEBUG("offset=0x%08x, count=0x%08x", offset, count);
	LOG_USER_N(NDSSPI_READ_START);
	if (nds_spi_rx == 1)
		retval = ndsspi_rx_buffer(bank, buffer, offset, count);
	else
		ndsspi_read_buffer(bank, buffer, offset, count, 1);

	restore_status(bank);
	ndsspi_data_clean(bank);
	LOG_USER_N(NDSSPI_READ_FINISH);
	return retval;
}

int count_writesize_perdot(uint32_t image_size, uint32_t page_size)
{
	int remain = 0;
	uint32_t new_data_size = nds_data_size;

	if ((new_data_size < (page_size + 8)) && (image_size > page_size)) {
		LOG_ERROR("data work size <= (page_size + tx_command_size) => NOT page_align");
		return ERROR_FAIL;
	}

	if (image_size % 512)
		remain = 1;

	if (new_data_size >= (image_size + (((image_size / 512) + remain) * 7) + 1)) {
		/* at least print 2 dot */

/*		remain = image_size % page_size;
		if (remain)
			ndsspi_write_bytes_per_dot = image_size - remain;
		else {
			remain = image_size - page_size;
			ndsspi_write_bytes_per_dot = remain;
		}
		if (ndsspi_write_bytes_per_dot < page_size)
			ndsspi_write_bytes_per_dot = page_size;
*/
		ndsspi_write_bytes_per_dot = image_size;
	} else {
		/*remain = new_data_size % page_size;
		ndsspi_write_bytes_per_dot = new_data_size - remain;

		remain = (ndsspi_write_bytes_per_dot % 512) ? 1 : 0;
		while (new_data_size < (ndsspi_write_bytes_per_dot + (((ndsspi_write_bytes_per_dot / 512) + remain) * 7) + 1)) {
			ndsspi_write_bytes_per_dot = ndsspi_write_bytes_per_dot - page_size;
			if (ndsspi_write_bytes_per_dot < page_size) {
				ndsspi_write_bytes_per_dot = page_size;
				return ERROR_OK;
			}
			remain = (ndsspi_write_bytes_per_dot % 512) ? 1 : 0;
		}
		*/
		remain = (new_data_size - 1) / (512 + 7);
		if (remain == 0)
			ndsspi_write_bytes_per_dot = page_size;
		else
			ndsspi_write_bytes_per_dot = remain * 512;
	}

	return ERROR_OK;
}

static int ndsspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;
	uint32_t cur_count, cur_offset, sector_size, page_offset, page_size;
	int sector;
	int retval = ERROR_OK;
	uint8_t *tmp_buffer;

	LOG_DEBUG("offset=0x%08x, count=0x%08x", offset, count);
	LOG_USER_N(NDSSPI_WRITE_START);
	ndsspi_cur_write_bytes = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		printf("Target not halted");
		fflush(stdout);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			printf("Flash sector %d protected", sector);
			fflush(stdout);
			return ERROR_FAIL;
		}
	}

	sector_size = ndsspi_info->dev->sectorsize;
	LOG_DEBUG("\nsector size: %d\n", sector_size);

	page_size = ndsspi_info->dev->pagesize;
	LOG_DEBUG("\npage size: %d\n", page_size);

	if (offset + count > ndsspi_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = ndsspi_info->dev->size_in_bytes - offset;
	}
	LOG_DEBUG("\ntotal size: %d\n", count);

	/* version > 0x100 : Count write size per dot */
	if (tgt_burn_new_version) {
		if (nds_tgt_version > TGT_BURN_VER) {
			if (count_writesize_perdot(count, page_size) != ERROR_OK) {
				tgt_burn_new_version = false;
				LOG_ERROR("count write size failed");
				return ERROR_FAIL;
			}
			/* IDE use ?/dot , if not support count_writesize_perdot, default is 8K/dot */
			LOG_USER_N("\nWRITESIZE: %d\n", ndsspi_write_bytes_per_dot);
		}
	}

	tmp_buffer = malloc(page_size);
	if (tmp_buffer == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* erase need sector, sector range 0~255 */
	retval = ndsspi_erase(bank, offset/sector_size, (offset + count + (sector_size - 1))/sector_size - 1);
	if (retval != ERROR_OK)
		goto ndsspi_write_finish;

	/* unaligned(page) buffer head */
	if ((offset % page_size) != 0) {
		/* read the 1st-unaligned-page, memcpy unaligned data */
		page_offset = (offset/page_size) * page_size;
		ndsspi_read_buffer(bank, tmp_buffer, page_offset, page_size, 0);

		cur_offset = (offset % page_size);
		cur_count = (page_size - cur_offset);
		if (cur_count > count)
			cur_count = count;

		memcpy(tmp_buffer+cur_offset, buffer, cur_count);

		/* write the 1st-unaligned-page */
		retval = ndsspi_write_sector_buffer(bank, tmp_buffer, page_offset, page_size);
		if (retval != ERROR_OK)
			goto ndsspi_write_finish;

		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	/* central part, aligned words */
	while (count >= page_size) {
		if (count >= ndsspi_write_bytes_per_dot) {
			cur_count = ndsspi_write_bytes_per_dot;
		} else {
			/* clip block at sector boundary */
			cur_count = (count/page_size) * page_size;
		}

		retval = ndsspi_write_sector_buffer(bank, buffer, offset, cur_count);
		if (retval != ERROR_OK)
			goto ndsspi_write_finish;

		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	/* buffer tail */
	if (count > 0) {
		/* read the last-unaligned-page, memcpy unaligned data */
		ndsspi_read_buffer(bank, tmp_buffer, offset, page_size, 0);
		memcpy(tmp_buffer, buffer, count);
		/* write the last-unaligned-page */
		retval = ndsspi_write_sector_buffer(bank, tmp_buffer, offset, page_size);
		if (retval != ERROR_OK)
			goto ndsspi_write_finish;
	}

ndsspi_write_finish:
	if (tmp_buffer)
		free(tmp_buffer);
	restore_status(bank);
	ndsspi_data_clean(bank);
	/* tx is the last step means nds_spi_rx = 0, so delete nds_as and         */
	/* free working area and setting default value to global variable at here */
	if (nds_as != NULL)
		nds_as = ndsspi_as_delete(nds_as);
	if (nds_data_wa != NULL && !tgt_burn_new_version) {
		target_free_working_area(target, nds_algorithm_wa);
		target_free_working_area(target, nds_data_wa);
	}
	nds_algorithm_wa = NULL;
	nds_data_wa = NULL;
	tgt_burn_new_version = false;

	if (retval != ERROR_OK)
		LOG_USER_N("\nwrite flash error\n");
	LOG_USER_N(NDSSPI_WRITE_FINISH);
	return retval;
}

struct flash_driver ndsspi_flash = {
	.name = "ndsspi",
	.flash_bank_command = ndsspi_flash_bank_command,
	.erase = ndsspi_erase,
	.protect = ndsspi_protect,
	.write = ndsspi_write,
	.read = ndsspi_read,
	.probe = ndsspi_probe,
	.auto_probe = ndsspi_auto_probe,
	.erase_check = ndsspi_erase_check,
	.protect_check = ndsspi_protect_check,
	.info = ndsspi_get_info,
};

/* ====================================================== */
/* algorithm apis                                         */
/* ====================================================== */

struct algorithm_steps *ndsspi_as_new(uint32_t size)
{
	struct algorithm_steps *as = calloc(1, sizeof(struct algorithm_steps));
	as->size = size;
	as->steps = calloc(size, sizeof(as->steps[0]));
	return as;
}

struct algorithm_steps *ndsspi_as_delete(struct algorithm_steps *as)
{
	for (uint32_t step = 0; step < as->used; step++) {
		free(as->steps[step]);
		as->steps[step] = NULL;
	}
	free(as);
	return NULL;
}

int ndsspi_as_empty(struct algorithm_steps *as)
{
	for (uint32_t s = 0; s < as->used; s++) {
		if (as->steps[s][0] != STEP_NOP)
			return 0;
	}
	return 1;
}

/* Return size of compiled program. */
uint32_t ndsspi_as_compile(struct algorithm_steps *as, uint8_t *target,
		uint32_t target_size)
{
	uint32_t offset = 0;
	bool finish_early = false;
	uint32_t tx_size, rx_size;

	LOG_DEBUG("as->used %d", as->used);
	for (uint32_t s = 0; s < as->used && !finish_early; s++) {
		uint32_t bytes_left = target_size - offset;
		LOG_DEBUG("as->steps[s][0] 0x%x", as->steps[s][0]);

		switch (as->steps[s][0]) {
			case STEP_NOP:
				break;
			case STEP_RX:
				rx_size = as->steps[s][5];
				rx_size |= (as->steps[s][6] << 8);
				LOG_DEBUG("rxsize %d", rx_size);
				if (rx_size + 8 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 7);
				offset += rx_size + 7;
				break;
			case STEP_TX:
				tx_size = as->steps[s][5];
				tx_size |= (as->steps[s][6] << 8);
				LOG_DEBUG("txsize %d", tx_size);
				if (tx_size + 8 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], tx_size + 7);
				offset += tx_size + 7;
				break;
			case STEP_ERASE:
				if (6 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 5);
				offset += 5;
				break;
			case STEP_INIT:
				if ((2 + sizeof(struct custom_flash_device_info)) > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 1 + sizeof(struct custom_flash_device_info));
				custom_flash_device_info_offset = (offset + 1);
				offset += (1 + sizeof(struct custom_flash_device_info));
				break;
			case STEP_LOCK:
			case STEP_UNLOCK:
				if (2 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 1);
				offset += 1;
				break;
			default:
				assert(0);
		}
		if (!finish_early)
			as->steps[s][0] = STEP_NOP;
	}
	assert(offset + 1 <= target_size);
	target[offset++] = STEP_EXIT;

	LOG_DEBUG("%d-byte program:", offset);
	for (uint32_t i = 0; i < offset;) {
		char buf[80];
		for (uint32_t x = 0; i < offset && x < 16; x++, i++)
			sprintf(buf + x*3, "%02x ", target[i]);
		LOG_DEBUG("%s", buf);
	}

	return offset;
}

void ndsspi_as_add_rx(struct algorithm_steps *as, uint32_t offset, uint32_t rx_count)
{
	LOG_DEBUG("rx_count=%d, as->used %d", rx_count, as->used);
	while (rx_count > 0) {
		uint32_t step_count = MIN(rx_count, 512);
		assert(as->used < as->size);
		as->steps[as->used] = malloc(step_count + 7);
		as->steps[as->used][0] = STEP_RX;
		as->steps[as->used][1] = (offset & 0xff);
		as->steps[as->used][2] = (offset & 0xff00) >> 8;
		as->steps[as->used][3] = (offset & 0xff0000) >> 16;
		as->steps[as->used][4] = (offset & 0xff000000) >> 24;
		as->steps[as->used][5] = (step_count & 0xff);
		as->steps[as->used][6] = (step_count & 0xff00) >> 8;
		as->used++;
		offset += step_count;
		rx_count -= step_count;
	}
}

void ndsspi_as_add_tx(struct algorithm_steps *as, uint32_t offset, uint32_t tx_count, const uint8_t *tx_data)
{
	LOG_DEBUG("tx_count=%d, as->used %d", tx_count, as->used);
	while (tx_count > 0) {
		uint32_t step_count = MIN(tx_count, 512);
		assert(as->used < as->size);
		as->steps[as->used] = malloc(step_count + 7);
		as->steps[as->used][0] = STEP_TX;
		as->steps[as->used][1] = (offset & 0xff);
		as->steps[as->used][2] = (offset & 0xff00) >> 8;
		as->steps[as->used][3] = (offset & 0xff0000) >> 16;
		as->steps[as->used][4] = (offset & 0xff000000) >> 24;
		as->steps[as->used][5] = (step_count & 0xff);
		as->steps[as->used][6] = (step_count & 0xff00) >> 8;
		memcpy(as->steps[as->used] + 7, tx_data, step_count);
		as->used++;
		tx_data += step_count;
		offset += step_count;
		tx_count -= step_count;
	}
}

void ndsspi_as_add_erase(struct algorithm_steps *as, uint32_t sector_1st, uint32_t sector_last)
{
	LOG_DEBUG("sector_1st=%d, sector_last=%d, as->used %d", sector_1st, sector_last, as->used);
	assert(as->used < as->size);
	as->steps[as->used] = malloc(5);
	as->steps[as->used][0] = STEP_ERASE;
	as->steps[as->used][1] = (sector_1st & 0xff);
	as->steps[as->used][2] = (sector_1st & 0xff00) >> 8;
	as->steps[as->used][3] = (sector_last & 0xff);
	as->steps[as->used][4] = (sector_last & 0xff00) >> 8;
	as->used++;
}

void ndsspi_as_add_init(struct algorithm_steps *as)
{
	LOG_DEBUG("as->used %d", as->used);
	assert(as->used < as->size);
	as->steps[as->used] = malloc(1 + sizeof(struct custom_flash_device_info));
	as->steps[as->used][0] = STEP_INIT;
	as->used++;
}

void ndsspi_as_add_onlycmd(struct algorithm_steps *as, int cmd)
{
	LOG_DEBUG("as->used %d", as->used);
	assert(as->used < as->size);
	as->steps[as->used] = malloc(1);
	as->steps[as->used][0] = cmd;
	as->used++;
}

int ndsspi_steps_execute(struct algorithm_steps *as, struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);

	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	/* //struct fespi_flash_bank *fespi_info = bank->driver_priv; */
	/* //uint32_t ctrl_base = 0x0; // do not need */
	uint8_t *data_buf = malloc(nds_data_size);
	struct reg_param reg_params[2];
	int num_used_reg = 0;
	uint64_t nds_end_addr = 0;

	nds_end_addr = nds_algorithm_ebreak_offset;

	if (strcmp(target_type_name(target), "nds32_v3") == 0)
		num_used_reg = 0;
	else {
		int xlen = riscv_xlen(target);

		/* if v5 and not rv32e, reset a7 avoid virtual hosting */
		if (rv32e == false) {
			num_used_reg = 1;
			init_reg_param(&reg_params[0], gpr_and_fpu_name[17], xlen, PARAM_OUT);     /* a7 */
			buf_set_u64(reg_params[0].value, 0, xlen, 0);
		}

		/* for v5 older target burn version use t4 to store cmd address */
		if (!tgt_burn_new_version) {
			num_used_reg = 2;
			init_reg_param(&reg_params[1], gpr_and_fpu_name[29], xlen, PARAM_OUT);     /* t4 */
			buf_set_u64(reg_params[1].value, 0, xlen, nds_data_addr);
			/* for v5 older target burn version: end_addr must add algorithm_addr */
			nds_end_addr = nds_end_addr + nds_algorithm_addr;
		}
	}

	while (!ndsspi_as_empty(as)) {
		keep_alive();
		uint32_t bytes = ndsspi_as_compile(as, data_buf, nds_data_size);
		LOG_DEBUG("write data to 0x%" TARGET_PRIxADDR ": %d bytes", nds_data_addr, bytes);

		int retval = target_write_buffer(target, nds_data_addr, bytes, data_buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write data to 0x%" TARGET_PRIxADDR ": %d", nds_data_addr,
					retval);
			free(data_buf);
			return retval;
		}
		/* algorithm_wa->address + ebreak_address => exit_point    */
		/* num_used_reg = 0 for new target burn and (rv32e or v3); */
		/* num_used_reg = 1 for new target burn and v5 not rv32e;  */
		/* num_used_reg = 2 for v5 older target burn               */
		retval = target_run_algorithm(target, 0, NULL, num_used_reg, reg_params,
				nds_algorithm_addr, nds_end_addr,
				100000, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d", nds_algorithm_addr,
					retval);
			free(data_buf);
			return retval;
		}
	}
	as->used = 0;
	free(data_buf);
	LOG_DEBUG("target_run_algorithm finish !!");
	return ERROR_OK;
}

int ndsspi_data_clean(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	int retval = ERROR_OK;
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	/* set 0 in data working area :do not executing algorithm-bin after target burn */
	uint8_t *data_buf = calloc(1, sizeof(uint8_t));
	retval = target_write_buffer(target, nds_data_addr, 1, data_buf);
	if (retval == ERROR_OK)
		LOG_DEBUG("write 0 to 0x%" TARGET_PRIxADDR, nds_data_addr);
	else
		LOG_ERROR("failed write 0 0x%" TARGET_PRIxADDR, nds_data_addr);
	free(data_buf);
	return retval;
}

struct flash_device nds_spi_dev;
extern char *log_output_path;
char *user_algorithm_path;
int ndsspi_algorithm_apis_init(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	int retval = ERROR_OK;
	struct target *target;
	for (target = all_targets; target; target = target->next) {
		if (target->target_number == nds_targetburn_targetnum) {
			riscv_set_current_hartid(target, nds_targetburn_corenum);
			target->coreid = nds_targetburn_corenum;
			if (riscv_rtos_enabled(target)) {
				RISCV_INFO(r);
				r->rtos_hartid = nds_targetburn_corenum;
			}
			break;
		}
	}
	if (target == NULL)
		target = bank->target;
	tgt_burn_new_version = false;

	if (user_algorithm_path == NULL) {
		LOG_DEBUG("NOT algorithm_mode");
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
#if 0
		/* open file algorithm.bin */
		char filename[2048];
		/* gmon.out output path depend on log file path */
		LOG_INFO("log_output_path: %s", log_output_path);
		memset(filename, 0, sizeof(filename));

		char *c = strstr(log_output_path, "iceman_debug0.log");
		if (c)
			*c = '\0';
		strncpy(filename, log_output_path, strlen(log_output_path));
		strncat(filename, "algorithm.bin", 13);
		user_algorithm_path = &filename[0];
#endif
	}
	LOG_INFO("filename: %s", user_algorithm_path);

	FILE *fp_algorithm = fopen(user_algorithm_path, "rb");
	if (fp_algorithm == NULL) {
		LOG_WARNING("Couldn't open algorithm bin file: %s", user_algorithm_path);
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
	}

	/* get algorithm bin size */
	fseek(fp_algorithm, 0L, SEEK_END);
	uint32_t sizeof_algorithm_bin = ftell(fp_algorithm);
	fseek(fp_algorithm, 0L, SEEK_SET);

	/* read algorithm content */
	p_algorithm_bin = malloc(sizeof_algorithm_bin);
	uint32_t read_size = fread(p_algorithm_bin, 1, sizeof_algorithm_bin, fp_algorithm);
	if (read_size < sizeof_algorithm_bin) {
		LOG_ERROR("read size 0x%x is less than filesize 0x%x", read_size, sizeof_algorithm_bin);
		free(p_algorithm_bin);
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
	}
	fclose(fp_algorithm);
	LOG_DEBUG("file read %s, filesize: 0x%x", user_algorithm_path, read_size);


	/* find tag_name, version and ebreak_offset from algorithm bin file */
	algo_hdr = (struct algorithm_header *) p_algorithm_bin;
	LOG_DEBUG("magic: %s", algo_hdr->magic);
	LOG_DEBUG("version:0x%x", algo_hdr->version);
	LOG_DEBUG("start_addr: %" PRIu64, algo_hdr->start_addr);
	LOG_DEBUG("max_size: %" PRIu64, algo_hdr->max_size);
	LOG_DEBUG("cmd_addr: %" PRIu64, algo_hdr->cmd_addr);
	LOG_DEBUG("cmd_arr_size: %" PRIu64, algo_hdr->cmd_size);
	LOG_DEBUG("ebreak_offset: %" PRIu64, algo_hdr->ebreak_addr);

	/* if new target burn: set cmd_offset and ebreak offset */
	if (strcmp(algo_hdr->magic, TGT_BURN_MAGIC) == 0) {
		if (algo_hdr->version >= TGT_BURN_VER) {
			LOG_DEBUG("NEW TARGET BURN");
			tgt_burn_new_version = true;
			nds_tgt_version = algo_hdr->version;
			/* target_burn stat.S prepare 64bit for nds_ebreadk, algorithm_addr and */
			/* cmd_addr(if v3 no support 64bit,the heigher 32bit is 0)              */
			nds_algorithm_ebreak_offset = algo_hdr->ebreak_addr;
			nds_algorithm_addr = algo_hdr->start_addr;
			nds_data_addr = algo_hdr->cmd_addr;
			nds_data_size = algo_hdr->cmd_size;

			/* check algorithm_bin_size < algo_hdr->max_size */
			if (algo_hdr->max_size < read_size) {
				LOG_ERROR("user defined max_size %" PRIu64 " < algorithm_bin file size %d",
						algo_hdr->max_size, read_size);
				retval = ERROR_FAIL;
				goto ndsspi_init_finish;
			}
		} else {
			LOG_ERROR("target burn version 0x%x is unknow", algo_hdr->version);
			retval = ERROR_FAIL;
			goto ndsspi_init_finish;
		}
	} else {
		LOG_DEBUG("OLD TARGET BURN");
		if (nds_algorithm_wa == NULL) {
			if (target_alloc_working_area(target, read_size,
						&nds_algorithm_wa) != ERROR_OK) {
				LOG_WARNING("Couldn't allocate %zd-byte working area.",
						read_size);
				retval = ERROR_FAIL;
				goto ndsspi_init_finish;
			}
		}

		if (nds_data_wa == NULL) {
			uint32_t data_wa_size = NDS_AS_DATA_AREA;
			while (1) {
				if (data_wa_size < 128) {
					LOG_ERROR("Couldn't allocate data working area.");
					target_free_working_area(target, nds_algorithm_wa);
					retval = ERROR_FAIL;
					goto ndsspi_init_finish;
				}
				if (target_alloc_working_area_try(target, data_wa_size, &nds_data_wa) == ERROR_OK) {
					/* reserved for target code may use _edata in virtual hosting mode */
					/* nds_data_wa->address += 16; */
					break;
				}
				data_wa_size /= 2;
			}
		}
		nds_algorithm_addr = nds_algorithm_wa->address;
		nds_data_addr = nds_data_wa->address;
		nds_data_size = nds_data_wa->size;

		/* if v5 older target burn: use 0x12 as ebreak offset and store cmd address to t4 */
		nds_algorithm_ebreak_offset = NDSSPI_EBREAK_OFFSET;
		LOG_DEBUG("use t4 and set ebreak_offset from openocd setting");
	}

	/* for v5 send fence.i before write algorithm bin to target */
	if (strcmp(target_type_name(target), "nds_v5") == 0) {
		LOG_DEBUG("send fence.i before load algorithm bin");
		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		if (riscv_program_exec(&program, target) != ERROR_OK)
			LOG_ERROR("Unable to execute fence.i");
	}

	/* write algorithm bin to target */
	retval = target_write_buffer(target, nds_algorithm_addr,
			read_size, p_algorithm_bin);
	/* print bin content */
	FILE *out_algorithm = fopen("./output_bin", "wb");
	if (out_algorithm == NULL) {
		LOG_WARNING("Couldn't open output_bin file");
		/*
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
		*/
	} else {
		fwrite(p_algorithm_bin, 1, read_size, out_algorithm);
		fclose(out_algorithm);
	}

	free(p_algorithm_bin);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d", nds_algorithm_addr,
				retval);
		if (!tgt_burn_new_version) {
			target_free_working_area(target, nds_algorithm_wa);
			target_free_working_area(target, nds_data_wa);
		}
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
	}
	LOG_DEBUG("write code to 0x%" TARGET_PRIxADDR ": 0x%x bytes", nds_algorithm_addr, read_size);


	if (nds_as == NULL)
		nds_as = ndsspi_as_new(NDS_AS_NUMS);

	/* algorithm mode */
	if ((nds_data_wa != NULL && !tgt_burn_new_version) || (tgt_burn_new_version && nds_data_wa == NULL)) {
		ndsspi_as_add_init(nds_as);
		retval = ndsspi_steps_execute(nds_as, bank);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to ndsspi_as_add_init()");
			if (!tgt_burn_new_version) {
				target_free_working_area(target, nds_algorithm_wa);
				target_free_working_area(target, nds_data_wa);
			}
			retval = ERROR_FAIL;
			goto ndsspi_init_finish;
		}
	} else {
		LOG_ERROR("NOT target burn");
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
	}
	/* initial bank info. (sectorsize, size_in_bytes) */
	struct ndsspi_flash_bank *ndsspi_info = bank->driver_priv;
	ndsspi_info->dev = &nds_spi_dev;
	ndsspi_info->dev->name = NDSSPI_DEV_MAME;
	/* get SPI info. from custom algorithm_bin */
	target_read_buffer(target, nds_data_addr + custom_flash_device_info_offset,
		sizeof(struct custom_flash_device_info), (uint8_t *)&nds_flash_info);

	ndsspi_info->dev->device_id = nds_flash_info.device_id;
	ndsspi_info->dev->pagesize = nds_flash_info.pagesize;
	ndsspi_info->dev->sectorsize = nds_flash_info.sectorsize;
	ndsspi_info->dev->size_in_bytes = nds_flash_info.size_in_bytes;
	if (tgt_burn_new_version)
		bank->base = nds_flash_info.device_addr;

	LOG_DEBUG("device_id 0x%x", ndsspi_info->dev->device_id);
	LOG_DEBUG("size_in_bytes 0x%x", (unsigned int)ndsspi_info->dev->size_in_bytes);
	LOG_DEBUG("sectorsize 0x%x", (unsigned int)ndsspi_info->dev->sectorsize);
	LOG_DEBUG("pagesize 0x%x", ndsspi_info->dev->pagesize);
	LOG_DEBUG("devicd_addr " TARGET_ADDR_FMT, bank->base);

	if (ndsspi_info->dev->size_in_bytes <= 0 ||
	    ndsspi_info->dev->sectorsize <= 0 ||
	    ndsspi_info->dev->pagesize <= 0 ||
	    nds_data_size <= 0) {
		LOG_ERROR("FLASH size or sectorsize or pagesize or command array size is less than 0");
		if (nds_data_wa != NULL && !tgt_burn_new_version) {
			target_free_working_area(target, nds_algorithm_wa);
			target_free_working_area(target, nds_data_wa);
		}
		retval = ERROR_FAIL;
		goto ndsspi_init_finish;
	}

	retval = ERROR_OK;

	/* for SPI_Burn check offset+image size < flash size */
	LOG_USER_N("\nflash total size:0x%x:\n", (unsigned int)ndsspi_info->dev->size_in_bytes);

ndsspi_init_finish:
	if (retval == ERROR_FAIL) {
		nds_algorithm_wa = NULL;
		nds_data_wa = NULL;
		tgt_burn_new_version = false;
		if (nds_as != NULL)
			nds_as = ndsspi_as_delete(nds_as);
		LOG_USER_N(NDSSPI_INIT_FAIL);
	}
	return retval;
}

