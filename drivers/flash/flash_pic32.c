/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic32_nvmctrl
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_pic32);

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <string.h>

#define FLASHC_MEM(_a) ((uint32_t *)((uint8_t *)((_a) + CONFIG_FLASH_BASE_ADDRESS)))

struct flash_pic32_data {
	uint8_t buf[256];
};

static const struct flash_parameters flash_pic32_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static const struct flash_parameters *flash_pic32_get_parameters(const struct device *dev)
{
	return &flash_pic32_parameters;
}

static void flash_pic32_wait_ready(void)
{
	while ((FLASHC_REGS->FLASHC_FC_FSR & FLASHC_FC_FSR_FRDY_Msk) == 0) {
	}
}

static int flash_pic32_check_status(void)
{
	uint32_t fc_error_flags = FLASHC_FC_FSR_FCMDE_Msk | FLASHC_FC_FSR_FLOCKE_Msk |
				  FLASHC_FC_FSR_FLERR_Msk | FLASHC_FC_FSR_WPERR_Msk;
	uint32_t status;

	flash_pic32_wait_ready();

	status = FLASHC_REGS->FLASHC_FC_FSR;
	if ((status & fc_error_flags) != 0) {
		return -EIO;
	}

	return 0;
}

static int flash_pic32_write_protection(const struct device *dev, bool enable)
{
	if (enable) {
		FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_WPEN(1);
	} else {
		FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_WPEN(0);
	}

	return flash_pic32_check_status();
}

static int flash_pic32_valid_range(off_t offset, size_t len)
{
	if (offset < 0) {
		LOG_WRN("0x%lx: before start of flash", (long)offset);
		return -EINVAL;
	}
	if ((offset + len) > FLASHC_TOTAL_FLASH_SIZE) {
		LOG_WRN("0x%lx: ends past the end of flash", (long)offset);
		return -EINVAL;
	}

	return 0;
}

static int flash_pic32_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	int ret;

	ret = flash_pic32_valid_range(offset, len);
	if (ret != 0) {
		return ret;
	}

	memcpy(data, (uint8_t *)CONFIG_FLASH_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_pic32_write_page(const struct device *dev, off_t offset,
				 const void *data, size_t len, uint32_t page_number)
{
	const uint32_t *src = data;
	const uint32_t *end = src + (len / sizeof(*src));
	uint32_t *dst = FLASHC_MEM(offset);
	int ret;

	/* Ensure writes happen 32 bits at a time. */
	for (; src != end; src++, dst++) {
		*dst = UNALIGNED_GET((uint32_t *)src);
	}

	flash_pic32_wait_ready();

	FLASHC_REGS->FLASHC_FC_FCR = FLASHC_FC_FCR_FKEY_PASSWD | FLASHC_FC_FCR_FCMD(FLASHC_FC_FCR_FCMD_WP_Val) | FLASHC_FC_FCR_FARG(page_number);
	
	ret = flash_pic32_check_status();
	if (ret != 0) {
		return ret;
	}

	if (memcmp(data, FLASHC_MEM(offset), len) != 0) {
		LOG_ERR("verify error at offset 0x%lx", (long)offset);
		return -EIO;
	}

	return 0;
}

static int flash_pic32_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const uint8_t *pdata = data;
	int ret;

	ret = flash_pic32_valid_range(offset, len);
	if (ret != 0) {
		return ret;
	}

	if (len == 0) {
		return 0;
	}

	/* Check 8-byte address and 8-byte size alignment */
	if (((offset & 0x7U) != 0) || ((len & 0x7U) != 0)) {
		return -EINVAL;
	}

	ret = flash_pic32_write_protection(dev, false);
	if (ret != 0) {
		return ret;
	}

	/* Write flash */
	
	size_t eop_len = FLASHC_PAGE_SIZE - (offset % FLASHC_PAGE_SIZE);
	size_t write_len = MIN(len, eop_len);

	while (len > 0) {
		uint32_t page_number = offset / FLASHC_PAGE_SIZE;

		ret = flash_pic32_write_page(dev, offset, pdata, write_len, page_number);
		if (ret != 0) {
			break;
		}

		offset += write_len;
		pdata += write_len;
		len -= write_len;
		write_len = MIN(len, FLASHC_PAGE_SIZE);
	}

	ret = flash_pic32_write_protection(dev, true);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int flash_pic32_erase_block(uint32_t block_number)
{
	FLASHC_REGS->FLASHC_FC_FCR = FLASHC_FC_FCR_FKEY_PASSWD | FLASHC_FC_FCR_FCMD(FLASHC_FC_FCR_FCMD_EUS_Val) |
				     FLASHC_FC_FCR_FARG(0);

	return flash_pic32_check_status();
}

static int flash_pic32_erase(const struct device *dev, off_t offset, size_t size)
{
	int ret;

	/* If erasewp or erasewl bits in the fc_wpmr are set, then not allowed and return false */
	if ((FLASHC_REGS->FLASHC_FC_WPMR & FLASHC_FC_WPMR_ERASEWP_Msk) ||
	    (FLASHC_REGS->FLASHC_FC_WPMR & FLASHC_FC_WPMR_ERASEWL_Msk)) {
		return -EPERM;
	}

	/* Has to be block aligned and size has to be multiple of block */
	if ((offset % FLASHC_BLOCK_SIZE != 0) || (size % FLASHC_BLOCK_SIZE != 0)) {
		return -EINVAL;
	}

	ret = flash_pic32_write_protection(dev, false);
	if (ret != 0) {
		return ret;
	}

	for (size_t addr = offset; addr < offset + size; addr += FLASHC_BLOCK_SIZE) {
		uint32_t block_number = addr / FLASHC_BLOCK_SIZE;

		ret = flash_pic32_erase_block(block_number);
		if (ret != 0) {
			break;
		}
	}

	return flash_pic32_write_protection(dev, true);
}

static int flash_pic32_init(const struct device *dev)
{
	/* Unlock flash configuration */
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_USRWP(0);
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_WPEN(0);

	/* Enable flash cache */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_CACHE_ENABLE(1);

	/* Enable sequential prefetch optimization */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_SEQ_PREFETCH_OPT(1);

	/* Enable program loop optimization */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_PROG_LOOP_OPT(1);

	/* Set flash wait state to 1 */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_WAIT_STATE(1);

	/* Set flash signature wait states to 1 */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_SIGNATURE_WAIT_STATES(1);

	/* Lock flash configuration */
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_USRWP(1);
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_ERASEWP(1);
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_LOCKWP(1);
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_GPNVMWP(1);
	FLASHC_REGS->FLASHC_FC_WPMR |= FLASHC_FC_WPMR_WPEN(1);

	return 0;
}

static const struct flash_driver_api flash_pic32_api = {
	.erase = flash_pic32_erase,
	.write = flash_pic32_write,
	.read = flash_pic32_read,
	.get_parameters = flash_pic32_get_parameters,
};

static struct flash_pic32_data flash_pic32_data_0;

//DEVICE_DT_INST_DEFINE(0, flash_pic32_init, NULL, &flash_pic32_data_0, NULL, POST_KERNEL,
//		      CONFIG_FLASH_INIT_PRIORITY, &flash_pic32_api);
