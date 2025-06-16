/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic32_clkrstgen

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define SERCOM0_PERIPH_ClOCK_ID		0x3
#define SERCOM1_PERIPH_ClOCK_ID		0x4
#define SERCOM2_PERIPH_ClOCK_ID		0x5
#define SERCOM3_PERIPH_ClOCK_ID		0x6
#define I2S_PERIPH_ClOCK_ID		0x7
#define BCLKC_PERIPH_ClOCK_ID		0x8
#define ADCC_PERIPH_ClOCK_ID		0x9

static int microchip_pic32_clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint8_t *periph_clk_id = (uint8_t *)(sys);
	clk_rst_gen_registers_t* regs = CLK_RST_GEN_REGS;

	/* Enable the peripheral clock */
	switch (*periph_clk_id) {
	case SERCOM0_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL0 |= CLK_RST_GEN_CG_PERIPH_CTRL0_MASK_Msk;
		break;
	case SERCOM1_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL1 |= CLK_RST_GEN_CG_PERIPH_CTRL1_MASK_Msk;
		break;
	case SERCOM2_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL2 |= CLK_RST_GEN_CG_PERIPH_CTRL2_MASK_Msk;
		break;
	case SERCOM3_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL3 |= CLK_RST_GEN_CG_PERIPH_CTRL3_MASK_Msk;
		break;
	case I2S_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL4 |= CLK_RST_GEN_CG_PERIPH_CTRL4_MASK_Msk;
		break;
	case BCLKC_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL5 |= CLK_RST_GEN_CG_PERIPH_CTRL5_MASK_Msk;
		break;
	case ADCC_PERIPH_ClOCK_ID:
		regs->CLK_RST_GEN_CG_PERIPH_CTRL6 |= CLK_RST_GEN_CG_PERIPH_CTRL6_MASK_Msk;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static DEVICE_API(clock_control, microchip_pic32_clock_control_api) = {
	.on = microchip_pic32_clock_control_on,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &microchip_pic32_clock_control_api);
