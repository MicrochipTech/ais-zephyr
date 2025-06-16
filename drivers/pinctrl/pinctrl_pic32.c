/*
 * Copyright (c) 2025, Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include "soc_port.h"

#define DT_DRV_COMPAT microchip_pic32_pinctrl

static void pinctrl_configure_pin(pinctrl_soc_pin_t pin)
{
	struct soc_port_pin soc_pin;
	uint8_t  port_idx, port_func;

	port_idx = PIC32_PINMUX_PORT_GET(pin);
	port_func = PIC32_PINMUX_FUNC_GET(pin);

	soc_pin.regs = (pincfg_registers_t *) DT_REG_ADDR(DT_NODELABEL(pinctrl));
	soc_pin.pinum = PIC32_PINMUX_PIN_GET(pin);
	soc_pin.flags = PIC32_PINCTRL_FLAGS_GET(pin) << SOC_PORT_FLAGS_POS;

	if (port_func == PIC32_PINMUX_FUNC_periph) {
		soc_pin.flags |= (PIC32_PINMUX_PERIPH_GET(pin) << SOC_PORT_FUNC_POS);
	}

	soc_port_configure(&soc_pin);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(*pins++);
	}

	return 0;
}
