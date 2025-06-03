/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

void soc_port_configure(const struct soc_port_pin *pin)
{
	pincfg_registers_t *pincfg = pin->regs;
	uint32_t flags = pin->flags;
	uint32_t func = (pin->flags & SOC_PORT_FUNC_MASK) >> SOC_PORT_FUNC_POS;
	pincfg_config_registers_t diocfg = 0;

	pincfg->DIO_CFG[pin->pinum] = diocfg;

	diocfg |= PINCFG_DIO_CFG_SEL(func);
	diocfg |= PINCFG_DIO_CFG_DRVSTR(drvstr);

	if (flags & SOC_PORT_OPEN_DRAIN_ENABLE)
		diocfg |= PINCFG_DIO_CFG_OD_enable;

	if (flags & SOC_PORT_INPUT_ENABLE)
		diocfg |= PINCFG_DIO_CFG_DIN_EN_enable;

	if (flags & SOC_PORT_PULLDOWN_ENABLE)
		diocfg |= PINCFG_DIO_CFG_PD_ENC_enable;

	if (flags & SOC_PORT_PULLUP_ENABLE)
		diocfg |= PINCFG_DIO_CFG_PU_ENC_enable;

	pincfg->DIO_CFG[pin->pinum] = diocfg;
}

void soc_port_list_configure(const struct soc_port_pin pins[],
			     unsigned int size)
{
	for (int i = 0; i < size; i++) {
		soc_port_configure(&pins[i]);
	}
}
