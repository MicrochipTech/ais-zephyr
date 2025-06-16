/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_port.h"

void soc_port_configure(const struct soc_port_pin *pin)
{
	pincfg_registers_t *pincfg = pin->regs;
	uint32_t flags = pin->flags;
	uint32_t func = (pin->flags & SOC_PORT_FUNC_MASK) >> SOC_PORT_FUNC_POS;

	pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_SEL(func);

	if (flags & SOC_PORT_STRENGTH_STRONGER)
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_DRVSTR(PINCFG_DIO_CFG_DRVSTR_4mA_Val);

	if (flags & SOC_PORT_OPENDRAIN_ENABLE)
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_OD_enable;
#if 0
	if (flags & SOC_PORT_INPUT_ENABLE)
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_DIN_EN_enable;
	else
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG &= ~PINCFG_DIO_CFG_DIN_EN_enable;
#endif
	if (flags & SOC_PORT_PULLDOWN)
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_PD_ENC_enable;

	if (flags & SOC_PORT_PULLUP)
		pincfg->DIO_CFG[pin->pinum].PINCFG_DIO_CFG |= PINCFG_DIO_CFG_PU_ENC_enable;
}

void soc_port_list_configure(const struct soc_port_pin pins[],
			     unsigned int size)
{
	for (int i = 0; i < size; i++) {
		soc_port_configure(&pins[i]);
	}
}
