/*
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_mdio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/atmel_sam_pmc.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_sam, CONFIG_MDIO_LOG_LEVEL);

/* GMAC */
#ifdef CONFIG_SOC_FAMILY_ATMEL_SAM0
#define GMAC_MAN        MAN.reg
#define GMAC_NSR        NSR.reg
#define GMAC_NCR        NCR.reg
#endif

#ifdef CONFIG_SOC_FAMILY_MICROCHIP_PIC32
#define Gmac            gmac_registers_t
#define GMAC_MAN        GMAC_PHY_MANAGEMENT
#define GMAC_NSR        GMAC_NETWORK_STATUS
#define GMAC_NCR        GMAC_NETWORK_CONTROL
#define GMAC_NCFGR      GMAC_NETWORK_CONFIG

#define GMAC_NCR_MPE		GMAC_NETWORK_CONTROL_MAN_PORT_EN_Msk
#define GMAC_NSR_IDLE		GMAC_NETWORK_STATUS_MAN_DONE_Msk

#define GMAC_MAN_DATA_Msk	GMAC_PHY_MANAGEMENT_PHY_WRITE_READ_DATA_Msk

#define GMAC_MAN_CLTTO		GMAC_PHY_MANAGEMENT_WRITE1_Msk
#define GMAC_MAN_OP(op)		GMAC_PHY_MANAGEMENT_OPERATION(op)
#define GMAC_MAN_WTN(val)	GMAC_PHY_MANAGEMENT_WRITE10(val)
#define GMAC_MAN_PHYA(prtad)	GMAC_PHY_MANAGEMENT_PHY_ADDRESS(prtad)
#define GMAC_MAN_REGA(regad)	GMAC_PHY_MANAGEMENT_REGISTER_ADDRESS(regad)
#define GMAC_MAN_DATA(data_in)	GMAC_PHY_MANAGEMENT_PHY_WRITE_READ_DATA(data_in)
#endif

struct mdio_sam_dev_data {
	struct k_sem sem;
};

struct mdio_sam_dev_config {
	Gmac * const regs;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SOC_FAMILY_ATMEL_SAM
	const struct atmel_sam_pmc_config clock_cfg;
#endif
};

static int mdio_transfer(const struct device *dev, uint8_t prtad, uint8_t regad,
			 enum mdio_opcode op, bool c45, uint16_t data_in,
			 uint16_t *data_out)
{
	const struct mdio_sam_dev_config *const cfg = dev->config;
	struct mdio_sam_dev_data *const data = dev->data;
	int timeout = 50;

	k_sem_take(&data->sem, K_FOREVER);

	/* Write mdio transaction */
	cfg->regs->GMAC_MAN = (c45 ? 0U : GMAC_MAN_CLTTO)
			    |  GMAC_MAN_OP(op)
			    |  GMAC_MAN_WTN(0x02)
			    |  GMAC_MAN_PHYA(prtad)
			    |  GMAC_MAN_REGA(regad)
			    |  GMAC_MAN_DATA(data_in);

	/* Wait until done */
	while (!(cfg->regs->GMAC_NSR & GMAC_NSR_IDLE)) {
		if (timeout-- == 0U) {
			LOG_ERR("transfer timedout %s", dev->name);
			k_sem_give(&data->sem);

			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(5));
	}

	if (data_out) {
		*data_out = cfg->regs->GMAC_MAN & GMAC_MAN_DATA_Msk;
	}

	k_sem_give(&data->sem);

	return 0;
}

static int mdio_sam_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			 uint16_t *data)
{
	return mdio_transfer(dev, prtad, regad, MDIO_OP_C22_READ, false,
			     0, data);
}

static int mdio_sam_write(const struct device *dev, uint8_t prtad,
			  uint8_t regad, uint16_t data)
{
	return mdio_transfer(dev, prtad, regad, MDIO_OP_C22_WRITE, false,
			     data, NULL);
}

#if (!CONFIG_SOC_FAMILY_MICROCHIP_PIC32)
static int mdio_sam_read_c45(const struct device *dev, uint8_t prtad,
			     uint8_t devad, uint16_t regad, uint16_t *data)
{
	int err;

	err = mdio_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true,
			    regad, NULL);
	if (!err) {
		err = mdio_transfer(dev, prtad, devad, MDIO_OP_C45_READ, true,
				    0, data);
	}

	return err;
}

static int mdio_sam_write_c45(const struct device *dev, uint8_t prtad,
			      uint8_t devad, uint16_t regad, uint16_t data)
{
	int err;

	err = mdio_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true,
			    regad, NULL);
	if (!err) {
		err = mdio_transfer(dev, prtad, devad, MDIO_OP_C45_WRITE, true,
				    data, NULL);
	}

	return err;
}
#endif

static void mdio_sam_bus_enable(const struct device *dev)
{
	const struct mdio_sam_dev_config *const cfg = dev->config;

	cfg->regs->GMAC_NCR |= GMAC_NCR_MPE;
}

static void mdio_sam_bus_disable(const struct device *dev)
{
	const struct mdio_sam_dev_config *const cfg = dev->config;

	cfg->regs->GMAC_NCR &= ~GMAC_NCR_MPE;
}

static int mdio_sam_initialize(const struct device *dev)
{
#if (!CONFIG_SOC_FAMILY_MICROCHIP_PIC32)
	const struct mdio_sam_dev_config *const cfg = dev->config;
	int retval;
#endif
	struct mdio_sam_dev_data *const data = dev->data;

	k_sem_init(&data->sem, 1, 1);

#if (CONFIG_SOC_FAMILY_MICROCHIP_PIC32)
	PINCFG_REGS->PINCFG_PHYRX_PAD_CFG &= ~PINCFG_PHYRX_PAD_CFG_PU_ENC_Msk;
	PINCFG_REGS->PINCFG_PHYED_PAD_CFG &= ~PINCFG_PHYED_PAD_CFG_PU_ENC_Msk;
	PINCFG_REGS->PINCFG_PHYTX_PAD_CFG |= PINCFG_PHYTX_PAD_CFG_DRVSTR(PINCFG_PHYTX_PAD_CFG_DRVSTR_4mA_Val);
	PINCFG_REGS->PINCFG_PHYRX_PAD_CFG |= PINCFG_PHYRX_PAD_CFG_DRVSTR(PINCFG_PHYRX_PAD_CFG_DRVSTR_2p8mA_Val);
	PINCFG_REGS->PINCFG_PHYED_PAD_CFG |= PINCFG_PHYED_PAD_CFG_DRVSTR(PINCFG_PHYED_PAD_CFG_DRVSTR_2p8mA_Val);
	return 0;
#else
#ifdef CONFIG_SOC_FAMILY_ATMEL_SAM
	/* Enable GMAC module's clock */
	(void) clock_control_on(SAM_DT_PMC_CONTROLLER, (clock_control_subsys_t) &cfg->clock_cfg);
#else
	/* Enable MCLK clock on GMAC */
	MCLK->AHBMASK.reg |= MCLK_AHBMASK_GMAC;
	*MCLK_GMAC |= MCLK_GMAC_MASK;
#endif

	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	return retval;
#endif
}

static DEVICE_API(mdio, mdio_sam_driver_api) = {
	.read = mdio_sam_read,
	.write = mdio_sam_write,
#if (!CONFIG_SOC_FAMILY_MICROCHIP_PIC32)
	.read_c45 = mdio_sam_read_c45,
	.write_c45 = mdio_sam_write_c45,
#endif
	.bus_enable = mdio_sam_bus_enable,
	.bus_disable = mdio_sam_bus_disable,
};

#define MDIO_SAM_CLOCK(n)						\
	COND_CODE_1(CONFIG_SOC_FAMILY_ATMEL_SAM,			\
		(.clock_cfg = SAM_DT_INST_CLOCK_PMC_CFG(n),), ()	\
	)

#define MDIO_SAM_CONFIG(n)						\
static const struct mdio_sam_dev_config mdio_sam_dev_config_##n = {	\
	.regs = (Gmac *)DT_INST_REG_ADDR(n),				\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	MDIO_SAM_CLOCK(n)						\
};

#define MDIO_SAM_DEVICE(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	MDIO_SAM_CONFIG(n);						\
	static struct mdio_sam_dev_data mdio_sam_dev_data##n;		\
	DEVICE_DT_INST_DEFINE(n,					\
			      &mdio_sam_initialize,			\
			      NULL,					\
			      &mdio_sam_dev_data##n,			\
			      &mdio_sam_dev_config_##n, POST_KERNEL,	\
			      CONFIG_MDIO_INIT_PRIORITY,		\
			      &mdio_sam_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_SAM_DEVICE)
