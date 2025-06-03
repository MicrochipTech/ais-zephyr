/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic32_watchdog

#include <soc.h>
#include <zephyr/drivers/watchdog.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(wdt_pic32);

static void wdt_pic32_wait_synchronization(void)
{
	/* Waiting for the completion of synchronization */
	while (WDT_REGS->WDT_SYNCBUSY & WDT_SYNCBUSY_CLEAR(1)) {
	}
}

static inline bool wdt_pic32_is_enabled(void)
{
	return WDT_REGS->WDT_CTRLA & WDT_CTRLA_ENABLE_Msk;
}

static inline void wdt_pic32_set_enable(bool enable)
{
	if (enable) {
		if ((WDT_REGS->WDT_CTRLA & WDT_CTRLA_ALWAYSON_Msk) == 0) {
			WDT_REGS->WDT_CTRLA |= WDT_CTRLA_ENABLE(1);
		} else {
			return -EFAULT;
		}
	} else {
		if (!(WDT_REGS->WDT_CTRLA & WDT_CTRLA_ALWAYSON_Msk)) {
			WDT_REGS->WDT_CTRLA &= (uint32_t)(~WDT_CTRLA_ENABLE(1));
		} else {
			return -EFAULT;
		}
	}

	wdt_pic32_wait_synchronization();

	return 0;
}

static int wdt_pic32_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	/* Normal mode */
	if (((WDT_REGS->WDT_CTRLA & WDT_CTRLA_ALWAYSON_Msk) == 0) &&
	    ((WDT_REGS->WDT_CTRLA & WDT_CTRLA_ENABLE(1)) == 0)) {
		// Configure the Time-Out Period in the Configuration register for Normal Mode operation
		/* timeout -> cfg->window.max */
		WDT_REGS->WDT_CONFIG_ = WDT_CONFIG__PER(cfg->window.max);
		// Early Warning interrupt time-offset(EWCTRL.EWOFFSET) should always be less than than the watchdog time-out period
		// Set the Early Warning Offset bits in EWCTRL.EWOFFSET
		/* earlyWarningTimeOut -> cfg->window.min */
		WDT_REGS->WDT_EWCTRL = WDT_EWCTRL_EWOFFSET(cfg->window.min);
	} else {
		return -EFAULT;
	}

	/* Window mode */
	if (((WDT_REGS->WDT_CTRLA & WDT_CTRLA_ALWAYSON_Msk) == 0) &&
	    ((WDT_REGS->WDT_CTRLA & WDT_CTRLA_ENABLE(1)) == 0)) {
		// Set Closed window period: by the Window Period bits in CONFIG.WINDOW
		// Set Open window period: by the Period bits in CONFIG.PER
		// Configure the Time-Out Period in the Configuration register for Window Mode operation
		/* closedWindowPeriod -> cfg->window.min */
		/* openWindowPeriod -> cfg->window.max */
		WDT_REGS->WDT_CONFIG_ = uint32_t(WDT_CONFIG__WINDOW(cfg->window.min) |
					WDT_CONFIG__PER(cfg->window.max));
		// Early Warning interrupt time-offset(EWCTRL.EWOFFSET) should always be less than than the watchdog time-out period
		// Set the Early Warning Offset bits in EWCTRL.EWOFFSET
		/* closedWindowPeriod -> cfg->window.min */
		WDT_REGS->WDT_EWCTRL = WDT_EWCTRL_EWOFFSET(cfg->window.min);
		// Setting the Window Enable bit (CTRLA.WEN=1) for Window mode operation
		WDT_REGS->WDT_CTRLA |= WDT_CTRLA_WEN(1);
		wdt_pic32_wait_synchronization();
	} else {
		return -EFAULT;
	}

	return 0;
}

static int wdt_pic32_setup(const struct device *dev, uint8_t options)
{
	if (wdt_pic32_is_enabled()) {
		LOG_ERR("Watchdog already setup");
		return -EBUSY;
	}

	/* Enable watchdog */
	wdt_pic32_set_enable(true);
	wdt_pic32_wait_synchronization();

	return 0;
}

static int wdt_pic32_disable(const struct device *dev)
{
	if (!wdt_pic32_is_enabled()) {
		return -EFAULT;
	}

	wdt_pic32_set_enable(0);
	wdt_pic32_wait_synchronization();

	return 0;
}

static int wdt_pic32_feed(const struct device *dev, int channel_id)
{
	struct wdt_sam0_dev_data *data = dev->data;

	WDT_REGS->WDT_CLEAR = WDT_CLEAR_CLEAR(0xA5);

	wdt_pic32_wait_synchronization();

	return 0;
}

static void wdt_pic32_isr(const struct device *dev)
{
	struct wdt_pic32_dev_data *data = dev->data;

	// Clear early warning interrupt flag by writing '1' to INTFLAG.EW
	WDT_REGS->WDT_INTFLAG = WDT_INTFLAG_EW(1);

	if (data->cb != NULL) {
		data->cb(dev, 0);
	}
}

static const struct wdt_driver_api wdt_pic32_api = {
	.setup = wdt_pic32_setup,
	.disable = wdt_pic32_disable,
	.install_timeout = wdt_pic32_install_timeout,
	.feed = wdt_pic32_feed,
};

static int wdt_pic32_init(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), wdt_pic32_isr, DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static struct wdt_pic32_dev_data wdt_pic32_data;

//DEVICE_DT_INST_DEFINE(0, wdt_pic32_init, NULL,
//		    &wdt_pic32_data, NULL, PRE_KERNEL_1,
//		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_pic32_api);
