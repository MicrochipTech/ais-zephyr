/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic32_gpio

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#define GPIO_INT_FALLING_EDGE		0x1
#define GPIO_INT_RISING_EDGE		0x2
#define GPIO_INT_BOTH_EDGES		0x3
#define GPIO_INT_LOW_LEVEL		0x4
#define GPIO_INT_HIGH_LEVEL		0x5

struct gpio_pic32_config {
	struct gpio_driver_config common;
	uint32_t regs;
};

struct gpio_pic32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

static int gpio_pic32_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;

	if (flags & GPIO_OUTPUT) {
		/* Output is incompatible with pull */
		if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
			return -ENOTSUP;
		}
		/* Bidirectional is supported */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			regs->GPIO_GPO_CLEAR = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			regs->GPIO_GPO_SET = BIT(pin);
		}
		regs->CONFIG[pin].GPIO_CFG = GPIO_CFG_DIR(1);
	} else {
		regs->CONFIG[pin].GPIO_CFG = GPIO_CFG_DIR(0);
	}

	return 0;
}

static int gpio_pic32_port_get_raw(const struct device *dev,
				   gpio_port_value_t *value)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;

	*value = regs->GPIO_GPI_VALUE;

	return 0;
}

static int gpio_pic32_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;
	uint32_t out = regs->GPIO_GPO_MASK;

	regs->GPIO_GPO_MASK = (out & ~mask) | (value & mask);

	return 0;
}

static int gpio_pic32_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;

	regs->GPIO_GPO_SET = pins;

	return 0;
}

static int gpio_pic32_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;

	regs->GPIO_GPO_CLEAR = pins;

	return 0;
}

static int gpio_pic32_port_toggle_bits(const struct device *dev, gpio_port_pins_t mask)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;
	uint32_t gpo_value = regs->GPIO_GPO_VALUE;

	regs->GPIO_GPO_CLEAR = gpo_value & mask;
	regs->GPIO_GPO_SET = (~gpo_value) & mask;

	return 0;
}

static int gpio_pic32_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_pic32_config *config = dev->config;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;
	uint32_t intAttr;

	if (mode == GPIO_INT_MODE_DISABLED) {
		regs->CONFIG[pin].GPIO_CFG &= ~BIT(4);
		regs->GPIO_C0A_INTSTATUS |= BIT(pin);
	} else {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			intAttr = ((mode == GPIO_INT_MODE_EDGE) ? GPIO_INT_FALLING_EDGE : GPIO_INT_LOW_LEVEL);
			break;
		case GPIO_INT_TRIG_HIGH:
			intAttr = ((mode == GPIO_INT_MODE_EDGE) ? GPIO_INT_RISING_EDGE : GPIO_INT_HIGH_LEVEL);
			break;
		case GPIO_INT_TRIG_BOTH:
			if (mode != GPIO_INT_MODE_EDGE) {
				return -ENOTSUP;
			}
			intAttr = GPIO_INT_BOTH_EDGES;
			break;
		default:
			return -ENOTSUP;
		}
		regs->CONFIG[pin].GPIO_CFG |= BIT(4) | intAttr;
	}

	return 0;
}

static int gpio_pic32_manage_callback(const struct device *dev, struct gpio_callback *callback,
				      bool set)
{
	struct gpio_pic32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_pic32_api = {
	.pin_configure = gpio_pic32_config,
	.port_get_raw = gpio_pic32_port_get_raw,
	.port_set_masked_raw = gpio_pic32_port_set_masked_raw,
	.port_set_bits_raw = gpio_pic32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_pic32_port_clear_bits_raw,
	.port_toggle_bits = gpio_pic32_port_toggle_bits,
	.pin_interrupt_configure = gpio_pic32_pin_interrupt_configure,
	.manage_callback = gpio_pic32_manage_callback,
};

static void gpio_pic32_isr(const struct device *dev)
{
	const struct gpio_pic32_config *config = dev->config;
	struct gpio_pic32_data *data = dev->data;
	gpio_registers_t *regs = (gpio_registers_t *)config->regs;
	uint32_t int_status;

	int_status = regs->GPIO_C0A_INTSTATUS;

	regs->GPIO_C0A_INTSTATUS = int_status;

	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

#define GPIO_PIC32_IRQ_INIT(inst)                                               \
	do {                                                                    \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    gpio_pic32_isr, DEVICE_DT_INST_GET(inst), 0);      	\
                                                                                \
		irq_enable(DT_INST_IRQN(inst));                                 \
	} while (0)

#define GPIO_PIC32_INIT(inst)                                       		\
static const struct gpio_pic32_config gpio_pic32_config_##inst = {              \
	.common = {                                                                        \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),               \
		},                                                                         \
	.regs = DT_INST_REG_ADDR(inst),				\
	};							\
static struct gpio_pic32_data gpio_pic32_data_##inst;                          	\
										\
static int gpio_pic32_init_##inst(const struct device *dev)                 	\
{                                                                               \
	IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 0), (GPIO_PIC32_IRQ_INIT(inst);))	\
										\
	return 0;								\
}										\
DEVICE_DT_INST_DEFINE(inst, gpio_pic32_init_##inst, NULL,			\
		      &gpio_pic32_data_##inst,					\
		      &gpio_pic32_config_##inst, PRE_KERNEL_1,			\
		      CONFIG_GPIO_INIT_PRIORITY, &gpio_pic32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PIC32_INIT)
