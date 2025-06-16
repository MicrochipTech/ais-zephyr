/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic32_uart

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <string.h>
#include <zephyr/irq.h>

/* Device constant configuration parameters */
struct uart_pic32_dev_cfg {
	sercom_usart_registers_t *regs;
	const struct device *clk_dev;
	uint32_t pads;
	const struct pinctrl_dev_config *pcfg;
	uint32_t baudrate;
	uint8_t periph_clk_id;
#if CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_pic32_dev_data {
	struct uart_config config_cache;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	uint8_t txc_cache;
#endif
};

static void wait_synchronization(sercom_usart_registers_t *const regs, uint32_t syncbusy_mask)
{
	while ((regs->SERCOM_USART_SYNCBUSY & syncbusy_mask) != 0) {
	}
};

static int uart_pic32_set_baudrate(sercom_usart_registers_t *const usart, uint32_t baudrate,
				  uint32_t clk_freq_hz)
{
	uint64_t tmp;
	uint16_t baud;

	tmp = (uint64_t)baudrate << 20;
	tmp = (tmp + (clk_freq_hz >> 1)) / clk_freq_hz;

	/* Verify that the calculated result is within range */
	if (tmp < 1 || tmp > UINT16_MAX) {
		return -ERANGE;
	}

	baud = 65536 - (uint16_t)tmp;
	usart->SERCOM_USART_BAUD_RXPL = SERCOM_USART_BAUD_RXPL_BAUD_BAUD(baud);

	return 0;
}

static int uart_pic32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_pic32_dev_cfg *config = dev->config;

	sercom_usart_registers_t * const regs = config->regs;

	if (!(regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk)) {
		return -EBUSY;
	}

	*c = (unsigned char)regs->SERCOM_USART_DATA;
	return 0;
}

static void uart_pic32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_pic32_dev_cfg *config = dev->config;

	sercom_usart_registers_t * const regs = config->regs;

	while (!(regs->SERCOM_USART_INTFLAG_STATUS &
	       SERCOM_USART_INTFLAG_STATUS_DRE_INTFLAG_Msk)) {
	}

	/* send a character */
	regs->SERCOM_USART_DATA = c;

	while ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Msk) == 0) {
	}
}

static int uart_pic32_err_check(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;

	sercom_usart_registers_t * const regs = config->regs;
	uint32_t err = 0U;

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_BUFOVF_STATUS_Msk) {
		err |= UART_ERROR_OVERRUN;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_FERR_STATUS_Msk) {
		err |= UART_ERROR_FRAMING;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_PERR_STATUS_Msk) {
		err |= UART_ERROR_PARITY;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXBRK_INTFLAG_Msk) {
		err |= UART_BREAK;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_COLL_STATUS_Msk) {
		err |= UART_ERROR_COLLISION;
	}

	regs->SERCOM_USART_INTFLAG_STATUS |= SERCOM_USART_INTFLAG_STATUS_BUFOVF_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_FERR_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_PERR_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_COLL_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_RXBRK_INTFLAG_Msk;
	return err;
}

#if CONFIG_UART_INTERRUPT_DRIVEN
static int uart_pic32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t *regs = config->regs;

	if ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_DRE_INTFLAG_Msk) &&
	    len >= 1) {
		regs->SERCOM_USART_DATA = tx_data[0];
		/* Wait until Tx complete */
		while ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Msk) == 0) {
		}
		/* Clear TXC */
		regs->SERCOM_USART_INTFLAG_STATUS |= SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Msk;
		return 1;
	} else {
		return 0;
	}
}

static int uart_pic32_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t *regs = config->regs;

	if ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk) != 0) {
		uint8_t ch = regs->SERCOM_USART_DATA;

		if (size >= 1) {
			*rx_data = ch;
			return 1;
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static void uart_pic32_irq_tx_enable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t *regs = config->regs;
	uint32_t temp = regs->SERCOM_USART_INTENCLR_INTENSET;

	temp |= SERCOM_USART_INTENCLR_INTENSET_DRE_INTENSET(1) | SERCOM_USART_INTENCLR_INTENSET_TXC_INTENSET(1);
	temp &= 0xFFFFFF00;
	regs->SERCOM_USART_INTENCLR_INTENSET = temp;
}

static void uart_pic32_irq_tx_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;
	uint32_t temp = regs->SERCOM_USART_INTENCLR_INTENSET;

	temp |= SERCOM_USART_INTENCLR_INTENSET_DRE_INTENCLR(1) | SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR(1);
	temp &= 0xFFFFFF03;
	regs->SERCOM_USART_INTENCLR_INTENSET = temp;
}

static int uart_pic32_irq_tx_ready(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	return ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_DRE_INTFLAG_Msk) != 0)
		&& ((regs->SERCOM_USART_INTENCLR_INTENSET & SERCOM_USART_INTENCLR_INTENSET_DRE_INTENSET_Msk) != 0);
}

static int uart_pic32_irq_tx_complete(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;
	sercom_usart_registers_t * const regs = config->regs;

	return (dev_data->txc_cache != 0) && ((regs->SERCOM_USART_INTENCLR_INTENSET & SERCOM_USART_INTENCLR_INTENSET_TXC_INTENSET_Msk) != 0);
}

static void uart_pic32_irq_rx_enable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;
	uint32_t temp = regs->SERCOM_USART_INTENCLR_INTENSET;

	temp |= SERCOM_USART_INTENCLR_INTENSET_RXC_INTENSET(1);
	temp &= 0xFFFFFF00;
	regs->SERCOM_USART_INTENCLR_INTENSET = temp;
}

static void uart_pic32_irq_rx_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;
	uint32_t temp = regs->SERCOM_USART_INTENCLR_INTENSET;

	temp |= SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);
	temp &= 0xFFFFFF04;
	regs->SERCOM_USART_INTENCLR_INTENSET = temp;
}

static int uart_pic32_irq_rx_ready(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	return (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk) != 0;
}

static int uart_pic32_irq_is_pending(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	return (regs->SERCOM_USART_INTENCLR_INTENSET & regs->SERCOM_USART_INTFLAG_STATUS) != 0;
}

static void uart_pic32_irq_err_enable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;
	uint32_t temp = regs->SERCOM_USART_INTENCLR_INTENSET;

	temp |= SERCOM_USART_INTENCLR_INTENSET_ERROR_INTENSET(1);
	temp &= 0xFFFFFF00;
	regs->SERCOM_USART_INTENCLR_INTENSET = temp;
}

static void uart_pic32_irq_err_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET |= SERCOM_USART_INTENCLR_INTENSET_ERROR_INTENCLR(1);
}

static int uart_pic32_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_pic32_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;

#if defined(CONFIG_UART_PIC32_ASYNC) && defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->async_cb = NULL;
	dev_data->async_cb_data = NULL;
#endif
}
#endif

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_pic32_configure(const struct device *dev, const struct uart_config *new_cfg)
{
	int retval;

	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;
	sercom_usart_registers_t *regs = cfg->regs;

	regs->SERCOM_USART_CTRLA &= ~SERCOM_USART_CTRLA_ENABLE_Msk;

	wait_synchronization(regs, SERCOM_USART_SYNCBUSY_ENABLE_Msk);

	if (new_cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		/* Flow control not yet supported though in principle possible
		 * on this soc family.
		 */
		return -ENOTSUP;
	}

	dev_data->config_cache.flow_ctrl = new_cfg->flow_ctrl;

	uint32_t CTRLA_temp = regs->SERCOM_USART_CTRLA;
	uint32_t CTRLB_temp = regs->SERCOM_USART_CTRLB;

	switch (new_cfg->parity) {
	case UART_CFG_PARITY_NONE:
		CTRLA_temp &= ~SERCOM_USART_CTRLA_FORM_Msk;
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(SERCOM_USART_CTRLA_FORM_USART_Val);
		break;
	case UART_CFG_PARITY_ODD:
		CTRLA_temp &= ~SERCOM_USART_CTRLA_FORM_Msk;
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(SERCOM_USART_CTRLA_FORM_USARTP_Val);
		CTRLB_temp &= ~SERCOM_USART_CTRLB_PMODE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_PMODE(SERCOM_USART_CTRLB_PMODE_ODD_Val);
		break;
	case UART_CFG_PARITY_EVEN:
		CTRLA_temp &= ~SERCOM_USART_CTRLA_FORM_Msk;
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(SERCOM_USART_CTRLA_FORM_USARTP_Val);
		CTRLB_temp &= ~SERCOM_USART_CTRLB_PMODE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_PMODE(SERCOM_USART_CTRLB_PMODE_EVEN_Val);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.parity = new_cfg->parity;

	switch (new_cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_SBMODE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_SBMODE(SERCOM_USART_CTRLB_SBMODE_ONE_Val);
		break;
	case UART_CFG_STOP_BITS_2:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_SBMODE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_SBMODE(SERCOM_USART_CTRLB_SBMODE_TWO_Val);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.stop_bits = new_cfg->stop_bits;

	switch (new_cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(SERCOM_USART_CTRLB_CHSIZE_5BITS_Val);
		break;
	case UART_CFG_DATA_BITS_6:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(SERCOM_USART_CTRLB_CHSIZE_6BITS_Val);
		break;
	case UART_CFG_DATA_BITS_7:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(SERCOM_USART_CTRLB_CHSIZE_7BITS_Val);
		break;
	case UART_CFG_DATA_BITS_8:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(SERCOM_USART_CTRLB_CHSIZE_8BITS_Val);
		break;
	case UART_CFG_DATA_BITS_9:
		CTRLB_temp &= ~SERCOM_USART_CTRLB_CHSIZE_Msk;
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(SERCOM_USART_CTRLB_CHSIZE_9BITS_Val);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.data_bits = new_cfg->data_bits;

	regs->SERCOM_USART_CTRLA = CTRLA_temp;

	regs->SERCOM_USART_CTRLB = CTRLB_temp;
	wait_synchronization(regs, SERCOM_USART_SYNCBUSY_CTRLB_Msk);

	retval = uart_pic32_set_baudrate(regs, new_cfg->baudrate, SOC_MICROCHIP_PIC32_HCLK_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	dev_data->config_cache.baudrate = new_cfg->baudrate;

	regs->SERCOM_USART_CTRLA |= SERCOM_USART_CTRLA_ENABLE(1);
	wait_synchronization(regs, SERCOM_USART_SYNCBUSY_ENABLE_Msk);

	return 0;
}

static int uart_pic32_config_get(const struct device *dev,
				struct uart_config *out_cfg)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

	memcpy(out_cfg, &(dev_data->config_cache), sizeof(dev_data->config_cache));

	return 0;
}
#endif

static int uart_pic32_init(const struct device *dev)
{
	int retval;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;
	sercom_usart_registers_t * const regs = cfg->regs;

	retval = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->periph_clk_id);
	if (retval != 0) {
		return retval;
	}

	/* Enable PINMUX based on PINCTRL */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	/* Set baud rate */
	retval = uart_pic32_set_baudrate(regs, cfg->baudrate, SOC_MICROCHIP_PIC32_HCLK_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	dev_data->config_cache.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	dev_data->config_cache.parity = UART_CFG_PARITY_NONE;
	dev_data->config_cache.stop_bits = UART_CFG_STOP_BITS_1;
	dev_data->config_cache.data_bits = UART_CFG_DATA_BITS_8;
	dev_data->config_cache.baudrate = cfg->baudrate;

	/* Enable Tx & Rx */
	regs->SERCOM_USART_CTRLB = SERCOM_USART_CTRLB_TXEN(1) | SERCOM_USART_CTRLB_RXEN(1);
	/* Wait for SERCOM_USART_CTRLB reguster write Synchronization busy */
	wait_synchronization(regs, SERCOM_USART_SYNCBUSY_CTRLB_Msk);

	/* Configure Usart with internal clock, LSB data order, immediate buffer overflow
	 * notification, LSB data order, and txpo & rxpo pads.
	 */
	regs->SERCOM_USART_CTRLA = SERCOM_USART_CTRLA_MODE(SERCOM_USART_CTRLA_MODE_INTCLK_Val) |
				   SERCOM_USART_CTRLA_DORD(SERCOM_USART_CTRLA_DORD_LSB_Val) |
				   SERCOM_USART_CTRLA_IBON(1) |
				   cfg->pads;

#if CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif

	/* Enable Usart communication */
	regs->SERCOM_USART_CTRLA |= SERCOM_USART_CTRLA_ENABLE(1); 
	/* Wait for SERCOM Enable Synchronization busy */
	wait_synchronization(regs, SERCOM_USART_SYNCBUSY_ENABLE_Msk);

	return 0;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

static void uart_pic32_isr(const struct device *dev)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

#if CONFIG_UART_INTERRUPT_DRIVEN
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
#endif
}

#endif

static const struct uart_driver_api uart_pic32_driver_api = {
	.poll_in = uart_pic32_poll_in,
	.poll_out = uart_pic32_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_pic32_configure,
	.config_get = uart_pic32_config_get,
#endif
	.err_check = uart_pic32_err_check,
#if CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_pic32_fifo_fill,
	.fifo_read = uart_pic32_fifo_read,
	.irq_tx_enable = uart_pic32_irq_tx_enable,
	.irq_tx_disable = uart_pic32_irq_tx_disable,
	.irq_tx_ready = uart_pic32_irq_tx_ready,
	.irq_tx_complete = uart_pic32_irq_tx_complete,
	.irq_rx_enable = uart_pic32_irq_rx_enable,
	.irq_rx_disable = uart_pic32_irq_rx_disable,
	.irq_rx_ready = uart_pic32_irq_rx_ready,
	.irq_is_pending = uart_pic32_irq_is_pending,
	.irq_err_enable = uart_pic32_irq_err_enable,
	.irq_err_disable = uart_pic32_irq_err_disable,
	.irq_update = uart_pic32_irq_update,
	.irq_callback_set = uart_pic32_irq_callback_set,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN

#define PIC32_UART_IRQ_CONNECT(n)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_pic32_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	} while (false)

#define UART_PIC32_IRQ_HANDLER_DECL(n)					\
	static void uart_pic32_irq_config_##n(const struct device *dev)
#define UART_PIC32_IRQ_HANDLER_FUNC(n)					\
	.irq_config_func = uart_pic32_irq_config_##n,

#define UART_PIC32_IRQ_HANDLER(n)					\
static void uart_pic32_irq_config_##n(const struct device *dev)		\
{									\
	PIC32_UART_IRQ_CONNECT(n);					\
}
#else
#define UART_PIC32_IRQ_HANDLER_DECL(n)
#define UART_PIC32_IRQ_HANDLER_FUNC(n)
#define UART_PIC32_IRQ_HANDLER(n)
#endif

#define UART_PIC32_SERCOM_PADS(n) \
	(DT_INST_PROP(n, rxpo) << SERCOM_USART_CTRLA_RXPO_Pos) |	\
	(DT_INST_PROP(n, txpo) << SERCOM_USART_CTRLA_TXPO_Pos)

#define UART_PIC32_CONFIG_DEFN(n)					\
static const struct uart_pic32_dev_cfg uart_pic32_config_##n = {	\
	.regs = (sercom_usart_registers_t *)DT_INST_REG_ADDR(n),	\
	.baudrate = DT_INST_PROP(n, current_speed),			\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_DRV_INST(n))),       \
        .periph_clk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, clkrstgen, id), \
	.pads = UART_PIC32_SERCOM_PADS(n),				\
	UART_PIC32_IRQ_HANDLER_FUNC(n)					\
}

#define UART_PIC32_DEVICE_INIT(n)					\
PINCTRL_DT_INST_DEFINE(n);						\
static struct uart_pic32_dev_data uart_pic32_data_##n;		\
UART_PIC32_IRQ_HANDLER_DECL(n);						\
UART_PIC32_CONFIG_DEFN(n);						\
DEVICE_DT_INST_DEFINE(n, uart_pic32_init, NULL,			\
		    &uart_pic32_data_##n,				\
		    &uart_pic32_config_##n, PRE_KERNEL_1,		\
		    CONFIG_SERIAL_INIT_PRIORITY,			\
		    &uart_pic32_driver_api);				\
UART_PIC32_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(UART_PIC32_DEVICE_INIT)
