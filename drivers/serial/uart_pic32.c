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
#include <zephyr/drivers/pinctrl.h>
#include <string.h>
#include <zephyr/irq.h>

/* Device constant configuration parameters */
struct uart_pic32_dev_cfg {
	sercom_usart_registers_t *regs;
	uint32_t pads;
	const struct pinctrl_dev_config *pcfg;
	uint32_t baudrate;
	uint8_t clk_gen_periph_id;
#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_PIC32_ASYNC
	void (*irq_config_func)(const struct device *dev);
#endif
#if CONFIG_UART_PIC32_ASYNC
	const struct device *dma_dev;
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
#endif
};

struct uart_pic32_dev_data {
	struct uart_config config_cache;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	uint8_t txc_cache;
#endif
#if CONFIG_UART_PIC32_ASYNC
	const struct device *dev;
	const struct uart_pic32_dev_cfg *cfg;

	uart_callback_t async_cb;
	void *async_cb_data;

	struct k_work_delayable tx_timeout_work;
	const uint8_t *tx_buf;
	size_t tx_len;

	struct k_work_delayable rx_timeout_work;
	size_t rx_timeout_time;
	size_t rx_timeout_chunk;
	uint32_t rx_timeout_start;
	uint8_t *rx_buf;
	size_t rx_len;
	size_t rx_processed_len;
	uint8_t *rx_next_buf;
	size_t rx_next_len;
	bool rx_waiting_for_irq;
	bool rx_timeout_from_isr;
#endif
};

static void wait_synchronization(sercom_usart_registers_t *const regs)
{
	/* SYNCBUSY is a register */
	while ((regs->SERCOM_USART_SYNCBUSY & SERCOM_USART_SYNCBUSY_Msk) != 0) {
	}
}

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
	wait_synchronization(usart);

	return 0;
}

#if CONFIG_UART_PIC32_ASYNC

static void uart_pic32_dma_tx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_pic32_dev_data *const dev_data =
		(struct uart_pic32_dev_data *const) arg;
	const struct uart_pic32_dev_cfg *const cfg = dev_data->cfg;

	sercom_usart_registers_t * const regs = cfg->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR(1);
}

static int uart_pic32_tx_halt(struct uart_pic32_dev_data *dev_data)
{
	const struct uart_pic32_dev_cfg *const cfg = dev_data->cfg;
	unsigned int key = irq_lock();
	size_t tx_active = dev_data->tx_len;
	struct dma_status st;

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = dev_data->tx_buf,
			.len = 0U,
		},
	};

	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0U;

	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);

	irq_unlock(key);

	if (dma_get_status(cfg->dma_dev, cfg->tx_dma_channel, &st) == 0) {
		evt.data.tx.len = tx_active - st.pending_length;
	}

	if (tx_active) {
		if (dev_data->async_cb) {
			dev_data->async_cb(dev_data->dev,
					   &evt, dev_data->async_cb_data);
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static void uart_pic32_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_pic32_dev_data *dev_data = CONTAINER_OF(dwork,
							   struct uart_pic32_dev_data, tx_timeout_work);

	uart_pic32_tx_halt(dev_data);
}

static void uart_pic32_notify_rx_processed(struct uart_pic32_dev_data *dev_data,
					  size_t processed)
{
	if (!dev_data->async_cb) {
		return;
	}

	if (dev_data->rx_processed_len == processed) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = dev_data->rx_buf,
			.offset = dev_data->rx_processed_len,
			.len = processed - dev_data->rx_processed_len,
		},
	};

	dev_data->rx_processed_len = processed;

	dev_data->async_cb(dev_data->dev,
			   &evt, dev_data->async_cb_data);
}

static void uart_pic32_dma_rx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_pic32_dev_data *const dev_data =
		(struct uart_pic32_dev_data *const)arg;
	const struct device *dev = dev_data->dev;
	const struct uart_pic32_dev_cfg *const cfg = dev_data->cfg;
	sercom_usart_registers_t * const regs = cfg->regs;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	uart_pic32_notify_rx_processed(dev_data, dev_data->rx_len);

	if (dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = dev_data->rx_buf,
			},
		};

		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	/* No next buffer, so end the transfer */
	if (!dev_data->rx_next_len) {
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;

		if (dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
		return;
	}

	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0U;
	dev_data->rx_processed_len = 0U;

	dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
		   (uint32_t)(&(regs->SERCOM_USART_DATA)),
		   (uint32_t)dev_data->rx_buf, dev_data->rx_len);

	/*
	 * If there should be a timeout, handle starting the DMA in the
	 * ISR, since reception resets it and DMA completion implies
	 * reception.  This also catches the case of DMA completion during
	 * timeout handling.
	 */
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		dev_data->rx_waiting_for_irq = true;
		regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);
		irq_unlock(key);
		return;
	}

	/* Otherwise, start the transfer immediately. */
	dma_start(cfg->dma_dev, cfg->rx_dma_channel);

	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	dev_data->async_cb(dev, &evt, dev_data->async_cb_data);

	irq_unlock(key);
}

static void uart_pic32_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_pic32_dev_data *dev_data = CONTAINER_OF(dwork,
							   struct uart_pic32_dev_data, rx_timeout_work);
	const struct uart_pic32_dev_cfg *const cfg = dev_data->cfg;
	sercom_usart_registers_t * const regs = cfg->regs;
	struct dma_status st;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}

	/*
	 * Stop the DMA transfer and restart the interrupt read
	 * component (so the timeout restarts if there's still data).
	 * However, just ignore it if the transfer has completed (nothing
	 * pending) that means the DMA ISR is already pending, so just let
	 * it handle things instead when we re-enable IRQs.
	 */
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel,
			   &st) == 0 && st.pending_length == 0U) {
		irq_unlock(key);
		return;
	}

	uint8_t *rx_dma_start = dev_data->rx_buf + dev_data->rx_len -
			     st.pending_length;
	size_t rx_processed = rx_dma_start - dev_data->rx_buf;

	/*
	 * We know we still have space, since the above will catch the
	 * empty buffer, so always restart the transfer.
	 */
	dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
		   (uint32_t)(&(regs->SERCOM_USART_DATA)),
		   (uint32_t)rx_dma_start,
		   dev_data->rx_len - rx_processed);

	dev_data->rx_waiting_for_irq = true;
	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);

	/*
	 * Never do a notify on a timeout started from the ISR: timing
	 * granularity means the first timeout can be in the middle
	 * of reception but still have the total elapsed time exhausted.
	 * So we require a timeout chunk with no data seen at all
	 * (i.e. no ISR entry).
	 */
	if (dev_data->rx_timeout_from_isr) {
		dev_data->rx_timeout_from_isr = false;
		k_work_reschedule(&dev_data->rx_timeout_work,
				      K_USEC(dev_data->rx_timeout_chunk));
		irq_unlock(key);
		return;
	}

	uint32_t now = k_uptime_get_32();
	uint32_t elapsed = now - dev_data->rx_timeout_start;

	if (elapsed >= dev_data->rx_timeout_time) {
		/*
		 * No time left, so call the handler, and let the ISR
		 * restart the timeout when it sees data.
		 */
		uart_pic32_notify_rx_processed(dev_data, rx_processed);
	} else {
		/*
		 * Still have time left, so start another timeout.
		 */
		uint32_t remaining = MIN(dev_data->rx_timeout_time - elapsed,
				      dev_data->rx_timeout_chunk);

		k_work_reschedule(&dev_data->rx_timeout_work,
				      K_USEC(remaining));
	}

	irq_unlock(key);
}

#endif


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
		err |= UART_ERROR_PARITY;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_PERR_STATUS_Msk) {
		err |= UART_ERROR_FRAMING;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_ISF_STATUS_Msk) {
		err |= UART_BREAK;
	}

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_COLL_STATUS_Msk) {
		err |= UART_ERROR_COLLISION;
	}

	regs->SERCOM_USART_INTFLAG_STATUS |= SERCOM_USART_INTFLAG_STATUS_BUFOVF_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_FERR_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_PERR_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_COLL_STATUS_Msk
					  |  SERCOM_USART_INTFLAG_STATUS_ISF_STATUS_Msk;

	wait_synchronization(regs);
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
		return 1;
	} else {
		return 0;
	}
}

static int uart_pic32_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t *regs = config->regs;

	if (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk) {
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

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_DRE_INTENCLR(1) |
					       SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR(1);
}

static void uart_pic32_irq_tx_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_DRE_INTENCLR(0) |
					       SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR(0);
}

static int uart_pic32_irq_tx_ready(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	return ((regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_DRE_INTFLAG_Msk) != 0)
		&& ((regs->SERCOM_USART_INTENCLR_INTENSET & SERCOM_USART_INTENCLR_INTENSET_DRE_INTENCLR_Msk) != 0);
}

static int uart_pic32_irq_tx_complete(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;
	sercom_usart_registers_t * const regs = config->regs;

	return (dev_data->txc_cache != 0) && ((regs->SERCOM_USART_INTENCLR_INTENSET & SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR_Msk) != 0);
}

static void uart_pic32_irq_rx_enable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);
}

static void uart_pic32_irq_rx_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(0);
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

	regs->SERCOM_USART_INTENCLR_INTENSET |= SERCOM_USART_INTENCLR_INTENSET_ERROR_INTENCLR(1);
	wait_synchronization(regs);
}

static void uart_pic32_irq_err_disable(const struct device *dev)
{
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	regs->SERCOM_USART_INTENCLR_INTENSET |= SERCOM_USART_INTENCLR_INTENSET_ERROR_INTENCLR(0);
	wait_synchronization(regs);
}

static int uart_pic32_irq_update(const struct device *dev)
{
	/* Clear sticky interrupts */
	const struct uart_pic32_dev_cfg *config = dev->config;
	sercom_usart_registers_t * const regs = config->regs;

	/*
	 * Cache the TXC flag, and use this cached value to clear the interrupt
	 * if we do not used the cached value, there is a chance TXC will set
	 * after caching...this will cause TXC to never cached.
	 */
	struct uart_pic32_dev_data *const dev_data = dev->data;

	dev_data->txc_cache = regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Msk;
	regs->SERCOM_USART_INTFLAG_STATUS = SERCOM_USART_INTFLAG_STATUS_ERROR_INTFLAG(1)
			  | SERCOM_USART_INTFLAG_STATUS_RXBRK_INTFLAG(1)
			  | SERCOM_USART_INTFLAG_STATUS_CTSIC_INTFLAG(1)
			  | SERCOM_USART_INTFLAG_STATUS_RXS_INTFLAG(1)
			  | (dev_data->txc_cache << SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Pos);
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

#ifdef CONFIG_UART_PIC32_ASYNC

static int uart_pic32_callback_set(const struct device *dev,
				  uart_callback_t callback,
				  void *user_data)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->cb = NULL;
	dev_data->cb_data = NULL;
#endif

	return 0;
}

static int uart_pic32_tx(const struct device *dev, const uint8_t *buf, size_t len,
			  int32_t timeout)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	sercom_usart_registers_t *regs = cfg->regs;
	int retval;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->tx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	dev_data->tx_buf = buf;
	dev_data->tx_len = len;

	irq_unlock(key);

	retval = dma_reload(cfg->dma_dev, cfg->tx_dma_channel, (uint32_t)buf,
			    (uint32_t)(&(regs->SERCOM_USART_DATA)), len);
	if (retval != 0U) {
		return retval;
	}

	if (timeout != SYS_FOREVER_US) {
		k_work_reschedule(&dev_data->tx_timeout_work,
				      K_USEC(timeout));
	}

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
err:
	irq_unlock(key);
	return retval;
}

static int uart_pic32_tx_abort(const struct device *dev)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;

	if (cfg->tx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	return uart_pic32_tx_halt(dev_data);
}

static int uart_pic32_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				 int32_t timeout)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	sercom_usart_registers_t *regs = cfg->regs;
	int retval;

	if (cfg->rx_dma_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	if (dev_data->rx_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	/* Read off anything that was already there */
	while (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk) {
		char discard = regs->SERCOM_USART_DATA;

		(void)discard;
	}

	retval = dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
			    (uint32_t)(&(regs->SERCOM_USART_DATA)),
			    (uint32_t)buf, len);
	if (retval != 0) {
		return retval;
	}

	dev_data->rx_buf = buf;
	dev_data->rx_len = len;
	dev_data->rx_processed_len = 0U;
	dev_data->rx_waiting_for_irq = true;
	dev_data->rx_timeout_from_isr = true;
	dev_data->rx_timeout_time = timeout;
	dev_data->rx_timeout_chunk = MAX(timeout / 4U, 1);

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);

	irq_unlock(key);
	return 0;

err:
	irq_unlock(key);
	return retval;
}

static int uart_pic32_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	struct uart_pic32_dev_data *const dev_data = dev->data;
	unsigned int key = irq_lock();
	int retval = 0;

	if (dev_data->rx_len == 0U) {
		retval = -EACCES;
		goto err;
	}

	if (dev_data->rx_next_len != 0U) {
		retval = -EBUSY;
		goto err;
	}

	dev_data->rx_next_buf = buf;
	dev_data->rx_next_len = len;

	irq_unlock(key);
	return 0;

err:
	irq_unlock(key);
	return retval;
}

static int uart_pic32_rx_disable(const struct device *dev)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	sercom_usart_registers_t *regs = cfg->regs;
	struct dma_status st;

	k_work_cancel_delayable(&dev_data->rx_timeout_work);

	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return -EINVAL;
	}

	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(0);
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);


	if (dma_get_status(cfg->dma_dev, cfg->rx_dma_channel,
			   &st) == 0 && st.pending_length != 0U) {
		size_t rx_processed = dev_data->rx_len - st.pending_length;

		uart_pic32_notify_rx_processed(dev_data, rx_processed);
	}

	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {
			.buf = dev_data->rx_buf,
		},
	};

	dev_data->rx_buf = NULL;
	dev_data->rx_len = 0U;

	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	if (dev_data->rx_next_len) {
		struct uart_event next_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = dev_data->rx_next_buf,
			},
		};

		dev_data->rx_next_buf = NULL;
		dev_data->rx_next_len = 0U;

		if (dev_data->async_cb) {
			dev_data->async_cb(dev, &next_evt, dev_data->async_cb_data);
		}
	}

	evt.type = UART_RX_DISABLED;
	if (dev_data->async_cb) {
		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	irq_unlock(key);

	return 0;
}
#endif

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_pic32_configure(const struct device *dev, const struct uart_config *new_cfg)
{
	int retval;

	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;
	sercom_usart_registers_t *regs = cfg->regs;

	wait_synchronization(regs);

	regs->SERCOM_USART_CTRLA |= SERCOM_USART_CTRLA_ENABLE(0);
	wait_synchronization(regs);

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
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(0);
		break;
	case UART_CFG_PARITY_ODD:
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(0x1);
		CTRLB_temp |= SERCOM_USART_CTRLB_PMODE(1);
		break;
	case UART_CFG_PARITY_EVEN:
		CTRLA_temp |= SERCOM_USART_CTRLA_FORM(0x1);
		CTRLB_temp |= SERCOM_USART_CTRLB_PMODE(0);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.parity = new_cfg->parity;

	switch (new_cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		CTRLB_temp |= SERCOM_USART_CTRLB_SBMODE(0);
		break;
	case UART_CFG_STOP_BITS_2:
		CTRLB_temp |= SERCOM_USART_CTRLB_SBMODE(1);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.stop_bits = new_cfg->stop_bits;

	switch (new_cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(0x5);
		break;
	case UART_CFG_DATA_BITS_6:
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(0x6);
		break;
	case UART_CFG_DATA_BITS_7:
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(0x7);
		break;
	case UART_CFG_DATA_BITS_8:
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(0x0);
		break;
	case UART_CFG_DATA_BITS_9:
		CTRLB_temp |= SERCOM_USART_CTRLB_CHSIZE(0x1);
		break;
	default:
		return -ENOTSUP;
	}

	dev_data->config_cache.data_bits = new_cfg->data_bits;

	CTRLB_temp |= SERCOM_USART_CTRLB_COLDEN(cfg->pads);

	regs->SERCOM_USART_CTRLA = CTRLA_temp;
	wait_synchronization(regs);

	regs->SERCOM_USART_CTRLB = CTRLB_temp;
	wait_synchronization(regs);

	retval = uart_pic32_set_baudrate(regs, new_cfg->baudrate, 50000000);
					//SOC_ATMEL_SAM0_GCLK0_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}

	dev_data->config_cache.baudrate = new_cfg->baudrate;

	regs->SERCOM_USART_CTRLA |= SERCOM_USART_CTRLA_ENABLE(1);
	wait_synchronization(regs);

	return 0;
}

static int uart_pic32_config_get(const struct device *dev,
				struct uart_config *out_cfg)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

	memcpy(out_cfg, &(dev_data->config_cache),
				sizeof(dev_data->config_cache));

	return 0;
}
#endif

static int uart_pic32_init(const struct device *dev)
{
#if 1
	int retval;
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	struct uart_pic32_dev_data *const dev_data = dev->data;

	sercom_usart_registers_t * const regs = cfg->regs;

	/* Enable the clock */
	if (cfg->clk_gen_periph_id == 3)
		CLK_RST_GEN_REGS->CLK_RST_GEN_CG_PERIPH_CTRL0 |= CLK_RST_GEN_CG_PERIPH_CTRL0_MASK_Msk;
	else if (cfg->clk_gen_periph_id == 4)
		CLK_RST_GEN_REGS->CLK_RST_GEN_CG_PERIPH_CTRL1 |= CLK_RST_GEN_CG_PERIPH_CTRL1_MASK_Msk;
	else if (cfg->clk_gen_periph_id == 5)
		CLK_RST_GEN_REGS->CLK_RST_GEN_CG_PERIPH_CTRL2 |= CLK_RST_GEN_CG_PERIPH_CTRL2_MASK_Msk;
	else if (cfg->clk_gen_periph_id == 6)
		CLK_RST_GEN_REGS->CLK_RST_GEN_CG_PERIPH_CTRL3 |= CLK_RST_GEN_CG_PERIPH_CTRL3_MASK_Msk;
	else
		return -ENOTSUP;

	/* Disable all USART interrupts */
	regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_Msk;
	wait_synchronization(regs);

	/* 8 bits of data, no parity, 1 stop bit in normal mode */
	regs->SERCOM_USART_CTRLA = 
	    cfg->pads
	    /* Internal clock */
	    | SERCOM_USART_CTRLA_MODE_INTCLK
#if defined(SERCOM_UART_CTRLA_SAMPR)
	    /* 16x oversampling with arithmetic baud rate generation */
	    | SERCOM_USART_CTRLA_SAMPR(0)
#endif
	    | SERCOM_USART_CTRLA_FORM(0) |
	    SERCOM_USART_CTRLA_CPOL(1) | SERCOM_USART_CTRLA_DORD(1);
	wait_synchronization(regs);

	/* Enable PINMUX based on PINCTRL */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	dev_data->config_cache.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	dev_data->config_cache.parity = UART_CFG_PARITY_NONE;
	dev_data->config_cache.stop_bits = UART_CFG_STOP_BITS_1;
	dev_data->config_cache.data_bits = UART_CFG_DATA_BITS_8;

	/* Enable receiver and transmitter */
	regs->SERCOM_USART_CTRLB = SERCOM_USART_CTRLB_CHSIZE(0) |
			   SERCOM_USART_CTRLB_RXEN(1) | SERCOM_USART_CTRLB_TXEN(1);
	wait_synchronization(regs);

	retval = uart_pic32_set_baudrate(regs, cfg->baudrate, 50000000);
					//SOC_ATMEL_SAM0_GCLK0_FREQ_HZ);
	if (retval != 0) {
		return retval;
	}
	dev_data->config_cache.baudrate = cfg->baudrate;

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_PIC32_ASYNC
	cfg->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_PIC32_ASYNC
	dev_data->dev = dev;
	dev_data->cfg = cfg;
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	k_work_init_delayable(&dev_data->tx_timeout_work, uart_pic32_tx_timeout);
	k_work_init_delayable(&dev_data->rx_timeout_work, uart_pic32_rx_timeout);

	if (cfg->tx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_pic32_dma_tx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->tx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.dest_address = (uint32_t)(&(regs->SERCOM_USART_DATA));
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

	if (cfg->rx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_pic32_dma_rx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->rx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.source_address = (uint32_t)(&(regs->SERCOM_USART_DATA));
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

#endif

	regs->SERCOM_USART_CTRLA |= SERCOM_USART_CTRLA_ENABLE(1);
	wait_synchronization(regs);
#endif
	return 0;
}

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_PIC32_ASYNC

static void uart_pic32_isr(const struct device *dev)
{
	struct uart_pic32_dev_data *const dev_data = dev->data;

#if CONFIG_UART_INTERRUPT_DRIVEN
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
#endif

#if CONFIG_UART_PIC32_ASYNC
	const struct uart_pic32_dev_cfg *const cfg = dev->config;
	sercom_usart_registers_t * const regs = cfg->regs;

	if (dev_data->tx_len && (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_TXC_INTFLAG_Msk)) {
		regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_TXC_INTENCLR(1);

		k_work_cancel_delayable(&dev_data->tx_timeout_work);

		unsigned int key = irq_lock();

		struct uart_event evt = {
			.type = UART_TX_DONE,
			.data.tx = {
				.buf = dev_data->tx_buf,
				.len = dev_data->tx_len,
			},
		};

		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0U;

		if (evt.data.tx.len != 0U && dev_data->async_cb) {
			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
	}

	if (dev_data->rx_len && (regs->SERCOM_USART_INTFLAG_STATUS & SERCOM_USART_INTFLAG_STATUS_RXC_INTFLAG_Msk) &&
	    dev_data->rx_waiting_for_irq) {
		dev_data->rx_waiting_for_irq = false;
		regs->SERCOM_USART_INTENCLR_INTENSET = SERCOM_USART_INTENCLR_INTENSET_RXC_INTENCLR(1);

		/* Receive started, so request the next buffer */
		if (dev_data->rx_next_len == 0U && dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_BUF_REQUEST,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		/*
		 * If we have a timeout, restart the time remaining whenever
		 * we see data.
		 */
		if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
			dev_data->rx_timeout_from_isr = true;
			dev_data->rx_timeout_start = k_uptime_get_32();
			k_work_reschedule(&dev_data->rx_timeout_work,
					      K_USEC(dev_data->rx_timeout_chunk));
		}

		/* DMA will read the currently ready byte out */
		dma_start(cfg->dma_dev, cfg->rx_dma_channel);
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
#if CONFIG_UART_PIC32_ASYNC
	.callback_set = uart_pic32_callback_set,
	.tx = uart_pic32_tx,
	.tx_abort = uart_pic32_tx_abort,
	.rx_enable = uart_pic32_rx_enable,
	.rx_buf_rsp = uart_pic32_rx_buf_rsp,
	.rx_disable = uart_pic32_rx_disable,
#endif
};

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_PIC32_ASYNC

#define PIC32_UART_IRQ_CONNECT(n)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq),		\
			    DT_INST_IRQ_BY_IDX(n, 0, priority),		\
			    uart_pic32_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));		\
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

//.clk_gen_periph_id = DT_INST_CLOCKS_CELL_BY_NAME(n, clkrstgen, clkctrl_id),
#define UART_PIC32_CONFIG_DEFN(n)					\
static const struct uart_pic32_dev_cfg uart_pic32_config_##n = {	\
	.regs = (sercom_usart_registers_t *)DT_INST_REG_ADDR(n),	\
	.baudrate = DT_INST_PROP(n, current_speed),			\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	.clk_gen_periph_id = 6, \
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

//DT_INST_FOREACH_STATUS_OKAY(UART_PIC32_DEVICE_INIT)
