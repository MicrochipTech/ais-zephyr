/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED2_NODE DT_ALIAS(led2)
#define LED6_NODE DT_ALIAS(led6)
#define LED8_NODE DT_ALIAS(led8)
#define LED10_NODE DT_ALIAS(led10)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led6 = GPIO_DT_SPEC_GET(LED6_NODE, gpios);
static const struct gpio_dt_spec led8 = GPIO_DT_SPEC_GET(LED8_NODE, gpios);
static const struct gpio_dt_spec led10 = GPIO_DT_SPEC_GET(LED10_NODE, gpios);

int main(void)
{
	int ret;
	//bool led_state = true;

	if (!gpio_is_ready_dt(&led2)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led6)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led6, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led8)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led8, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	if (!gpio_is_ready_dt(&led10)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led10, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led2);
		if (ret < 0) {
			return 0;
		}

		ret = gpio_pin_toggle_dt(&led6);
		if (ret < 0) {
			return 0;
		}

		ret = gpio_pin_toggle_dt(&led8);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_toggle_dt(&led10);
		if (ret < 0) {
			return 0;
		}

		//led_state = !led_state;
		//printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
