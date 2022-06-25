/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <inttypes.h>
#include <string.h>

#define UART_DEVICE_NODE DT_CHOSEN(bt_device)
#define MSG_SIZE 32
#define SLEEP_TIME_MS 1
/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															  {0});
static struct gpio_callback button_cb_data;

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev))
	{
		return;
	}

	while (uart_irq_rx_ready(uart_dev))
	{

		uart_fifo_read(uart_dev, &c, 1);
		
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0)
		{

			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		}
		else if (rx_buf_pos < (sizeof(rx_buf) - 1))
		{
			rx_buf[rx_buf_pos++] = c;
			
		}
		/* else: characters beyond buffer size are dropped */
	}
	
}

/*
 * Print a string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++)
	{
		uart_poll_out(uart_dev, buf[i]);
	}
	printk("TX: %s\r\n",buf);
}
void button_pressed(const struct device *dev, struct gpio_callback *cb,
					   uint32_t pins)
{
	print_uart("AT+BAUD?\r\n");
}

void main(void)
{
	int ret;
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev))
	{
		printk("UART device not found!");
		return;
	}

	if (!device_is_ready(button.port))
	{
		printk("Error: button device %s is not ready\n",
			   button.port->name);
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	printk("Press the button\n");
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0)
	{
		printk("RX: %s\r\n",tx_buf);
	}
}
