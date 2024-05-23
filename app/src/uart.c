#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "uart.h"
#include "ui.h"

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));

static char rx_buf[CMD_MSG_SIZE];
static int rx_buf_pos;

LOG_MODULE_REGISTER(uart);

/* Queue to store up to 4 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, CMD_MSG_SIZE, 4, 4);

struct k_work uart_work;

void uart_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart)) {
		return;
	}

	if (!uart_irq_rx_ready(uart)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart, &c, 1) == 1) {
		//LOG_INF("%d", c);
		if (c == '\n') {

		} else if ((c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';
			//LOG_INF("[%s]",rx_buf);

			/* if queue is full, message is silently dropped */
			if (rx_buf[0] == 0) {
				// Remove null at start of string
				k_msgq_put(&uart_msgq, &rx_buf[1], K_NO_WAIT);
			} else { 
				k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			}
			k_work_submit(&uart_work);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

void uart_work_function(struct k_work *item)
{
	char buffer[CMD_MSG_SIZE];
    char uart_message[100];

	//LOG_INF("ESP32/UART Work Function");

	// Keep processing until there are no more NMEA messages in queue:
	while (k_msgq_num_used_get(&uart_msgq)) {
		// 1 or more messages in message queue:
		if (k_msgq_get(&uart_msgq, &buffer, K_FOREVER) == 0) {
            uart_send("Echo: ");
		    uart_send(buffer);
		    uart_send("\r\n");      
            sprintf(uart_message, "UART Echo: %s", buffer); 
            update_text(uart_message);  
		}
	}

}

int uart_init(void)
{
	int ret;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	ret = uart_irq_callback_user_data_set(uart, uart_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		} else {
			LOG_ERR("Error setting UART callback: %d", ret);
		}
		return -1;
	}

	k_work_init(&uart_work, uart_work_function);

	uart_irq_rx_enable(uart);

	return ret;
}

void uart_send(char *buffer)
{
	for (int i = 0; i < strlen(buffer); i++) {
		uart_poll_out(uart, buffer[i]);
	}
}