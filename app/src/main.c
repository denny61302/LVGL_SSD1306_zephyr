#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#include <dk_buttons_and_leds.h>

#include <lvgl.h>

#include <stdio.h>
#include <string.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define UART_BUF_SIZE 40
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX 50000

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));

#define CMD_MSG_SIZE 82
static char rx_buf[CMD_MSG_SIZE];
static int rx_buf_pos;

/* Queue to store up to 4 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, CMD_MSG_SIZE, 4, 4);

struct k_work uart_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

uint32_t count = 0U;
char count_str[11] = {0};
const struct device *display_dev;
lv_obj_t *hello_world_label;
lv_obj_t *count_label;

LOG_MODULE_REGISTER(main);

void uart_send(char *buffer)
{
	for (int i = 0; i < strlen(buffer); i++) {
		uart_poll_out(uart, buffer[i]);
	}
}

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
            lv_label_set_text(count_label, uart_message);  
		}
	}

}

static int uart_init(void)
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

static void MTU_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done. %d", bt_gatt_get_mtu(current_conn)-3); 
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void request_mtu_exchange(void)
{	int err;
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = MTU_exchange_cb;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	} else {
		LOG_INF("MTU exchange pending");
	}
}

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

static void request_data_len_update(void)
{
	int err;
	err = bt_conn_le_data_len_update(current_conn, BT_LE_DATA_LEN_PARAM_MAX);
		if (err) {
			LOG_ERR("LE data length update request failed: %d",  err);
		}
}
static void request_phy_update(void)
{
	int err;

	err = bt_conn_le_phy_update(current_conn, BT_CONN_LE_PHY_PARAM_2M);
		if (err) {
			LOG_ERR("Phy update request failed: %d",  err);
		}
}

#define INTERVAL_MIN  6     
#define INTERVAL_MAX  6  
static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);
static int update_connection_parameters(void)
{	
	int err;
	err = bt_conn_le_param_update(current_conn, conn_param);
		if (err) {
			LOG_ERR("Cannot update conneciton parameter (err: %d)", err);
			return err;
		}
	LOG_INF("Connection parameters update requested");
	return 0;
}
static void conn_params_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	uint32_t interval_int= interval*1.25;
	LOG_INF("Conn params updated: interval %d ms, latency %d, timeout: %d0 ms",interval_int, latency, timeout);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
	update_connection_parameters();
	request_mtu_exchange();
	request_data_len_update();
	request_phy_update();

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.le_param_updated= conn_params_updated,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
    char message[40];
    memcpy(message, data, MIN(sizeof(message) - 1, len));
    message[MIN(sizeof(message) - 1, len)] = '\0';

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);	
    LOG_INF("Received data: %s", message);
    LOG_INF("Received data length: %d", len);

    char ble_message[100];
    
    int packet_len = sprintf(ble_message, "BLE Echo: %s", message);
    bt_nus_send(NULL, ble_message, packet_len);
	lv_label_set_text(count_label, ble_message);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

int main(void)
{
	int blink_status = 0;
    int err = 0;

    configure_gpio();

	err = uart_init();
	if (err) {
		error();
	}
    
    err = bt_enable(NULL);
	if (err) {
		error();
	}

	bt_gatt_cb_register(&gatt_callbacks);

    k_sem_give(&ble_init_ok);

    err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
	}

	bt_le_adv_stop();

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}  

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (display_dev == NULL) {
        LOG_ERR("Display device not found. Aborting application.");
	}

	hello_world_label = lv_label_create(lv_scr_act());
	
	lv_label_set_text(hello_world_label, "Hello world!");
	lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

	count_label = lv_label_create(lv_scr_act());
	lv_obj_align(count_label, LV_ALIGN_BOTTOM_MID, 0, 0);

	lv_task_handler();
	display_blanking_off(display_dev);

	while (1) {		
		lv_task_handler();
		k_sleep(K_MSEC(100));
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
	}
}