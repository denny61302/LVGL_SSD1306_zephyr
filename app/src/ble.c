#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#include <dk_buttons_and_leds.h>

#include "main.h"
#include "ble.h"
#include "ui.h"

LOG_MODULE_REGISTER(ble);

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

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
    ble_send(ble_message, packet_len);
    update_text(ble_message);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

int ble_init(void)
{
    int err;
    
    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
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

    return err;
}

void ble_send(char *buffer, int len)
{
    int err;
    if (current_conn) {
        err = bt_nus_send(current_conn, buffer, len);
        if (err) {
            LOG_ERR("Failed to send data over BLE (err %d)", err);
        }
    } else {
        LOG_ERR("No connection to send data over BLE");
    }
}