/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

#include <string.h>


LOG_MODULE_REGISTER(gatt_client_telemetry, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_LED_BLINK_INTERVAL 1000

#define BT_UUID_TELEMETRY_VAL \
    BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_WRITE_VAL \
    BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_READ_VAL \
    BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

#define BT_UUID_TELEMETRY_128 BT_UUID_DECLARE_128(BT_UUID_TELEMETRY_VAL)
#define BT_UUID_WRITE_128 BT_UUID_DECLARE_128(BT_UUID_WRITE_VAL)
#define BT_UUID_READ_128 BT_UUID_DECLARE_128(BT_UUID_READ_VAL)

// static const struct bt_uuid_128 telemetry_uuid = BT_UUID_INIT_128(BT_UUID_TELEMETRY_VAL);
// static const struct bt_uuid_128 telemetry_tx_uuid = BT_UUID_INIT_128(BT_UUID_TELEMETRY_VAL);
// static const struct bt_uuid_128 telemetry_rx_uuid = BT_UUID_INIT_128(BT_UUID_TELEMETRY_VAL);

#define BUF_RAM_SIZE 4096

static uint8_t 	buf_ram[BUF_RAM_SIZE];
static uint16_t buf_write_pos = 0;
static uint16_t buf_bytes_stored = 0;
static bool 	buf_overflow = false;

static bool notifications_enabled;

static struct k_work adv_work;
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_TELEMETRY_VAL),
};

static void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", notifications_enabled ? "enabled" : "disabled");
}

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
	const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (buf_write_pos + len > BUF_RAM_SIZE) {
		buf_overflow = true;
		printk("RAM BUFFER FULL! Dropping %d bytes\n", len);
		return -ENOMEM;
	}

	memcpy(buf_ram + buf_write_pos, buf, len);


	buf_write_pos 	 += len;
	buf_bytes_stored += len;

	const uint8_t *data = (const uint8_t *)buf;

	// printk("Value received: ");
	// for (uint16_t i = 0; i < len; i++) {
	// 	printk("%02X ", data[i]);
	// }
	// for (uint16_t i = 0; i < len; i++) {
	// 	printk("%d ", data[i]);
	// }
	printk("Received string: %.*s\n", len, data);
	printk("\n");


	printk("Received %d bytes | Buffer: %d / %d bytes used\n", len, buf_bytes_stored, BUF_RAM_SIZE);

	

	return len;
}

BT_GATT_SERVICE_DEFINE(my_telemtery_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_TELEMETRY_128),
        BT_GATT_CHARACTERISTIC(BT_UUID_WRITE_128, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP, 
			BT_GATT_PERM_WRITE, NULL, on_write, NULL),
        //BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
        //BT_GATT_CHARACTERISTIC(BT_UUID_RECEIVE_128, BT_GATT_PERM_WRITE, BT_GATT_PERM_NONE, NULL, NULL, NULL),
);

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

// Connection callback
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	}

	LOG_INF("Connected\n");

}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)\n", reason);

}

static void recycled_cb(void)
{
	printk("Connection object available from previous conn. Disconnect is complete!\n");
	advertising_start();
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u\n", addr, level);
	} else {
		LOG_INF("Security failed: %s level %u err %d\n", addr, level, err);
	}
}
struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.recycled         = recycled_cb,
	.security_changed = on_security_changed,
};

// Pairing Security Callback
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};



int main(void)
{
	int err;

	LOG_INF("Starting nrf54l15 TELEMETRY\n");

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_INF("Failed to register authorization callbacks\n");
		return -1;
	}

	bt_conn_cb_register(&connection_callbacks);

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	LOG_INF("Bluetooth initialized\n");

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		k_sleep(K_FOREVER);
	}
}
