/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

uint8_t uart_status = 0;
volatile char *receivedData;

//State variables for the connected client.
struct bt_conn *ConnectedClientConn;
bool connectedToClient = false;
bool EchoValues = true;

static ssize_t ble_uart_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);
static ssize_t ble_uart_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static struct bt_uuid_128 bt_uuid_uart_base =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E));

static struct bt_uuid_128 bt_uuid_uart_rx =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E));

static struct bt_uuid_128 bt_uuid_uart_tx =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E));

static bool signal_notify = false;

static void signal_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  signal_notify = value == BT_GATT_CCC_NOTIFY;
}

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

BT_GATT_SERVICE_DEFINE(uart_tx_svc,
                       BT_GATT_PRIMARY_SERVICE(&bt_uuid_uart_base.uuid),

                       /* UART tx */
                       BT_GATT_CHARACTERISTIC(&bt_uuid_uart_tx.uuid,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_uart_read, ble_uart_write, &uart_status),

                       /* tx characteristic configuration */
                       BT_GATT_CCC(signal_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* UART rx */
                       BT_GATT_CHARACTERISTIC(&bt_uuid_uart_rx.uuid,
                                              BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_uart_read, ble_uart_write, &uart_status), );

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0x30, 0x5F, 0x1A, 0xC8, 0x46, 0x0B, 0x42, 0x7E,
                  0x8C, 0xA1, 0x52, 0xB9, 0xFF, 0x1F, 0xC0, 0x5F)};

static ssize_t ble_uart_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
  printk("uart_read: %d\n", uart_status);
  const char *value = attr->user_data;
  receivedData = value;

  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(uart_status));
}

static ssize_t ble_uart_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  int lenval = len;
  printk("uart_write: %d\n", uart_status);
  uint8_t *value = attr->user_data;
  // if (offset + len > sizeof(uart_status))
  // {
  //   return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  // }

  memcpy(value + offset, buf, len);

  //Inefficient, but it creates a new character array with the data that was received.
  char DataRecv[lenval];
  memcpy(DataRecv, value + offset, lenval);

  //then, try to echo values.

  if (EchoValues == true)
  {
    //Set the notify data on the tx characteristic. In this case, it is the data that is sent to the rx characteristic.
    bt_gatt_notify(conn, &uart_tx_svc.attrs[2], DataRecv, lenval);
  }

  return len;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
  if (err)
  {
    printk("Connection failed (err 0x%02x)\n", err);
  }
  else
  {
    printk("Connected\n");
    ConnectedClientConn = conn;
    connectedToClient = true;
  }
}

//Uses the global connection handle and the attributes we set up earlier. Simple enterface to send string values over BLE NUS UART.
void SendBLEString(void *StringData, uint16_t len)
{
  //Do stuff only if we know we're connected to a client.
  if (connectedToClient == true)
  {
    //Send the String
    bt_gatt_notify(ConnectedClientConn, &uart_tx_svc.attrs[2], StringData, len);
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
  printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(void)
{
  printk("Bluetooth initialized\n");
  int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err)
  {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }
  printk("Advertising successfully started\n");
}

void main(void)
{

  int err = bt_enable(NULL);
  if (err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  bt_ready();

  bt_conn_cb_register(&conn_callbacks);

  while (1)
  {
    k_sleep(K_SECONDS(1));
  }
}