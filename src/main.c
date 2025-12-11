/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#ifdef BT_UUID_NUS_VAL
#undef BT_UUID_NUS_VAL
#endif

#ifdef BT_UUID_NUS_TX_VAL
#undef BT_UUID_NUS_TX_VAL
#endif

// Define custom UUIDs

#define BT_UUID_NUS_VAL \
    BT_UUID_128_ENCODE(0x0000AAAA, 0x1212, 0xEFDE, 0x1523, 0x785FEF13D123)

#define BT_UUID_NUS_TX_VAL 0x1111

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/counter.h>

#define BREADBOARD 0 // Set 1 for breadboard, 0 for PCB

// Get RTC Device
#define RTC_NODE DT_NODELABEL(rtc0)
static const struct device *rtc_dev = DEVICE_DT_GET(RTC_NODE);

// MUX Pins
#define A0_PIN 20
#define A1_PIN 3
#define A2_PIN 10
#define A3_PIN 5
#define EN_PIN 8

// SPI Chip Select Pins
#define DAC_CS_PIN 6
#define ADC_CS_PIN 4

// Get GPIO device
#define GPIO_NODE DT_NODELABEL(gpio0)
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);

// Get SPI Device
#define SPI_NODE DT_NODELABEL(spi0)  // Changed from spi1 to spi0
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

// SPI configuration for DAC (Mode 2: CPOL=1, CPHA=0)
static struct spi_config dac_spi_cfg = {
    .frequency = 250000,  // 250 kHz to match Segger SDK
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_MODE_CPOL,  // Mode 2
};

// SPI configuration for ADC (Mode 3: CPOL=1, CPHA=1) 
static struct spi_config adc_spi_cfg = {
    .frequency = 250000,  // 250 kHz
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA,  // Mode 3
};

#if !BREADBOARD
static uint8_t m_tx_dac_use_internal_ref[] = {0b00111000, 0b00000000, 0b00000001};
static uint8_t m_tx_dac_set_500mV[] = {0b00011000, 0b00010011, 0b01100101};
#endif

#if BREADBOARD
static uint8_t m_tx_dac_set_500mV_breadboard[] = {0b00000000, 0b00011001, 0b10011010};
#endif

static const uint8_t m_length_dac = 3;

#define MUX_SETTLE_DELAY K_MSEC(125)  // 125ms/channel x 16 channels = 2s total

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED 9
#define RUN_LED_BLINK_INTERVAL 1000

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

// static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
// static struct k_work_delayable uart_work;

// struct uart_data_t {
// 	void *fifo_reserved;
// 	uint8_t data[UART_BUF_SIZE];
// 	uint16_t len;
// };

// static K_FIFO_DEFINE(fifo_uart_tx_data);
// static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

// static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
// {
// 	ARG_UNUSED(dev);

// 	static size_t aborted_len;
// 	struct uart_data_t *buf;
// 	static uint8_t *aborted_buf;
// 	static bool disable_req;

// 	switch (evt->type) {
// 	case UART_TX_DONE:
// 		LOG_DBG("UART_TX_DONE");
// 		if ((evt->data.tx.len == 0) ||
// 		    (!evt->data.tx.buf)) {
// 			return;
// 		}

// 		if (aborted_buf) {
// 			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
// 					   data[0]);
// 			aborted_buf = NULL;
// 			aborted_len = 0;
// 		} else {
// 			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
// 					   data[0]);
// 		}

// 		k_free(buf);

// 		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
// 		if (!buf) {
// 			return;
// 		}

// 		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
// 			LOG_WRN("Failed to send data over UART");
// 		}

// 		break;

// 	case UART_RX_RDY:
// 		LOG_DBG("UART_RX_RDY");
// 		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
// 		buf->len += evt->data.rx.len;

// 		if (disable_req) {
// 			return;
// 		}

// 		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
// 		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
// 			disable_req = true;
// 			uart_rx_disable(uart);
// 		}

// 		break;

// 	case UART_RX_DISABLED:
// 		LOG_DBG("UART_RX_DISABLED");
// 		disable_req = false;

// 		buf = k_malloc(sizeof(*buf));
// 		if (buf) {
// 			buf->len = 0;
// 		} else {
// 			LOG_WRN("Not able to allocate UART receive buffer");
// 			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
// 			return;
// 		}

// 		uart_rx_enable(uart, buf->data, sizeof(buf->data),
// 			       UART_WAIT_FOR_RX);

// 		break;

// 	case UART_RX_BUF_REQUEST:
// 		LOG_DBG("UART_RX_BUF_REQUEST");
// 		buf = k_malloc(sizeof(*buf));
// 		if (buf) {
// 			buf->len = 0;
// 			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
// 		} else {
// 			LOG_WRN("Not able to allocate UART receive buffer");
// 		}

// 		break;

// 	case UART_RX_BUF_RELEASED:
// 		LOG_DBG("UART_RX_BUF_RELEASED");
// 		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
// 				   data[0]);

// 		if (buf->len > 0) {
// 			k_fifo_put(&fifo_uart_rx_data, buf);
// 		} else {
// 			k_free(buf);
// 		}

// 		break;

// 	case UART_TX_ABORTED:
// 		LOG_DBG("UART_TX_ABORTED");
// 		if (!aborted_buf) {
// 			aborted_buf = (uint8_t *)evt->data.tx.buf;
// 		}

// 		aborted_len += evt->data.tx.len;
// 		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
// 				   data);

// 		uart_tx(uart, &buf->data[aborted_len],
// 			buf->len - aborted_len, SYS_FOREVER_MS);

// 		break;

// 	default:
// 		break;
// 	}
// }

// static void uart_work_handler(struct k_work *item)
// {
// 	struct uart_data_t *buf;

// 	buf = k_malloc(sizeof(*buf));
// 	if (buf) {
// 		buf->len = 0;
// 	} else {
// 		LOG_WRN("Not able to allocate UART receive buffer");
// 		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
// 		return;
// 	}

// 	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
// }

// static bool uart_test_async_api(const struct device *dev)
// {
// 	const struct uart_driver_api *api =
// 			(const struct uart_driver_api *)dev->api;

// 	return (api->callback_set != NULL);
// }

// static int uart_init(void)
// {
// 	int err;
// 	int pos;
// 	struct uart_data_t *rx;
// 	struct uart_data_t *tx;

// 	if (!device_is_ready(uart)) {
// 		return -ENODEV;
// 	}

// 	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
// 		err = usb_enable(NULL);
// 		if (err && (err != -EALREADY)) {
// 			LOG_ERR("Failed to enable USB");
// 			return err;
// 		}
// 	}

// 	rx = k_malloc(sizeof(*rx));
// 	if (rx) {
// 		rx->len = 0;
// 	} else {
// 		return -ENOMEM;
// 	}

// 	k_work_init_delayable(&uart_work, uart_work_handler);


// 	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
// 		/* Implement API adapter */
// 		uart_async_adapter_init(async_adapter, uart);
// 		uart = async_adapter;
// 	}

// 	err = uart_callback_set(uart, uart_cb, NULL);
// 	if (err) {
// 		k_free(rx);
// 		LOG_ERR("Cannot initialize UART callback");
// 		return err;
// 	}

// 	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
// 		LOG_INF("Wait for DTR");
// 		while (true) {
// 			uint32_t dtr = 0;

// 			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
// 			if (dtr) {
// 				break;
// 			}
// 			/* Give CPU resources to low priority threads. */
// 			k_sleep(K_MSEC(100));
// 		}
// 		LOG_INF("DTR set");
// 		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
// 		if (err) {
// 			LOG_WRN("Failed to set DCD, ret code %d", err);
// 		}
// 		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
// 		if (err) {
// 			LOG_WRN("Failed to set DSR, ret code %d", err);
// 		}
// 	}

// 	tx = k_malloc(sizeof(*tx));

// 	if (tx) {
// 		pos = snprintf(tx->data, sizeof(tx->data),
// 			       "Starting Nordic UART service sample\r\n");

// 		if ((pos < 0) || (pos >= sizeof(tx->data))) {
// 			k_free(rx);
// 			k_free(tx);
// 			LOG_ERR("snprintf returned %d", pos);
// 			return -ENOMEM;
// 		}

// 		tx->len = pos;
// 	} else {
// 		k_free(rx);
// 		return -ENOMEM;
// 	}

// 	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
// 	if (err) {
// 		k_free(rx);
// 		k_free(tx);
// 		LOG_ERR("Cannot display welcome message (err: %d)", err);
// 		return err;
// 	}

// 	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
// 	if (err) {
// 		LOG_ERR("Cannot enable uart reception (err: %d)", err);
// 		/* Free the rx buffer only because the tx buffer will be handled in the callback */
// 		k_free(rx);
// 	}

// 	return err;
// }

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
    
	gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		gpio_pin_set(gpio_dev, RUN_STATUS_LED, 0);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

// static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
// 			  uint16_t len)
// {
// 	int err;
// 	char addr[BT_ADDR_LE_STR_LEN] = {0};

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

// 	LOG_INF("Received data from: %s", addr);

// 	for (uint16_t pos = 0; pos != len;) {
// 		struct uart_data_t *tx = k_malloc(sizeof(*tx));

// 		if (!tx) {
// 			LOG_WRN("Not able to allocate UART send data buffer");
// 			return;
// 		}

// 		/* Keep the last byte of TX buffer for potential LF char. */
// 		size_t tx_data_size = sizeof(tx->data) - 1;

// 		if ((len - pos) > tx_data_size) {
// 			tx->len = tx_data_size;
// 		} else {
// 			tx->len = (len - pos);
// 		}

// 		memcpy(tx->data, &data[pos], tx->len);

// 		pos += tx->len;

// 		/* Append the LF character when the CR character triggered
// 		 * transmission from the peer.
// 		 */
// 		if ((pos == len) && (data[len - 1] == '\r')) {
// 			tx->data[tx->len] = '\n';
// 			tx->len++;
// 		}

// 		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
// 		if (err) {
// 			k_fifo_put(&fifo_uart_tx_data, tx);
// 		}
// 	}
// }

// static struct bt_nus_cb nus_cb = {
// 	.received = bt_receive_cb,
// };

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}


static int gpio_init(void)
{
    int err;
    
    // Check if GPIO device is ready
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    // Configure GPIO pins as outputs
    err = gpio_pin_configure(gpio_dev, A0_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure A0 pin: %d", err);
        return err;
    }
    
    err = gpio_pin_configure(gpio_dev, A1_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure A1 pin: %d", err);
        return err;
    }
    
    err = gpio_pin_configure(gpio_dev, A2_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure A2 pin: %d", err);
        return err;
    }
    
    err = gpio_pin_configure(gpio_dev, A3_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure A3 pin: %d", err);
        return err;
    }
    
    err = gpio_pin_configure(gpio_dev, EN_PIN, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("Failed to configure EN pin: %d", err);
        return err;
    }

	err = gpio_pin_configure(gpio_dev, DAC_CS_PIN, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("Failed to configure DAC CS pin: %d", err);
        return err;
    }

	err = gpio_pin_configure(gpio_dev, ADC_CS_PIN, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("Failed to configure ADC CS pin: %d", err);
        return err;
    }

	err = gpio_pin_configure(gpio_dev, RUN_STATUS_LED, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure LED pin: %d", err);
        return err;
    }
    
    LOG_INF("GPIO initialized successfully!!");
    return 0;
}

static int dac_write(uint8_t *data, size_t len)
{
    struct spi_buf tx_buf = {
        .buf = data,
        .len = len
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    gpio_pin_set(gpio_dev, DAC_CS_PIN, 0);
	k_busy_wait(1);
    int err = spi_write(spi_dev, &dac_spi_cfg, &tx);
	k_busy_wait(1);
	gpio_pin_set(gpio_dev, DAC_CS_PIN, 1);

	if (err) {
        LOG_ERR("DAC SPI write failed: %d", err);
    }
	return err;

}

static void set_mux_channel(uint8_t channel)
{
    gpio_pin_set(gpio_dev, A0_PIN, (channel >> 0) & 0x01);
    gpio_pin_set(gpio_dev, A1_PIN, (channel >> 1) & 0x01);
    gpio_pin_set(gpio_dev, A2_PIN, (channel >> 2) & 0x01);
    gpio_pin_set(gpio_dev, A3_PIN, (channel >> 3) & 0x01);
}

// External ADC Functions

static int ad7789_reset()
{
    uint8_t reset_cmd[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	struct spi_buf tx_buf = {
        .buf = reset_cmd,
        .len = 5
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    
    gpio_pin_set(gpio_dev, ADC_CS_PIN, 0);
	k_busy_wait(1);
    int err = spi_write(spi_dev, &adc_spi_cfg, &tx);
	k_busy_wait(1);
	gpio_pin_set(gpio_dev, ADC_CS_PIN, 1);

	if (err) {
        LOG_ERR("AD7789 reset failed: %d", err);
    }
	return err;
}

uint8_t ad7789_read_status()
{
    uint8_t tx_data[2] = {0x08, 0x00};
    uint8_t rx_data[2] = {0};

	struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 2
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

	struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 2
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };
    
	gpio_pin_set(gpio_dev, ADC_CS_PIN, 0);
	k_busy_wait(1);
    int err = spi_transceive(spi_dev, &adc_spi_cfg, &tx, &rx);
	k_busy_wait(1);
	gpio_pin_set(gpio_dev, ADC_CS_PIN, 1);
	k_busy_wait(10);

	if (err) {
        LOG_ERR("AD7789 Read Status Error: %d", err);
    }
    
    return rx_data[1];
}

bool ad7789_wait_for_data(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    uint8_t status;
    
    do {
        status = ad7789_read_status();
        
        // Check if RDY bit (bit 7) is cleared
        if ((status & 0x80) == 0) {
            // Data is ready
            if (status & 0x40) {
                LOG_WRN("Data ready but ERR flag set! Status: 0x%02X", status);
            }
            return true;
        }
        
        k_busy_wait(10);
        elapsed += 10;
        
        if (elapsed >= timeout_ms) {
            LOG_WRN("Timeout waiting for data. Status: 0x%02X", status);
            return false;
        }
    } while (1);
}

uint32_t ad7789_read_data()
{
    // Wait for data to be ready
    if (!ad7789_wait_for_data(200)) {
        LOG_ERR("No data ready!");
        return 0xFFFFFFFF;
    }
    
    // Read data register in one transaction
    uint8_t tx_data[4] = {0x38, 0x00, 0x00, 0x00}; // Read data register
    uint8_t rx_data[4] = {0};

	struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 4
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

	struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 4
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };

	gpio_pin_set(gpio_dev, ADC_CS_PIN, 0);
	k_busy_wait(1);
    int err = spi_transceive(spi_dev, &adc_spi_cfg, &tx, &rx);
	k_busy_wait(1);
	gpio_pin_set(gpio_dev, ADC_CS_PIN, 1);
	k_busy_wait(1);

	if (err) {
        LOG_ERR("AD7789 Read Status Error: %d", err);
    }
    
    // Combine bytes
    uint32_t adc_value = ((uint32_t)rx_data[1] << 16) | 
                         ((uint32_t)rx_data[2] << 8) | 
                         rx_data[3];
    
    return adc_value;
}

// Get timestamp in milliseconds since startup
uint32_t get_timestamp_ms(void)
{
    uint32_t ticks;
    uint32_t freq;
    
    if (counter_get_value(rtc_dev, &ticks) != 0) {
        LOG_ERR("Failed to read RTC counter");
        return 0;
    }
    
    freq = counter_get_frequency(rtc_dev);
    
    // Convert ticks to milliseconds
    // RTC typically runs at 32.768 kHz, so freq = 32768
    return ((uint32_t)ticks * 1000UL) / freq;
}

int main(void)
{
	int err = 0;

	configure_gpio();
	
	err = gpio_init();
    if (err) {
        LOG_ERR("Failed to initialize GPIO: %d", err);
        error();
    }

	k_msleep(5);

    // Initialize RTC
    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("RTC device not ready");
        error();
    }

    // Start the counter
    counter_start(rtc_dev);
    LOG_INF("RTC initialized and started");

	k_msleep(5);

	if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
		error();
    }

	k_msleep(5);
	// Initialize DAC with internal reference to 0.5V
	#if !BREADBOARD
	dac_write(m_tx_dac_use_internal_ref, m_length_dac);
	k_msleep(5);
	dac_write(m_tx_dac_set_500mV, m_length_dac);
	#endif

	#if BREADBOARD
	dac_write(m_tx_dac_set_500mV_breadboard, m_length_dac);
	#endif
	LOG_INF("DAC initialized successfully");


	ad7789_reset();
	LOG_INF("ADC Reset Successful");

	// err = uart_init();
	// if (err) {
	// 	error();
	// }

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(NULL);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);

	 // Wait for BLE to initialize
    k_sem_take(&ble_init_ok, K_FOREVER);

	// Create packet data string
	char packet_data[9];
	packet_data[0] = 0x05;

	uint32_t timestamp;
	int channel;
	uint32_t sensor_data;

	for (;;) {
		
		// Loop through all 16 channels
		for (channel = 0; channel < 16; channel++) {
			set_mux_channel(channel);
			k_sleep(MUX_SETTLE_DELAY);
			sensor_data = ad7789_read_data();

			// Add timestamp (4 bytes)
			timestamp = get_timestamp_ms();
			packet_data[1] = (unsigned char)((timestamp >> 24) & 0xFF);
			packet_data[2] = (unsigned char)((timestamp >> 16) & 0xFF);
			packet_data[3] = (unsigned char)((timestamp >> 8) & 0xFF);
			packet_data[4] = (unsigned char)(timestamp & 0xFF);

			packet_data[5] = (uint8_t)channel; // Channel Number

			// Add sensor data as 3 bytes
			packet_data[6] = (unsigned char)((sensor_data >> 16) & 0xFF);
    		packet_data[7] = (unsigned char)((sensor_data >> 8) & 0xFF);
    		packet_data[8] = (unsigned char)(sensor_data & 0xFF);

			// If connected, send data over BLE
			if (current_conn) {

				gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);
				
				if (bt_nus_send(NULL, packet_data, sizeof(packet_data))) {
					LOG_WRN("Failed to send sensor data");
				} else {
					LOG_INF("Sent Data: %u, %d, %u", timestamp, channel, sensor_data);
				}

				gpio_pin_set(gpio_dev, RUN_STATUS_LED, 0);
			}
		}
		
	}
}

// void ble_write_thread(void)
// {
// 	/* Don't go any further until BLE is initialized */
// 	k_sem_take(&ble_init_ok, K_FOREVER);
// 	struct uart_data_t nus_data = {
// 		.len = 0,
// 	};

// 	for (;;) {
// 		/* Wait indefinitely for data to be sent over bluetooth */
// 		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
// 						     K_FOREVER);

// 		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
// 		int loc = 0;

// 		while (plen > 0) {
// 			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
// 			nus_data.len += plen;
// 			loc += plen;

// 			if (nus_data.len >= sizeof(nus_data.data) ||
// 			   (nus_data.data[nus_data.len - 1] == '\n') ||
// 			   (nus_data.data[nus_data.len - 1] == '\r')) {
// 				if (bt_nus_send(NULL, nus_data.data, nus_data.len)) {
// 					LOG_WRN("Failed to send data over BLE connection");
// 				}
// 				nus_data.len = 0;
// 			}

// 			plen = MIN(sizeof(nus_data.data), buf->len - loc);
// 		}

// 		k_free(buf);
// 	}
// }

// K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
// 		NULL, PRIORITY, 0, 0);


// Add this near the top with other defines

// Add this function before main()
void sensor_thread(void)
{
    /* Wait for BLE to initialize */
    k_sem_take(&ble_init_ok, K_FOREVER);
        
    for (;;) {

		// Loop through all 16 channels
		uint32_t sensor_data;
		for (int channel = 0; channel < 16; channel++) {
			set_mux_channel(channel);
			k_sleep(MUX_SETTLE_DELAY);
			sensor_data = ad7789_read_data();

			// Create packet data string
			char packet_data[9];
			packet_data[0] = 0x05;

			// Add timestamp (4 bytes)
			uint32_t timestamp = get_timestamp_ms();
			packet_data[1] = (unsigned char)(timestamp & 0xFF);
			packet_data[2] = (unsigned char)((timestamp >> 8) & 0xFF);
			packet_data[3] = (unsigned char)((timestamp >> 16) & 0xFF);
			packet_data[4] = (unsigned char)((timestamp >> 24) & 0xFF);

			packet_data[5] = (uint8_t)channel; // Channel Number

			// Add sensor data as 3 bytes
			packet_data[6] = (unsigned char)(sensor_data & 0xFF);
    		packet_data[7] = (unsigned char)((sensor_data >> 8) & 0xFF);
			packet_data[8] = (unsigned char)((sensor_data >> 16) & 0xFF);

			// If connected, send data over BLE
			if (current_conn) {

				gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);
				
				if (bt_nus_send(NULL, packet_data, sizeof(packet_data))) {
					LOG_WRN("Failed to send sensor data");
				} else {
					LOG_INF("Sent Data: %u, %d, %u", timestamp, channel, sensor_data);
				}

				gpio_pin_set(gpio_dev, RUN_STATUS_LED, 0);
			}
		}

		// // Send timestamp first
		// char timestamp_data[20];
		// sprintf(timestamp_data, "T:%llu", (unsigned long long)timestamp);
		// if (current_conn) {

		// 	gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);
			
		// 	if (bt_nus_send(NULL, timestamp_data, sizeof(timestamp_data))) {
		// 		LOG_WRN("Failed to send sensor data");
		// 	} else {
		// 		LOG_INF("Sent Data: %s", timestamp_data);
		// 	}

		// 	gpio_pin_set(gpio_dev, RUN_STATUS_LED, 0);
		// 	// Clear packet data for next packet
			
		// }

		// // Send in 2 packets of 8 channels each
		// char packet_data[20];
		// for (int packet = 0; packet < 2; packet++) {
		// 	packet_data[0] = '\0';
		// 	// add the raw bytes to the packet data
		// 	memcpy(packet_data, &sensor_data[packet*8], 16);

		// 	// If connected, send data over BLE
		// 	if (current_conn) {

		// 		gpio_pin_set(gpio_dev, RUN_STATUS_LED, 1);
				
		// 		if (bt_nus_send(NULL, packet_data, sizeof(packet_data))) {
		// 			LOG_WRN("Failed to send sensor data");
		// 		} else {
		// 			LOG_INF("Sent Data: %s", packet_data);
		// 		}

		// 		gpio_pin_set(gpio_dev, RUN_STATUS_LED, 0);
		// 		// Clear packet data for next packet
		// 		packet_data[0] = '\0';
				
		// 	}
		// }

    }
}

// Add this at the end of the file, after the ble_write_thread definition
// K_THREAD_DEFINE(sensor_thread_id, STACKSIZE, sensor_thread, NULL, NULL,
//                 NULL, PRIORITY, 0, 0);