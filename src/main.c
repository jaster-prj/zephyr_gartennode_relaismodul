/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BOARD_NATIVE_SIM)
#define _POSIX_C_SOURCE 200809L
#endif

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <canopennode.h>
#include "relay.h"
#include "update_control.h"
#include "update_diagnostics.h"
#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <time.h>
#include <zephyr/drivers/can.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
K_SEM_DEFINE(native_sim_can_rx_wakeup, 0, 1);

static int64_t native_sim_monotonic_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void native_sim_can_bus_activity(const struct device *dev,
					struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(frame);
	ARG_UNUSED(user_data);
	k_sem_give(&native_sim_can_rx_wakeup);
}
#endif

#define LOG_LEVEL CONFIG_CANOPENNODE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

static struct gpio_dt_spec led_green_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(green_led), gpios, {0});
static struct gpio_dt_spec led_red_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(red_led), gpios, {0});

/**
 * @brief Callback for setting LED indicator state.
 *
 * @param value true if the LED indicator shall be turned on, false otherwise.
 * @param arg argument that was passed when LEDs were initialized.
 */
static void led_callback(bool value, void *arg)
{
	struct gpio_dt_spec *led_gpio = arg;

	if (!led_gpio || !led_gpio->port) {
		return;
	}

	gpio_pin_set_dt(led_gpio, value);
}

/**
 * @brief Configure LED indicators pins and callbacks.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @param nmt CANopenNode NMT object.
 */
static void config_leds(CO_NMT_t *nmt)
{
	int err;

	if (!led_green_gpio.port) {
		LOG_INF("Green LED not available");
	} else if (!gpio_is_ready_dt(&led_green_gpio)) {
		LOG_ERR("Green LED device not ready");
		led_green_gpio.port = NULL;
	} else {
		err = gpio_pin_configure_dt(&led_green_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("failed to configure Green LED gpio: %d", err);
			led_green_gpio.port = NULL;
		}
	}

	if (!led_red_gpio.port) {
		LOG_INF("Red LED not available");
	} else if (!gpio_is_ready_dt(&led_red_gpio)) {
		LOG_ERR("Red LED device not ready");
		led_red_gpio.port = NULL;
	} else {
		err = gpio_pin_configure_dt(&led_red_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("failed to configure Red LED gpio: %d", err);
			led_red_gpio.port = NULL;
		}
	}

	canopen_leds_init(nmt,
			  led_callback, &led_green_gpio,
			  led_callback, &led_red_gpio);
}

/**
 * @brief Main application entry point.
 *
 * The main application thread is responsible for initializing the
 * CANopen stack and doing the non real-time processing.
 */
int main(void)
{
	CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
	CO_ReturnError_t err;
	struct canopen_context can;
	uint16_t timeout;
	uint32_t elapsed;
	int64_t timestamp;
#ifdef CONFIG_CANOPENNODE_STORAGE
	int ret;
#endif /* CONFIG_CANOPENNODE_STORAGE */

	can.dev = CAN_INTERFACE;
	if (!device_is_ready(can.dev)) {
		LOG_ERR("CAN interface not ready");
		return 0;
	}

#ifdef CONFIG_CANOPENNODE_STORAGE
	ret = settings_subsys_init();
	if (ret) {
		LOG_ERR("failed to initialize settings subsystem (err = %d)",
			ret);
		return 0;
	}

	ret = settings_load();
	if (ret) {
		LOG_ERR("failed to load settings (err = %d)", ret);
		return 0;
	}
#endif /* CONFIG_CANOPENNODE_STORAGE */

	OD_powerOnCounter++;

	while (reset != CO_RESET_APP) {
		elapsed =  0U; /* milliseconds */

#if defined(CONFIG_BOARD_NATIVE_SIM)
		/* Start node 11 in Operational and periodically report relay states. */
		OD_NMTStartup = 8U;
		OD_producerHeartbeatTime = 1000U;
		OD_TPDOCommunicationParameter[0].eventTimer = 250U;
#endif

		err = CO_init(&can, CONFIG_CANOPEN_NODE_ID, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			return 0;
		}

		LOG_INF("CANopen stack initialized: %d", CONFIG_CANOPEN_NODE_ID);

#ifdef CONFIG_CANOPENNODE_STORAGE
		canopen_storage_attach(CO->SDO[0], CO->em);
#endif /* CONFIG_CANOPENNODE_STORAGE */

		config_leds(CO->NMT);
		relay_init();

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0],
							CO->em);
			relay_update_control_attach(CO->SDO[0]);
#if defined(CONFIG_RELAY_UPDATE_DIAGNOSTICS)
			relay_update_diagnostics_attach(CO->SDO[0]);
#endif
		}

#if defined(CONFIG_BOARD_NATIVE_SIM)
		const struct can_filter native_sim_wakeup_filter = {
			.id = 0U,
			.mask = 0U,
		};
		int native_sim_wakeup_filter_id =
			can_add_rx_filter(can.dev, native_sim_can_bus_activity, NULL,
					  &native_sim_wakeup_filter);
		if (native_sim_wakeup_filter_id < 0) {
			LOG_ERR("Failed to add native-sim CAN activity filter: %d",
				native_sim_wakeup_filter_id);
		}
#endif

		CO_CANsetNormalMode(CO->CANmodule[0]);

		while (true) {
			timeout = 1U; /* default timeout in milliseconds */
#if defined(CONFIG_BOARD_NATIVE_SIM)
			timestamp = native_sim_monotonic_ms();
#else
			timestamp = k_uptime_get();
#endif
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);

			if (reset != CO_RESET_NOT) {
				break;
			}
#if defined(CONFIG_BOARD_NATIVE_SIM)
			/*
			 * The native simulator has no deterministic SYNC worker. Process
			 * the asynchronous relay RPDO/TPDO in the application loop, just
			 * as the regular CANopen worker does on the hardware target.
			 */
			CO_LOCK_OD();
			CO_process_RPDO(CO, false);
			CO_process_TPDO(CO, false, elapsed * 1000U);
			CO_UNLOCK_OD();
#endif

#if defined(CONFIG_BOARD_NATIVE_SIM)
			/*
			 * CANopenNode timers use elapsed time, but native_sim's kernel
			 * uptime can remain static while waiting on UDP CAN. Use the
			 * monotonic host clock and wake promptly on incoming bus traffic.
			 */
			relay_process();
			(void)k_sem_take(&native_sim_can_rx_wakeup, K_MSEC(MAX(timeout, 10U)));
			elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
			if (elapsed == 0U) {
				elapsed = 1U;
			}
#else
			if (timeout > 0) {
				relay_process();

#ifdef CONFIG_CANOPENNODE_STORAGE
				ret = canopen_storage_save(
					CANOPEN_STORAGE_EEPROM);
				if (ret) {
					LOG_ERR("failed to save EEPROM");
				}
#endif /* CONFIG_CANOPENNODE_STORAGE */
				/*
				 * Try to sleep for as long as the
				 * stack requested and calculate the
				 * exact time elapsed.
				 */
				k_sleep(K_MSEC(timeout));
				elapsed = (uint32_t)k_uptime_delta(&timestamp);
			} else {
				/*
				 * Do not sleep, more processing to be
				 * done by the stack.
				 */
#if defined(CONFIG_BOARD_NATIVE_SIM)
				(void)k_sem_take(&native_sim_can_rx_wakeup, K_MSEC(10));
				elapsed = (uint32_t)(native_sim_monotonic_ms() - timestamp);
				if (elapsed == 0U) {
					elapsed = 1U;
				}
#else
				elapsed = 0U;
#endif
			}
#endif
		}

		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	LOG_INF("Resetting device");

	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}
