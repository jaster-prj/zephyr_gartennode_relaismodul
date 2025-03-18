/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <canopennode.h>

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);


#if !DT_NODE_EXISTS(DT_NODELABEL(pressure_sw))
#error "Overlay for pressure_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(ultrasonic_sw))
#error "Overlay for pressure_sw output node not properly defined."
#endif
#if !DT_NODE_EXISTS(DT_NODELABEL(temperature_sw))
#error "Overlay for pressure_sw output node not properly defined."
#endif

static const struct gpio_dt_spec pressure_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pressure_sw), gpios, {0});
static const struct gpio_dt_spec ultrasonic_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(ultrasonic_sw), gpios, {0});
static const struct gpio_dt_spec temperature_sw =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(temperature_sw), gpios, {0});


void us_pressure_func(void *d0, void *d1, void *d2);
void us_ultrasonic_func(void *d0, void *d1, void *d2);
void us_temperature_func(void *d0, void *d1, void *d2);

#define US_PRESSURE_STACK 1024
#define US_ULTRASONIC_STACK 1024
#define US_TEMPERATURE_STACK 1024

static bool us_pressure_loop = false;
static bool us_ultrasonic_loop = false;
static bool us_temperature_loop = false;
K_THREAD_STACK_DEFINE(us_pressure_stack_area, US_PRESSURE_STACK);
K_THREAD_STACK_DEFINE(us_ultrasonic_stack_area, US_ULTRASONIC_STACK);
K_THREAD_STACK_DEFINE(us_temperature_stack_area, US_TEMPERATURE_STACK);
static _thread_t us_pressure_thread;
static _thread_t us_ultrasonic_thread;
static _thread_t us_temperature_thread;
static k_tid_t us_pressure_tid;
static k_tid_t us_ultrasonic_tid;
static k_tid_t us_temperature_tid;
//K_THREAD_DEFINE(us_pressure_tid, US_PRESSURE_STACK, us_pressure_func, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
//K_THREAD_DEFINE(us_ultrasonic_tid, US_ULTRASONIC_STACK, us_ultrasonic_func, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
//K_THREAD_DEFINE(us_temperature_tid, US_TEMPERATURE_STACK, us_temperature_func, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
static uint32_t pressure_value = 0;
static bool pressure_update = false;
static uint32_t ultrasonic_value = 0;
static bool ultrasonic_update = false;
static int32_t temperature_value[ODL_temperatureValue_arrayLength] = {0};
static bool temperature_update = false;

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

#define CAN_INTERFACE DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus))
#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

static struct gpio_dt_spec led_green_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(green_led), gpios, {0});
static struct gpio_dt_spec led_red_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(red_led), gpios, {0});

struct led_indicator {
	const struct device *dev;
	gpio_pin_t pin;
};

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
 * @brief Configure LED indicators pins and callbacks.
 *
 * This routine configures the GPIOs for the red and green LEDs (if
 * available).
 *
 * @param nmt CANopenNode NMT object.
 */
static void config_powerstates()
{
	int err;
	if (!gpio_is_ready_dt(&pressure_sw)) {
		LOG_ERR("The pressure switch pin GPIO port is not ready.\n");
		return; // CO_SDO_AB_GENERAL;
	}
	err = gpio_pin_configure_dt(&pressure_sw, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Configuring pressure switch GPIO pin failed: %d\n", err);
		return; // CO_SDO_AB_GENERAL;
	}
	if (!gpio_is_ready_dt(&ultrasonic_sw)) {
		LOG_ERR("The ultrasonic switch pin GPIO port is not ready.\n");
		return; // CO_SDO_AB_GENERAL;
	}
	err = gpio_pin_configure_dt(&ultrasonic_sw, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Configuring ultrasonic switch GPIO pin failed: %d\n", err);
		return; // CO_SDO_AB_GENERAL;
	}
	if (!gpio_is_ready_dt(&temperature_sw)) {
		LOG_ERR("The temperature switch pin GPIO port is not ready.\n");
		return; // CO_SDO_AB_GENERAL;
	}
	err = gpio_pin_configure_dt(&temperature_sw, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Configuring temperature switch GPIO pin failed: %d\n", err);
		return; // CO_SDO_AB_GENERAL;
	}
}

void us_pressure_func(void *d0, void *d1, void *d2) {
	int err;
	uint32_t channel = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

	if (!adc_is_ready_dt(&adc_channels[channel])) {
		LOG_INF("ADC controller device %s not ready\n", adc_channels[channel].dev->name);
		return;
	}
	err = adc_channel_setup_dt(&adc_channels[channel]);
	if (err < 0) {
		LOG_INF("Could not setup channel #%d (%d)\n", channel, err);
		return;
	}

    while(us_pressure_loop) {
		int32_t val_mv;
		(void)adc_sequence_init_dt(&adc_channels[channel], &sequence);
		err = adc_read_dt(&adc_channels[channel], &sequence);
		if (err < 0) {
			LOG_INF("Could not read (%d)\n", err);
			continue;
		}
		val_mv = (int32_t)buf;
		LOG_INF("%"PRId32, val_mv);

		if (pressure_value != (uint32_t)(buf)) {
			pressure_value = (uint32_t)(buf);
			CO_LOCK_OD();
			OD_pressureMeasurement = pressure_value;
			CO_UNLOCK_OD();
		}
		if (OD_pressureDistanceMax != 0) {
			CO_LOCK_OD();
			OD_pressureDistance = (uint32_t)(OD_pressureDistanceMax/4096*OD_pressureMeasurement);
			CO_UNLOCK_OD();
		}
        k_sleep(K_MSEC(OD_pressurePeriode));
    }
}

void us_ultrasonic_func(void *d0, void *d1, void *d2) {

    int ret;
    struct sensor_value distance;
    const struct device *dev;
    dev = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(us0)));
    if (dev == NULL) {
        LOG_ERR("Failed to get HC-SR04 binding");
        return;
    }
    while(us_ultrasonic_loop) {
        ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
        switch (ret) {
        case 0:
            ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
            if (ret) {
                LOG_ERR("sensor_channel_get for HC-SR04 failed ret %d", ret);
                break;
            }
			if (ultrasonic_value != (uint32_t)((distance.val1*1000)+(distance.val2 / 1000))) {
				ultrasonic_value = (uint32_t)((distance.val1*1000)+(distance.val2 / 1000));
				CO_LOCK_OD();
				OD_ultrasonicDistance = ultrasonic_value;
				CO_UNLOCK_OD();
			}
            break;
        case -EIO:
            LOG_WRN("%s: Could not read device", dev->name);
            break;
        default:
            LOG_ERR("Error when reading device: %s", dev->name);
            break;
        }
        k_sleep(K_MSEC(OD_ultrasonicPeriode));
    }
}

void w1_search_callback(struct w1_rom rom, void *user_data)
{
	LOG_INF("Device found; family: 0x%02x, serial: 0x%016llx", rom.family,
		w1_rom_to_uint64(&rom));
}

void us_temperature_func(void *d0, void *d1, void *d2) {
    const struct device *dev[2];
	struct w1_rom w1rom[2];
	struct sensor_value temp;
	struct sensor_value value[2];
    dev[0] = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(temp_0)));
    dev[1] = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(temp_1)));

	// const struct device *w1_dev = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(w1_0)));
	// if (!device_is_ready(w1_dev)) {
	// 	LOG_ERR("Device not ready");
	// 	return;
	// }
	// int num_devices = w1_search_rom(w1_dev, w1_search_callback, NULL);
	// LOG_INF("Number of devices found on bus: %d", num_devices);

	uint64_t rom_address[2] = {0x281e1a4398230066,0x2872f7e062200123};
    if (dev[0]==NULL || !device_is_ready(dev[0])) {
        LOG_ERR("Failed to initialize temp_0");
    }else{
		w1_uint64_to_rom(rom_address[0], &w1rom[0]);
		w1_rom_to_sensor_value(&w1rom[0], &value[0]);
	}
    if (dev[1]==NULL || !device_is_ready(dev[1])) {
        LOG_ERR("Failed to initialize temp_1");
    }else{
		w1_uint64_to_rom(rom_address[1], &w1rom[1]);
		w1_rom_to_sensor_value(&w1rom[1], &value[1]);
	}
	bool set_update = false;
    while(us_temperature_loop) {
		//set_update = false;
		for (int i=0;i<2;i++) {
			if (dev[i]==NULL) {
				continue;
			}
			sensor_attr_set(dev[i], SENSOR_CHAN_ALL, SENSOR_ATTR_W1_ROM, &value[i]);
			if (sensor_sample_fetch(dev[i]) < 0) {
				LOG_INF("Failed to fetch data from temp_%d with ROM %llx.\n", i, rom_address[i]);
			} else {
				sensor_channel_get(dev[i], SENSOR_CHAN_AMBIENT_TEMP, &temp);
				LOG_INF("Temperature temp from temp_%d at ROM %llx: %d.%06d°C\n", i, rom_address[i], temp.val1, temp.val2);
				temperature_value[i] = (int16_t)((temp.val1*1000)+(temp.val2 / 1000));
				//set_update = true;
				CO_LOCK_OD();
				OD_temperatureValue[i] = temperature_value[i];
				CO_UNLOCK_OD();
			}
		}
		// if (set_update == true) {
		// 	temperature_update = true;
		// }
        k_sleep(K_MSEC(OD_temperaturePeriode));
    }
}

// Enable pressure thread
static CO_SDO_abortCode_t odf_2102(CO_ODF_arg_t *odf_arg)
{
	uint8_t value;
	int err;

	value = *(odf_arg->data);
	
	if (!gpio_is_ready_dt(&pressure_sw)) {
		LOG_ERR("The pressure switch pin GPIO port is not ready.\n");
		return CO_SDO_AB_GENERAL;
	}
    if (value != 0) {
		err = gpio_pin_set_dt(&pressure_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring pressure switch GPIO pin failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
		us_pressure_loop = true;
		us_pressure_tid = k_thread_create(&us_pressure_thread, us_pressure_stack_area,
                                 K_THREAD_STACK_SIZEOF(us_pressure_stack_area),
                                 us_pressure_func,
                                 NULL, NULL, NULL,
                                 K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	} else {
		us_pressure_loop = false;
		err = gpio_pin_set_dt(&pressure_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting pressure switch GPIO pin level failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
    }
    return CO_SDO_AB_NONE;
}

// Enable ultrasonic thread
static CO_SDO_abortCode_t odf_2107(CO_ODF_arg_t *odf_arg)
{
	uint8_t value;
	int err;

	value = *(odf_arg->data);
	
	if (!gpio_is_ready_dt(&ultrasonic_sw)) {
		LOG_ERR("The ultrasonic switch pin GPIO port is not ready.\n");
		return CO_SDO_AB_GENERAL;
	}
    if (value != 0) {
		err = gpio_pin_set_dt(&ultrasonic_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring ultrasonic switch GPIO pin failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
		us_ultrasonic_loop = true;
		us_ultrasonic_tid = k_thread_create(&us_ultrasonic_thread, us_ultrasonic_stack_area,
                                 K_THREAD_STACK_SIZEOF(us_ultrasonic_stack_area),
                                 us_ultrasonic_func,
                                 NULL, NULL, NULL,
                                 K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	} else {
		us_ultrasonic_loop = false;
		err = gpio_pin_set_dt(&ultrasonic_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting ultrasonic switch GPIO pin level failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
    }
    return CO_SDO_AB_NONE;
}

// Enable temperature thread
static CO_SDO_abortCode_t odf_2110(CO_ODF_arg_t *odf_arg)
{
	uint8_t value;
	int err;

	value = *(odf_arg->data);
	
	if (!gpio_is_ready_dt(&temperature_sw)) {
		LOG_ERR("The temperature switch pin GPIO port is not ready.\n");
		return CO_SDO_AB_GENERAL;
	}
    if (value != 0) {
		err = gpio_pin_set_dt(&temperature_sw, 1);
		if (err != 0) {
			LOG_ERR("Configuring temperature switch GPIO pin failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
		us_temperature_loop = true;
		us_temperature_tid = k_thread_create(&us_temperature_thread, us_temperature_stack_area,
                                 K_THREAD_STACK_SIZEOF(us_temperature_stack_area),
                                 us_temperature_func,
                                 NULL, NULL, NULL,
                                 K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	} else {
		us_temperature_loop = false;
		err = gpio_pin_set_dt(&temperature_sw, 0);
		if (err != 0) {
			LOG_ERR("Setting temperature switch GPIO pin level failed: %d\n", err);
    		return CO_SDO_AB_GENERAL;
		}
    }
    return CO_SDO_AB_NONE;
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

		OD_pressureEnable = 0;
		OD_ultrasonicEnable = 0;
		OD_temperatureEnable = 0;

		LOG_INF("CANopen stack initialize");
		err = CO_init(&can, CONFIG_CANOPEN_NODE_ID, CAN_BITRATE);
		if (err != CO_ERROR_NO) {
			LOG_ERR("CO_init failed (err = %d)", err);
			return 0;
		}
		LOG_INF("CANopen stack initialized");

#ifdef CONFIG_CANOPENNODE_STORAGE
		canopen_storage_attach(CO->SDO[0], CO->em);
#endif /* CONFIG_CANOPENNODE_STORAGE */

		config_leds(CO->NMT);
		config_powerstates();
		CO_OD_configure(CO->SDO[0], OD_2102_pressureEnable,
				odf_2102, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2107_ultrasonicEnable,
				odf_2107, NULL, 0U, 0U);
		CO_OD_configure(CO->SDO[0], OD_2110_temperatureEnable,
				odf_2110, NULL, 0U, 0U);

		if (IS_ENABLED(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)) {
			canopen_program_download_attach(CO->NMT, CO->SDO[0],
							CO->em);
		}

		CO_CANsetNormalMode(CO->CANmodule[0]);

		while (true) {
			timeout = 1U; /* default timeout in milliseconds */
			timestamp = k_uptime_get();
			reset = CO_process(CO, (uint16_t)elapsed, &timeout);

			if (reset != CO_RESET_NOT) {
				break;
			}

			if (timeout > 0) {
#ifdef CONFIG_CANOPENNODE_STORAGE
				ret = canopen_storage_save(
					CANOPEN_STORAGE_EEPROM);
				if (ret) {
					LOG_ERR("failed to save EEPROM");
				}
				// if pressure_update {
				// 	CO_LOCK_OD();
				// 	OD_pressureMeasurement = pressure_value;
				// 	CO_UNLOCK_OD();
				// 	pressure_update = false;
				// }
				// if ultrasonic_update {
				// 	CO_LOCK_OD();
				// 	OD_ultrasonicDistance = ultrasonic_value;
				// 	CO_UNLOCK_OD();
				// 	ultrasonic_update = false;
				// }
				// if temperature_update {
				// 	for (i=0;i<ODL_temperatureValue_arrayLength;i++) {
				// 		CO_LOCK_OD();
				// 		OD_temperatureValue[i] = temperature_value[i];
				// 		CO_UNLOCK_OD();
				// 	}
				// 	temperature_update = false;
				// }
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
				elapsed = 0U;
			}
		}

        us_pressure_loop = false;
        us_ultrasonic_loop = false;
        us_temperature_loop = false;

		if (reset == CO_RESET_COMM) {
			LOG_INF("Resetting communication");
		}
	}

	LOG_INF("Resetting device");

	CO_delete(&can);
	sys_reboot(SYS_REBOOT_COLD);
}
