#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <zephyr/drivers/gpio/gpio_emul.h>
#endif
#include <zephyr/devicetree.h>
#include <canopennode.h>
#include "native_sim_gpio.h"

#define LOG_LEVEL CONFIG_CANOPENNODE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(relay);

#define RELAY_COUNT 6U

static const int output_lookup_table[RELAY_COUNT] = {2, 1, 0, 5, 4, 3};
static uint8_t output[RELAY_COUNT];

#if defined(CONFIG_BOARD_NATIVE_SIM)
static const struct gpio_dt_spec relay_gpio[RELAY_COUNT] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay1_sw), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay2_sw), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay3_sw), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay4_sw), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay5_sw), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(relay6_sw), gpios),
};

static void publish_relay_gpio(void)
{
	uint8_t states[RELAY_COUNT];

	for (size_t i = 0; i < ARRAY_SIZE(relay_gpio); i++) {
		states[i] = (uint8_t)(gpio_emul_output_get_dt(&relay_gpio[i]) > 0);
	}
	native_sim_gpio_publish(46006U, states, ARRAY_SIZE(states));
}
#else
static const struct device *pcf857x;
#endif

void relay_init(void)
{
#if defined(CONFIG_BOARD_NATIVE_SIM)
	for (size_t i = 0; i < ARRAY_SIZE(relay_gpio); i++) {
		if (!gpio_is_ready_dt(&relay_gpio[i])) {
			LOG_ERR("Relay output %u is not ready", (unsigned int)i + 1U);
			continue;
		}

		int ret = gpio_pin_configure_dt(&relay_gpio[i], GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure relay output %u: %d",
				(unsigned int)i + 1U, ret);
		}
	}
	publish_relay_gpio();
#else
	pcf857x = DEVICE_DT_GET(DT_NODELABEL(pcf8575));
	if (!device_is_ready(pcf857x)) {
		LOG_ERR("PCF8575 device is not ready");
		pcf857x = NULL;
		return;
	}

	for (size_t i = 0; i < RELAY_COUNT; i++) {
		int ret = gpio_pin_configure(pcf857x, output_lookup_table[i],
					     GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("Failed to configure relay output %u: %d",
				(unsigned int)i + 1U, ret);
		}
	}
#endif
}

void relay_process(void)
{
#if defined(CONFIG_BOARD_NATIVE_SIM)
	for (size_t i = 0; i < ARRAY_SIZE(relay_gpio); i++) {
		uint8_t requested = (uint8_t)OD_outputs[i];

		if (output[i] != requested &&
		    gpio_pin_set_dt(&relay_gpio[i], requested != 0U) == 0) {
			output[i] = requested;
		}
	}
#else
	if (pcf857x == NULL) {
		return;
	}

	for (size_t i = 0; i < RELAY_COUNT; i++) {
		uint8_t requested = (uint8_t)OD_outputs[i];

		if (output[i] != requested &&
		    gpio_pin_set(pcf857x, output_lookup_table[i],
				 requested != 0U) == 0) {
			output[i] = requested;
		}
	}
#endif
#if defined(CONFIG_BOARD_NATIVE_SIM)
	publish_relay_gpio();
#endif
}
