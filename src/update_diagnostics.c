/* Optional, non-invasive instrumentation of the upstream download callbacks. */
#include "update_diagnostics.h"

#if defined(CONFIG_RELAY_UPDATE_DIAGNOSTICS)
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(update_diagnostics, LOG_LEVEL_INF);

/* Read only after the transfer: live debugger accesses can disturb timing. */
volatile struct {
	uint32_t reset_cause;
	uint32_t bytes;
	uint32_t calls;
	uint32_t maximum_data_us;
	uint32_t maximum_control_us;
	uint32_t abort_code;
	uint32_t last_command;
	uint32_t last_callback_ms;
} relay_update_diagnostics;

static CO_SDO_abortCode_t (*data_callback)(CO_ODF_arg_t *);
static CO_SDO_abortCode_t (*control_callback)(CO_ODF_arg_t *);

static int diagnostics_init(void)
{
	static const char *const noisy_sources[] = {
		"gpio_pcf857x",
	};
	uint32_t cause = 0;

	(void)hwinfo_get_reset_cause(&cause);
	relay_update_diagnostics.reset_cause = cause;
	LOG_INF("Image %s, reset cause 0x%08x, watchdog support %u",
		CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION, cause,
		IS_ENABLED(CONFIG_WATCHDOG));
	(void)hwinfo_clear_reset_cause();
	for (size_t i = 0; i < ARRAY_SIZE(noisy_sources); i++) {
		int id = log_source_id_get(noisy_sources[i]);

		if (id >= 0) {
			log_filter_set(NULL, 0, id, LOG_LEVEL_NONE);
		}
	}
	return 0;
}

SYS_INIT(diagnostics_init, APPLICATION, 0);

static CO_SDO_abortCode_t record_data(CO_ODF_arg_t *arg)
{
	if (arg->reading || arg->subIndex != 1U) {
		return data_callback(arg);
	}

	uint32_t length = arg->dataLength;
	uint32_t start = k_cycle_get_32();
	CO_SDO_abortCode_t result;

	if (arg->firstSegment) {
		relay_update_diagnostics.bytes = 0;
		relay_update_diagnostics.calls = 0;
		relay_update_diagnostics.maximum_data_us = 0;
		LOG_INF("Image download started: %u bytes", arg->dataLengthTotal);
	}
	result = data_callback(arg);
	uint32_t duration = k_cyc_to_us_floor32(k_cycle_get_32() - start);

	relay_update_diagnostics.bytes += length;
	relay_update_diagnostics.calls++;
	relay_update_diagnostics.maximum_data_us =
		MAX(relay_update_diagnostics.maximum_data_us, duration);
	relay_update_diagnostics.abort_code = result;
	relay_update_diagnostics.last_callback_ms = k_uptime_get_32();
	if (arg->lastSegment || result != CO_SDO_AB_NONE || duration > 100000U) {
		LOG_INF("Download bytes=%u max=%u us abort=%x final=%u",
			relay_update_diagnostics.bytes,
			relay_update_diagnostics.maximum_data_us,
			(unsigned int)result, arg->lastSegment);
	}
	return result;
}

static CO_SDO_abortCode_t record_control(CO_ODF_arg_t *arg)
{
	uint8_t command = arg->reading ? 0xff : arg->data[0];
	uint32_t start = k_cycle_get_32();
	CO_SDO_abortCode_t result = control_callback(arg);
	uint32_t duration = k_cyc_to_us_floor32(k_cycle_get_32() - start);

	if (!arg->reading) {
		relay_update_diagnostics.last_command = command;
		relay_update_diagnostics.maximum_control_us =
			MAX(relay_update_diagnostics.maximum_control_us, duration);
		LOG_INF("Control %u: %u us, abort=%x", command, duration,
			(unsigned int)result);
	}
	return result;
}

void relay_update_diagnostics_attach(CO_SDO_t *sdo)
{
	uint16_t data = CO_OD_find(sdo, 0x1f50);
	uint16_t control = CO_OD_find(sdo, 0x1f51);

	if (data == 0xffffU || control == 0xffffU) {
		LOG_ERR("Update objects absent");
		return;
	}
	data_callback = sdo->ODExtensions[data].pODFunc;
	control_callback = sdo->ODExtensions[control].pODFunc;
	if (data_callback == NULL || control_callback == NULL) {
		LOG_ERR("Update callbacks absent");
		return;
	}
	CO_OD_configure(sdo, 0x1f50, record_data, NULL, 0, 0);
	CO_OD_configure(sdo, 0x1f51, record_control, NULL, 0, 0);
}
#endif
