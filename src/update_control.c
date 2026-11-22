#include "update_control.h"

#if defined(CONFIG_CANOPENNODE_PROGRAM_DOWNLOAD)
#include <string.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(update_control, CONFIG_CANOPENNODE_LOG_LEVEL);

#define PROGRAM_CONTROL 0x1f51
#define CONFIRM_IMAGE 0x80

static CO_SDO_abortCode_t (*upstream_control)(CO_ODF_arg_t *);

static CO_SDO_abortCode_t software_version(CO_ODF_arg_t *arg)
{
	static const char version[] = CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION;

	_Static_assert(sizeof(version) > 1, "Firmware version must not be empty");
	_Static_assert(sizeof(version) - 1 <= CONFIG_CANOPENNODE_SDO_BUFFER_SIZE,
		       "Firmware version must fit the SDO buffer");
	if (!arg->reading) {
		return CO_SDO_AB_READONLY;
	}
	/* Exclude the C terminator: the gateway compares the VISIBLE_STRING. */
	memcpy(arg->data, version, sizeof(version) - 1);
	arg->dataLength = sizeof(version) - 1;
	arg->dataLengthTotal = sizeof(version) - 1;
	return CO_SDO_AB_NONE;
}

static CO_SDO_abortCode_t program_control(CO_ODF_arg_t *arg)
{
	if (arg->reading || arg->subIndex != 1U || arg->data[0] != CONFIRM_IMAGE) {
		return upstream_control(arg);
	}

	/*
	 * 0x80 is Zephyr's image-confirmation extension, not a program stop/start.
	 * The gateway may verify a test image after bringing it OPERATIONAL,
	 * then confirm it without another NMT transition.
	 * Keep STOP/CLEAR/START subject to upstream's PRE-OPERATIONAL requirement,
	 * but permit explicit confirmation in either active NMT state.
	 */
	memcpy(arg->data, arg->ODdataStorage, sizeof(uint8_t));
	if (!boot_is_img_confirmed()) {
		int err = boot_write_img_confirmed();

		if (err != 0) {
			LOG_ERR("Image confirmation failed: %d", err);
			return CO_SDO_AB_HW;
		}
		LOG_INF("Running image confirmed");
	}
	return CO_SDO_AB_NONE;
}

void relay_update_control_attach(CO_SDO_t *sdo)
{
	uint16_t entry = CO_OD_find(sdo, PROGRAM_CONTROL);

	if (entry == 0xffffU || sdo->ODExtensions[entry].pODFunc == NULL) {
		LOG_ERR("Program control callback absent");
		return;
	}
	upstream_control = sdo->ODExtensions[entry].pODFunc;
	CO_OD_configure(sdo, PROGRAM_CONTROL, program_control, NULL, 0, 0);
	CO_OD_configure(sdo, 0x100a, software_version, NULL, 0, 0);
}
#endif
