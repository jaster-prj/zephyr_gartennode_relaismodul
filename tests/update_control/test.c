#include <assert.h>
#include <string.h>
#include "update_control.h"

#define DEVICE_STATE_ABORT 0x08000022U
static bool confirmed;
static bool missing_entry;
static int confirmation_result;
static int confirmation_calls;
static int upstream_calls;
static CO_SDO_abortCode_t (*version_callback)(CO_ODF_arg_t *);
static CO_OD_extension_t extension;
static CO_SDO_t sdo = {.ODExtensions = &extension};

bool boot_is_img_confirmed(void)
{
	return confirmed;
}

int boot_write_img_confirmed(void)
{
	confirmation_calls++;
	if (confirmation_result == 0) {
		confirmed = true;
	}
	return confirmation_result;
}

uint16_t CO_OD_find(CO_SDO_t *object, uint16_t index)
{
	assert(object == &sdo && index == 0x1f51);
	return missing_entry ? 0xffff : 0;
}

void CO_OD_configure(CO_SDO_t *object, uint16_t index,
		    CO_SDO_abortCode_t (*callback)(CO_ODF_arg_t *),
		    void *context, uint8_t *flags, uint16_t size)
{
	assert(object == &sdo && (index == 0x1f51 || index == 0x100a));
	assert(context == NULL && flags == NULL && size == 0);
	if (index == 0x1f51) {
		extension.pODFunc = callback;
	} else {
		version_callback = callback;
	}
}

static CO_SDO_abortCode_t upstream(CO_ODF_arg_t *arg)
{
	(void)arg;
	upstream_calls++;
	/* Model upstream's rejection while OPERATIONAL. */
	return DEVICE_STATE_ABORT;
}

static void reset(void)
{
	confirmed = false;
	missing_entry = false;
	confirmation_result = 0;
	confirmation_calls = 0;
	upstream_calls = 0;
	version_callback = NULL;
	extension.pODFunc = upstream;
	relay_update_control_attach(&sdo);
}

int main(void)
{
	uint8_t value = 0x80;
	uint8_t old_value = 0x55;
	CO_ODF_arg_t arg = {
		.reading = false, .subIndex = 1,
		.data = &value, .ODdataStorage = &old_value,
	};

	reset();
	assert(extension.pODFunc(&arg) == CO_SDO_AB_NONE);
	assert(confirmed && confirmation_calls == 1 && upstream_calls == 0);
	assert(value == old_value);

	reset();
	confirmed = true;
	value = 0x80;
	assert(extension.pODFunc(&arg) == CO_SDO_AB_NONE);
	assert(confirmation_calls == 0 && upstream_calls == 0);

	reset();
	confirmation_result = -5;
	value = 0x80;
	assert(extension.pODFunc(&arg) == CO_SDO_AB_HW);
	assert(!confirmed && confirmation_calls == 1);
	assert(value == old_value);

	reset();
	value = 0x80;
	arg.reading = true;
	assert(extension.pODFunc(&arg) == DEVICE_STATE_ABORT);
	assert(upstream_calls == 1 && confirmation_calls == 0);
	arg.reading = false;

	for (uint8_t command = 0; command <= 3; command++) {
		reset();
		value = command;
		assert(extension.pODFunc(&arg) == DEVICE_STATE_ABORT);
		assert(upstream_calls == 1 && confirmation_calls == 0);
		assert(value == command);
	}

	reset();
	value = 0x80;
	arg.subIndex = 0;
	assert(extension.pODFunc(&arg) == DEVICE_STATE_ABORT);
	assert(upstream_calls == 1 && confirmation_calls == 0);
	arg.subIndex = 1;

	reset();
	missing_entry = true;
	extension.pODFunc = upstream;
	relay_update_control_attach(&sdo);
	assert(extension.pODFunc == upstream);

	reset();
	extension.pODFunc = NULL;
	relay_update_control_attach(&sdo);
	assert(extension.pODFunc == NULL);

	reset();
	uint8_t version[32] = {0};
	CO_ODF_arg_t version_arg = {
		.reading = true, .subIndex = 0, .data = version,
		.dataLength = 6, .dataLengthTotal = 6,
	};
	assert(version_callback != NULL);
	assert(version_callback(&version_arg) == CO_SDO_AB_NONE);
	assert(version_arg.dataLength == 5 && version_arg.dataLengthTotal == 5);
	assert(memcmp(version, "1.3.0", 5) == 0);
	version_arg.reading = false;
	assert(version_callback(&version_arg) == CO_SDO_AB_READONLY);
	return 0;
}
