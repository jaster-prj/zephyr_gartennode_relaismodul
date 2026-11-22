#ifndef TEST_CANOPENNODE_H
#define TEST_CANOPENNODE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint32_t CO_SDO_abortCode_t;
#define CO_SDO_AB_NONE 0U
#define CO_SDO_AB_HW 0x06060000U
#define CO_SDO_AB_READONLY 0x06010002U

typedef struct {
	bool reading;
	uint8_t subIndex;
	uint8_t *data;
	void *ODdataStorage;
	uint16_t dataLength;
	uint32_t dataLengthTotal;
} CO_ODF_arg_t;

typedef struct {
	CO_SDO_abortCode_t (*pODFunc)(CO_ODF_arg_t *);
} CO_OD_extension_t;

typedef struct {
	CO_OD_extension_t *ODExtensions;
} CO_SDO_t;

uint16_t CO_OD_find(CO_SDO_t *, uint16_t);
void CO_OD_configure(CO_SDO_t *, uint16_t,
		    CO_SDO_abortCode_t (*)(CO_ODF_arg_t *),
		    void *, uint8_t *, uint16_t);
#endif
