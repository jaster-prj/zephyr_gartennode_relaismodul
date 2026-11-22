#ifndef NATIVE_SIM_GPIO_H
#define NATIVE_SIM_GPIO_H

#include <stddef.h>
#include <stdint.h>

void native_sim_gpio_publish(uint16_t companion_port, const uint8_t *values,
			     size_t count);

#endif
