#ifndef RELAY_UPDATE_DIAGNOSTICS_H
#define RELAY_UPDATE_DIAGNOSTICS_H

#include <canopennode.h>

#if defined(CONFIG_RELAY_UPDATE_DIAGNOSTICS)
void relay_update_diagnostics_attach(CO_SDO_t *sdo);
#endif

#endif
