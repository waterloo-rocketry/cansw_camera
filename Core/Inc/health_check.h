#ifndef HEALTH_CHECK_H
#define HEALTH_CHECK_H

#include <stdbool.h>

bool check_bus_current_error(void);
bool check_bus_voltage_error(void);

#endif
