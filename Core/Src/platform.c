#include "platform.h"

uint32_t millis(void) {
    return HAL_GetTick();
}
