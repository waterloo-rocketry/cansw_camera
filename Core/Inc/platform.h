#ifndef PLATFORM_H
#define PLATFORM_H

#include "main.h"

#define LED_RED_ON() (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET))
#define LED_RED_OFF() (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET))
#define LED_RED_TOGGLE() (HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0))

#define LED_GREEN_ON() (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET))
#define LED_GREEN_OFF() (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET))
#define LED_GREEN_TOGGLE() (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8))

// number of milliseconds since bootup
uint32_t millis(void);

#endif /* PLATFORM_H */
