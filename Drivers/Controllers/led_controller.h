/*
 * led_controller.h
 */

#ifndef LED_CONTROLLER_H_
#define LED_CONTROLLER_H_

#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOA

// System states
#define SD_CARD_NOT_EXIST		0x01
#define SD_CARD_NOT_MOUNTED 	0x02
#define SD_CARD_READ_WRITE_ERR	0x03
#define SD_CARD_NO_FREE_SPACE	0x04
#define ML_READ_ERR				0x05
#define UNDEFINED_ERR			0x07

typedef enum
{
	LED1 = LED1_Pin,
	LED2 = LED2_Pin,
	LED3 = LED3_Pin
} LED;

void LED_Init(GPIO_InitTypeDef GPIO_InitStruct);
void LED_SetLedOn(LED led);
void LED_SetLedOff(LED led);
void LED_ToggleLed(LED led);
void LED_SetLed(LED led, bool state);
void LED_DisplayStatus(uint8_t status);

#endif /* LED_CONTROLLER_H_ */