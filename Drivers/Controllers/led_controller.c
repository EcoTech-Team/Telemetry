/*
 * led_controller.c
 */
#include "led_controller.h"

void LED_Init(GPIO_InitTypeDef GPIO_InitStruct)
{
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void LED_SetLedOn(LED led)
{
	HAL_GPIO_WritePin(GPIOA, led, GPIO_PIN_SET);
}

void LED_SetLedOff(LED led)
{
	HAL_GPIO_WritePin(GPIOA, led, GPIO_PIN_RESET);
}

void LED_ToggleLed(LED led)
{
    HAL_GPIO_TogglePin(GPIOA, led);
}

void LED_SetLed(LED led, bool state)
{
	if(state) LED_SetLedOn(led);
	else LED_SetLedOff(led);
}

void LED_DisplayStatus(uint8_t status)
{
  switch (status)
  {
  case SD_CARD_NOT_EXIST:
    LED_SetLed(LED3, false);
    LED_SetLed(LED2, false);
    LED_SetLed(LED1, true);
    break;
  case SD_CARD_NOT_MOUNTED:
    LED_SetLed(LED3, false);
    LED_SetLed(LED2, true);
    LED_SetLed(LED1, false);
    break;
  case SD_CARD_READ_WRITE_ERR:
    LED_SetLed(LED3, false);
    LED_SetLed(LED2, true);
    LED_SetLed(LED1, true);
    break;
  case SD_CARD_NO_FREE_SPACE:
    LED_SetLed(LED3, true);
    LED_SetLed(LED2, false);
    LED_SetLed(LED1, false);
    break;
  case ML_READ_ERR:
    LED_SetLed(LED3, true);
    LED_SetLed(LED2, false);
    LED_SetLed(LED1, true);
    break;
  case UNDEFINED_ERR:
    LED_SetLed(LED3, true);
    LED_SetLed(LED2, true);
    LED_SetLed(LED1, true);
    break;
  default:
    LED_SetLed(LED3, false);
    LED_SetLed(LED2, false);
    LED_SetLed(LED1, false);
    break;
  }
}
