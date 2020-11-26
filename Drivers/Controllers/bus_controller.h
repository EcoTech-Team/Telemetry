/*
 * bus_controller.h
 */

#ifndef BUS_CONTROLLER_H_
#define BUS_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

//! Length of single frame buffer
#define BUS_BUFF_LEN    60
//! Number of frame buffers
#define BUS_BUFF_NUM    6

void MainLine_IRQHandler(UART_HandleTypeDef *);
//! Function called after receive. Has to be implemented in other file
void __attribute__((weak)) BUS_Received(uint8_t *buff, uint8_t len);
//! Send passed buff to the bus
void BUS_Send(UART_HandleTypeDef *huart, uint8_t *buff, uint8_t len);

#endif /* BUS_CONTROLLER_H_ */
