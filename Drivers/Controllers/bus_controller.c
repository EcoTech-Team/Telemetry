#include "bus_controller.h"

//! Receive buffer
static uint8_t _RxBuff[BUS_BUFF_NUM][BUS_BUFF_LEN];
//! Rx message index
static uint8_t _RxMsg = 0;
//! Rx data intex
static uint8_t _RxData = 0;
//! Rx overflowed
static bool _RxOverflow = false;

static uint8_t _TempDR;

uint16_t _sr;

void MainLine_IRQHandler(UART_HandleTypeDef *huart3)
{
	_sr = huart3->Instance->SR;
	if(_sr & USART_SR_RXNE)
	{
		if(_RxData < BUS_BUFF_LEN) _RxBuff[_RxMsg][_RxData++] = huart3->Instance->DR;
		else _RxOverflow = true;
	}
	if(_sr & USART_SR_IDLE)
	{
		//! Read DR to clear flag
		_TempDR = huart3->Instance->DR;

		uint8_t temp_RxMsg = _RxMsg;
		uint8_t temp_RxData = _RxData;
		bool temp_RxOverflow = _RxOverflow;

		_RxMsg = (_RxMsg + 1) % BUS_BUFF_NUM;
		_RxData = 0;
		_RxOverflow = false;

		//! Call receive function if receive buffer is not overflowed
		if(!temp_RxOverflow) BUS_Received(_RxBuff[temp_RxMsg], temp_RxData);
	}
	if(_sr & (USART_SR_ORE || USART_SR_FE || USART_SR_NE || USART_SR_PE))
	{
		_TempDR = huart3->Instance->DR;
	}
}
