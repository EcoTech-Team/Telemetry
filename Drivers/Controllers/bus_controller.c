#include "bus_controller.h"

//! Free bus flag
static volatile bool _BusFree = true;
//! Message is pending to send
static volatile bool _TxPending = false;
//! Tranmission ongoing
static volatile bool _TxOngoing = false;
//! Receive buffer
static uint8_t _RxBuff[BUS_BUFF_NUM][BUS_BUFF_LEN];
//! Rx message index
static uint8_t _RxMsg = 0;
//! Rx data intex
static uint8_t _RxData = 0;
//! Rx overflowed
static bool _RxOverflow = false;
//! Transmit buffer
static uint8_t _TxBuff[BUS_BUFF_LEN];
//! Tx data index
static uint8_t _TxData = 0;
//! Tx data to send
static uint8_t _TxLen = 0;

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
		//! Send message if was pending
		if(_TxPending)
		{
			_TxOngoing = true;
			_TxPending = false;
			// Set interupt on clearing TX register.
			huart3->Instance->CR1 |= USART_CR1_TXEIE;
			// start uart transmission
			huart3->Instance->DR = _TxBuff[_TxData++];
		}
		else _BusFree = true;
	}
	if(_sr & USART_SR_TXE && _TxOngoing)
	{
		if(_TxData < _TxLen) huart3->Instance->DR = _TxBuff[_TxData++];
		else
		{
			// clear interupt on clearing TX register.
			huart3->Instance->CR1 &= ~USART_CR1_TXEIE;
			_BusFree = true;
			_TxOngoing = false;
		}
	}
	if(_sr & (USART_SR_ORE || USART_SR_FE || USART_SR_NE || USART_SR_PE))
	{
		_TempDR = huart3->Instance->DR;
	}
}

void BUS_Send(UART_HandleTypeDef *huart, uint8_t *buff, uint8_t len)
{
	//! Dump message if transmission is already ongoing or buffered
	if(_TxOngoing || _TxPending) return;

	if( len > BUS_BUFF_LEN) len = BUS_BUFF_LEN;

	//! Copy passed message to internal buffer
	for(uint8_t i=0; i<len; i++)
		_TxBuff[i] = buff[i];
	//! Set tx size and pointer
	_TxLen = len;
	_TxData = 0;

	//! Start transmission if bus is free
	if(_BusFree)
	{
		_BusFree = false;
		_TxOngoing = true;
		// Set interupt on clearing TX register.
		huart->Instance->CR1 |= USART_CR1_TXEIE;
		huart->Instance->DR = _TxBuff[_TxData++];
	}
	//! Pend message to be send after received frame
	else _TxPending = true;
}
