/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi_controller.h"
#include "led_controller.h"
#include "msg_lib.h"
#include "bus_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Private variables ---------------------------------------------------------*/
#define ML_TIMEOUT        0xA5
#define SD_CARD_FULL      0xA6
#define PAYLOAD_MAX_LEN   100
#define MOTOR_CONTROLLER  0x01
#define MOTOR_DRIVER      0x02

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;

FATFS fs; // file system
FIL fil; // file object
FILINFO fno; // file info
uint8_t res = FR_NOT_READY; // to store the result

/* capacity related variables */
/* check capasity of the card */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

static uint8_t MC_Payload[PAYLOAD_MAX_LEN]; // MotorController data
static uint8_t MD_Payload[PAYLOAD_MAX_LEN]; // MotorDriver data
static uint8_t mc_received = false;
static uint8_t md_received = false;
static uint8_t offset_mc = 0;
static uint8_t offset_md = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void MSG_Received(uint8_t *, uint8_t);
void WriteDataToCard (void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MSG_CrcInit();

  /* Configure the system clock */
  SystemClock_Config();

  while (1)
  {
    if (res == FR_NOT_READY) {
      // Mount and initialize card
      res = f_mount(&fs, "", 1);
      if (res != FR_OK) {
        Error_Handler();
        continue;
      }

      f_getfree("", &fre_clust, &pfs);
      total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
      free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
      if (free_space == 0 && total != 0) {
        res = SD_CARD_FULL;
        Error_Handler();
        continue;
      }

      res = f_open(&fil, "data_from_ride.txt", FA_CREATE_NEW|FA_WRITE);
      if (res == FR_EXIST) {
        res = FR_OK;
        continue;
      }
      if (f_puts("|--MC--|----MD----|\n", &fil) == -1) res = FR_DISK_ERR;
      f_close(&fil);
    }
    else if (res == FR_OK) {
      WriteDataToCard();
      Error_Handler();
    }
  }
}

// Receive cut frame [without addr and crc]
void MSG_Received(uint8_t *buff, uint8_t len)
{
  uint8_t cmd, p_len;
  cmd = buff[0];    // 2nd byte is CMD
  p_len = buff[1];  // 3rd byte is payload length

  if (cmd == MOTOR_CONTROLLER) {
    for (uint8_t byte = 0; byte < p_len; byte++)
      MC_Payload[offset_mc+byte] = buff[byte+2];
    mc_received = true;
    offset_mc += 4;
  }
  else if (cmd == MOTOR_DRIVER) {
    for (uint8_t byte = 0; byte < p_len; byte++)
      MD_Payload[offset_md+byte] = buff[byte+2];
    md_received = true;
    offset_md += 4;
  }
  if (offset_mc >= PAYLOAD_MAX_LEN)
    offset_mc = 0;
  if (offset_md >= PAYLOAD_MAX_LEN)
    offset_md = 0;
}

void WriteDataToCard (void)
{
  TCHAR data[40] = {0};
  // Data received from MotorController
  if (mc_received == true && md_received == true
     && offset_mc == 0 && offset_md == 0) {
    // volts, ampere, rpm
    uint8_t vol, amp;
    uint16_t rpm;
    uint8_t buttonA, buttonB, buttonC, buttonD;

    f_open(&fil, "data_from_ride.txt", FA_OPEN_ALWAYS|FA_WRITE);
    for (uint8_t pos=0; pos < PAYLOAD_MAX_LEN; pos+=4) {
      buttonA = MC_Payload[pos+0];
      buttonB = MC_Payload[pos+1];
      buttonC = MC_Payload[pos+2];
      buttonD = MC_Payload[pos+3];

      vol = MD_Payload[pos+0];
      amp = MD_Payload[pos+1];
      rpm = (uint16_t)(MD_Payload[pos+2]) << 8; // MSB
      rpm += (uint16_t)(MD_Payload[pos+3]);     // LSB

      sprintf(data, "%u-%u-%u-%u|%u-%u-%u\n", buttonA, buttonB, buttonC, buttonD, vol, amp, rpm);
      f_lseek(&fil, f_size(&fil));
      if (f_puts(data, &fil) == -1) res = FR_DISK_ERR;
    }
    f_close(&fil);

    mc_received = false;
    md_received = false;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 17050;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  LED_Init(GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW3_Pin */
  GPIO_InitStruct.Pin = SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW3_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  switch (res)
  {
  case FR_OK:
    LED_DisplayStatus(0);
    break;
  case FR_NOT_READY:
    LED_DisplayStatus(SD_CARD_NOT_EXIST);
    break;
  case FR_NOT_ENABLED:
    LED_DisplayStatus(SD_CARD_NOT_MOUNTED);
    break;
  case FR_DISK_ERR:
    LED_DisplayStatus(SD_CARD_READ_WRITE_ERR);
    break;
  case SD_CARD_FULL:
    LED_DisplayStatus(SD_CARD_NO_FREE_SPACE);
    break;
  case ML_TIMEOUT:
    LED_DisplayStatus(ML_READ_ERR);
    break;
  default:
    LED_DisplayStatus(UNDEFINED_ERR);
    break;
  }
  /* USER CODE END Error_Handler_Debug */
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
