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
//#include "msg_lib.h"
#include <string.h>
#include <stdio.h>


/* Private variables ---------------------------------------------------------*/
#define ML_TIMEOUT        0xA5
#define SD_CARD_FULL      0xA6

struct
{
  // uint8_t addr; // For now only one address exist (0x01)
  uint8_t Command;
  uint8_t Length;
  uint8_t *Payload;
} ML_Frame;


SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;

FATFS fs; // file system
FIL fil; // file
FILINFO fno; // file info
uint8_t res = FR_NOT_READY; // to store the result
//MSG_Message msg;  // received message

/* capacity related variables */
/* check capasity of the card */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* To send the data to the uart */
/*void send_uart (char *string)
{
  uint8_t len = strlen(string);
  HAL_UART_Transmit(&huart3, (uint8_t *) string, len, 2000); // transmit in blocking mode
}*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
uint8_t RX_Byte (void);
void MainLine_FrameHandler (void);
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
  //MSG_CrcInit();

  /* Configure the system clock */
  SystemClock_Config();

  //MSG_Message *msg;
  HAL_Delay(500);

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

      // Check whether sub-directories exist. If not create it
      if (f_stat("MotorController", &fno) == FR_NO_FILE) {
        res = f_mkdir("MotorController");
        Error_Handler();
      }
      if (f_stat("MotorDriver", &fno) == FR_NO_FILE) {
        res = f_mkdir("MotorDriver");
        Error_Handler();
      }
    }
    else if (res == FR_OK) {
      WriteDataToCard();
    }
  }
}

void MSG_Received(uint8_t *buff, uint8_t len)
{
  // Receive cut frame [without addr and crc]
  ML_Frame.Command = buff[0]; // 2nd byte is CMD
  ML_Frame.Length = buff[1];  // 3rd byte is payload length
  ML_Frame.Payload = calloc(ML_Frame.Length, sizeof(ML_Frame.Payload));
  for (int byte = 2; byte < len; byte ++)
    ML_Frame.Payload[byte-2] = buff[byte];
}

void WriteDataToCard (void)
{
  // Data received from MotorController
  if (ML_Frame.Command == 0x01) {
    for (int i = 1; i <= ML_Frame.Length; i++) {
      switch (i)
      {
        case 1:
          f_open(&fil, "MotorController/buttonA.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        case 2:
          f_open(&fil, "MotorController/buttonB.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        case 3:
          f_open(&fil, "MotorController/buttonC.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        case 4:
          f_open(&fil, "MotorController/buttonD.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        default:
          f_open(&fil, "MotorController/default.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
      }
      f_lseek(&fil, f_size(&fil));
      res = f_putc(ML_Frame.Payload[i-1], &fil);
      f_puts("\n", &fil);
      f_close(&fil);
    }
  }
  // Data received from MotorDriver
  else if (ML_Frame.Command == 0x02) {
    for (int i = 1; i <= ML_Frame.Length; i++) {
      switch (i)
      {
        case 1:
          f_open(&fil, "MotorDriver/voltage.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        case 2:
          f_open(&fil, "MotorDriver/current.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        case 3:
          f_open(&fil, "MotorDriver/rpm.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
        default:
          f_open(&fil, "MotorDriver/default.txt", FA_OPEN_ALWAYS|FA_WRITE);
          break;
      }
      f_lseek(&fil, f_size(&fil));
      res = f_putc(ML_Frame.Payload[i-1], &fil);
      f_puts("\n", &fil);
      f_close(&fil);
    }
  }
  // Clear information about last frame
  ML_Frame.Command = 0;
  ML_Frame.Length = 0;
  free(ML_Frame.Payload);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
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
