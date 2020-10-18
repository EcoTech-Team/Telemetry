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
#include <string.h>
#include <stdio.h>


/* Private variables ---------------------------------------------------------*/
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
FRESULT fresult; // to store the result
uint8_t buffer[1024]; // to store data

/* capacity related variables */
/* check capasity of the card */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* To send the data to the uart */
void send_uart (char *string)
{
  uint8_t len = strlen(string);
  HAL_UART_Transmit(&huart3, (uint8_t *) string, len, 2000); // transmit in blocking mode
}

/* Return size of data in the buffer */
int bufsize (uint8_t *buf)
{
  int i=0;
  while (*buf++ != '\0') i++;
  return i;
}

void bufclear (void)
{
  for (int i = 0; i < ML_Frame.Length; i++)
  {
      ML_Frame.Payload[i] = '\0';
  }
}

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

  /* Configure the system clock */
  SystemClock_Config();

  HAL_Delay(500);
  /* Mount SD Card */
  fresult = f_mount(&fs, "", 1);
  if (fresult != FR_OK)
    send_uart("Error: can't mount SD Card...\n");
  else
    send_uart("SD Card mounted successfully...\n");

/******************* Card capacity detals *******************/

  /* Check free space */
  fresult = f_getfree("", &fre_clust, &pfs);

  if (fresult == FR_OK)
    send_uart("f_getfree FR_OK...\n");
  else if (fresult == FR_NOT_ENABLED)
    send_uart("f_getfree FR_NOT_ENABLED...\n");
  else if (fresult == FR_NOT_READY)
    send_uart("f_getfree FR_NOT_READY...\n");
  else if (fresult == FR_NO_FILESYSTEM)
    send_uart("f_getfree FR_NO_FILESYSTEM...\n");
  else if (fresult == FR_DISK_ERR)
    send_uart("f_getfree FR_DISK_ERR...\n");
  else if (fresult == FR_INVALID_DRIVE)
    send_uart("f_getfree FR_INVALID_DRIVE...\n");
  else if (fresult == FR_INT_ERR)
    send_uart("f_getfree FR_INT_ERR...\n");
  else if (fresult == FR_TIMEOUT)
    send_uart("f_getfree FR_TIMEOUT...\n");

  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

  // Check whether sub-directories exist. If not create it
  if (f_stat("MotorController", &fno) == FR_NO_FILE)
    fresult = f_mkdir("MotorController");

  if (f_stat("MotorDriver", &fno) == FR_NO_FILE)
    fresult = f_mkdir("MotorDriver");

  while (1)
  {
    MainLine_FrameHandler();
    WriteDataToCard();
  }
}

uint8_t RX_Byte (void)
{
  uint8_t data = 0;
  while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY);
  HAL_UART_Receive(&huart3, &data, 1, 2000);
  return data;
}

void MainLine_FrameHandler (void)
{
  // First received byte is an address
  if (RX_Byte() == 0x01) {
    // Read CMD
    ML_Frame.Command = RX_Byte();
    // Read payload lenght
    ML_Frame.Length = RX_Byte();
    // Initialize array
    ML_Frame.Payload = calloc(ML_Frame.Length, sizeof(ML_Frame.Payload));

    for (int i = 0; i < ML_Frame.Length; i++)
      ML_Frame.Payload[i] = RX_Byte();

    // Read CRC byte
    RX_Byte();
  }
}

void WriteDataToCard (void)
{
  // Data received from MotorController
  if (ML_Frame.Command == 0x01) {
    for (int i = 0; i < ML_Frame.Length; i++) {
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
          continue;
          break;
      }
      f_lseek(&fil, f_size(&fil));
      f_puts(ML_Frame.Payload[i]+"\n", &fil);
      f_close(&fil);
    }
  }
  // Data received from MotorDriver
  else if (ML_Frame.Command == 0x02) {
    for (int i = 0; i < ML_Frame.Length; i++) {
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
          continue;
          break;
      }
      f_lseek(&fil, f_size(&fil));
      f_puts(ML_Frame.Payload[i]+"\n", &fil);
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
  huart3.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED2_Pin|LED1_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin LED2_Pin LED1_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin|LED1_Pin|SPI1_CS_Pin;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
