/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <sx127x.h>
#include <sx127x_spi.h>
#include <inttypes.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SPI_TIMEOUT 10000

#define ERROR_CHECK(x)                                                                 \
  do {                                                                                 \
    int __err_rc = (x);                                                                \
    if (__err_rc != HAL_OK) {                                                          \
      stm_log("invalid response code: %d at %s:%d\r\n", __err_rc, __func__, __LINE__); \
      Error_Handler();                                                                 \
    }                                                                                  \
  } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

sx127x device;
volatile int interrupt_received = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void stm_log(const char *format, ...);
void rx_callback(void *ctx, uint8_t *data, uint16_t data_length);
void cad_callback(void *ctx, int cad_detected);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  stm_log("system initialized\r\n");
  ERROR_CHECK(sx127x_create(&hspi2, &device));
  stm_log("sx127x connected\r\n");
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, &device));
  ERROR_CHECK(sx127x_set_frequency(TEST_FREQUENCY, &device));
  ERROR_CHECK(sx127x_lora_reset_fifo(&device));
  ERROR_CHECK(sx127x_rx_set_lna_boost_hf(true, &device));
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, &device));
  ERROR_CHECK(sx127x_rx_set_lna_gain(SX127x_LNA_GAIN_G4, &device));
  ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, &device));
  ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
  ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, &device));
  ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
  ERROR_CHECK(sx127x_set_preamble_length(8, &device));
  sx127x_rx_set_callback(rx_callback, &device, &device);
  sx127x_lora_cad_set_callback(cad_callback, &device, &device);
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, &device));
  stm_log("waiting for messages\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (interrupt_received > 0) {
	  interrupt_received = 0;
	  //stm_log("interrupt\n");
	  sx127x_handle_interrupt(&device);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void stm_log(const char *format, ...) {
  char temp[1024];
  memset(temp, '\0', 1024);
  va_list args;
  va_start(args, format);
  vsnprintf(temp, 1023, format, args);
  HAL_UART_Transmit(&huart1, (uint8_t *)temp, strlen(temp), 1000);
  va_end(args);
}

void rx_callback(void *ctx, uint8_t *data, uint16_t data_length) {
  sx127x *device = (sx127x *)ctx;
  uint8_t payload[514];
  const char SYMBOLS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < data_length; i++) {
    uint8_t cur = data[i];
    payload[2 * i] = SYMBOLS[cur >> 4];
    payload[2 * i + 1] = SYMBOLS[cur & 0x0F];
  }
  payload[data_length * 2] = '\0';

  int16_t rssi;
  ERROR_CHECK(sx127x_rx_get_packet_rssi(device, &rssi));
  float snr;
  ERROR_CHECK(sx127x_lora_rx_get_packet_snr(device, &snr));
  int32_t frequency_error;
  ERROR_CHECK(sx127x_rx_get_frequency_error(device, &frequency_error));
  stm_log("received: %d %s rssi: %d snr: %f freq_error: %" PRId32 "\r\n", data_length, payload, rssi, snr, frequency_error);
}

void cad_callback(void *ctx, int cad_detected) {
  sx127x *device = (sx127x *)ctx;
  if (cad_detected == 0) {
    stm_log("cad not detected\r\n");
    ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_CAD, SX127x_MODULATION_LORA, device));
    return;
  }
  // put into RX mode first to handle interrupt as soon as possible
  ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_RX_CONT, SX127x_MODULATION_LORA, device));
  stm_log("cad detected\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == DIO0_Pin) {
    interrupt_received = 1;
  }
}

int sx127x_spi_read_registers(int reg, void *spi_device, size_t data_length, uint32_t *result) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  uint64_t tx_buf = ((uint8_t)reg & 0x7F);
  uint64_t rx_buf = 0;
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
  int code = HAL_SPI_TransmitReceive(spi_device, (uint8_t *)&tx_buf, (uint8_t *)&rx_buf, data_length + 1, SPI_TIMEOUT);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
  if (code != 0) {
    *result = 0;
    return code;
  }
  *result = __builtin_bswap32(rx_buf >> 8) >> (4 - data_length) * 8;
  return 0;
}

int sx127x_spi_read_buffer(int reg, uint8_t *buffer, size_t buffer_length, void *spi_device) {
  if (buffer_length < 1) {
    return 0;
  }
  uint8_t *rx_buf = malloc(buffer_length + 1);
  uint8_t *tx_buf = malloc(buffer_length + 1);
  if (rx_buf == NULL || tx_buf == NULL) {
    return ENOMEM;
  }
  memset(tx_buf, 0, buffer_length + 1); // just in case
  tx_buf[0] = ((uint8_t)reg & 0x7F);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
  int code = HAL_SPI_TransmitReceive(spi_device, tx_buf, rx_buf, buffer_length + 1, SPI_TIMEOUT);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
  if (code == 0) {
    // shift 1 byte to the right
    memcpy(buffer, rx_buf + 1, buffer_length);
  }
  free(rx_buf);
  free(tx_buf);
  return 0;
}

int sx127x_spi_write_register(int reg, const uint8_t *data, size_t data_length, void *spi_device) {
  if (data_length == 0 || data_length > 4) {
    return -1;
  }
  uint8_t tmp[5] = {0};
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, data, data_length);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
  int code = HAL_SPI_Transmit(spi_device, tmp, data_length + 1, SPI_TIMEOUT);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
  return code;
}

int sx127x_spi_write_buffer(int reg, const uint8_t *buffer, size_t buffer_length, void *spi_device) {
  uint8_t *tmp = malloc(buffer_length + 1);
  if (tmp == NULL) {
    return ENOMEM;
  }
  tmp[0] = reg | 0x80;
  memcpy(tmp + 1, buffer, buffer_length);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
  int code = HAL_SPI_Transmit(spi_device, tmp, buffer_length + 1, SPI_TIMEOUT);
  HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
  free(tmp);
  return code;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
