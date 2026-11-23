/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	  BMP280_OK = 0,
	  BMP280_ERR_COMM,
	  BMP280_ERR_WRONG_ID
  } BMP280_Status_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
BMP280_CalibData_t calib;
int32_t t_fine;
uint8_t motion_flag = 0;
SemaphoreHandle_t motionMutexHandle = NULL;
QueueHandle_t sensorQueueHandle = NULL;

TaskHandle_t envTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;
TaskHandle_t commTaskHandle = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
BMP280_Status_t BMP280_Init();
void BMP280_Config();
uint32_t bmp280_compensate_P_int64(int32_t adc_P);
int32_t bmp280_compensate_T_int32(int32_t adc_T);
BaseType_t Read_BMP280_Data(System_Data_t *data);

BaseType_t AHT20_Init();
BaseType_t Read_AHT20_Data(System_Data_t *data);

BaseType_t Read_Sensors_Data(System_Data_t *data);
uint8_t Calculate_CRC8(const uint8_t *data, size_t length);

void StartEnvTask(void *argument);
void StartMotionTask(void *argument);
void StartCommTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  sensorQueueHandle = xQueueCreate(4, sizeof(System_Data_t));
  if (sensorQueueHandle == NULL)
  {
	  Error_Handler();
  }
  motionMutexHandle = xSemaphoreCreateMutex();
  if (motionMutexHandle == NULL)
  {
	  Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if (xTaskCreate(StartEnvTask, "EnvTask", 512, NULL, tskIDLE_PRIORITY + 2, &envTaskHandle) != pdPASS)
  {
	  Error_Handler();
  }
  if (xTaskCreate(StartCommTask, "CommTask", 512, NULL, tskIDLE_PRIORITY + 4, &commTaskHandle) != pdPASS)
  {
	  Error_Handler();
  }
  if (xTaskCreate(StartMotionTask, "MotionTask", 256, NULL, tskIDLE_PRIORITY + 3, &motionTaskHandle) != pdPASS)
  {
	  Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)calib.dig_T1<<1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)calib.dig_T1)) * ((adc_T>>4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3)>>8) + ((var1 * (int64_t)calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib.dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7)<<4);
    return (uint32_t)p;
}

BMP280_Status_t BMP280_Init()
{
  uint8_t chip_id;

  if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100) != HAL_OK)
  {
	  return BMP280_ERR_COMM;
  }
  if (chip_id != BMP280_ID)
  {
	  return BMP280_ERR_WRONG_ID;
  }
  return BMP280_OK;
}

uint8_t Calculate_CRC8(const uint8_t *data, size_t length)
{
	if (data == NULL || length == 0)
	{
		return 0x00;
	}

	uint8_t crc = 0x00;

	for(size_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (size_t j = 0; j < 8; j++)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ 0x07;
			} else
			{
				crc = crc << 1;
			}
		}
	}
	return crc;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(commTaskHandle, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void BMP280_Config()
{
	uint8_t ctrl_meas = (0b010 << 5) | (0b001 << 2) | (0b11);
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, 100);

	uint8_t config = (0b110 << 5) | (0b001 << 2);
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, 0xF5, I2C_MEMADD_SIZE_8BIT, &config, 1, 100);

	uint8_t raw_calib[24];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, raw_calib, 24, 100);
	memcpy(&calib, raw_calib, 24);
}

BaseType_t AHT20_Init()
{
	uint8_t status_byte = 0;
	vTaskDelay(pdMS_TO_TICKS(40));
	if (HAL_I2C_Master_Receive(&hi2c1, (0x38 << 1), &status_byte, 1, 100) != HAL_OK) return pdFALSE;
	if (!(status_byte & 0x8))
	{
		uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
		if (HAL_I2C_Master_Transmit(&hi2c1, (0x38 << 1), init_cmd, 3, 100) != HAL_OK) return pdFALSE;
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return pdTRUE;
}

BaseType_t Read_BMP280_Data(System_Data_t *data)
{
	uint8_t raw_bmp280_data[6];
	int32_t adc_T;
	int32_t adc_P;

	if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, raw_bmp280_data, 6, 100) == HAL_OK)
	{
		adc_P = (int32_t)((((uint32_t)raw_bmp280_data[0]) << 12) | (((uint32_t)raw_bmp280_data[1]) << 4) | (((uint32_t)raw_bmp280_data[2]) >> 4));
		adc_T = (int32_t)((((uint32_t)raw_bmp280_data[3]) << 12) | (((uint32_t)raw_bmp280_data[4]) << 4) | (((uint32_t)raw_bmp280_data[5]) >> 4));
		data->temp = (int16_t)bmp280_compensate_T_int32(adc_T);
		data->press = bmp280_compensate_P_int64(adc_P);
		printf("Temp: %" PRId16 ", Press: %" PRIu32 "\r\n", data->temp, data->press);
		return pdTRUE;
	}
	return pdFALSE;
}

BaseType_t Read_AHT20_Data(System_Data_t *data)
{
	uint8_t raw_aht20_data[4];
	int32_t s_RH;

	uint8_t trig_cmd[3] = {0xAC, 0x33, 0x00};
	if (HAL_I2C_Master_Transmit(&hi2c1, (0x38 << 1), trig_cmd, 3, 100) != HAL_OK) return pdFALSE;
	vTaskDelay(pdMS_TO_TICKS(80));

	if (HAL_I2C_Master_Receive(&hi2c1, (0x38 << 1), raw_aht20_data, 4, 100) != HAL_OK) return pdFALSE;
	if (raw_aht20_data[0] & 0x80) return pdFALSE;

	s_RH = (int32_t)((((uint32_t)raw_aht20_data[1]) << 12) | (((uint32_t)raw_aht20_data[2]) << 4) | (((uint32_t)raw_aht20_data[3]) >> 4));
	data->hum = (uint8_t)((s_RH * 100) >> 20);

	return pdTRUE;
}

BaseType_t Read_Sensors_Data(System_Data_t *data)
{
	return (Read_AHT20_Data(data) && Read_BMP280_Data(data));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(motionTaskHandle, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void StartEnvTask(void *argument)
{

	while(AHT20_Init() != pdTRUE)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	while(BMP280_Init() != BMP280_OK)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	BMP280_Config();

	System_Data_t tx_data = {0};
	for(;;)
	{
		if (Read_Sensors_Data(&tx_data) == pdTRUE)
		{
			if (xSemaphoreTake(motionMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE)
			{
				if (motion_flag == 1 || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET)
				{
					tx_data.motion = 1;
				}
				else
				{
					tx_data.motion = 0;
				}
				motion_flag = 0;
				xSemaphoreGive(motionMutexHandle);
			}
			xQueueSend(sensorQueueHandle, &tx_data, pdMS_TO_TICKS(10));
		}
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void StartMotionTask(void *argument)
{
  for(;;)
  {
	  if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
	  {
		  if (xSemaphoreTake(motionMutexHandle, pdMS_TO_TICKS(100)) == pdTRUE)
		  {
			  motion_flag = 1;
			  xSemaphoreGive(motionMutexHandle);
		  }
	  }
  }
}

void StartCommTask(void *argument)
{
	static Sensor_Packet_t packet;
	System_Data_t rx_data;

	packet.header[0] = 0xAA;
	packet.header[1] = 0x55;
	for(;;)
	{
		if (xQueueReceive(sensorQueueHandle, &rx_data, portMAX_DELAY) == pdTRUE)
		{
			packet.temp = rx_data.temp;
			packet.press = rx_data.press;
			packet.hum = rx_data.hum;
			packet.motion = (rx_data.motion == 1) ? 0xE0 : 0x1F;
			packet.crc = Calculate_CRC8((uint8_t *)&packet.temp, 8);
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&packet, sizeof(packet));

			uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
			if (notified == 0)
			{
				HAL_UART_AbortTransmit(&huart3);
			}
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelete(NULL);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
