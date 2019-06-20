/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "motors_servos.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_SLAVE_PACKET_LENGTH 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1;

osThreadId SmoothDCStartTaskHandle;
//osSemaphoreId i2cSemaphoreHandle;
osThreadId I2CTaskHandle;
osThreadId EXTIHandlerTaskHandle;

static uint8_t i2c_rx_buffer[I2C_SLAVE_PACKET_LENGTH];
static uint8_t i2c_ret_code = SM_OK;

//volatile uint8_t old_data_flag = 0;
volatile uint32_t error_counter;
/* USER CODE END Variables */
osThreadId mainTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SmoothDCStartTask(void const * argument);
void I2CHandlerTask(void const * arg);
void EXTIHandlerTask(void const *arg);

static void free_buffer(uint8_t *buffer, uint16_t length);
volatile uint32_t encoder_irq_counter = 0;
/* USER CODE END FunctionPrototypes */

void StartMainTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//	osSemaphoreDef(osSemaphore);
//	i2cSemaphoreHandle = osSemaphoreCreate(osSemaphore(osSemaphore), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 128);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
//	osThreadDef(SmoothDC, SmoothDCStartTask, osPriorityNormal, 0, 128);
//	SmoothDCStartTaskHandle = osThreadCreate(osThread(SmoothDC), NULL);
	osThreadDef(I2CHandle, I2CHandlerTask, osPriorityNormal, 0, 512);
	I2CTaskHandle = osThreadCreate(osThread(I2CHandle), NULL);

	osThreadDef(EXTIHandle, EXTIHandlerTask, osPriorityAboveNormal, 0, 128);
	EXTIHandlerTaskHandle = osThreadCreate(osThread(EXTIHandle), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
 * @brief  Function implementing the mainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{

  /* USER CODE BEGIN StartMainTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END StartMainTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void SmoothDCStartTask(void const * argument) {
//	for (;;) {
////		M_move(MOVE_RIGHT, 45);
////		osDelay(5000);
////		M_move(MOVE_STOP, 0);
////		osDelay(1000);
////		M_move(MOVE_REVERSE, 0);
////		osDelay(1000);
////		M_move(MOVE_STOP, 0);
////		osDelay(1000 / portTICK_RATE_MS);
//	}
//	vTaskDelete(NULL);
//}
void I2CHandlerTask(void const * arg) {
//	HAL_I2C_EnableListen_IT(&hi2c1);
	osEvent event;
	while (1) {
		event = osSignalWait(1, osWaitForever);
		if (event.value.signals == 1) {
			HAL_I2C_EnableListen_IT(&hi2c1);
			if (i2c_rx_buffer[0] == COMM_INIT) {
				uint32_t temp;
				switch (i2c_rx_buffer[1]) {
				case COMM_MOVE:
					i2c_ret_code = SM_BUSY;
					temp = (i2c_rx_buffer[3] << 8) | i2c_rx_buffer[4];
					M_move(i2c_rx_buffer[2], temp);
					i2c_ret_code = SM_OK;
					break;
				case COMM_SET_SERVOS_POS:
					i2c_ret_code = SM_BUSY;
					S_setPosition(i2c_rx_buffer[2], i2c_rx_buffer[3]);
					i2c_ret_code = SM_OK;
					break;
				case COMM_GET_STATUS:
//				HAL_I2C_Slave_Transmit(&hi2c1, &i2c_ret_code, 1, 0x1000);
					break;
				case COMM_ERROR:
					i2c_ret_code = SM_ERROR;
					break;
				}
			}
		}
	}
	vTaskDelete(NULL);
}

void EXTIHandlerTask(void const *arg) {
	osEvent event;
	while (1) {
		event = osSignalWait(1, osWaitForever);
		if (event.value.signals == 1)
			encoder_irq_counter++;
	}
	vTaskDelete(NULL);
}

//static void free_buffer(uint8_t *buffer, uint16_t length) {
//	for (uint8_t i = 0; i < length; i++)
//		buffer[i] = 0;
//}

//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2CHandle) {
//	if (hi2c1.Instance == I2CHandle->Instance) {
//		uint8_t old_position = 0;
//		for (uint8_t i = 0; i < I2C_SLAVE_PACKET_LENGTH; i++) {
//			if (i2c_rx_buffer[i] == COMM_INIT) {
//				while (old_position != i) {
//					i2c_rx_buffer[old_position] = i2c_rx_buffer[i];
//					old_position += 1;
//				}
//				break;
//			}
//		}
//	}
//
//}
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle){
//	old_data_flag = 1;
//}
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if (i2c_rx_buffer[0] == COMM_INIT) {
//		switch (i2c_rx_buffer[1]) {
//		case COMM_GET_STATUS:
//			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &i2c_ret_code, 1,
//					I2C_LAST_FRAME);
//			break;
//		default:
//			break;
//		}
//	}
//}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_7) {
		osSignalSet(EXTIHandlerTaskHandle, 1);
	}
}

uint8_t trans_dir = 0;
uint16_t slave_receive_index = 0;
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	if (AddrMatchCode == (uint16_t) (0x1C << 1)) {
		trans_dir = TransferDirection;
		if (trans_dir == I2C_DIRECTION_TRANSMIT) {
			HAL_I2C_Slave_Seq_Receive_DMA(&hi2c1, i2c_rx_buffer, 255,
					I2C_FIRST_FRAME);
//			slave_receive_index++;
		} else {
			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &i2c_ret_code, 1,
			I2C_LAST_FRAME);
			slave_receive_index = 0;
		}
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef * hi2c) {
	osSignalSet(I2CTaskHandle, 2);
//	HAL_I2C_EnableListen_IT(&hi2c1);
//	if (i2c_rx_buffer[0] == COMM_INIT) {
//		switch (i2c_rx_buffer[1]) {
//		case COMM_GET_STATUS:
//			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &i2c_ret_code, 1,
//			I2C_LAST_FRAME);
//			break;
//		default:
//			break;
//		}
//	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
