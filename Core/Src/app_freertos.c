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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1;

osThreadId SmoothDCStartTaskHandle;
osSemaphoreId i2cSemaphoreHandle;
osThreadId I2CTaskHandle;

uint8_t i2c_rx_buffer[10];
/* USER CODE END Variables */
osThreadId ServoTestTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SmoothDCStartTask(void const * argument);
void I2CHandlerTask(void const * arg);
/* USER CODE END FunctionPrototypes */

void StartServoTestTask(void const * argument);

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
	osSemaphoreDef(osSemaphore);
	i2cSemaphoreHandle = osSemaphoreCreate(osSemaphore(osSemaphore), 1);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of ServoTestTask */
	osThreadDef(ServoTestTask, StartServoTestTask, osPriorityNormal, 0, 128);
	ServoTestTaskHandle = osThreadCreate(osThread(ServoTestTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(SmoothDC, SmoothDCStartTask, osPriorityNormal, 0, 128);
	SmoothDCStartTaskHandle = osThreadCreate(osThread(SmoothDC), NULL);

	osThreadDef(I2CHandle, I2CHandlerTask, osPriorityNormal, 0, 128);
	I2CTaskHandle = osThreadCreate(osThread(I2CHandle), NULL);
	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartServoTestTask */
/**
 * @brief  Function implementing the ServoTestTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartServoTestTask */
void StartServoTestTask(void const * argument) {

	/* USER CODE BEGIN StartServoTestTask */
	/* Infinite loop */
	for (;;) {
//		for (uint8_t j = 0; j < 10; j++) {
//			for (uint8_t i = 0; i < 180; i++) {
//				S_setPosition(j, i);
//				osDelay(10);
//			}
//		}
//		for (uint8_t j = 9; j > 0; j--) {
//			for (uint8_t i = 180; i > 0; i--) {
//				S_setPosition(j, i);
//				osDelay(10);
//			}
//		}
		osDelay(1000);
	}
	/* USER CODE END StartServoTestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void SmoothDCStartTask(void const * argument) {
	for (;;) {
//		M_move(MOVE_FORWARD, 0);
//		osDelay(1000);
//		M_move(MOVE_STOP, 0);
//		osDelay(1000);
//		M_move(MOVE_REVERSE, 0);
//		osDelay(1000);
//		M_move(MOVE_STOP, 0);
		osDelay(1000);
	}
}

void I2CHandlerTask(void const * arg) {
	HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buffer, 10);
	while (1) {
		if (i2cSemaphoreHandle != NULL) {
			if (osSemaphoreWait(i2cSemaphoreHandle, 0) == osOK) {
				uint32_t temp;
				switch (i2c_rx_buffer[0]) {
				case COMM_MOVE:
					temp = (i2c_rx_buffer[2] << 8) | i2c_rx_buffer[3];
					M_move(i2c_rx_buffer[1], temp);
					break;
				case COMM_SET_SERVOS_POS:
					S_setPosition(i2c_rx_buffer[1], i2c_rx_buffer[2]);
					break;
				default:
					break;
				}
				HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buffer, 10);
			}
		}
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2CHandle) {
	if (hi2c1.Instance == I2CHandle->Instance)
		osSemaphoreRelease(i2cSemaphoreHandle);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
