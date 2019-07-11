/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motors_servos.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SM_memory {
	uint8_t status;
	uint8_t WHO_AM_I;
	uint8_t voltage;
} SM_memory_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_PACKET_LENGTH 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t adc_raw_sensor_data;
volatile uint8_t adc_irq_flag = 0;

driver_t driver_settings;

uint8_t i2c_rx_buffer[I2C_PACKET_LENGTH] = { 0 };
uint8_t i2c_tx_buffer[I2C_PACKET_LENGTH] = { 0 };

SM_memory_t device_memory;

//uint8_t request_code = 0;

volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void StartIT(void);
static void ActivateAndCalibrate_ADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
//	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
//		Error_Handler();
//	}
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_sensor_data, 2);
//	HAL_ADC_Start_IT(&hadc1);
	ActivateAndCalibrate_ADC();

	driver_settings.dc_motors_ports = 0x30
			| ( TERMINAL_ONE | TERMINAL_TWO | TERMINAL_THREE | TERMINAL_FOUR);
	driver_settings.servo_ports = 0x1FF;
	driver_settings.PID_kp = 0.5;
	M_init(&driver_settings);
	S_init(&driver_settings);
	StartIT();

	device_memory.status = SM_OK;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//		M_move(MOVE_LEFT, 180);
//		HAL_Delay(2000);
//		M_move(MOVE_FORWARD, 0);
//		HAL_Delay(2000);
//		M_move(MOVE_LEFT, 0);
//		HAL_Delay(2000);
//		M_move(MOVE_RIGHT, 0);
//		HAL_Delay(2000);
		if (i2c_rx_buffer[0] == COMM_INIT/* && old_data_flag == 1*/) {
			i2c_rx_buffer[0] = 0; // disable loop

			uint32_t temp;
			switch (i2c_rx_buffer[1]) {
			case COMM_MOVE:
				device_memory.status = SM_BUSY;
				temp = (i2c_rx_buffer[3] << 8) | i2c_rx_buffer[4];
				M_move(i2c_rx_buffer[2], temp);
				device_memory.status = SM_OK;
				break;
			case COMM_SET_SERVOS_POS:
				device_memory.status = SM_BUSY;
				S_setPosition(i2c_rx_buffer[2], i2c_rx_buffer[3]);
				device_memory.status = SM_OK;
				break;
			case COMM_GET_STATUS:
				i2c_tx_buffer[0] = device_memory.status;
				break;
			case COMM_GET_SENSORS:
				i2c_tx_buffer[0] = device_memory.status;
				i2c_tx_buffer[1] = device_memory.status;
				i2c_tx_buffer[2] = device_memory.status;
				break;
			case COMM_ERROR:
				device_memory.status = SM_ERROR;
				break;
			}
		}

	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC
			| RCC_PERIPHCLK_TIM15 | RCC_PERIPHCLK_TIM1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
	PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
////	if (hadc->Instance == hadc1.Instance) {
////		adc_irq_flag = 1;
////	}
//}
void Slave_Ready_To_Transmit_Callback(void) {
	/* Send the Byte requested by the Master */
	LL_I2C_TransmitData8(I2C1, i2c_tx_buffer[tx_index++]);
}

void Slave_Reception_Callback(void) {
	/* Read character in Receive Data register.
	 RXNE flag is cleared by reading data in RXDR register */
	i2c_rx_buffer[rx_index++] = LL_I2C_ReceiveData8(I2C1);
}

void Slave_Complete_Callback(void) {
	tx_index = 0;
	rx_index = 0;
}

static void StartIT(void) {
	LL_I2C_EnableIT_ADDR(I2C1);
	LL_I2C_EnableIT_NACK(I2C1);
	LL_I2C_EnableIT_ERR(I2C1);
	LL_I2C_EnableIT_STOP(I2C1);
}

static void ActivateAndCalibrate_ADC(void) {
	volatile uint32_t wait_loop_index = 0U;
	volatile uint32_t backup_setting_adc_dma_transfer = 0U;

	if (LL_ADC_IsEnabled(ADC1) == 0) {
		LL_ADC_EnableInternalRegulator(ADC1);

		//delay for ADC internal regulator stabilization
		wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US
				* (SystemCoreClock / (100000 * 2))) / 10);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
		LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
		LL_ADC_StartCalibration(ADC1);

		while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
			;

		LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
		wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		LL_ADC_Enable(ADC1);
		while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
			;

	}
}
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
////	i2c_ret_code = SM_NOT_RESPOND_ERROR;
////	HAL_I2C_Slave_Transmit(&hi2c1, &i2c_ret_code, 1, 0x1000);
////	HAL_I2C_Slave_Receive_DMA(&hi2c1, i2c_rx_buffer, 10);
//}
//
//uint8_t trans_dir = 0;
//uint16_t slave_receive_index = 0;
//void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
//		uint16_t AddrMatchCode) {
//	if(AddrMatchCode == hi2c1.Init.OwnAddress1){
//		trans_dir = TransferDirection;
//		if(trans_dir == I2C_DIRECTION_TRANSMIT){
//			HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, &i2c_rx_buffer[slave_receive_index], 1, I2C_FIRST_FRAME);
//			slave_receive_index++;
//		} else {
//			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, &i2c_ret_code, 1, I2C_LAST_FRAME);
//			slave_receive_index = 0;
//		}
//	}
//}
//
//void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef * hi2c){
//	HAL_I2C_EnableListen_IT(&hi2c1);
//}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
