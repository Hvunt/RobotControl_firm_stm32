/*
 * motors.c
 *
 *  Created on: 18 февр. 2019 г.
 *      Author: hvunt
 */

#include "motors_servos.h"

static uint16_t angle_to_time(uint8_t angle);

static void _m_move_d(uint8_t port, uint8_t direction);
static void _m_move_r(uint8_t port, uint8_t direction);
static void _m_move_s(uint8_t port);
static void _m_move_smooth(void);

driver_t *settings;

void M_init(driver_t *driver) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	settings = driver;
}

void S_init(driver_t *driver) {
	settings = driver;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void M_move(uint8_t direction, uint32_t distance) {

	switch (direction) {
	case MOVE_FORWARD:
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, SET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, SET);
		_m_move_d(settings->dc_motors_ports, SET);
		_m_move_smooth();
		break;
	case MOVE_REVERSE:
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, SET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, SET);
		_m_move_d(settings->dc_motors_ports, RESET);
		_m_move_smooth();
		break;
	case MOVE_STOP:
	default:
		_m_move_s(settings->dc_motors_ports);
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, RESET);
		break;
	}

}

void M_rotate(uint8_t direction, uint16_t angle) {
	htim1.Instance->CCR1 = 1000;
	htim1.Instance->CCR2 = 1000;
	htim1.Instance->CCR3 = 1000;
	htim1.Instance->CCR4 = 1000;

	switch (direction) {
	case MOVE_RIGHT:
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, SET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, SET);
		_m_move_r(settings->dc_motors_ports, RESET);
		break;
	case MOVE_LEFT:
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, SET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, SET);
		_m_move_r(settings->dc_motors_ports, SET);
		break;
	case MOVE_STOP:
	default:
		_m_move_s(settings->dc_motors_ports);
		HAL_GPIO_WritePin(MOTOR_PORT_STB1, MOTOR_PIN_STB1, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_STB2, MOTOR_PIN_STB2, RESET);
		break;
	}
}

static void _m_move_d(uint8_t port, uint8_t direction) {
	if (port & TERMINAL_ONE) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV1, MOTOR_PIN_DRV1, direction);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV2, MOTOR_PIN_DRV2, !direction);
	}
	if (port & TERMINAL_TWO) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV3, MOTOR_PIN_DRV3, direction);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV4, MOTOR_PIN_DRV4, !direction);
	}
	if (port & TERMINAL_THREE) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV5, MOTOR_PIN_DRV5, direction);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV6, MOTOR_PIN_DRV6, !direction);
	}
	if (port & TERMINAL_FOUR) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV7, MOTOR_PIN_DRV7, direction);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV8, MOTOR_PIN_DRV8, !direction);
	}
}

static void _m_move_r(uint8_t port, uint8_t direction) {
	if (port & TERMINAL_ONE) {
		uint8_t temp = (port >> (TERMINAL_ONE + 3));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV1, MOTOR_PIN_DRV1,
				(temp ? direction : !direction));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV2, MOTOR_PIN_DRV2,
				(temp ? !direction : direction));
	}
	if (port & TERMINAL_TWO) {
		uint8_t temp = (port >> (TERMINAL_TWO + 3));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV3, MOTOR_PIN_DRV3,
				(temp ? direction : !direction));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV4, MOTOR_PIN_DRV4,
				(temp ? !direction : direction));
	}
	if (port & TERMINAL_THREE) {
		uint8_t temp = (port >> (TERMINAL_THREE + 3));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV5, MOTOR_PIN_DRV5,
				(temp ? direction : !direction));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV6, MOTOR_PIN_DRV6,
				(temp ? !direction : direction));
	}
	if (port & TERMINAL_FOUR) {
		uint8_t temp = (port >> (TERMINAL_FOUR + 3));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV7, MOTOR_PIN_DRV7,
				(temp ? direction : !direction));
		HAL_GPIO_WritePin(MOTOR_PORT_DRV8, MOTOR_PIN_DRV8,
				(temp ? !direction : direction));
	}
}

static void _m_move_smooth(void) {
	uint16_t speed_value = 0;
	while (speed_value < 1000) {
		htim1.Instance->CCR1 = speed_value;
		htim1.Instance->CCR2 = speed_value;
		htim1.Instance->CCR3 = speed_value;
		htim1.Instance->CCR4 = speed_value;
		speed_value += 10;
		osDelay((1000 - speed_value) / 200 );
	}
}

static void _m_move_s(uint8_t port) {
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = 0;
	if (port & TERMINAL_ONE) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV1, MOTOR_PIN_DRV1, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV2, MOTOR_PIN_DRV2, RESET);
	}
	if (port & TERMINAL_TWO) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV3, MOTOR_PIN_DRV3, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV4, MOTOR_PIN_DRV4, RESET);
	}
	if (port & TERMINAL_THREE) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV5, MOTOR_PIN_DRV5, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV6, MOTOR_PIN_DRV6, RESET);
	}
	if (port & TERMINAL_FOUR) {
		HAL_GPIO_WritePin(MOTOR_PORT_DRV7, MOTOR_PIN_DRV7, RESET);
		HAL_GPIO_WritePin(MOTOR_PORT_DRV8, MOTOR_PIN_DRV8, RESET);
	}
}

void S_setPosition(uint8_t servos_port, uint8_t angle) {
	uint16_t time = angle_to_time(angle);
	switch (servos_port) {
	case 1:
		htim15.Instance->CCR1 = time;
		break;
	case 2:
		htim15.Instance->CCR2 = time;
		break;
	case 3:
		htim2.Instance->CCR1 = time;
		break;
	case 4:
		htim3.Instance->CCR1 = time;
		break;
	case 5:
		htim3.Instance->CCR2 = time;
		break;
	case 6:
		htim3.Instance->CCR3 = time;
		break;
	case 7:
		htim3.Instance->CCR4 = time;
		break;
	case 8:
		htim2.Instance->CCR3 = time;
		break;
	case 9:
		htim2.Instance->CCR4 = time;
		break;
	default:
		break;
	}
}

static uint16_t angle_to_time(uint8_t angle) {
	return 100 * angle / 18 + 1000;
}
