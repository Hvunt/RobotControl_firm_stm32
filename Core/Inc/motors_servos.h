/*
 * motors.h
 *
 *  Created on: 18 февр. 2019 г.
 *      Author: hvunt
 */

#ifndef INC_MOTORS_SERVOS_H_
#define INC_MOTORS_SERVOS_H_

#include "stm32g0xx_hal.h"
//#include "cmsis_os.h"
#include "tim.h"
#include <math.h>

#include "encoder.h"

typedef struct {
	//[0:3] - used terminals M1 = 0, M2, etc. Set bits (M1 = 0s bit, M2 = 1st bit, etc.)
	//[4:8] - position of motor. 1 - left, 0 - right. For example: if fourth bit is 1 then M1 is left motor
	uint8_t dc_motors_ports;
	//optical / magnet (??) or just rotation counter
	uint8_t encoder_types;
	// First port = 1, second port = 2 etc.
	uint16_t servo_ports;
	//Battery voltage for battery charge monitor
	uint8_t battery_v;
	//Type of the robot: walked, wheeled, combined
	uint8_t robot_type; //??
	//Type of the wheels: Mecanum, Omni, Normal

	//PID regulator. 1 - DC-motors will use PID, 0 - no.
	uint8_t usePID;
	//PID coefficients
	double PID_kp, PID_ki, PID_kd;
}driver_t;

/*
 * Pins definition
 */

//MOTOR PINS
////Front motors

//Front motors
#define MOTOR_PORT_DRV1 		GPIOA
#define MOTOR_PIN_DRV1 			GPIO_PIN_4
#define MOTOR_PORT_DRV2 		GPIOB
#define MOTOR_PIN_DRV2 			GPIO_PIN_2
#define MOTOR_PORT_DRV3 		GPIOB
#define MOTOR_PIN_DRV3 			GPIO_PIN_13
#define MOTOR_PORT_DRV4 		GPIOB
#define MOTOR_PIN_DRV4			GPIO_PIN_14
#define MOTOR_PORT_STB1 		GPIOB
#define MOTOR_PIN_STB1 			GPIO_PIN_12

//Rear motors
#define MOTOR_PORT_DRV5 		GPIOB
#define MOTOR_PIN_DRV5 			GPIO_PIN_15
#define MOTOR_PORT_DRV6 		GPIOC
#define MOTOR_PIN_DRV6 			GPIO_PIN_6
#define MOTOR_PORT_DRV7 		GPIOA
#define MOTOR_PIN_DRV7 			GPIO_PIN_12
#define MOTOR_PORT_DRV8 		GPIOA
#define MOTOR_PIN_DRV8 			GPIO_PIN_15
#define MOTOR_PORT_STB2 		GPIOC
#define MOTOR_PIN_STB2 			GPIO_PIN_7

////////////////////////////////////////////////////////////////

/*
 * Terminals definitions
 */
#define TERMINAL_ONE		 0b00000001
#define TERMINAL_TWO		 0b00000010
#define TERMINAL_THREE		 0b00000100
#define TERMINAL_FOUR		 0b00001000

////////////////////////////////////////////////////////////////

/*
 * Commands definitions
 */
enum
{
	COMM_INIT = 0xF0,
	COMM_ERROR = 0xD0,
	COMM_MOVE = 0xC0,
	COMM_SET_SERVOS_POS,
	COMM_SET_SETTINGS,
	COMM_GET_STATUS,
	COMM_GET_SPEED,
	COMM_GET_SERVOS_POS,
	COMM_GET_SETTINGS,
	COMM_GET_SENSORS
};
//SET
//#define COMM_MOVE					0x50
//#define COMM_SET_SERVOS_POS			0x80
//
////GET
//#define COMM_GET_SPEED				0x70
//#define COMM_GET_SERVOS_POS 		0xB0

enum {
	MOVE_FORWARD = 0x02,
	MOVE_REVERSE = 0x04,
	MOVE_LEFT = 0x20,
	MOVE_RIGHT = 0x22,
	MOVE_STOP = 0x44
};

////////////////////////////////////////////////////////////////

/*
 * Functions prototypes
 */
void M_init(driver_t *driver);
void M_move(uint8_t direction, uint32_t distance);
void M_rotate(uint8_t direction, uint16_t angle);

// speed - rotations per second
uint16_t M_getSpeed(void);
void M_setSpeed(uint16_t speed);

void S_init(driver_t *driver);
void S_setPosition(uint8_t servos, uint8_t angle);

////////////////////////////////////////////////////////////////

/*
 * TEMPLATES
 */

//MOTOR PINS
//#define MOTOR_PORT_F_L1 GPIO
//#define MOTOR_PIN_F_L1 GPIO_PIN_
//#define MOTOR_PORT_F_L2 GPIO
//#define MOTOR_PIN_F_L2 GPIO_PIN_
//#define MOTOR_PORT_F_R1 GPIO
//#define MOTOR_PIN_F_R1 GPIO_PIN_
//#define MOTOR_PORT_F_R2 GPIO
//#define MOTOR_PIN_F_R2 GPIO_PIN_
//#define MOTOR_PORT_STB_1 GPIO
//#define MOTOR_PIN_STB_1 GPIO_PIN_
//
//#define MOTOR_PORT_R_L1 GPIO
//#define MOTOR_PIN_R_L1 GPIO_PIN_
//#define MOTOR_PORT_R_L2 GPIO
//#define MOTOR_PIN_R_L2 GPIO_PIN_
//#define MOTOR_PORT_R_R1 GPIO
//#define MOTOR_PIN_R_R1 GPIO_PIN_
//#define MOTOR_PORT_R_R2 GPIO
//#define MOTOR_PIN_R_R2 GPIO_PIN_
//#define MOTOR_PORT_STB_2 GPIO
//#define MOTOR_PIN_STB_2 GPIO_PIN_

//#define MOTOR_PORT_F_L1
//#define MOTOR_PIN_F_L1
//#define MOTOR_PORT_F_L2
//#define MOTOR_PIN_F_L2
//#define MOTOR_PORT_F_R1
//#define MOTOR_PIN_F_R1
//#define MOTOR_PORT_F_R2
//#define MOTOR_PIN_F_R2
//
//#define MOTOR_PORT_R_L1
//#define MOTOR_PIN_R_L1
//#define MOTOR_PORT_R_L2
//#define MOTOR_PIN_R_L2
//#define MOTOR_PORT_R_R1
//#define MOTOR_PIN_R_R1
//#define MOTOR_PORT_R_R2
//#define MOTOR_PIN_R_R2

////////////////////////////////////////////////////////////////

#endif /* INC_MOTORS_SERVOS_H_ */
