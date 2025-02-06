/*
 * constants.h
 *
 *  Created on: Feb 1, 2025
 *      Author: Jacob Cohen
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#include <stdint.h>
#include "gpio.h"

#define NUM_ENCODERS 4
#define MAX_TICK_COUNT 65353
#define ACCEL_SCALE 16384.0
#define GYRO_SCALE  131.0
#define MAG_SCALE  0.15;




static const uint8_t LIDAR_ADDR  = 0x10 << 1;
// static const uint8_t DIST_OUT  = 0x21 << 1;
//static const uint8_t DIST_LOW  = 0x00 << 1;




//test change

static const uint8_t MPU9250_ADDR  = 0x68 << 1;
static const uint8_t REG_TEMP = 0x00;
static const uint8_t WHO_AM_I_REG = 0x75;
static const uint8_t PWR_MGMT_1_REG = 0x6B;
static const uint8_t ACCEL_XOUT_H = 0x3B;

static const uint8_t AK8963_ADDR = 0x0C << 1;
static const uint8_t AK8963_XOUT_L = 0x03;
static const uint8_t INT_PIN_CFG = 0x37;
static const uint8_t AK8963_CNTL1 = 0x0A;
static const uint8_t AK8963_CNTL2 = 0x0B;


static const uint8_t IMU_SAMPLE_RATE = 100;

extern volatile float asax;
extern volatile float asay;
extern volatile float asaz;

typedef enum{
    STOP = 'x',
    MOVE_FORWARD = 'w',
    MOVE_BACKWARD = 's',
    MOVE_LEFT = 'l',
    MOVE_RIGHT = 'r',
    ROTATE_LEFT = 'a',
    ROTATE_RIGHT = 'd',
    DIAG_FORWARD_RIGHT = 'e',
    DIAG_BACKWARD_RIGHT = 'c',
    DIAG_FORWARD_LEFT = 'q',
    DIAG_BACKWARD_LEFT = 'z',
    FASTER = '+',
    SLOWER = '-',
    INVALID = '?'
}RobotMovement;
//
//
typedef enum{
    FORWARD = 1,
    BACKWARD = -1,
	STOPPED = 100
}Direction;
//
//typedef struct {
//    Direction left_front;
//    Direction right_front;
//    Direction left_back;
//    Direction right_back;
//} MovementDirections;
//
//// Define static movement directions
//const MovementDirections DIR_MOVE_FORWARD = {FORWARD, FORWARD, BACKWARD, BACKWARD};
//const MovementDirections DIR_MOVE_BACKWARD = {BACKWARD, BACKWARD, FORWARD, FORWARD};
//const MovementDirections DIR_MOVE_LEFT = {BACKWARD, FORWARD, BACKWARD, FORWARD};
//const MovementDirections DIR_MOVE_RIGHT = {FORWARD, BACKWARD, FORWARD, BACKWARD};
//const MovementDirections DIR_ROTATE_LEFT = {BACKWARD, FORWARD, BACKWARD, FORWARD};
//const MovementDirections DIR_ROTATE_RIGHT = {FORWARD, BACKWARD, FORWARD, BACKWARD};
//const MovementDirections DIR_DIAG_FORWARD_RIGHT = {FORWARD, STOPPED, FORWARD, STOPPED};
//const MovementDirections DIR_DIAG_BACKWARD_RIGHT = {STOPPED, BACKWARD, STOPPED, BACKWARD};
//const MovementDirections DIR_DIAG_FORWARD_LEFT = {STOPPED, FORWARD, STOPPED, FORWARD};
//const MovementDirections DIR_DIAG_BACKWARD_LEFT = {BACKWARD, STOPPED, BACKWARD, STOPPED};


//Direction* test = (Direction*)mallac(sizeof(Direction)*4);
static const Direction DIR_MOVE_FORWARD[] = {BACKWARD, FORWARD, FORWARD, BACKWARD};
static const Direction DIR_MOVE_BACKWARD[] = {FORWARD, BACKWARD, BACKWARD, FORWARD};
static const Direction DIR_MOVE_LEFT[] = {BACKWARD, FORWARD, BACKWARD, FORWARD};
static const Direction DIR_MOVE_RIGHT[] = {FORWARD, BACKWARD, FORWARD, BACKWARD};
static const Direction DIR_ROTATE_LEFT[] = {BACKWARD, FORWARD, BACKWARD, FORWARD};
static const Direction DIR_ROTATE_RIGHT[] = {FORWARD, BACKWARD, FORWARD, BACKWARD};
static const Direction DIR_DIAG_FORWARD_RIGHT[] = {FORWARD, STOPPED, FORWARD, STOPPED};
static const Direction DIR_DIAG_BACKWARD_RIGHT[] = {STOPPED, BACKWARD, STOPPED, BACKWARD};
static const Direction DIR_DIAG_FORWARD_LEFT[] = {STOPPED, FORWARD, STOPPED, FORWARD};
static const Direction DIR_DIAG_BACKWARD_LEFT[] = {BACKWARD, STOPPED, BACKWARD, STOPPED};
//
typedef enum{
    BACK = 0,
    FRONT = 1,
}WheelPosition;
//
////volatile int32_t encoder_ticks_wheel1 = 0;
////volatile uint8_t prev_encoder1_state = 0;
//
typedef struct{
	volatile int32_t tick_count;
	volatile uint8_t prev_state;
	Direction wheel_spin;
	WheelPosition wheel_pos;
	GPIO_TypeDef* gpio_port;
	uint16_t gpio_pin;
}Encoder;
//
//
////forward wheel and back spin in opposite direction to achieve forward/backward
//// TODO: when moving direction right/left or diagonal, how to calcualte odometry?
extern Encoder right_back; //  = {0, 0, BACKWARD, BACK, Encoder_RB_Input_GPIO_Port, Encoder_RB_Input_Pin};
extern Encoder right_front; // = {0, 0, FORWARD, FRONT, Encoder_RF_Input_GPIO_Port, Encoder_RF_Input_Pin};
extern Encoder left_front; // = {0, 0, FORWARD, FRONT, Encoder_LF_Input_GPIO_Port,Encoder_LF_Input_Pin};
extern Encoder left_back; // = {0, 0, BACKWARD, BACK, Encoder_LB_Input_GPIO_Port, Encoder_LB_Input_Pin};

extern Encoder* encoders[NUM_ENCODERS];//  = {&right_back, &right_front, &left_front, &left_back};
//
extern uint8_t imu_buf[14];
extern uint8_t uart_buf[100];
extern uint8_t i2c_buf[100];

extern uint8_t mag_buf[7];
//
//uint8_t uart_lidar_buf[10];
extern uint8_t i2c_lidar_buf[10];
//
//HAL_StatusTypeDef ret;
//HAL_StatusTypeDef ret_lidar;
//float accel[3], gyro[3];


#endif /* INC_CONSTANTS_H_ */
