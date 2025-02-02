/*
 * constants.c
 *
 *  Created on: Feb 1, 2025
 *      Author: Jacob Cohen
 */
#include "constants.h"

uint8_t imu_buf[14];
uint8_t uart_buf[100];
uint8_t i2c_buf[100];

Encoder* encoders[NUM_ENCODERS] = {&right_back, &right_front, &left_front, &left_back};


extern Encoder right_back = {0, 0, BACKWARD, BACK, Encoder_RB_Input_GPIO_Port, Encoder_RB_Input_Pin};
extern Encoder right_front = {0, 0, FORWARD, FRONT, Encoder_RF_Input_GPIO_Port, Encoder_RF_Input_Pin};
extern Encoder left_front = {0, 0, FORWARD, FRONT, Encoder_LF_Input_GPIO_Port,Encoder_LF_Input_Pin};
extern Encoder left_back = {0, 0, BACKWARD, BACK, Encoder_LB_Input_GPIO_Port, Encoder_LB_Input_Pin};

