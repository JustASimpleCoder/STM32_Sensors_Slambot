//
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
//
//#include "main.h"
//#include <string.h>
//#include <stdio.h>
//#include <inttypes.h>
//#include <stdbool.h>
//
//I2C_HandleTypeDef hi2c1;
//I2C_HandleTypeDef hi2c2;
//
//TIM_HandleTypeDef htim2;
//
//UART_HandleTypeDef huart4;
//UART_HandleTypeDef huart2;
//
//#define NUM_ENCODERS 4
//#define MAX_TICK_COUNT 65353
//#define ACCEL_SCALE 16384.0
//#define GYRO_SCALE  131.0
//
//static const uint8_t MPU9250_ADDR  = 0x68 << 1;
//static const uint8_t REG_TEMP = 0x00;
//static const uint8_t WHO_AM_I_REG = 0x75;
//static const uint8_t PWR_MGMT_1_REG = 0x6B;
//static const uint8_t ACCEL_XOUT_H = 0x3B;
////static const uint8_t GYRO_XOUT_H = 0x43;
//
//
//extern uint8_t uart_buf[100];
//extern uint8_t imu_buf[14];
//
//
//uint8_t i2c_buf[100];
//
//HAL_StatusTypeDef ret;
//
//float accel[3], gyro[3];
//void MPU9250_Init(void);
//void MPU9250_read_accel_gyro(float *accel, float *gyro);
//void handle_IMU();
//
//void MPU9250_Init(void)
//{
//    uint8_t check, data;
//
//    // Check WHO_AM_I register
//    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
//    if (check != 0x71)  // 0x71 is the expected response for MPU-9250
//    {
//        char error_msg[] = "MPU9250 not found\r\n";
//        HAL_UART_Transmit(&huart2, (uint8_t *)error_msg, strlen(error_msg), HAL_MAX_DELAY);
//        while (1);  // Stop execution if MPU-9250 is not detected
//    }
//
//    // Wake up the MPU-9250 by clearing sleep mode bit (6) in PWR_MGMT_1
//    data = 0x00;
//    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
//}
//
//
//
//void handle_IMU(){
//	uart_buf[0] = REG_TEMP;
//
//	// verify connection
//	bool connect_imu_success = true;
//	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, i2c_buf, 1, HAL_MAX_DELAY);
//	if(ret != HAL_OK){
//		strcpy((char*)uart_buf, "MPU9250 I2C Transmit Failed in handle_IMU\r\n");
//		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//		connect_imu_success = false;
//	}
//
//	ret = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, i2c_buf, 2, HAL_MAX_DELAY);
//	if(ret != HAL_OK){
//		strcpy((char*)uart_buf, "MPU9250 I2C Receive Failed in handle_IMU\r\n");
//		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//		connect_imu_success = false;
//	}
//
//	//read and print
//	if(connect_imu_success){
//		MPU9250_read_accel_gyro(accel, gyro);
//		sprintf((char*)uart_buf,
//					"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n",
//					accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
//		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//	}
//}
//
///**
// * @brief Read accelerometer and gyroscope data
// * @param accel: Pointer to store accelerometer data (X, Y, Z)
// * @param gyro: Pointer to store gyroscope data (X, Y, Z)
// *
// */
//void MPU9250_read_accel_gyro(float *accel, float *gyro)
//{
//    // Read accelerometer and gyroscope data (14 bytes)
//    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1, imu_buf, 14, HAL_MAX_DELAY);
//
//    // Convert accelerometer raw data to g
//    accel[0] = (int16_t)((imu_buf[0] << 8) | imu_buf[1]) / ACCEL_SCALE;
//    accel[1] = (int16_t)((imu_buf[2] << 8) | imu_buf[3]) / ACCEL_SCALE;
//    accel[2] = (int16_t)((imu_buf[4] << 8) | imu_buf[5]) / ACCEL_SCALE;
//
//    // Convert gyroscope raw data to degrees/second
//    gyro[0] = (int16_t)((imu_buf[8] << 8) | imu_buf[9]) / GYRO_SCALE;
//    gyro[1] = (int16_t)((imu_buf[10] << 8) | imu_buf[11]) / GYRO_SCALE;
//    gyro[2] = (int16_t)((imu_buf[12] << 8) | imu_buf[13]) / GYRO_SCALE;
//
//
//}
//
