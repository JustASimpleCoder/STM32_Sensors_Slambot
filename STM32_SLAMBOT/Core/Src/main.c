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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>


#include "cmsis_os2.h"
#include "constants.h"
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

/* USER CODE BEGIN PV */

//#define NUM_ENCODERS 4
//#define MAX_TICK_COUNT 65353
//#define ACCEL_SCALE 16384.0
//#define GYRO_SCALE  131.0
//
//static const uint8_t LIDAR_ADDR  = 0x10 << 1;
//// static const uint8_t DIST_OUT  = 0x21 << 1;
////static const uint8_t DIST_LOW  = 0x00 << 1;
//
//
//
//
////test change
//
//static const uint8_t MPU9250_ADDR  = 0x68 << 1;
//static const uint8_t REG_TEMP = 0x00;
//static const uint8_t WHO_AM_I_REG = 0x75;
//static const uint8_t PWR_MGMT_1_REG = 0x6B;
//static const uint8_t ACCEL_XOUT_H = 0x3B;
//static const uint8_t GYRO_XOUT_H = 0x43;

//typedef enum{
//    STOP = 'x',
//    MOVE_FORWARD = 'w',
//    MOVE_BACKWARD = 's',
//    MOVE_LEFT = 'l',
//    MOVE_RIGHT = 'r',
//    ROTATE_LEFT = 'a',
//    ROTATE_RIGHT = 'd',
//    DIAG_FORWARD_RIGHT = 'e',
//    DIAG_BACKWARD_RIGHT = 'c',
//    DIAG_FORWARD_LEFT = 'q',
//    DIAG_BACKWARD_LEFT = 'z',
//    FASTER = '+',
//    SLOWER = '-',
//    INVALID = '?'
//}RobotMovement;


//typedef enum{
//    FORWARD = 1,
//    BACKWARD = -1,
//	STOPPED = 100
//}Direction;

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
//Direction DIR_MOVE_FORWARD[] = {BACKWARD, FORWARD, FORWARD, BACKWARD};
//Direction DIR_MOVE_BACKWARD[] = {FORWARD, BACKWARD, BACKWARD, FORWARD};
//Direction DIR_MOVE_LEFT[] = {BACKWARD, FORWARD, BACKWARD, FORWARD};
//Direction DIR_MOVE_RIGHT[] = {FORWARD, BACKWARD, FORWARD, BACKWARD};
//Direction DIR_ROTATE_LEFT[] = {BACKWARD, FORWARD, BACKWARD, FORWARD};
//Direction DIR_ROTATE_RIGHT[] = {FORWARD, BACKWARD, FORWARD, BACKWARD};
//Direction DIR_DIAG_FORWARD_RIGHT[] = {FORWARD, STOPPED, FORWARD, STOPPED};
//Direction DIR_DIAG_BACKWARD_RIGHT[] = {STOPPED, BACKWARD, STOPPED, BACKWARD};
//Direction DIR_DIAG_FORWARD_LEFT[] = {STOPPED, FORWARD, STOPPED, FORWARD};
//Direction DIR_DIAG_BACKWARD_LEFT[] = {BACKWARD, STOPPED, BACKWARD, STOPPED};

//typedef enum{
//    BACK = 0,
//    FRONT = 1,
//}WheelPosition;

//volatile int32_t encoder_ticks_wheel1 = 0;
//volatile uint8_t prev_encoder1_state = 0;

//typedef struct{
//	volatile int32_t tick_count;
//	volatile uint8_t prev_state;
//	Direction wheel_spin;
//	WheelPosition wheel_pos;
//	GPIO_TypeDef* gpio_port;
//	uint16_t gpio_pin;
//}Encoder;


//forward wheel and back spin in opposite direction to achieve forward/backward


//Encoder right_back = {0, 0, BACKWARD, BACK, Encoder_RB_Input_GPIO_Port, Encoder_RB_Input_Pin};
//Encoder right_front = {0, 0, FORWARD, FRONT, Encoder_RF_Input_GPIO_Port, Encoder_RF_Input_Pin};
//Encoder left_front = {0, 0, FORWARD, FRONT, Encoder_LF_Input_GPIO_Port,Encoder_LF_Input_Pin};
//Encoder left_back = {0, 0, BACKWARD, BACK, Encoder_LB_Input_GPIO_Port, Encoder_LB_Input_Pin};

//Encoder* encoders[NUM_ENCODERS] = {&right_back, &right_front, &left_front, &left_back};

//uint8_t imu_buf[14];
//uint8_t uart_buf[100];
//uint8_t i2c_buf[100];

uint8_t uart_lidar_buf[10];
//uint8_t i2c_lidar_buf[10];

HAL_StatusTypeDef ret;
HAL_StatusTypeDef ret_lidar;
float accel[3], gyro[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void MPU9250_Init(void);
void MPU9250_read_accel_gyro(float *accel, float *gyro);
void handle_IMU();
void handle_lidar();
void handle_lidar_i2c();
void handle_encoder();


void update_multiple_directions(const Direction dir[]);
void update_encoder_directon(Encoder * enc, Direction wheel_dir);
void handle_encoder_direction(RobotMovement move);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  strcpy((char*)uart_buf, "Starting Timer2 \r\n");
  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);

  // void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
//  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
//  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

  /* Init scheduler */
   osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
   MX_FREERTOS_Init();

  /* Start scheduler */
   osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	 never reach here due to freeRTOS, but keep for debugging sensors when needed (also need tp comment osKernelStart)
//	 handle_IMU();
//	 handle_lidar_i2c();
//	 handle_encoder();


//	HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void lidar_Init(){

}

void MPU9250_Init(void)
{
    uint8_t check, data;

    // Check WHO_AM_I register
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
    if (check != 0x71)  // 0x71 is the expected response for MPU-9250
    {
        char error_msg[] = "MPU9250 not found\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        while (1);  // Stop execution if MPU-9250 is not detected
    }

    // Wake up the MPU-9250 by clearing sleep mode bit (6) in PWR_MGMT_1
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
}


void handle_lidar(){

   // bool receive_lidar_data = true;
    // Receive data from Lidar (9 bytes for a typical distance data frame)
//    uint8_t continous_output[] = {0x5A, 0x05, 0x07, 0x01, 0x00};
//    ret_lidar = HAL_UART_Transmit(&huart3, continous_output , strlen((char*)continous_output), HAL_MAX_DELAY);
//    if(ret_lidar == HAL_OK){
//        strcpy((char*)uart_buf, "Continous output command succeded \r\n");
//        HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//    }

//	ret_lidar = HAL_UART_Receive(&huart4, uart_lidar_buf, strlen((char*)uart_lidar_buf), HAL_MAX_DELAY);
//    if (ret_lidar != HAL_OK) {
//        // Handle UART receive failure
//        strcpy((char*)uart_buf, "Lidar UART Receive Failed in handle_lidar\r\n");
//        HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//        receive_lidar_data = false;
//    }
//
//    if (receive_lidar_data) {
//    	 sprintf((char*)uart_buf, "Recieved data");
//    	 HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//        // Validate header
//        if (uart_lidar_buf[0] != 0x5A) {
//            strcpy((char*)uart_buf, "Invalid Lidar Data Header\r\n");
//            HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//            return;
//        }
//
//        // Extract the payload length
//        uint8_t len = uart_lidar_buf[1];
//        if (len < 4 || len > 255) {
//            strcpy((char*)uart_buf, "Invalid Lidar Data Length\r\n");
//            HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//            return;
//        }
//
//        sprintf((char*)uart_buf, "Header and payload okay data");
//        HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//
//        // Parse distance data (Payload bytes)
//        uint16_t distance = (uart_lidar_buf[3] << 8) | uart_lidar_buf[2];
//
//        // Optional: Validate checksum (if checksum verification is enabled)
//        uint8_t checksum = 0;
//        for (int i = 0; i < len - 1; i++) {
//            checksum += uart_lidar_buf[i];
//        }
//        sprintf((char*)uart_buf, "Checksum okay data");
//        HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//        if ((checksum & 0xFF) != uart_lidar_buf[len - 1]) {
//            strcpy((char*)uart_buf, "Lidar Checksum Error\r\n");
//            HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//            return;
//        }
//
//        // Transmit distance over UART
//        sprintf((char*)uart_buf, "LiDAR Distance: %u cm\r\n", distance);
//        HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//    }
}

void handle_lidar_i2c(){
	uart_buf[0] = REG_TEMP;
	// verify connection

	bool connect_lidar_success = true;
	ret = HAL_I2C_Master_Transmit(&hi2c2, LIDAR_ADDR, i2c_lidar_buf, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)uart_buf, "Lidar I2C Transmit Failed in handle_IMU\r\n");
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		connect_lidar_success = false;
	}

	ret = HAL_I2C_Master_Receive(&hi2c2, LIDAR_ADDR, i2c_lidar_buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)uart_buf, "Lidar I2C Receive Failed in handle_IMU\r\n");
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		connect_lidar_success = false;
	}

	//read and print
	if(connect_lidar_success){
		// uint8_t i2c__test_buf[14];
	    HAL_I2C_Mem_Read(&hi2c2, LIDAR_ADDR, 0x00, 1, i2c_lidar_buf, 14, HAL_MAX_DELAY);
	    uint8_t distance = (i2c_lidar_buf[1] << 8) | i2c_lidar_buf[0];
        sprintf((char *)uart_buf, "LiDAR Distance: %u cm\r\n", distance);
        HAL_UART_Transmit(&huart2, uart_buf, strlen((char *)uart_buf), HAL_MAX_DELAY);
	}


}


void handle_IMU(){


	uart_buf[0] = REG_TEMP;

	// verify connection
	bool connect_imu_success = true;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, i2c_buf, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)uart_buf, "MPU9250 I2C Transmit Failed in handle_IMU\r\n");
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		connect_imu_success = false;
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, i2c_buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		strcpy((char*)uart_buf, "MPU9250 I2C Receive Failed in handle_IMU\r\n");
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		connect_imu_success = false;
	}

	//read and print
	if(connect_imu_success){
		MPU9250_read_accel_gyro(accel, gyro);
		sprintf((char*)uart_buf,
					"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n",
					accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
}

/**
 * @brief Read accelerometer and gyroscope data
 * @param accel: Pointer to store accelerometer data (X, Y, Z)
 * @param gyro: Pointer to store gyroscope data (X, Y, Z)
 *
 */

void MPU9250_read_accel_gyro(float *accel, float *gyro)
{
    // Read accelerometer and gyroscope data (14 bytes)
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1, imu_buf, 14, HAL_MAX_DELAY);

    // Convert accelerometer raw data to g
    accel[0] = (int16_t)((imu_buf[0] << 8) | imu_buf[1]) / ACCEL_SCALE;
    accel[1] = (int16_t)((imu_buf[2] << 8) | imu_buf[3]) / ACCEL_SCALE;
    accel[2] = (int16_t)((imu_buf[4] << 8) | imu_buf[5]) / ACCEL_SCALE;

    // Convert gyroscope raw data to degrees/second
    gyro[0] = (int16_t)((imu_buf[8] << 8) | imu_buf[9]) / GYRO_SCALE;
    gyro[1] = (int16_t)((imu_buf[10] << 8) | imu_buf[11]) / GYRO_SCALE;
    gyro[2] = (int16_t)((imu_buf[12] << 8) | imu_buf[13]) / GYRO_SCALE;
}


/*
 *
 * enum RobotMovement{
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
};
 *
 *
 *
 *
 *
 * */
void handle_encoder(){
	 for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
		// int32_t ticks =; // __HAL_TIM_GET_COUNTER(&htim2);
		sprintf((char*)uart_buf, "Pin: %u\r\n",  encoders[i]->gpio_pin);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		sprintf((char*)uart_buf, "encoder_ticks_wheel_%d:%" PRId32 "\r\n", i+1,  encoders[i]->tick_count);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	 }
}

/*
 *
 * 0 = right_back
 * 1 = right_front
 * 2 = left_front
 * 3 = left_back
 *{BACKWARD, FORWARD, FORWARD, BACKWARD}
{FORWARD, BACKWARD, BACKWARD, FORWARD}
{BACKWARD, FORWARD, BACKWARD, FORWARD}
{FORWARD, BACKWARD, FORWARD, BACKWARD}
{BACKWARD, FORWARD, BACKWARD, FORWARD}
{FORWARD, BACKWARD, FORWARD, BACKWARD}
{FORWARD, NONE, FORWARD, NONE}
{NONE, BACKWARD, NONE, BACKWARD}
{NONE, FORWARD, NONE, FORWARD}
{BACKWARD, NONE, BACKWARD, NONE}
 * */

void handle_encoder_direction(RobotMovement move){

	switch (move){
		case STOP:
			//Fall Through, potentially add check to ensure motors are stopped
			break;
		case MOVE_FORWARD:
			update_multiple_directions(DIR_MOVE_FORWARD);
			break;
		case MOVE_BACKWARD:
			update_multiple_directions(DIR_MOVE_BACKWARD);
			break;
		case MOVE_LEFT:
			update_multiple_directions(DIR_MOVE_LEFT);
			break;
		case MOVE_RIGHT:
			update_multiple_directions(DIR_MOVE_RIGHT);
			break;
		case ROTATE_LEFT:
			update_multiple_directions(DIR_ROTATE_LEFT);
			break;
		case ROTATE_RIGHT:
			update_multiple_directions(DIR_ROTATE_RIGHT);
			break;
		case DIAG_FORWARD_RIGHT:
			update_multiple_directions(DIR_DIAG_FORWARD_RIGHT);
			break;
		case DIAG_BACKWARD_RIGHT:
			update_multiple_directions(DIR_DIAG_BACKWARD_RIGHT);
			break;
		case DIAG_FORWARD_LEFT:
			update_multiple_directions(DIR_DIAG_FORWARD_LEFT);
			break;
		case DIAG_BACKWARD_LEFT:
			update_multiple_directions(DIR_DIAG_BACKWARD_LEFT);
			break;
		case FASTER:
			//fall through
			break;
		case SLOWER:
			//fall through
			break;
		case INVALID:
			//fall through
			break;
	}
}

void update_multiple_directions(const Direction  dir[]) {
	if(dir == NULL){
		//error
		return;
	}

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    	if(dir[i] == STOPPED){
    		continue;
    	}
        update_encoder_directon(encoders[i], dir[i]);
    }
}

void update_encoder_directon(Encoder * enc, Direction wheel_dir){
	if(enc == NULL){
		return;
	}
	enc->wheel_spin = wheel_dir;
}

//Encoder right_back = {0, 0, FORWARD, BACK, Encoder_RB_Input_GPIO_Port, Encoder_RB_Input_Pin};
//Encoder right_front = {0, 0, BACKWARD,FRONT, Encoder_RF_Input_GPIO_Port, Encoder_RF_Input_Pin};
//Encoder left_front = {0, 0, FORWARD,FRONT, Encoder_LF_Input_GPIO_Port,Encoder_LF_Input_Pin};
//Encoder left_back = {0, 0, BACKWARD,BACK, Encoder_LB_Input_GPIO_Port, Encoder_LB_Input_Pin};

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// uint8_t local_buffer[100];
}
*/

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim == &htim2)  // Check which timer triggered the interrupt
    {
        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            Encoder* enc = encoders[i];
            uint8_t curr_state = HAL_GPIO_ReadPin(enc->gpio_port, enc->gpio_pin);
            // Process state change
            if (curr_state != enc->prev_state) {
                int32_t tick_change = (enc->wheel_spin == FORWARD) ? 1 : -1;
                enc->tick_count += (enc->wheel_pos == FRONT) ? tick_change : -tick_change;
                enc->prev_state = curr_state;
            }
        }
    }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
