/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdbool.h>
#include <stdlib.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/util/time.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"

#include "usart.h"
#include "i2c.h"
#include "tim.h"
#include "constants.h"
#include "MadgwickAHRS.h"




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
rcl_publisher_t publisher;
rcl_publisher_t encoder_publisher;

rcl_publisher_t encoder_RB_publisher;
rcl_publisher_t encoder_RF_publisher;
rcl_publisher_t encoder_LF_publisher;
rcl_publisher_t encoder_LB_publisher;


rcl_publisher_t* encoder_pub[NUM_ENCODERS] = {&encoder_RB_publisher, &encoder_RF_publisher, &encoder_LF_publisher, &encoder_LB_publisher};

rcl_publisher_t imu_publisher;
rcl_publisher_t lidar_publisher;
//rcl_publisher_t encoder_pub_check;

rcl_subscription_t encoder_dir_subscriber;


std_msgs__msg__Int32 encoder_msg;
sensor_msgs__msg__Imu  imu_msg;
std_msgs__msg__Float32 lidar_msg;
std_msgs__msg__Int32 msg;
std_msgs__msg__String sub_encoder_dir_msg;
std_msgs__msg__String pub_encoder_dir_msg;

sensor_msgs__msg__LaserScan lidar_msg_test;

uint32_t servo_angle = 0;
uint32_t step = 10;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);



int32_t get_encoder_data();
void update_encs_dir(RobotMovement move);
void update_multi_dir(const Direction  dir[]);
void update_enc_dir(Encoder * enc, Direction wheel_dir);
void sub_encoder_dir_callback(const void * msgin);


void  init_imu_msg(sensor_msgs__msg__Imu *imu_data);
void get_imu_data(sensor_msgs__msg__Imu *imu_data);

void lidar_sweep(sensor_msgs__msg__LaserScan *lidar_msg_test);
bool get_lidar_data();



void madgwick_init();
void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void madgwick_get_quaternion(float *qx, float *qy, float *qz, float *qw);
void read_magnetometer(float *mag_x, float *mag_y, float *mag_z);


void publisherInit();

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	// micro-ROS configuration
	rmw_uros_set_custom_transport(
	  true,
	  (void *) &huart2,
	  cubemx_transport_open,
	  cubemx_transport_close,
	  cubemx_transport_write,
	  cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  // strcpy((char*)uart_buf, "Error on default allocators \r\n");
	  // HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	   printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app
//	rcl_publisher_t publisher;
//	rcl_publisher_t encoder_publisher;
//	rcl_publisher_t imu_publisher;
//	rcl_publisher_t lidar_publisher;

//	std_msgs__msg__Float32 encoder_msg;
//	sensor_msgs__msg__Imu  imu_msg;
//	std_msgs__msg__Float32 lidar_msg;
//	std_msgs__msg__Int32 msg;

	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;

	allocator = rcl_get_default_allocator();

	//create init_options
	rclc_support_init(&support, 0, NULL, &allocator);

	// create node
	rclc_node_init_default(&node, "cubemx_node", "", &support);
    // strcpy((char*)uart_buf, "Finished creating support_init and node_init \r\n");
    // HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);

	// create publisher
	rclc_publisher_init_default(
	&publisher,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	"cubemx_publisher");

    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_data"
    );
    rclc_publisher_init_default(
        &encoder_RB_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_RB_data"
    );
    rclc_publisher_init_default(
        &encoder_RF_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_RF_data"
    );
    rclc_publisher_init_default(
        &encoder_LB_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_LB_data"
    );
    rclc_publisher_init_default(
        &encoder_LF_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_LF_data"
    );

    rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    );

//    rclc_publisher_init_default(
//        &lidar_publisher,
//        &node,
//        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//        "lidar_data"
//    );


    rclc_publisher_init_default(
        &lidar_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "lidar_data"
    );

//    rclc_publisher_init_default(
//        &encoder_pub_check,
//        &node,
//        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
//        "encoder_check"
//    );

    rmw_qos_profile_t qos_profile = {
        .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST, // Keep last message
        .depth = 1, // Depth of 1
        .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, // BestEffort reliability
        .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE // Volatile durability
    };

    rclc_subscription_init(
        &encoder_dir_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "motor_control",
		&qos_profile
	);

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(
    		&executor,
			&encoder_dir_subscriber,
			&sub_encoder_dir_msg,
			&sub_encoder_dir_callback,
			ON_NEW_DATA
	);

    sub_encoder_dir_msg.data.data = (char * ) malloc(200 * sizeof(char));
    sub_encoder_dir_msg.data.size = 0;
    sub_encoder_dir_msg.data.capacity = 200;

    pub_encoder_dir_msg.data.data = (char * ) malloc(200 * sizeof(char));
    pub_encoder_dir_msg.data.size = 0;
    pub_encoder_dir_msg.data.capacity = 200;



    // strcpy((char*)uart_buf, "Finished init deault of all publishers \r\n");
    // HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);

    // Start FreeRTOS tasks
//    osThreadNew(handleEncoder, NULL, &encoderTask_attributes);
//    osThreadNew(handleIMU, NULL, &imuTask_attributes);
//    osThreadNew(handleLidar, NULL, &lidarTask_attributes);

    // init_MPU9250();

	msg.data = 0;

	init_imu_msg(&imu_msg);
//	float lidar_dist;

	rcl_ret_t ret;
	rcl_ret_t imu_ret;
	/* Infinite loop */
	for(;;)
	{

		ret = rcl_publish(&publisher, &msg, NULL);
		if (ret != RCL_RET_OK)
		{
          //strcpy((char*)uart_buf, "main uRos msg failed \r\n");
          //HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		  printf("Error publishing (line %d)\n", __LINE__);
		}

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

		for(int i = 0; i < NUM_ENCODERS; i++){
			encoder_msg.data = get_encoder_data(i);
			ret = rcl_publish(encoder_pub[i], &encoder_msg, NULL);
			if (ret != RCL_RET_OK)
			{
	//	      strcpy((char*)uart_buf, "encoder uRos msg failed \r\n");
	//	      HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
			  printf("Error publishing encoder data (line %d)\n", __LINE__);
			}
		}

        get_imu_data(&imu_msg);
		imu_ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
		if (imu_ret != RCL_RET_OK)
		{
//		  strcpy((char*)uart_buf, "imu uRos msg failed \r\n");
//		  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		  msg.data = 0;
		  printf("Error publishing imu data (line %d)\n", __LINE__);
		}

		lidar_sweep(&lidar_msg_test);
	    rcl_ret_t ret = rcl_publish(&lidar_publisher, &lidar_msg_test, NULL);
	    if (ret != RCL_RET_OK) {
	        printf("Error publishing LIDAR data (line %d)\n", __LINE__);
	    }

//		if(get_lidar_data(&lidar_dist)){
//			 lidar_msg.data = lidar_dist;
//			 ret = rcl_publish(&lidar_publisher, &lidar_msg, NULL);
//			 if (ret != RCL_RET_OK)
//			 {
////				 strcpy((char*)uart_buf, "lidar uRos msg failed \r\n");
////				 HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
//				 printf("Error publishing lidar data (line %d)\n", __LINE__);
//			 }
//		}


		msg.data++;

		if(msg.data < 100){
			 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 250);
		}
		else if(msg.data > 200 && msg.data < 300){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 750);
		}
		else if(msg.data > 400 && msg.data < 500){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 1250);
		}
		else if(msg.data > 600 && msg.data < 700){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 750);
		}else{
			msg.data = 0;
		}



		osDelay(100);
	}

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void sub_encoder_dir_callback(const void * msgin)
{
	const std_msgs__msg__String *msg_enc = (const std_msgs__msg__String *)msgin;
    RobotMovement current_move = (RobotMovement)msg_enc->data.data[0]; // Assuming the message contains a single character
    update_encs_dir(current_move);

    // pub_encoder_dir_msg = *msg_enc;
    // pub_encoder_dir_msg.data.size = strlen(pub_encoder_dir_msg.data.data);
    // rcl_publish(&encoder_pub_check, &pub_encoder_dir_msg, NULL);

}

void update_encs_dir(RobotMovement move){

	switch (move){
		case STOP:
			//Fall Through, potentially add check to ensure motors are stopped
			break;
		case MOVE_FORWARD:
			update_multi_dir(DIR_MOVE_FORWARD);
			break;
		case MOVE_BACKWARD:
			update_multi_dir(DIR_MOVE_BACKWARD);
			break;
		case MOVE_LEFT:
			update_multi_dir(DIR_MOVE_LEFT);
			break;
		case MOVE_RIGHT:
			update_multi_dir(DIR_MOVE_RIGHT);
			break;
		case ROTATE_LEFT:
			update_multi_dir(DIR_ROTATE_LEFT);
			break;
		case ROTATE_RIGHT:
			update_multi_dir(DIR_ROTATE_RIGHT);
			break;
		case DIAG_FORWARD_RIGHT:
			update_multi_dir(DIR_DIAG_FORWARD_RIGHT);
			break;
		case DIAG_BACKWARD_RIGHT:
			update_multi_dir(DIR_DIAG_BACKWARD_RIGHT);
			break;
		case DIAG_FORWARD_LEFT:
			update_multi_dir(DIR_DIAG_FORWARD_LEFT);
			break;
		case DIAG_BACKWARD_LEFT:
			update_multi_dir(DIR_DIAG_BACKWARD_LEFT);
			break;
		case FASTER:
			//fall through
			break;
		case SLOWER:
			//fall through
			break;
		default:
			//fall through
			break;
	}
}

void update_multi_dir(const Direction  dir[]) {
	if(dir == NULL){
		//error
		return;
	}

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    	if(dir[i] == STOPPED){
    		continue;
    	}
    	update_enc_dir(encoders[i], dir[i]);
    }
}

void update_enc_dir(Encoder * enc, Direction wheel_dir){
	if(enc == NULL){
		return;
	}
	enc->wheel_spin = wheel_dir;
}

int32_t  get_encoder_data(int enc_num){
	return encoders[enc_num]->tick_count;
}
void  init_imu_msg(sensor_msgs__msg__Imu *imu_data){
	imu_data->header.frame_id.data = "MPU9250_fr";
	imu_data->header.frame_id.size = 10;

    imu_data->linear_acceleration_covariance[0] = 0.00009;  // X variance
    imu_data->linear_acceleration_covariance[1] = 0;
    imu_data->linear_acceleration_covariance[2] = 0;
    imu_data->linear_acceleration_covariance[3] = 0;
    imu_data->linear_acceleration_covariance[4] = 0.00009;  // Y variance
    imu_data->linear_acceleration_covariance[5] = 0;
    imu_data->linear_acceleration_covariance[6] = 0;
    imu_data->linear_acceleration_covariance[7] = 0;
    imu_data->linear_acceleration_covariance[8] = 0.00009;  // Z variance

    imu_data->angular_velocity_covariance[0] = 0.0001;
    imu_data->angular_velocity_covariance[1] = 0;
    imu_data->angular_velocity_covariance[2] = 0;
    imu_data->angular_velocity_covariance[3] = 0;
    imu_data->angular_velocity_covariance[4] = 0.0001;
    imu_data->angular_velocity_covariance[5] = 0;
    imu_data->angular_velocity_covariance[6] = 0;
    imu_data->angular_velocity_covariance[7] = 0;
    imu_data->angular_velocity_covariance[8] = 0.0001;

    imu_data->orientation_covariance[0] = 9999;  // Set high for unknown values
    imu_data->orientation_covariance[1] = 0;
    imu_data->orientation_covariance[2] = 0;
    imu_data->orientation_covariance[3] = 0;
    imu_data->orientation_covariance[4] = 9999;
    imu_data->orientation_covariance[5] = 0;
    imu_data->orientation_covariance[6] = 0;
    imu_data->orientation_covariance[7] = 0;
    imu_data->orientation_covariance[8] = 9999;


}
void  get_imu_data(sensor_msgs__msg__Imu *imu_data){

	HAL_StatusTypeDef imu_ret_func;
	float mag_x, mag_y, mag_z;


	imu_ret_func = HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, ACCEL_XOUT_H, 1, imu_buf, 14, HAL_MAX_DELAY);
    if(imu_ret_func != HAL_OK){
    	// printf("Error reading I2C imu data \n");
    	return;
    }

    imu_data->linear_acceleration.x = (int16_t)((imu_buf[0] << 8) | imu_buf[1]) / ACCEL_SCALE;
    imu_data->linear_acceleration.y = (int16_t)((imu_buf[2] << 8) | imu_buf[3]) / ACCEL_SCALE;
    imu_data->linear_acceleration.z = (int16_t)((imu_buf[4] << 8) | imu_buf[5]) / ACCEL_SCALE;

    // Convert gyroscope raw data to degrees/second
    imu_data->angular_velocity.x = (int16_t)((imu_buf[8] << 8) | imu_buf[9]) / GYRO_SCALE;
    imu_data->angular_velocity.y = (int16_t)((imu_buf[10] << 8) | imu_buf[11]) / GYRO_SCALE;
    imu_data->angular_velocity.z = (int16_t)((imu_buf[12] << 8) | imu_buf[13]) / GYRO_SCALE;


    read_magnetometer(&mag_x, &mag_y, &mag_z);

    MadgwickAHRSupdate(
        imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z,
        imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z,
        mag_x, mag_y, mag_z
    );

    float qx, qy, qz, qw;
    madgwick_get_quaternion(&qx, &qy, &qz, &qw);

    imu_data->orientation.x = qx;
    imu_data->orientation.y = qy;
    imu_data->orientation.z = qz;
    imu_data->orientation.w = qw;


    uint32_t time_now = uxr_millis();
    imu_data->header.stamp.sec = time_now / 1000;
    imu_data->header.stamp.nanosec = (time_now % 1000) * 1000000;



}

void madgwick_init() {
    // madgwick_begin(IMU_SAMPLE_RATE);  // IMU_SAMPLE_RATE is the sampling rate in Hz
}

//void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
//	MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
//}

void madgwick_get_quaternion(float *qx, float *qy, float *qz, float *qw) {
    *qx = q0;
    *qy = q1;
    *qz = q2;
    *qw = q3;
}

void read_magnetometer(float *mag_x, float *mag_y, float *mag_z) {
    uint8_t mag_buf[7];
    HAL_StatusTypeDef ret;

    // Read magnetometer data
    ret = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR, AK8963_XOUT_L, 1, mag_buf, 7, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error reading magnetometer data\n");
        return;
    }

    // Convert raw data to microteslas (uT)
    *mag_x = (int16_t)((mag_buf[1] << 8) | mag_buf[0]) * MAG_SCALE;
    *mag_y = (int16_t)((mag_buf[3] << 8) | mag_buf[2]) * MAG_SCALE;
    *mag_z = (int16_t)((mag_buf[5] << 8) | mag_buf[4]) * MAG_SCALE;
}


void lidar_sweep(sensor_msgs__msg__LaserScan *lidar_msg_test){

    int steps = 18;  // 180° / 10° step size = 18 readings

    if (lidar_msg_test->ranges.data == NULL) {
        lidar_msg_test->ranges.data = (float *)malloc(steps * sizeof(float));
        lidar_msg_test->ranges.size = steps;
        lidar_msg_test->ranges.capacity = steps;
    }

    float lidar_readings[steps];

    // Set up LaserScan message parameters
    lidar_msg_test->angle_min = 0.0;                       // Start angle (radians)
    lidar_msg_test->angle_max = 3.14159;                   // End angle (π radians)
    lidar_msg_test->angle_increment = (3.14159 / steps);   // Angle resolution per step
    lidar_msg_test->range_min = 0.1;                       // Min valid range (10cm)
    lidar_msg_test->range_max = 4.0;                       // Max valid range (4m)

    // 🔄 Sweep through 180 degrees
    for (int i = 0; i < steps; i++) {
        int servo_angle = i * 10; // Move in 10° steps (0, 10, ..., 180)
        uint32_t angle_pwm = 500 + ((servo_angle * 500) / 180);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angle_pwm); // Move Servo
        //osDelay(100);  // Allow servo to move before reading LIDAR

        // Read LIDAR distance
        float distance = 0.0;
        if (get_lidar_data(&distance)) {
            lidar_readings[i] = distance;
        } else {
            lidar_readings[i] = 0.0; // Mark as invalid if LIDAR fails
        }
    }

    // Copy collected data into LaserScan message
   for (int i = 0; i < steps; i++) {
        lidar_msg_test->ranges.data[i] = lidar_readings[i];
    }
}


bool get_lidar_data(float * dist_out){

	uart_buf[0] = REG_TEMP;
	HAL_StatusTypeDef lidar_ret_func;

	lidar_ret_func = HAL_I2C_Master_Transmit(&hi2c2, LIDAR_ADDR, uart_buf, 1, HAL_MAX_DELAY);
	if(lidar_ret_func != HAL_OK){
//		strcpy((char*)uart_buf, "Lidar I2C Transmit Failed in handle_IMU\r\n");
//		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		return false;
	}

	lidar_ret_func = HAL_I2C_Master_Receive(&hi2c2, LIDAR_ADDR, i2c_lidar_buf, 2, HAL_MAX_DELAY);
	if(lidar_ret_func != HAL_OK){
//		strcpy((char*)uart_buf, "Lidar I2C Receive Failed in handle_IMU\r\n");
//		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		return false;
	}


	lidar_ret_func =  HAL_I2C_Mem_Read(&hi2c2, LIDAR_ADDR, 0x00, 1, i2c_lidar_buf, 14, HAL_MAX_DELAY);
	uint8_t distance = (i2c_lidar_buf[1] << 8) | i2c_lidar_buf[0];
//	sprintf((char *)uart_buf, "LiDAR Distance: %u cm\r\n", distance);
//	HAL_UART_Transmit(&huart2, uart_buf, strlen((char *)uart_buf), HAL_MAX_DELAY);



	*dist_out = (float)distance;

	return true;
}
/* USER CODE END Application */

