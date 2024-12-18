# STM32_Sensors_Slambot
This repository contains the firmware for STM32 NUCLEO-F446RE that handles the MPU9250 IMU, 4 photo-interrupt encoders and lidar sensor readings. Right now it is just sending the data via UART through the standard buffer (hopefully will move this to micro_ros).

- IMU: I2C connection 
- LIDAR: another I2c Connection
- Encoder: One interrupt timer, that will check all four wheel ticks