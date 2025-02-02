# STM32_Sensors_Slambot
This repository contains the firmware for STM32 NUCLEO-F446RE that handles the MPU9250 IMU, 4 photo-interrupt encoders and lidar sensor readings using microROS via freeRTOS. 

- MPU9250 IMU: I2C connection 
- Luna LIDAR: another I2c Connection
- Photo-interuptor Encoder: One interrupt timer, that will update all four wheel ticks

# microROS

The microcontroller was programmed via WSL and docker on windows 11 following steps https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy as well as help from https://github.com/lFatality/stm32_micro_ros_setup, who also had a nice youtube video to go along with it. This code should work outside the box and you only need to hit run. 

I used usbipd in windows powershell to connect my stm32 to WSL and it defaulted to the device /dev/ttyACM0 (it may be different, especially when i move from testing on my windows computer to testing this out on a raspberry pi). 

When you have the microros agent setup, which you can follow from the micro_ros_stm32cubemx_utils repo, you can run the agent with 

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

I found sometimes you neet to detach and attatch using usbipd on windows powershell, to get the agent to work correctly: 

usbipd detach --busid=6-1
usbipd attach --wsl --busid=6-1

but in general you can follow these steps to get microros agent running:
1. Start the agent before attachting to WSL.  
2. Run themicrocontroller in the STM32 IDE, 
3. attactch to WSL
4. if it is getting stuck with "serial port not found", double check the port name and correct it if necessary.
5. if it is getting stuck with "session established" and "session re-established", try to detach and re-attach the device with usbipd. 
6. if it is still not working try to detach the device, restart the agent and then attatch again. 


# Topics, Publishers and Subscriber
This code has four main topics: 
1. "cubemx_publisher" - just publishes basic integer data to ensure everything is working correctly (may remove later)
2. "encoder_data" - publishes encoder tick count from the photo-interupt (currently all wheels are publishes on this one topic -> may move to 4 topics, one for each wheel)
3. "imu_data" - publishes imu data utilizing the sensor_msg/Imu msg 
4. "lidar_data" - publishes distance values in cm (will integrate this with more ROS2 laser msg)


# pin configuration

I2C pin connections for MPU9250 IMU:
- VCC to 3.3 V
- GND to GND
- SDA to pin D14 (PB_9 in CN5)
- SCL to pin D15 (PB8 in CN5)

I2C pin connections for Luna LIDAR: 
- VCC to 5V
- GND to GND
- SDA to PC12 (top left of CN7) 
- SCL to PB10 (7 from boot on left of CN10)
- Config Input to GND

Photo-interuptor Encoder: 
- VCC to 5V
- GND to GND
- Right Back Wheel PA9 (D8 in CN9)
- Right front Wheel PA10 (D2 in CN9)
- left Back Wheel PA6 (D12 in CN9)
- left front Wheel PA7 (D11 in CN9)


# future updates
- adding more topics for each encoder
- add lidar data to a mor standard ros msg
- integrate this with the nav stack or other slam methods