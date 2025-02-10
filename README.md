# STM32_Sensors_Slambot
This repository contains the firmware for STM32 NUCLEO-F446RE that handles the MPU9250 IMU, 4 photo-interrupt encoders and lidar sensor readings using microROS via freeRTOS. This is the microrcontrolelr that handles all of my sensor processing for my slambot project here https://github.com/JustASimpleCoder/slambot_ws. The sensors and their communication protocols are smarized below

- MPU9250 IMU: I2C connection 
- Luna LIDAR: another I2C Connection
- Photo-interuptor Encoder: One interrupt timer, that will update all four wheel ticks

# Micro-ROS

The microcontroller was programmed via WSL and docker on windows 11 following steps https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy as well as help from https://github.com/lFatality/stm32_micro_ros_setup, who also had a nice youtube video to go along with it. This code should work outside the box and you only need to hit run. 

I used usbipd in windows powershell to connect my stm32 to WSL and it defaulted to the device /dev/ttyACM0 (it may be different, especially when i move from testing on my windows computer to testing this out on a raspberry pi). FOr more details look at https://learn.microsoft.com/en-us/windows/wsl/connect-usb .

When you have the microros agent setup, which you can follow from the micro_ros_stm32cubemx_utils repo, you can run the agent with 

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

I found sometimes you neet to detach and attatch using usbipd on windows powershell, to get the agent to work correctly: 

```
usbipd detach --busid=6-1
usbipd attach --wsl --busid=6-1
```

In general you can follow these steps to get microros agent running:
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
5. "motor_control" - subscriber to the current motor command (see the arduino code running my motors for my slambot project here https://github.com/JustASimpleCoder/Ardunio_Motor_Driver_Slambot), that updates whether to increase or decrease enocder count (i.e. forwards we increment, backwards we decrement)  

# Pin Configuration

I2C pin connections for MPU9250 IMU:
- VCC to 3.3 V
- GND to GND
- SDA to pin D14 (PB9 in CN5)
- SCL to pin D15 (PB8 in CN5)

I2C pin connections for Luna LIDAR: 
- VCC to 5V
- GND to GND
- SDA to PC12 (Top left of CN7) 
- SCL to PB10 (7 from bottom on left of CN10)
- Config Input to GND

Photo-interuptor Encoder: 
- VCC to 5V
- GND to GND
- Right Back Wheel PA9 (D8 in CN9)
- Right front Wheel PA10 (D2 in CN9)
- Left Back Wheel PA6 (D12 in CN9)
- Left front Wheel PA7 (D11 in CN9)


# Future Updates
- adding a topic for each encoder instead of just one
- integrate this with the nav stack or other slam methods
- abstracting some of the publisher/subscriber definitions
