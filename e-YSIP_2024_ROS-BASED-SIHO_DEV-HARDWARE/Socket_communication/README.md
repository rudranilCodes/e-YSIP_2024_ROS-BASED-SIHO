# 1_ROS_based_SIHO
## Week -2

**Socket Communication for imu**

Imu UDP folder contains the code for wifi UDP connection to transfer imu data to ros2. Upload the sketch in esp32 and run the python script to see the data in ros2 topics.

**Socket Communication for LiDAR**

LiDAR UDP folder contains the code for wifi UDP connection to transfer LiDAR data to ros2. First upload the sketch in esp32 and run the python script to publish the data in ros2 topic /scan.

**Socket Communication for sensor_data_combine**

sensor_data_combine_with_cmd folder contains the esp32 and python code, Upload esp32 code and run the python then it can transfer the data of LiDAR , IMU to ros2 and receive the cmd vel data in esp32 by using 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
node in terminal 

It uses multiple ports to communicate.


