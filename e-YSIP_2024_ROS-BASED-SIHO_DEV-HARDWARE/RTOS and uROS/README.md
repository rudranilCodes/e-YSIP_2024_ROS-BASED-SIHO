# 1_ROS_based_SIHO
## Week -1

**Setup for uROS**
1) First install the micro-ros library in Arduino IDE
   Use this github link: https://github.com/micro-ROS/micro_ros_arduino/tree/humble to download the library and install it in the Arduino IDE  
2) Once you have a ROS 2 installation in the computer, follow these steps to install the micro-ROS build system:
```
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

3)Creating and building a new firmware workspace

```
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
export_executable(<my_app>)
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

**Running Micro-ros**
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
And then, in another command line, run the micro-ROS node (remember sourcing the ROS 2 and micro-ROS installations and setting the RMW Micro XRCE-DDS implementation):
```
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Use RMW Micro XRCE-DDS implementation
export RMW_IMPLEMENTATION=rmw_microxrcedds

```

**Setup for FreeRTOS in Arduino IDE**

1) Add this URL to the "Additional Boards Manager URLs" window in Arduino for ESP32 support: https://dl.espressif.com/dl/package_esp32_index.json
2) Install esp32 by Expressif in Board Manager.

**Running code in Arduino IDE**

You can refer to RTOS example code Arduino folder to code using FreeRTOS. It can be compiled normally and can be uploaded to esp32.

**Setup for FreeRTOS in esp-idf**

1) First we need to install esp-idf in anyone of the platform like Platform IO, Vscode, windows/linux installer.
2) Use this link for detalied installation setup : https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#installation

**Running code in esp-idf**

Here, I show an example of running a sample code of Hello World using esp-idf through Windows.
1) Open the ESP-IDF 5.2 CMD which will be installed default on the Desktop when we follow the instructions in setting up

```
cd hello_world
idf.py set-target esp32
idf.py menuconfig
```
We don't need to change anything here, but you can explore what you can do with it.

2) Build the project

```
idf.py build
```

3) Flash onto the device
Change the PORT to the port number of your esp32. eg COM5
```
idf.py -p <PORT> flash
```
4) Monitor the Output
```
 idf.py -p <PORT> monitor
```
