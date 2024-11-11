# Hardware Setup
* Take reference from table below for LiDAR's pin diagram.
  
| Signal | Colour |
| ------------- | ------------- |
| TX  | Yellow  |
| RX  | Green |
| VCC_5V  | Orange |
| GND  | White |
| GND_MOTO  | Black |
| CTRL_MOTO  | Blue |
| 5V_MOTO  | Red |

* Connect the TX and RX to ESP's 16th and 17th pin (Serial2).
* Connect the CTRL_MOTO pin to 25th pin of ESP as declared in the sketch.
* Make sure to have proper power management for the setup.

# Micro-ROS setup
## 1. Micro-ros on Esp32
* Now, we are moving to set up the micro_ros_arduino package in your Arduino IDE.
* Download the zip file from the [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble) repository.
* Open Arduino IDE and include the downloaded zip file as library.
  
**ESP Board Setup :**
  * Open Arduino IDE, go to File -> Preferences.
  * Paste the following link in the Additional Board manager. Once done, click OK.
    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```
  * Go to Tools -> Board -> Boards manager.
  * Search for ESP32 and press install button for the "ESP32 by Espressif Systems"
  * Once the installation is completed you can close the window.
    
**LiDAR Library :**
  * Download the zip file from the [RPLidar](https://github.com/thijses/rplidar/tree/main) repository.
  * Open Arduino IDE and include the downloaded zip file as library.

## 2. Micro-ROS Agent on the computer 
* We will use a micro-ROS Agent to act as the bridge between DDS-XRCE and normal DDS. This means that ROS and micro-ROS nodes can communicate with the agent.
* Workspace setup :
```sh
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
* After this source your workspace in bashrc.

* Now create a micro-ROS agent
```sh
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
* To give micro-ROS access to the ROS 2 dataspace, run the agent:
```sh
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```