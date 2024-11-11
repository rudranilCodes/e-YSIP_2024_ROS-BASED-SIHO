# 1_ROS_based_SIHO
# Skid-Steered Mini Bot ROS2 Package

## Overview

This package contains a ROS2 (Humble) implementation for a four-wheeled skid-steered mini bot. The package includes:
- Skid steer controller
- Laser sensor plugin
- IMU sensor plugin

It allows you to simulate the robot in Gazebo and visualize its data in RViz.

## Features

- **Skid Steer Control**: Implements skid steering for the robot.
- **Laser Sensor**: Adds a laser sensor plugin for obstacle detection and mapping.
- **IMU Sensor**: Includes an IMU plugin for orientation and motion sensing.

## Prerequisites

Ensure you have the following installed:
- ROS2 Humble
- Gazebo
- RViz
- colcon (for building the package)

## Installation

1. Navigate to your ROS workspace (replace `~/ros2_ws` with the path to your actual workspace if different):
    ```bash
    cd ~/ros2_ws/src
    ```

2. Clone the repository:
    ```bash
    git clone https://github.com/albertrichard080/1_ROS_based_SIHO.git
    ```

3. Navigate back to your ROS workspace:
    ```bash
    cd ~/ros2_ws
    ```

4. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

5. Build the package:
    ```bash
    colcon build
    ```

6. Source the workspace:
    ```bash
    source install/setup.bash
    ```


## Usage

### Running the Simulation in Gazebo

Launch the Gazebo simulation with the skid-steered mini bot:
```bash
ros2 launch 1_ROS_based_SIHO gazebo.launch.py
```

### Visualizing in RViz

Launch RViz to visualize the robot's data:

```bash
ros2 launch 1_ROS_based_SIHO display.launch.py
```

## Teleoperation

```bash
sudo apt-get install ros-humble-teleop-twist-keyboard
```
Then, run the teleop node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard to control the robot:

    i to move forward
    , to move backward
    j to turn left
    l to turn right
    u and o for diagonal movements
    k to stop

--You can also able to see the /scan data ,/imu/data and the /odom data from this robot.
