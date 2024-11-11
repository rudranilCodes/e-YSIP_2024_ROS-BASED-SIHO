# 1_ROS_based_SIHO
# Mini Bot ROS2 Packages

This repository contains all necessary ROS 2 packages for operating a skid steer mini bot in various environments. It includes navigation capabilities, custom worlds, and specific launch files for different scenarios in simulation and also for real world.

## Dependencies to install

Ensure you have the following ROS 2 packages installed(Dependencies):

    sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard python3-colcon-common-extensions python3-rosdep python3-vcstool gazebo11 ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro ros-humble-rviz2 ros-humble-tf2-ros ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-planner ros-humble-nav2-controller ros-humble-nav2-recoveries ros-humble-nav2-life-cycle-manager ros-humble-nav2-behavior-tree ros-humble-rqt ros-humble-rqt-common-plugins ros-humble-rqt-robot-plugins

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

### Custom Worlds:

There are two custom worlds available:

    mini_bot_house.launch.py
    mini_bot_office.launch.py

## Launch Files:
## Gazebo

 To launch the bot in an empty world in Gazebo:

   ```bash
    ros2 launch mini_bot_description gazebo.launch.py
   ```
To launch the bot in the custom house world:

   ```bash
    ros2 launch mini_bot_description mini_bot_house.launch.py
   ```
To launch the bot in the custom office world:

   ```bash
    ros2 launch mini_bot_description mini_bot_office.launch.py
   ```
## RViz

To visualize the bot in RViz:

   ```bash
    ros2 launch mini_bot_description display.launch.py
   ```
## Navigation

Predefined maps are available inside the maps folder.

If you have multiple odometry sources, uncomment the comment which i've been commented in launch description for robot_localization node in navigation.launch.py.

For navigation in simulation (after launching the custom world in Gazebo):

   ```bash
    ros2 launch mini_bot_description navigation.launch.py use_sim_time:=true
   ```
For real-world navigation (after the required topics are published on ROS 2):

   ```bash
    ros2 launch mini_bot_description navigation.launch.py use_sim_time:=false
   ```
## Teleoperation

To teleoperate the bot using the keyboard:

   ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
## Autonomous Navigation:

For autonomous navigation, set the goal pose in RViz after launching the navigation stack. The bot has been tuned for the real-world navigation and mini_bot_office world.

### Notes:

 Ensure the appropriate topics are being published for odometry, laser scan, etc., before launching the navigation stack.   
 The bot's navigation parameters are tuned for the real world and office world and may require adjustments for other environments.
