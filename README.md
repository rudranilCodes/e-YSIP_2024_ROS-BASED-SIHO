# 1_ROS_based_SIHO

This package contains all the necessary folders and files to spawn a simple 4- wheel skid-steer drive robot on Gazebo. This package can be used as a refernce to generate URDF/xacro files along with the setup of the .gazebo file in which the kinematics plugin for skid-steer drive has been added and is somewhat different from ROS 1.

ROS version: ROS 2 Humble
OS: Ubuntu 22.04


SETUP:

1) Download the package in the ROS 2 workspace.
2) Build the package using :
	$ colcon build
		or
	$ colcon build --symlink-install 

3) Source the workspace if needed

4) To spawn the robot on Gazebo use:
	
	$ ros2 launch diff_drive_bot gazebo_model.launch.py 

5) To control the bot using teleop use:

	$ ros2 run teleop_twist_keyboard teleop_twist_keyboard 

6) To view odometry information use:

	$ ros2 topic echo /odom

