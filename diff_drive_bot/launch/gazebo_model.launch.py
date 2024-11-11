import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    #must match the robot name in xacro file
    robotXacroName = 'skid_steer_bot'

    namePackage = 'diff_drive_bot'

    modelFileRelativePath = 'model/bot_description.xacro'

    worldFileRelativePath = 'model/empty_world.world'

    pathModeFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)

    robotDescription = xacro.process_file(pathModeFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
    
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world':pathWorldFile}.items())

    spawnModelNode = Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-topic', 'robot_description','-entity',robotXacroName],output='screen')
    
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robotDescription,
                     'use_sim_time':True}])

    launchDescriptionObject= LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject


