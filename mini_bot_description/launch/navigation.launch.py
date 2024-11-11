from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('mini_bot_description')
    pkg_share = FindPackageShare(package='mini_bot_description').find('mini_bot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'mini_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_localization_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mini_bot_navigation2.rviz')
    map_file_path = os.path.join(pkg_share, 'maps', 'mini_bot_office.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_cmd = DeclareLaunchArgument(
        name='map',
        default_value=map_file_path,
        description='Full path to the map file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_urdf,  'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, robot_localization_file_path]
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
   # ld.add_action(start_robot_localization_cmd)
    ld.add_action(nav2_bringup_launch)

    return ld

