import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='serial_port', 
            default_value='/dev/ttyACM0', #TODO: Figure out what's the real port name
            description='LD06 Serial Port'
        ),
        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='LD06 Topic Name'
        ),
        DeclareLaunchArgument(
            name='lidar_frame', 
            default_value='laser',
            description='LD06 Topic Name'
        ),

        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration("serial_port")},
                {'topic_name': LaunchConfiguration("topic_name")},
                {'lidar_frame': LaunchConfiguration("lidar_frame")}
            ]
        )
    ])