from http.server import executable
import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    ros_serial = Node(
        package='ros2serial',
        executable='ros2serial',
        name='ros_serial',
        parameters=[
            {'serial_port': '/dev/ttyACM0', 
            'baud': 115200},
        ],
    ) 

    return LaunchDescription([
        ros_serial,
])