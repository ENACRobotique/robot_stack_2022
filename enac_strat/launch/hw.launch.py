from launch import LaunchDescription
from launch_ros.actions import Node
import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

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
    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/enac_ws/install/ldlidar/share/ldlidar/launch/ldlidar.launch.py']),
            launch_arguments={'serial_port': '/dev/ttyUSB0'}.items(),
      )

    return LaunchDescription([
        ros_serial,
        launch_lidar,
])