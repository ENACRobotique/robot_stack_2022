from http.server import executable
import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    strat = Node(
        package='enac_strat',
        executable='enac_strat',
        name='strat',
        parameters=[
        ],
    ) 

    return LaunchDescription([
        strat,
])