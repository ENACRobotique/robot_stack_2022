from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

#TODO : add namespace to manage multiple robots running on the same network

def generate_launch_description():
    bringup_dir = get_package_share_directory('ros2serial')

    ros_serial = Node(
        package='ros2serial',
        executable='ros2serial',
        name='ros_serial',
        parameters=[
            {'port': '/dev/ttyACM0', 
            'baud': 115200},
        ],
    ) 

    #launch ldlidar.launch.py which is the driver for the lidar
    #https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
    #https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master


    #launch comm_node from lidar_location package
    lidar_location_comm_node = Node(
        package='lidar_location',
        executable='comm_node',
        name='lidar_location_comm_node',
        parameters=[
            {'port': '/dev/ttyACM0',
            'baud': 115200},
        ],
    )

    #launch basic navigation
    robot_nav_node = Node(
        package='robot_simu_enac',
        executable='robot_nav_node',
        name='robot_nav_node',
        parameters=[
        ],
    )

    return LaunchDescription([
        ros_serial,
        lidar_location_comm_node,
        robot_nav_node,
])