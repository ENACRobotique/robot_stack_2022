from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution




def generate_launch_description():

    data_src = Node(
        package='v4l2_camera',
        namespace='camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[
            {'image_size':[1920,1080]},
        ],
    )

    detect_node = Node(
        package='aruco_analysis_enac',
        namespace='camera',
        executable='detect_aruco',
        name='detect_aruco',
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'debug_mode': True},
        ],
        output="screen",
    )

    return LaunchDescription([
        data_src,
        detect_node,
    ])