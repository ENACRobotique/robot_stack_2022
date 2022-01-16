import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution


#For reference : https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21
def launch_setup(context, *args, **kwargs):
    path_bag = LaunchConfiguration('path_bag').perform(context)
    path_config = LaunchConfiguration('path_camera_config').perform(context)
    if path_config == "0":
        path_config = None

    #data source - either a ros2 bag or a usb_camera
    if path_bag != "0":
        data_src = ExecuteProcess(
            cmd= ['ros2', 'bag', 'play', LaunchConfiguration('path_bag'), '-l'],
            output='screen',
        )
    else:
        data_src = Node(
            package='usb_cam',
            namespace='camera',
            executable='usb_cam',
            name='camera',
            parameters=[
                path_config,
            ],
        )

    #node for image_processing (pose estimation)
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

    #visualisation of image processing for debug
    rviz_params_path = os.path.dirname(os.path.abspath(__file__)) + "/detect_debug_view.rviz"
    #TODO : add rviz file
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['d', rviz_params_path],
        output='screen'
    )
    return [data_src,
            detect_node,
            start_rviz_cmd,
            ]

def generate_launch_description():
    path_desc = "play a bag file given its path - provides camera/image_raw and camera/camera_info -  how to set this param ?\n" \
                "1. data come from a node already launched - put random strings here \n" \
                "2. data come from a bag - give its (absolute ?) path  \n" \
                "3. data come from a usb_camera - don't set this argument, a usb_cam node will be launched"
    path_bag = DeclareLaunchArgument("path_bag", default_value=TextSubstitution(text="0"), description=path_desc)
    path_camera_config = DeclareLaunchArgument("path_camera_config", default_value=TextSubstitution(text="0"), description="set if you use usb_camera and you have the config file")

    return LaunchDescription([
        path_bag,
        path_camera_config,
        OpaqueFunction(function= launch_setup)
    ])