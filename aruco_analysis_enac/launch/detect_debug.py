import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.actions import ExecuteProcess

def generate_launch_description():
    path_bag = DeclareLaunchArgument("path_bag", default_value=None)

    if path_bag == None:
        data_src = ExecuteProcess(
            cmd= ['ros2', 'bag', 'play', path_bag, '-l'],
            output='screen',
        )
        pass
    else:
        data_src = Node(
            package='usb_cam',
            executable='usb_cam',
            #TODO : arguments fichier
        )
        pass

    detect_node = Node(
        package='aruco_analysis_enac',
        executable='detect_aruco',
    )

    rviz_params_path = os.path.dirname(os.path.abspath(__file__)) + "/detect_debug_view.rviz"
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['d', rviz_params_path],
        output='screen'
    )
    #TODO : add rviz file
    return LaunchDescription([
        path_bag,
        data_src,
        detect_node,
        start_rviz_cmd,
    ])