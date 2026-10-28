"""색 분류 전체: arm · camera · color · task (+ robot_state_publisher).

    ros2 launch so101_ros sort.launch.py port:=/dev/ttyACM0 camera:=0

먼저: 프로젝트 1 로 project/color.json (ROI), 프로젝트 3 으로 자세 7개 (data/poses.json).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("so101_ros")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "arm.launch.py")),
            launch_arguments={"port": LaunchConfiguration("port"), "rviz": LaunchConfiguration("rviz")}.items()),
        Node(package="so101_ros", executable="camera_node",
             parameters=[{"device": LaunchConfiguration("camera"), "fps": 10.0}]),
        Node(package="so101_ros", executable="color_node"),
        Node(package="so101_ros", executable="task_node"),
    ])
