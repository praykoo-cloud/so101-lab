"""공 따라가기 전체: arm (+ robot_state_publisher) · camera · ball · follow.

    ros2 launch so101_ros follow.launch.py port:=/dev/ttyACM0 camera:=0
    ros2 service call /arm/torque std_srvs/srv/SetBool "{data: true}"      팔 잠그기
    ros2 service call /follow/enable std_srvs/srv/SetBool "{data: true}"   따라가기 켜기

카메라는 집게(손목)에 달린 것. 받침 = 좌우, 손목 = 위아래로 돌려 공을 화면 가운데에 둔다.
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
        DeclareLaunchArgument("color", default_value="red"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "arm.launch.py")),
            launch_arguments={"port": LaunchConfiguration("port")}.items()),
        Node(package="so101_ros", executable="camera_node",
             parameters=[{"device": LaunchConfiguration("camera"), "fps": 10.0}]),
        Node(package="so101_ros", executable="ball_node", parameters=[{"color": LaunchConfiguration("color")}]),
        Node(package="so101_ros", executable="follow_node"),
    ])
