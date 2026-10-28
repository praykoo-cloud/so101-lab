"""arm_node + robot_state_publisher (+ RViz).

    ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0 rviz:=true

HiWonder 차처럼 /joint_states · /robot_description · base_link 를 쓰는 로봇과 한 도메인에 있을 때:

    ros2 launch so101_ros arm.launch.py port:=/dev/ttyACM0 namespace:=so101

→ /so101/joint_states · /so101/robot_description, TF 프레임 so101/base_link ... (서비스 /arm/* 는 그대로)

URDF 는 so101-lab/model/so101.urdf 를 쓴다 (soarm 패키지 위치로 찾는다).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from soarm.robot import ROOT


def robot_description():
    model = ROOT / "model"
    urdf = (model / "so101.urdf").read_text(encoding="utf-8")
    # 메시 경로 assets/... → 절대 경로 (RViz 가 찾을 수 있게)
    return urdf.replace('filename="assets/', f'filename="file://{model}/assets/')


def nodes(context):
    ns = LaunchConfiguration("namespace").perform(context).strip("/")
    share = get_package_share_directory("so101_ros")
    rviz_args = ["-d", os.path.join(share, "rviz", "so101.rviz")]
    if ns:
        rviz_args += ["-f", f"{ns}/base_link"]
    return [
        Node(package="so101_ros", executable="arm_node", namespace=ns,
             parameters=[{"port": LaunchConfiguration("port")}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns,
             parameters=[{"robot_description": robot_description(), "frame_prefix": f"{ns}/" if ns else ""}]),
        Node(package="rviz2", executable="rviz2", arguments=rviz_args,
             remappings=[("/robot_description", f"/{ns}/robot_description")] if ns else [],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("namespace", default_value="", description="차 등 다른 로봇과 한 도메인일 때 예: so101"),
        OpaqueFunction(function=nodes),
    ])
