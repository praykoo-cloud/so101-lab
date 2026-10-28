"""arm_node — 로봇과 이야기하는 노드는 이것 하나. 다른 노드는 토픽 · 서비스 이름만 안다.

    ros2 run so101_ros arm_node --ros-args -p port:=/dev/ttyACM0

발행   /joint_states      sensor_msgs/JointState   관절 각도 (rad, URDF 기준), rate Hz
서비스 /arm/torque        std_srvs/SetBool         잠그기 · 풀기
       /arm/go_pose       so101_interfaces/GoPose  저장한 자세로 (도착할 때까지 기다린 뒤 응답)
       /arm/save_pose     so101_interfaces/SavePose 지금 자세 저장 (티칭)
       /arm/gripper       so101_interfaces/SetGripper
       /arm/stop          std_srvs/Trigger         그 자리에서 멈춤
"""
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from so101_interfaces.srv import GoPose, SavePose, SetGripper
from soarm import Robot, RobotError

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")
        port = self.declare_parameter("port", "/dev/ttyACM0").value
        speed = int(self.declare_parameter("speed", 200).value)
        rate = float(self.declare_parameter("rate", 20.0).value)
        self.bot = Robot(port, speed=speed)
        self.get_logger().info(f"연결: {port}")

        # 상태 발행과 서비스를 다른 콜백 그룹에 둔다 → go_pose 로 기다리는 동안에도 /joint_states 가 나간다
        state_group, srv_group = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
        # 상대 이름: 네임스페이스 없이 /joint_states, namespace:=so101 이면 /so101/joint_states (차와 겹치지 않게)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self.publish_state, callback_group=state_group)
        self.create_service(SetBool, "/arm/torque", self.on_torque, callback_group=srv_group)
        self.create_service(GoPose, "/arm/go_pose", self.on_go_pose, callback_group=srv_group)
        self.create_service(SavePose, "/arm/save_pose", self.on_save_pose, callback_group=srv_group)
        self.create_service(SetGripper, "/arm/gripper", self.on_gripper, callback_group=srv_group)
        self.create_service(Trigger, "/arm/stop", self.on_stop, callback_group=srv_group)

    def publish_state(self):
        rad = self.bot.joint_radians()
        if any(v is None for v in rad.values()):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NAMES
        msg.position = [float(rad[i]) for i in range(1, 7)]
        self.pub.publish(msg)

    def on_torque(self, req, res):
        self.bot.torque(req.data)
        res.success, res.message = True, "잠금" if req.data else "풀림"
        self.get_logger().info(f"torque {req.data}")
        return res

    def on_go_pose(self, req, res):
        try:
            self.bot.go_pose(req.name, speed=req.speed or None, timeout=15.0)
            res.success, res.message = True, f"도착: {req.name}"
        except RobotError as e:
            res.success, res.message = False, str(e)
        self.get_logger().info(f"go_pose {req.name} → {res.message}")
        return res

    def on_save_pose(self, req, res):
        try:
            self.bot.save_pose(req.name)
            res.success, res.message = True, f"저장: {req.name}"
        except (RobotError, ValueError) as e:
            res.success, res.message = False, str(e)
        return res

    def on_gripper(self, req, res):
        try:
            self.bot.gripper(req.percent, speed=300)
            self.bot.wait_until_settled(timeout=6.0)
            res.load = int(self.bot.driver.get_load(6) or 0)
            res.success, res.message = True, f"그리퍼 {self.bot.gripper_percent()}%"
        except RobotError as e:
            res.success, res.message = False, str(e)
        self.get_logger().info(f"gripper {req.percent:.0f}% → {res.message}")
        return res

    def on_stop(self, req, res):
        self.bot.stop()
        res.success, res.message = True, "멈춤"
        return res


def main():
    rclpy.init()
    node = ArmNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.bot.stop()
        node.bot.close()
        node.destroy_node()
        rclpy.try_shutdown()
