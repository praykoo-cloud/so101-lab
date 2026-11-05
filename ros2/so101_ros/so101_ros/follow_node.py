"""follow_node — 공이 화면 가운데에 오게 받침(좌우)과 손목(위아래)을 돌린다 (제어). 카메라는 모른다.

    ros2 run so101_ros follow_node
    ros2 service call /follow/enable std_srvs/srv/SetBool "{data: true}"     켜기 (처음엔 꺼진 채로 시작)

구독 /ball/pixel        geometry_msgs/PointStamped  ball_node 가 낸 공 중심 (픽셀)
     /joint_states      sensor_msgs/JointState      지금 관절 각도 (arm_node)
발행 /arm/joint_command sensor_msgs/JointState      받침 · 손목 목표 (rad) → arm_node 가 움직인다
서비스 /follow/enable   std_srvs/SetBool            따라가기 켜기 · 끄기

비례 제어(P): 공이 가운데에서 벗어난 만큼(-1 ~ 1) × gain 만큼 돈다. 가운데 근처(deadband)는 무시.
공을 0.5 초 넘게 못 보면 멈춘다. 받침 · 손목은 시작 자세에서 ±pan_range · ±tilt_range 안에서만.
"""
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

PAN, TILT = "shoulder_pan", "wrist_flex"


class FollowNode(Node):
    def __init__(self):
        super().__init__("follow_node")
        # 파라미터는 돌아가는 중에도 바꿀 수 있다: ros2 param set /follow_node gain 0.05
        for name, value in (("width", 640.0), ("height", 480.0),
                            ("gain", 0.08),          # rad, 공이 화면 끝에 있을 때 한 번(0.2 초)에 도는 양
                            ("deadband", 0.1),       # 가운데 ±10 % 는 그대로
                            ("pan_sign", -1.0),      # 방향이 반대면 1.0 (카메라를 다는 법에 따라)
                            ("tilt_sign", 1.0),
                            ("pan_range", 0.26),     # 켤 때 자세에서 ±15°
                            ("tilt_range", 0.35)):   # ±20°
            self.declare_parameter(name, value)
        self.enabled = False
        self.ball = None                                         # (u, v, 받은 시각)
        self.joints = None
        self.center = None                                       # 켤 때의 자세 = 움직일 범위의 가운데
        self.create_subscription(PointStamped, "/ball/pixel", self.on_ball, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.cmd = self.create_publisher(JointState, "/arm/joint_command", 10)
        self.create_service(SetBool, "/follow/enable", self.on_enable)
        self.create_timer(0.2, self.step)                        # 5 Hz
        self.get_logger().info("대기 중. 켜기: ros2 service call /follow/enable std_srvs/srv/SetBool \"{data: true}\"")

    def on_ball(self, msg: PointStamped):
        self.ball = (msg.point.x, msg.point.y, self.get_clock().now())

    def on_joints(self, msg: JointState):
        self.joints = dict(zip(msg.name, msg.position))

    def on_enable(self, req, res):
        if req.data and self.joints is None:
            res.success, res.message = False, "/joint_states 를 아직 못 받음 (arm_node 확인)"
            return res
        self.enabled = req.data
        if req.data:
            self.center = (self.joints[PAN], self.joints[TILT])
        res.success, res.message = True, "켜짐" if req.data else "꺼짐"
        self.get_logger().info(res.message)
        return res

    def step(self):
        if not self.enabled or self.ball is None or self.joints is None:
            return
        u, v, t = self.ball
        if (self.get_clock().now() - t).nanoseconds > 0.5e9:      # 공이 안 보이면 그 자리에서 멈춤
            return
        g = {n: self.get_parameter(n).value for n in ("width", "height", "gain", "deadband",
                                                       "pan_sign", "tilt_sign", "pan_range", "tilt_range")}
        ex = (u - g["width"] / 2) / (g["width"] / 2)             # -1 (왼쪽 끝) ~ 1 (오른쪽 끝)
        ey = (v - g["height"] / 2) / (g["height"] / 2)           # -1 (위) ~ 1 (아래)
        pan, tilt = self.joints[PAN], self.joints[TILT]
        if abs(ex) > g["deadband"]:
            pan += g["pan_sign"] * g["gain"] * ex
        if abs(ey) > g["deadband"]:
            tilt += g["tilt_sign"] * g["gain"] * ey
        c_pan, c_tilt = self.center
        pan = max(c_pan - g["pan_range"], min(c_pan + g["pan_range"], pan))
        tilt = max(c_tilt - g["tilt_range"], min(c_tilt + g["tilt_range"], tilt))
        if (pan, tilt) == (self.joints[PAN], self.joints[TILT]):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = [PAN, TILT], [pan, tilt]
        self.cmd.publish(msg)


def main():
    rclpy.init()
    node = FollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
