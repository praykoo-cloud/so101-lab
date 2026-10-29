"""HiWonder MentorPi 차를 ROS2 표준 메시지로만 다루는 작은 도구.

차 컨테이너(IntelPi)에서 controller.launch.py (+ lidar.launch.py) 가 돌고 있고,
이 컴퓨터(so101-lab 컨테이너 · WSL)가 같은 ROS_DOMAIN_ID 이면 된다.
HiWonder 메시지 패키지는 필요 없다 (Twist · Odometry · Imu · LaserScan · UInt16).

    from carlib import Car
    car = Car()
    car.drive(vx=0.05, sec=2)      # 앞으로 0.05 m/s 로 2초 → 멈춤
    print(car.front())             # 앞쪽 ±15° 에서 가장 가까운 거리 (m)
    car.close()
"""
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import UInt16

MAX_V = 0.2       # m/s   차 쪽 /cmd_vel 은 이 값에서 자른다. /controller/cmd_vel 은 자르지 않으므로 여기서 자른다
MAX_W = 1.0       # rad/s
SELF = 0.10       # m     라이다가 차 자신(기둥 · 카메라 받침)을 보는 거리. 이보다 가까운 점은 버린다


def _interrupt(signum, frame):
    raise KeyboardInterrupt


class Car:
    def __init__(self, name="car_client"):
        if not rclpy.ok():
            # rclpy 가 Ctrl+C 를 받으면 먼저 꺼져서 마지막 0 을 못 보낸다 → 신호는 우리가 받는다
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            signal.signal(signal.SIGINT, _interrupt)
            signal.signal(signal.SIGTERM, _interrupt)     # docker stop · timeout
        self.node = rclpy.create_node(name)
        self.cmd = self.node.create_publisher(Twist, "/controller/cmd_vel", 10)
        self.battery_mv = None
        self.yaw_deg = None
        self.x = self.y = None
        self.scan = None
        self.node.create_subscription(UInt16, "/ros_robot_controller/battery", self._on_battery, 1)
        self.node.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.node.create_subscription(LaserScan, "/scan_raw", self._on_scan, 10)

    # ── 받기 ──
    def _on_battery(self, m):
        self.battery_mv = m.data

    def _on_imu(self, m):
        q = m.orientation
        self.yaw_deg = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _on_odom(self, m):
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y

    def _on_scan(self, m):
        self.scan = m

    def spin(self, sec):
        """sec 초 동안 토픽을 받는다 (time.sleep 대신 이것을 쓴다)"""
        end = time.time() + sec
        try:
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.02)
        except ExternalShutdownException:     # docker stop · timeout (SIGTERM) 도 Ctrl+C 처럼
            raise KeyboardInterrupt
        if not rclpy.ok():
            raise KeyboardInterrupt

    def wait_ready(self, timeout=5.0, lidar=False):
        """차의 토픽이 들어올 때까지. 안 들어오면 무엇이 빠졌는지 알려 준다"""
        def missing():
            need = [("battery", self.battery_mv), ("imu", self.yaw_deg), ("odom", self.x)]
            if lidar:
                need.append(("scan_raw (lidar.launch.py)", self.scan))
            return [k for k, v in need if v is None]

        end = time.time() + timeout
        while time.time() < end and missing():
            self.spin(0.1)
        if missing():
            raise RuntimeError(f"차 토픽이 안 들어온다: {missing()}. 차에서 controller.launch.py 가 돌고 있는지, "
                               "ROS_DOMAIN_ID 가 같은지 확인")

    def angle_range(self, deg_from, deg_to):
        """라이다에서 deg_from ~ deg_to (차 앞 = 0°, 왼쪽 = +90°) 사이 가장 가까운 거리. 없으면 None"""
        s = self.scan
        if s is None:
            return None
        best = None
        for i, r in enumerate(s.ranges):
            if not (max(s.range_min, SELF) < r < s.range_max):
                continue
            a = math.degrees(s.angle_min + i * s.angle_increment)
            a = (a + 180) % 360 - 180
            if deg_from <= a <= deg_to and (best is None or r < best):
                best = r
        return best

    def front(self, half=15):
        return self.angle_range(-half, half)

    # ── 보내기 ──
    def send(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x = max(-MAX_V, min(MAX_V, vx))
        t.linear.y = max(-MAX_V, min(MAX_V, vy))
        t.angular.z = max(-MAX_W, min(MAX_W, wz))
        self.cmd.publish(t)

    def stop(self):
        """차는 0 을 받아야 멈춘다. 마지막 명령을 계속 따르므로 끝낼 때 꼭 부른다"""
        for _ in range(3):
            self.send()
            self.spin(0.05)

    def drive(self, vx=0.0, vy=0.0, wz=0.0, sec=1.0, until=None):
        """sec 초 동안 10 Hz 로 보내고 멈춘다. until() 이 True 가 되면 일찍 멈춘다"""
        end = time.time() + sec
        try:
            while time.time() < end and rclpy.ok():
                if until is not None and until():
                    break
                self.send(vx, vy, wz)
                self.spin(0.1)
        finally:
            self.stop()

    def close(self):
        if rclpy.ok():
            self.stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
