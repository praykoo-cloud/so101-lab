"""task_node — 색 분류 순서와 판단. 로봇 · 카메라 코드가 없다: 토픽 하나와 서비스만 쓴다.

    ros2 run so101_ros task_node

구독  /color/detected   (color_node)
호출  /arm/go_pose · /arm/gripper   (arm_node)

프로젝트 4(p4_sort.py)의 상태 기계를 그대로 옮겼다.
  WAIT  같은 색 stable 번 연속 → PICK → PLACE → CLEAR  검사 자리가 none 이 될 때까지 → WAIT
"""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from so101_interfaces.srv import GoPose, SetGripper


class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")
        self.open = float(self.declare_parameter("open", 60.0).value)
        self.close = float(self.declare_parameter("close", 5.0).value)
        self.fast = int(self.declare_parameter("fast", 200).value)
        self.slow = int(self.declare_parameter("slow", 80).value)
        self.stable = int(self.declare_parameter("stable", 5).value)
        self.go = self.create_client(GoPose, "/arm/go_pose")
        self.grip = self.create_client(SetGripper, "/arm/gripper")
        self.torque = self.create_client(SetBool, "/arm/torque")
        self.create_subscription(String, "/color/detected", self.on_color, 10)
        self.color, self.count = None, 0
        self.lock = threading.Lock()

    def on_color(self, msg: String):
        with self.lock:
            self.count = self.count + 1 if msg.data == self.color else 1
            self.color = msg.data

    def seen(self):
        with self.lock:
            return self.color, self.count

    def say(self, text):
        self.get_logger().info(text)

    def call(self, client, req):
        """서비스 응답을 기다린다. client.call() 은 끝없이 기다려서, Ctrl+C 로 arm_node 가 먼저 꺼지면 이 노드가 내려가지 않았다"""
        fut = client.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        if not fut.done():
            raise KeyboardInterrupt
        return fut.result()

    def pose(self, name, speed):
        res = self.call(self.go, GoPose.Request(name=name, speed=speed))
        if not res.success:
            raise RuntimeError(res.message)

    def gripper(self, pct):
        res = self.call(self.grip, SetGripper.Request(percent=pct))
        if not res.success:
            raise RuntimeError(res.message)
        return res.load

    def run(self):
        """상태 기계. 서비스 응답(call)을 기다리므로 spin 과 다른 스레드에서 돈다"""
        for c in (self.go, self.grip, self.torque):
            while rclpy.ok() and not c.wait_for_service(timeout_sec=2.0):
                self.say(f"기다리는 중: {c.srv_name} (arm_node 가 떠 있나?)")
        self.call(self.torque, SetBool.Request(data=True))    # 잠그기 (arm_node 는 목표 = 지금 위치로 맞춘 뒤 켠다)
        self.pose("home", self.fast)
        state, done = "WAIT", {"red": 0, "blue": 0}
        self.say("WAIT: 검사 자리에 물체를 놓으세요")
        while rclpy.ok():
            color, count = self.seen()
            if state == "WAIT" and count >= self.stable and color in ("red", "blue"):
                self.say(f"PICK: {color}")
                self.pose("home", self.fast)
                self.gripper(self.open)
                self.pose("above_pick", self.fast)
                self.pose("pick", self.slow)
                load = self.gripper(self.close)
                self.pose("above_pick", self.slow)
                self.say(f"PLACE: {color} 상자 (그리퍼 부하 {load})")
                self.pose(f"above_{color}", self.fast)
                self.pose(color, self.slow)
                self.gripper(self.open)
                self.pose(f"above_{color}", self.slow)
                self.pose("home", self.fast)
                done[color] += 1
                self.say(f"끝: 빨강 {done['red']} · 파랑 {done['blue']}")
                state = "CLEAR"
                with self.lock:
                    self.count = 0
            elif state == "CLEAR" and count >= self.stable and color == "none":
                self.say("WAIT: 다음 물체를 놓으세요")
                state = "WAIT"
            time.sleep(0.1)


def main():
    rclpy.init()
    node = TaskNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        node.get_logger().error(f"멈춤: {e}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
