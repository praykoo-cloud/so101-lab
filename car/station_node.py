"""station_node — 팔 쪽 '공 넣어 주는 곳'. 차가 서비스를 부르면: 바구니 색 보기 → 같은 색 공 집기 → 바구니에 넣기.

    ros2 run so101_ros arm_node --ros-args -p port:=/dev/ttyACM0          # 팔 (터미널 1)
    ros2 run so101_ros camera_node --ros-args -p device:=0                 # 집게 카메라 (터미널 2)
    ros2 run so101_ros color_node                                          # 색 판별 → /color/detected
    python3 car/station_node.py                                            # 이 노드 (터미널 3)

차(또는 car/car_station_mock.py)가 부르는 것:
    서비스 /arm_station/serve   std_srvs/Trigger   → success, message = "red" · "blue" 또는 실패 까닭
    토픽   /arm_station/state   std_msgs/String     idle · looking · picking · placing · done · error
std_srvs · std_msgs 만 쓴다 → 차 컨테이너에 so101 패키지를 깔지 않아도 부를 수 있다.

색은 /color/detected (std_msgs/String) 를 듣는다. YOLO 로 바꾸려면 그 노드가 같은 토픽에 "red" · "blue" 를 내면 된다.
팔은 arm_node 의 서비스(/arm/go_pose · /arm/gripper)로만 움직인다 → 팔 포트를 여는 노드는 arm_node 하나.

미리 저장할 자세 (lessons/06 이나 /arm/save_pose):
    look_basket  집게 카메라로 바구니를 내려다보는 자세 (대기도 여기)
    above_red · pick_red · above_blue · pick_blue   공 받침대 위 · 집는 자리
    above_basket 바구니 위 (여기서 놓는다)
"""
import time
from collections import Counter

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_interfaces.srv import GoPose, SetGripper

COLORS = ("red", "blue")
LOOK_SECONDS = 3.0        # 바구니를 이만큼 보고 가장 많이 나온 색으로 정한다
MIN_VOTES = 5             # 그중 이만큼은 같은 색이어야 (손 · 그림자로 한두 번 튄 것은 버림)
SPEED = 150


class Station(Node):
    def __init__(self):
        super().__init__("station_node")
        group = ReentrantCallbackGroup()             # 서비스 안에서 다른 서비스를 기다리려면 재진입 가능해야 한다
        self.go = self.create_client(GoPose, "/arm/go_pose", callback_group=group)
        self.grip = self.create_client(SetGripper, "/arm/gripper", callback_group=group)
        self.seen = []                               # (시각, 색)
        self.create_subscription(String, "/color/detected", self.on_color, 10, callback_group=group)
        self.state = self.create_publisher(String, "/arm_station/state", 10)
        self.create_service(Trigger, "/arm_station/serve", self.on_serve, callback_group=group)
        self.busy = False
        self.say("idle")

    def say(self, s):
        self.state.publish(String(data=s))
        self.get_logger().info(s)

    def on_color(self, msg):
        self.seen.append((time.time(), msg.data))
        self.seen = self.seen[-200:]

    def call(self, client, req, timeout=20.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{client.srv_name} 가 없다 (arm_node 가 도는지)")
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done():
            if time.time() > end:
                raise RuntimeError(f"{client.srv_name} 응답 없음")
            time.sleep(0.05)
        res = fut.result()
        if not res.success:
            raise RuntimeError(f"{client.srv_name}: {res.message}")
        return res

    def pose(self, name):
        self.call(self.go, GoPose.Request(name=name, speed=SPEED))

    def gripper(self, percent):
        self.call(self.grip, SetGripper.Request(percent=float(percent)))

    def look(self):
        """LOOK_SECONDS 동안 들어온 색을 세어 정한다. 없으면 None"""
        start = time.time()
        time.sleep(LOOK_SECONDS)
        votes = Counter(c for t, c in self.seen if t >= start and c in COLORS)
        self.get_logger().info(f"본 색: {dict(votes)}")
        if not votes:
            return None
        color, n = votes.most_common(1)[0]
        return color if n >= MIN_VOTES else None

    def on_serve(self, req, res):
        if self.busy:
            res.success, res.message = False, "이미 하는 중"
            return res
        self.busy = True
        try:
            self.say("looking")
            self.pose("look_basket")
            color = self.look()
            if color is None:
                raise RuntimeError("바구니에서 빨강 · 파랑을 못 봄")
            self.say("picking")
            self.gripper(100)
            self.pose(f"above_{color}")
            self.pose(f"pick_{color}")
            self.gripper(0)
            self.pose(f"above_{color}")
            self.say("placing")
            self.pose("above_basket")
            self.gripper(100)
            self.pose("look_basket")
            self.say("done")
            res.success, res.message = True, color
        except RuntimeError as e:
            self.say("error")
            res.success, res.message = False, str(e)
            self.get_logger().error(str(e))
        finally:
            self.busy = False
        return res


def main():
    rclpy.init()
    node = Station()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
