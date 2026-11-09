"""자동차 쪽 흉내 (시험용). 실제 자동차 코드는 자동차 담당자가 만든다 — 이 파일은 "약속" 을 보여 주는 예시.

    ros2 run 이 아니라:  python -m mission.car_stub      (ROS2 가 있는 곳, 팔과 같은 ROS_DOMAIN_ID)

자동차가 지킬 약속 (기획안 D7)
1. 상차 구역에 서면 스스로 출발을 잠근다 (팔 메시지와 무관).
2. 출발 조건: 현재 job 의 확정 결과(done) + 같은 job 의 clear + 자기 잠금 상태. ack 는 출발 조건이 아니다.
3. 받은 결과는 먼저 저장하고 나서 ack 한다 (여기서는 화면 출력만).
4. 재시작하면 hello 를 보내고, 저장된 "출발 허가" 를 그대로 실행하지 않는다. 새 clear 를 받아야 한다.
"""
import json
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CarStub(Node):
    def __init__(self):
        super().__init__("car_stub")
        self.boot = uuid.uuid4().hex[:8]
        self.locked_job = None
        self.last = {}
        self.released = False
        self.ack_pub = self.create_publisher(String, "/car/ack", 10)
        self.hello_pub = self.create_publisher(String, "/car/hello", 10)
        self.create_subscription(String, "/arm/state", self.on_state, 10)
        self.create_timer(1.0, self.tick)
        self.hello_pub.publish(String(data=json.dumps({"car_boot_id": self.boot})))
        self.get_logger().info(f"car_stub 시작 boot={self.boot}")

    def ack(self, job_id, step):
        self.ack_pub.publish(String(data=json.dumps({"job_id": job_id, "step": step, "car_boot_id": self.boot})))

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.last = s
        if s.get("job_id", 0) and self.locked_job != s["job_id"] and not self.released:
            self.locked_job = s["job_id"]                       # 정지·잠금 상태에 들어감
        if s.get("done") and s.get("result"):
            self.ack(s["job_id"], "done")
        if s.get("clear"):
            self.ack(s["job_id"], "clear")

    def tick(self):
        s = self.last
        if (not self.released and s.get("job_id") == self.locked_job and s.get("done")
                and s.get("result") in ("ok", "fail_attempts") and s.get("clear") and not s.get("status")):
            self.released = True
            self.get_logger().info(f"출발 허가! job={s['job_id']} 결과={s['result']}")


def main():
    rclpy.init()
    node = CarStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
