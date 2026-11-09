"""자동차 Pi 와의 통신 (ROS2, std_msgs/String 안에 JSON). 두 Pi 사이 통신만 사용한다.

팔이 계속 발행하는 상태 메시지 `/arm/state` (기본 1 Hz, 값이 바뀌면 바로):
    {"arm_boot_id": "...", "job_id": 3, "step": "working|done|clear|abort|idle|wait",
     "result": null|"ok"|"fail_attempts", "done": false, "clear": false, "status": "", "t": 1730000000.0}
자동차가 보내는 것:
    `/car/ack`   {"job_id": 3, "step": "done"|"clear", "car_boot_id": "..."}   (받았다는 응답)
    `/car/hello` {"car_boot_id": "..."}                                          (자동차가 (재)시작했을 때)

규칙 (기획안 D7)
- 팔의 동작은 ack 를 기다리지 않는다 (ack 가 유실돼도 팔이 멈추지 않는다). 발행은 별도 스레드에서 계속한다.
- 자동차는 "확정 결과(done) + 현재 job 의 clear + 자신의 상차 대기 상태" 가 모두 맞을 때만 출발한다 (자동차 쪽 구현).
- 이 통신은 팔의 카메라 도착 감지를 대신하지 않는다. 시작 조건에 쓰지 않는다.
- 팀 전용 ROS_DOMAIN_ID (환경변수) 를 두 Pi 에 같은 값으로 설정한다.
"""
from __future__ import annotations

import json
import threading
import time
import uuid


def new_boot_id() -> str:
    return uuid.uuid4().hex[:8]


class Link:
    """공통 동작: 상태 필드를 합쳐서 들고 있다가 발행한다. 하위 클래스가 실제 발행을 맡는다"""

    def __init__(self):
        self.boot_id = new_boot_id()
        self._state = {"arm_boot_id": self.boot_id, "job_id": 0, "step": "idle", "result": None,
                       "done": False, "clear": False, "status": ""}
        self._lock = threading.Lock()
        self.acks: set[tuple] = set()          # (job_id, step, car_boot_id)
        self.car_boot_id: str | None = None

    def set_state(self, **fields) -> None:
        with self._lock:
            self._state.update(fields)
        self._publish_now()

    def state(self) -> dict:
        with self._lock:
            return dict(self._state)

    def acked(self, job_id: int, step: str) -> bool:
        return any(a[0] == job_id and a[1] == step for a in self.acks)

    def on_ack(self, msg: dict) -> None:
        self.car_boot_id = msg.get("car_boot_id", self.car_boot_id)
        self.acks.add((msg.get("job_id"), msg.get("step"), msg.get("car_boot_id")))

    def on_hello(self, msg: dict) -> None:
        """자동차가 (재)시작했다: 이전 ack 는 믿지 않고 현재 상태를 다시 발행한다"""
        cb = msg.get("car_boot_id")
        if cb != self.car_boot_id:
            self.car_boot_id = cb
            self.acks.clear()
        self._publish_now()

    def _publish_now(self) -> None:           # 하위 클래스가 채운다
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class FakeLink(Link):
    """시험 · 가상 실행용. 발행한 상태를 모두 기록한다 (history)"""

    def __init__(self):
        super().__init__()
        self.history: list[dict] = []

    def _publish_now(self) -> None:
        self.history.append({**self.state(), "t": time.time()})


class RosLink(Link):
    """진짜 ROS2 (Pi 의 Docker, Humble). rclpy 가 있어야 한다"""

    def __init__(self, hz: float = 1.0):
        super().__init__()
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        self._rclpy, self._String = rclpy, String
        rclpy.init()
        self.node = Node("arm_mission_link")
        self.pub = self.node.create_publisher(String, "/arm/state", 10)
        self.node.create_subscription(String, "/car/ack", lambda m: self._safe(self.on_ack, m), 10)
        self.node.create_subscription(String, "/car/hello", lambda m: self._safe(self.on_hello, m), 10)
        self.node.create_timer(1.0 / max(0.1, hz), self._publish_now)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    @staticmethod
    def _safe(fn, msg):
        try:
            fn(json.loads(msg.data))
        except (ValueError, TypeError):
            pass                              # 이상한 메시지는 무시

    def _spin(self):
        try:
            self._rclpy.spin(self.node)
        except Exception:
            pass

    def _publish_now(self) -> None:
        self.pub.publish(self._String(data=json.dumps({**self.state(), "t": time.time()})))

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        try:
            self.node.destroy_node()
            self._rclpy.try_shutdown()
        except Exception:
            pass
