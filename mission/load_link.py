"""자동차 Pi 와의 통신 (팔이 차를 세우고 출발시키는 방식). 팀 레포 docs/interfaces.md (2026-11-10 합의) 를 그대로 따른다.

    팔 → 차 : /arm/loading_status  (common_interfaces/msg/LoadingStatus)   상태 + 정지/출발 신호
    차 → 팔 : /car/status          (common_interfaces/msg/CarStatus)       자동차의 지금 상태

자동차가 지키는 규칙 (메시지 정의에 적힌 것)
    STOP_REQUEST, IN_PROGRESS → 즉시 멈추고 서 있는다       SUCCESS, FAILED → 출발한다
    WAITING, TRACKING         → 평소대로 주행 (TRACKING 이면 천천히)

- 상태가 바뀌면 바로, 안 바뀌어도 1초마다 다시 보낸다 ("살아 있음" 신호. 10초 넘게 끊기면 차는 공 없이 출발한다).
- QoS: RELIABLE + TRANSIENT_LOCAL + depth 1 (보내는 쪽 · 받는 쪽이 똑같아야 한다).
- 팀마다 ROS_DOMAIN_ID 를 다르게 (docker/.env). 두 Pi 는 같은 값.

단어장
- 토픽(topic)  : ROS2 에서 메시지를 주고받는 통로의 이름
- QoS          : 메시지를 얼마나 확실하게 전달할지 정하는 설정
- FakeLoadLink : ROS2 없이 연습 · 시험할 때 쓰는 가짜 통신 (보낸 상태를 화면에 찍고 기록만 한다)
"""
from __future__ import annotations

import threading
import time

ARM_STATES = ["WAITING", "TRACKING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS", "FAILED"]      # 번호 = 순서 (0~5)
CAR_STATES = ["IDLE", "DRIVING", "STOPPED", "AT_LOADING_ZONE", "PARKED"]                     # 번호 = 순서 (0~4)
ARM_TOPIC = "/arm/loading_status"
CAR_TOPIC = "/car/status"


class LoadLink:
    """공통: 지금 상태(state, color, attempt)를 들고 있다가 발행한다. 실제 발행은 하위 클래스가 한다"""

    def __init__(self):
        self._lock = threading.Lock()
        self.state, self.color, self.attempt = "WAITING", "", 0
        self.car_state: str | None = None        # 자동차가 마지막으로 알려 준 상태 (받은 적 없으면 None)
        self.car_seen_at: float | None = None

    def set(self, state: str, color: str | None = None, attempt: int | None = None) -> None:
        if state not in ARM_STATES:
            raise ValueError(f"모르는 상태: {state}")
        with self._lock:
            self.state = state
            if color is not None:
                self.color = color if color in ("red", "blue") else ""
            if attempt is not None:
                self.attempt = max(0, min(255, int(attempt)))
        self._publish_now()

    def snapshot(self) -> dict:
        with self._lock:
            return {"state": self.state, "color": self.color, "attempt": self.attempt}

    def on_car(self, state: str) -> None:
        self.car_state = state
        self.car_seen_at = time.monotonic()

    def _publish_now(self) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class FakeLoadLink(LoadLink):
    """ROS2 없이: 보낸 상태를 history 에 쌓고, echo=True 면 화면에도 찍는다"""

    def __init__(self, echo: bool = False):
        super().__init__()
        self.history: list[dict] = []
        self.echo = echo

    def _publish_now(self) -> None:
        snap = self.snapshot()
        if not self.history or self.history[-1] != snap:
            self.history.append(snap)
            if self.echo:
                print(f"  [차에게 보냄] {snap['state']}  색={snap['color'] or '-'}  시도={snap['attempt']}", flush=True)

    def states(self) -> list[str]:
        return [h["state"] for h in self.history]


class RosLoadLink(LoadLink):
    """진짜 ROS2 (Pi 의 Docker, Humble). rclpy 와 팀의 common_interfaces 패키지가 빌드돼 있어야 한다"""

    def __init__(self, hz: float = 1.0):
        super().__init__()
        import rclpy
        from common_interfaces.msg import CarStatus, LoadingStatus
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self._rclpy, self._Msg = rclpy, LoadingStatus
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rclpy.init()
        self.node = Node("arm_loading")
        self.pub = self.node.create_publisher(LoadingStatus, ARM_TOPIC, qos)
        self.node.create_subscription(CarStatus, CAR_TOPIC, self._on_msg, qos)
        self.node.create_timer(1.0 / max(0.1, hz), self._publish_now)          # 1초마다 다시 보낸다
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def _on_msg(self, msg) -> None:
        if 0 <= msg.state < len(CAR_STATES):
            self.on_car(CAR_STATES[msg.state])

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self.node)
        except Exception:
            pass

    def _publish_now(self) -> None:
        snap = self.snapshot()
        msg = self._Msg()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.state = ARM_STATES.index(snap["state"])
        msg.color = snap["color"]
        msg.attempt = snap["attempt"]
        self.pub.publish(msg)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass
