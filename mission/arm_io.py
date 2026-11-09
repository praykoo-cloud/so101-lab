"""로봇팔을 미션 용어로 감싼 것 (soarm.Robot 위에 얇게). 모든 움직임은 티칭한 자세 이름으로 한다.

    arm = ArmIO.open(cfg)            # cfg.port == "fake" 면 가상 팔
    arm.torque_on()
    arm.go("wait")
    ...

주의
- 포트는 한 프로그램만 쓴다 (ArmStudio · 다른 터미널이 쓰고 있으면 연결 실패).
- 이 파일은 실물 팔을 움직인다. 처음에는 cfg.port = "fake" 또는 환경변수 SOARM_PORT=fake 로 흐름만 확인할 것.
"""
from __future__ import annotations

import time

from soarm import Robot
from soarm.driver_sdk import JOINT_IDS

from .config import MissionConfig


class ArmIO:
    def __init__(self, robot: Robot, cfg: MissionConfig):
        self.robot = robot
        self.cfg = cfg

    @classmethod
    def open(cls, cfg: MissionConfig) -> "ArmIO":
        return cls(Robot(cfg.port, speed=cfg.speed_slow), cfg)

    # ── 상태 ────────────────────────────────────────────────────────────
    def preflight(self, needed_poses: list[str]) -> list[str]:
        """시작 전 점검. 문제 목록을 돌려준다 (비어 있으면 통과)"""
        problems = []
        saved = self.robot.poses()
        for name in needed_poses:
            if name not in saved:
                problems.append(f"자세 '{name}' 이 티칭되지 않았습니다 (m0_teach_poses.py)")
        for sid in JOINT_IDS:
            t = self.robot.driver.get_temperature(sid)
            if t is not None and t >= self.cfg.temp_stop:
                problems.append(f"모터 {sid} 온도 {t}도 (중단 기준 {self.cfg.temp_stop})")
        return problems

    def max_temperature(self) -> float | None:
        temps = [self.robot.driver.get_temperature(sid) for sid in JOINT_IDS]
        temps = [t for t in temps if t is not None]
        return max(temps) if temps else None

    def too_hot(self) -> bool:
        t = self.max_temperature()
        return t is not None and t >= self.cfg.temp_stop

    def warm(self) -> bool:
        t = self.max_temperature()
        return t is not None and t >= self.cfg.temp_warn

    def torque_on(self) -> None:
        self.robot.torque(True)

    def torque_off(self) -> None:
        self.robot.torque(False)

    def hold(self) -> None:
        """그 자리에서 멈춘다 (목표 = 지금 위치)"""
        self.robot.stop()

    def close(self) -> None:
        self.robot.close()

    # ── 움직이기 ────────────────────────────────────────────────────────
    def go(self, pose: str, speed: int | None = None, wait: bool = True, timeout: float = 12.0) -> bool:
        """티칭한 자세로 간다 (그리퍼는 그대로). 도착하면 True"""
        self.robot.speed = self.cfg.speed_slow if speed is None else speed
        self.robot.go_pose(pose, speed=self.robot.speed, wait=False)
        return self.robot.wait_until_settled(timeout=timeout) if wait else True

    def grip(self, pct: float, wait: bool = True) -> None:
        self.robot.gripper(pct, speed=self.cfg.speed_slow)
        if wait:
            self.robot.wait_until_settled(timeout=4.0)
            time.sleep(0.3)

    def grip_pct(self) -> float | None:
        return self.robot.gripper_percent()

    # ── 자세 확인 (clear 를 내기 전 "실제로 그 자세인가") ────────────────
    def pose_error_deg(self, pose: str) -> float | None:
        """지금 관절과 저장된 자세의 가장 큰 각도 차이 (그리퍼 제외). 못 읽으면 None"""
        saved = self.robot.poses().get(pose)
        if saved is None:
            return None
        now = self.robot.angles()
        worst = 0.0
        for sid, target in saved["angles"].items():
            if sid == 6 or target is None:
                continue
            cur = now.get(sid)
            if cur is None:
                return None
            worst = max(worst, abs(cur - target))
        return worst

    def at_pose(self, pose: str, tol_deg: float | None = None) -> bool:
        err = self.pose_error_deg(pose)
        return err is not None and err <= (self.cfg.pose_tolerance_deg if tol_deg is None else tol_deg)

    def nearest_pose(self, candidates: list[str], tol_deg: float | None = None) -> str | None:
        """candidates 중 지금 자세와 허용 오차 안에서 가장 가까운 것 (없으면 None)"""
        best = None
        for name in candidates:
            err = self.pose_error_deg(name)
            if err is not None and err <= (self.cfg.pose_tolerance_deg if tol_deg is None else tol_deg):
                if best is None or err < best[0]:
                    best = (err, name)
        return best[1] if best else None
