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

ARM_MAX_SPEED = 3400                # STS3215 서보 최고 속도 (틱/초, 12V 에서 약 0.22초/60°). arm_fast 일 때 쓴다
DEFAULT_ACCEL = 32                  # 기본 드라이버(soarm) 가 연결할 때 쓰는 가속도
NO_PAUSE_TOL_DEG = 3.0              # arm_no_pause: 목표에서 이만큼 안에 들어오면 '도착' → 바로 다음 동작
NO_PAUSE_STILL_S = 0.15             # arm_no_pause: 쥔 공처럼 목표까지 못 가는 관절은 이만큼 멈춰 있으면 '도착' (기본 0.5초)


def run_speed(cfg: MissionConfig) -> int:
    """동작 속도: arm_fast 면 서보 최고 속도, 아니면 speed_slow"""
    return ARM_MAX_SPEED if cfg.arm_fast else cfg.speed_slow


def apply_accel(robot, cfg: MissionConfig) -> None:
    """설정에 맞는 가속도를 서보에 쓴다 (바뀔 때만). 가짜 팔(driver 없음)은 건너뛴다"""
    want = max(0, min(254, int(cfg.arm_accel))) if cfg.arm_fast else DEFAULT_ACCEL
    driver = getattr(robot, "driver", None)
    if driver is None or not hasattr(driver, "set_acceleration") or getattr(robot, "_mission_accel", None) == want:
        return
    for sid in JOINT_IDS:
        driver.set_acceleration(sid, want)
    robot._mission_accel = want


def settle(robot, cfg: MissionConfig, timeout: float) -> bool:
    """움직임이 끝날 때까지 기다린다. arm_no_pause 면 3° 안에 들어오면 바로 (멈칫 없음)"""
    if cfg.arm_no_pause:
        return robot.wait_until_settled(timeout=timeout, tolerance_deg=NO_PAUSE_TOL_DEG, still_seconds=NO_PAUSE_STILL_S)
    return robot.wait_until_settled(timeout=timeout)


def play_options(cfg: MissionConfig) -> dict:
    """motions.play 에 넘길 속도 · 멈칫 설정"""
    if cfg.arm_no_pause:
        return {"speed": run_speed(cfg), "tolerance_deg": NO_PAUSE_TOL_DEG, "still_seconds": NO_PAUSE_STILL_S,
                "no_pause": True}
    return {"speed": run_speed(cfg)}


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
        apply_accel(self.robot, self.cfg)
        self.robot.speed = run_speed(self.cfg) if speed is None else speed
        self.robot.go_pose(pose, speed=self.robot.speed, wait=False)
        return settle(self.robot, self.cfg, timeout) if wait else True

    def grip(self, pct: float, wait: bool = True) -> None:
        self.robot.gripper(pct, speed=run_speed(self.cfg))
        if wait:
            settle(self.robot, self.cfg, 4.0)
            if not self.cfg.arm_no_pause:
                time.sleep(0.3)

    def grip_pct(self) -> float | None:
        return self.robot.gripper_percent()

    # ── 받침(1번 관절)만 조금씩 돌려 차를 따라간다 ──────────────────────
    def pan_angle(self) -> float | None:
        return self.robot.angles().get(1)

    def nudge_pan(self, delta_deg: float, center_deg: float, limit_deg: float) -> float | None:
        """받침을 delta 만큼 돌린다. center ± limit 밖으로는 절대 안 간다 (안전). 돌린 목표 각도를 돌려준다"""
        current = self.pan_angle()
        if current is None:
            return None
        target = max(center_deg - limit_deg, min(center_deg + limit_deg, current + delta_deg))
        if abs(target - current) < 0.05:
            return current
        self.robot.move(1, target, speed=self.cfg.speed_slow)      # 기다리지 않고 명령만 (다음 프레임에서 다시 본다)
        return target

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
