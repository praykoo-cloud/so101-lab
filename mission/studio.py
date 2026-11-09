"""ArmStudio(PC 앱)의 가상 팔 · 가상 카메라에 미션을 연결하는 어댑터. 팔 · Pi 없이 이 PC 에서 연습한다.

ArmStudio 앱을 켜 두면 http://127.0.0.1:8000 으로 SDK(armstudio.Robot)가 연결된다.
- StudioRobot : soarm.Robot 과 같은 이름의 메서드를 흉내 → ArmIO · 티칭 도구가 그대로 돈다
- StudioCap   : cv2.VideoCapture 처럼 read() / release() → 손목(wrist) 가상 카메라 영상을 준다

사용
    python -m mission.m0_teach_poses --studio          # 앱의 가상 팔로 자세 티칭
    python -m mission.m0_teach_rois --studio --only note
    python -m mission.m0_teach_colors teach note_red --studio
    python -m mission.mission_main --studio            # 가상 팔 + 가상 카메라로 미션

주의
- 앱 창이 열려 있어야 카메라가 나온다 (앱 화면이 가상 카메라를 그린다).
- 자세는 앱 안에 저장된다 (앱의 티칭 탭 · 이 도구 모두 같은 저장소). 이름은 mission.config.POSE_HELP 와 같아야 한다.
- 가상 팔은 실물과 달리 부딪힘 · 모터 온도가 단순하다. 실물에서 다시 확인해야 한다.
"""
from __future__ import annotations

import math
import os
import sys

RAD2DEG = 180.0 / math.pi


def _import_sdk():
    try:
        from armstudio import Robot, RobotError
    except ImportError:
        sys.path.insert(0, os.path.join(os.path.expanduser("~"), "ArmStudio"))
        from armstudio import Robot, RobotError        # noqa: F811
    return Robot, RobotError


class _Driver:
    """soarm.Robot.driver 흉내 (온도만)"""

    def __init__(self, arm):
        self._arm = arm

    def get_temperature(self, servo_id: int):
        try:
            return self._arm.get_temperature(servo_id)
        except Exception:
            return None


class StudioRobot:
    real = False

    def __init__(self, url: str | None = None, speed: int = 120):
        Robot, self._RobotError = _import_sdk()
        self._arm = Robot(url, name="mission")
        self.speed = speed                         # 서보 틱/초 (실물 Robot 과 같은 의미). 이동 시간으로 환산
        self.driver = _Driver(self._arm)
        self.port = "ArmStudio"

    def __repr__(self):
        return f"StudioRobot({self._arm.url})"

    # 이동 시간: speed 120 → 3 초, 200 → 1.8 초 (실물 틱 속도와 비슷한 느낌)
    def _duration(self, speed: int | None) -> float:
        sp = self.speed if speed is None else speed
        return max(1.0, 360.0 / max(1, sp))

    # ── 자세 ────────────────────────────────────────────────────────────
    def poses(self) -> dict:
        raw = self._arm._req("GET", "/v1/poses")
        out = {}
        for name, p in raw.items():
            q = p.get("q") or []
            out[name] = {**p, "angles": {i + 1: round(v * RAD2DEG, 2) for i, v in enumerate(q)}}
        return out

    def save_pose(self, name: str):
        self._arm.save_pose(name)

    def go_pose(self, name: str, speed: int | None = None, wait: bool = True, timeout: float = 15.0,
                gripper: bool = False):
        if name not in self.poses():
            from soarm import RobotError
            raise RobotError(f"자세 '{name}' 이 앱에 없습니다. 있는 것: {sorted(self.poses())}")
        self._arm.go_pose(name, duration=self._duration(speed), wait=False)
        if wait:
            self.wait_until_settled(timeout=timeout)

    # ── 읽기 ────────────────────────────────────────────────────────────
    def angles(self) -> dict:
        q = self._arm.get_joints()
        out = {i + 1: v for i, v in enumerate(q)}
        out[6] = None                                   # 그리퍼는 gripper_percent() 로
        return out

    def gripper_percent(self):
        return round(float(self._arm.get_gripper()), 1)

    def wait_until_settled(self, tolerance_deg: float = 0.5, timeout: float = 5.0) -> bool:
        return self._arm.wait(timeout)

    # ── 움직이기 ────────────────────────────────────────────────────────
    def gripper(self, percent: float, speed: int | None = None):
        self._arm.gripper(float(percent), wait=True)

    def torque(self, enable: bool, joint=None):
        self._arm.torque(bool(enable))
        self._torque = bool(enable)

    def is_locked(self, joint=None) -> bool:
        st = self._arm._state().get("torque")
        if isinstance(st, bool):
            return st
        if isinstance(st, dict):
            return all(bool(v) for v in st.values())
        if isinstance(st, (list, tuple)):
            return all(bool(v) for v in st)
        return getattr(self, "_torque", True)

    def stop(self):
        self._arm.stop()

    def close(self):
        pass


class StudioCap:
    """cv2.VideoCapture 처럼 쓰는 가상 카메라 (손목 카메라가 기본)"""

    def __init__(self, camera: str = "wrist", width: int = 640, height: int = 480, url: str | None = None):
        Robot, self._RobotError = _import_sdk()
        self._arm = Robot(url, name="mission-cam")
        self.camera, self.w, self.h = camera, width, height

    def read(self):
        try:
            img = self._arm.get_image(self.camera, self.w, self.h)
        except self._RobotError:
            return False, None                          # 앱 창이 닫혀 있으면 503
        if isinstance(img, bytes) or img is None:
            return False, None
        return True, img

    def release(self):
        pass


def open_capture(cfg, studio: bool):
    """실물 카메라(soarm.color.open_camera) 또는 ArmStudio 가상 카메라"""
    if studio:
        return StudioCap(getattr(cfg, "studio_camera", "wrist"), cfg.width, cfg.height)
    from soarm.color import open_camera
    return open_camera(cfg.camera, cfg.width, cfg.height)
