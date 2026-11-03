"""SO-ARM101 를 USB 로 바로 움직이는 학생용 Robot.

ArmStudio 앱의 SDK 와 하는 일은 같고 이름이 일부 다르다 (get_joints → angles, move_joint → move).
그쪽은 앱에 HTTP 로 보내고, 이쪽은 driver_sdk.py 로 시리얼 포트를 직접 연다.
그래서 앱 없이, 팔을 꽂은 PC · 라즈베리파이에서 바로 돈다.

    from soarm import Robot

    bot = Robot("COM4")              # Windows.  리눅스 · 라즈베리파이는 "/dev/ttyACM0"
    bot.torque(True)                 # 잠그기 (목표 = 지금 위치로 맞춘 뒤 켬 → 튀지 않음)
    bot.move("base", 10)             # 관절 하나, 도 (0° = 가운데)
    bot.save_pose("home")            # 지금 자세를 data/poses.json 에 저장
    bot.go_pose("home")
    print(bot.angles())              # {1: -9.5, 2: -100.8, ...}

자세 · 녹화 파일은 컨트롤러 앱과 같은 형식(관절 가동범위 안의 0~1 비율)이라 두 쪽에서 서로 읽는다.
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Union

from . import driver_sdk as drv
from .driver_sdk import (
    JOINT_IDS, JOINT_LIMITS, JOINT_NAMES, POS_CENTER, POS_MAX, POS_MIN, POS_RANGE,
    STS3215Driver, position_from_fraction, unwrap_position, window_fraction, wrap_position,
)

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "data"            # driver_sdk 가 calibration.json 을 읽는 곳과 같다
POSES_FILE = DATA_DIR / "poses.json"
RECORDINGS_DIR = DATA_DIR / "recordings"
POSE_FORMAT = "normalized"          # 컨트롤러 앱 backend/controller.py 와 같은 값

DEFAULT_SPEED = 200                 # 서보 목표 속도 (틱/초). 200 ≈ 17.6°/s
DEFAULT_ACCEL = 32

# 관절 이름 → id. soarm.py 와 같은 이름 + LeRobot 이름도 받는다
JOINT_BY_NAME = {
    "base": 1, "shoulder_pan": 1,
    "shoulder": 2, "shoulder_lift": 2,
    "elbow": 3, "elbow_flex": 3,
    "wrist_pitch": 4, "wrist": 4, "wrist_flex": 4,
    "wrist_roll": 5, "roll": 5,
    "gripper": 6,
}


class RobotError(RuntimeError):
    pass


def _resolve_joint(joint: Union[int, str]) -> int:
    if isinstance(joint, int):
        if joint in JOINT_IDS:
            return joint
    else:
        key = str(joint).strip().lower()
        if key.isdigit() and int(key) in JOINT_IDS:
            return int(key)
        if key in JOINT_BY_NAME:
            return JOINT_BY_NAME[key]
    raise ValueError(f"관절 '{joint}' 을 모릅니다. 1~6 또는 {sorted(set(JOINT_BY_NAME))}")


def deg_to_pos(deg: float) -> int:
    return drv.STS3215Driver.degrees_to_position(float(deg))


def pos_to_deg(pos: Optional[int]) -> Optional[float]:
    return drv.STS3215Driver.position_to_degrees(pos)


def find_ports() -> list[str]:
    """USB 시리얼 포트 중 SO-ARM101 보드(CH340/CH343 등)로 보이는 것"""
    from serial.tools import list_ports
    ids = {(0x1A86, 0x55D3), (0x1A86, 0x7523), (0x1A86, 0x55D4), (0x0403, 0x6001), (0x10C4, 0xEA60)}
    out = [p.device for p in list_ports.comports() if (p.vid, p.pid) in ids]
    return sorted(out)


def default_port() -> Optional[str]:
    """포트를 안 적었을 때: 환경변수 SOARM_PORT → 보드가 하나만 보이면 그 포트. 정할 수 없으면 None"""
    port = os.environ.get("SOARM_PORT")
    if port:
        return port
    ports = find_ports()
    return ports[0] if len(ports) == 1 else None


class Robot:
    """SO-ARM101 한 대. 포트 하나 = 팔 하나 (팔로워든 리더든)."""

    def __init__(self, port: Optional[str] = None, *, speed: int = DEFAULT_SPEED,
                 accel: int = DEFAULT_ACCEL, connect: bool = True):
        if port is None:
            port = default_port()
        if port is None:
            raise RobotError(f"포트를 정해 주세요: Robot('COM4') 처럼. 보이는 포트: {find_ports() or '없음'}")
        self.port = port
        self.real = str(port).lower() != "fake"   # False = 가상 팔
        self.speed = int(speed)
        self.accel = int(accel)
        self.driver = STS3215Driver(port=port)
        if str(port).lower() == "fake":       # 가상 팔 (soarm/fake.py): 실물 없이 같은 코드를 돌린다
            from .fake import FOLDED, FakeBus, FakeSerial
            FakeSerial.bus = FakeSerial.bus or FakeBus(present=FOLDED)

            def _fake_connect(d=self.driver):
                d.serial = FakeSerial()
                return True
            self.driver.connect = _fake_connect
        self._last_speed: dict[int, int] = {}
        self._goals: dict[int, int] = {}      # 마지막으로 보낸 목표 (wait_until_settled 가 쓴다)
        if connect:
            self.connect()

    # ── 연결 ─────────────────────────────────────────────────────────────
    def connect(self) -> "Robot":
        if not self.driver.connect():
            raise RobotError(f"{self.port} 을 열 수 없습니다. 포트 이름, 다른 프로그램(LeRobot · ArmStudio)이 "
                             f"쓰고 있지 않은지 확인하세요")
        missing = [sid for sid in JOINT_IDS if not self.driver.ping(sid)]
        if missing:
            self.driver.disconnect()
            raise RobotError(f"{self.port}: 모터 {missing} 이 응답하지 않습니다. 전원 · 3핀 케이블 확인")
        # 토크는 그대로 둔다. 여기서 목표 위치를 쓰지 않는다 — 토크가 꺼진 서보에 목표를 쓰면
        # 서보가 토크를 스스로 켠다 (실측: 6개 모두 켜짐). 예전 목표는 torque(True) 할 때 지운다.
        for sid in JOINT_IDS:
            self.driver.set_acceleration(sid, self.accel)
            self._set_speed(sid, self.speed)
        return self

    def close(self) -> None:
        """포트를 닫는다. 토크는 바꾸지 않는다 (잠긴 팔은 그 자세를 유지)."""
        self.driver.disconnect()

    disconnect = close

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"Robot({self.port!r}, {'실물' if self.real else '가상 팔'})"

    # ── 읽기 ─────────────────────────────────────────────────────────────
    def positions(self) -> dict[int, Optional[int]]:
        """서보 원시값 0~4095"""
        return self.driver.get_all_positions()

    def angles(self) -> dict[int, Optional[float]]:
        """관절 각도 (도). 0° = 서보 가운데(2048). id 6 은 그리퍼 서보 각도"""
        return {sid: pos_to_deg(p) for sid, p in self.positions().items()}

    def joint_radians(self) -> dict[int, Optional[float]]:
        """URDF 관절 각도 (rad) — ROS2 /joint_states · RViz 용.
        컨트롤러 앱 디지털 트윈(twin.js _mapPosToRad)과 같은 식: 캘리브레이션 범위를 URDF 범위에 겹친다"""
        out = {}
        for sid, pos in self.positions().items():
            lim = JOINT_LIMITS.get(sid) or {}
            t = window_fraction(pos, lim)
            if pos is None:
                rad = None
            elif t is not None and "rad_min" in lim and "rad_max" in lim:
                rad = lim["rad_min"] + t * (lim["rad_max"] - lim["rad_min"])
            else:
                rad = (pos - POS_CENTER) / 4095 * 2 * 3.141592653589793
            out[sid] = -rad if (rad is not None and lim.get("invert")) else rad
        return out

    def gripper_percent(self) -> Optional[float]:
        """그리퍼 열린 정도 0(닫힘)~100(열림). data/calibration.json 의 범위 기준"""
        t = window_fraction(self.driver.get_position(6), JOINT_LIMITS.get(6))
        return None if t is None else round(t * 100, 1)

    def status(self) -> dict[int, dict]:
        return {sid: s.to_dict() for sid, s in self.driver.get_all_status().items()}

    def is_locked(self, joint: Union[int, str, None] = None) -> bool:
        ids = JOINT_IDS if joint is None else [_resolve_joint(joint)]
        return all(self.driver.get_torque(sid) for sid in ids)

    # ── 토크 ─────────────────────────────────────────────────────────────
    def torque(self, enable: bool, joint: Union[int, str, None] = None) -> None:
        """잠그기(True) · 풀기(False). joint 를 안 주면 6개 모두.

        켤 때는 목표 위치를 지금 위치로 먼저 쓴다. 모터는 예전에 받은 목표를 기억하고 있어서,
        그동안 손으로 옮겼다면 토크를 켜는 순간 그 자리로 튄다 (실측: 41°까지).
        이 목표 쓰기가 서보 토크를 그 자리에서 켜기도 한다 — 그래서 켤 때만 쓴다.
        """
        ids = JOINT_IDS if joint is None else [_resolve_joint(joint)]
        if enable:
            self._hold_here(ids)
        else:
            self._goals = {s: g for s, g in self._goals.items() if s not in ids}
        for sid in ids:
            self.driver.set_torque(sid, enable)

    def lock(self, joint: Union[int, str, None] = None) -> None:
        self.torque(True, joint)

    def unlock(self, joint: Union[int, str, None] = None) -> None:
        self.torque(False, joint)

    def _hold_here(self, ids) -> None:
        """목표 = 지금 위치. 토크를 켜기 직전(torque)과 토크가 켜진 관절(stop)에만 쓴다"""
        for sid in ids:
            p = self.driver.get_position(sid)
            if p is not None:
                self.driver.set_position(sid, p)

    # ── 움직이기 ─────────────────────────────────────────────────────────
    def move(self, joint: Union[int, str], angle_deg: float, speed: Optional[int] = None) -> None:
        """관절 하나를 angle_deg 로. speed 는 서보 속도(틱/초, 기본 200 ≈ 17.6°/s)"""
        sid = _resolve_joint(joint)
        self.move_raw(sid, deg_to_pos(angle_deg), speed)

    def move_raw(self, joint: Union[int, str], position: int, speed: Optional[int] = None) -> None:
        """관절 하나를 원시값(0~4095)으로"""
        sid = _resolve_joint(joint)
        self._require_locked([sid])
        self._set_speed(sid, self.speed if speed is None else speed)
        goal = self._clamp(sid, position)
        self._goals = {sid: goal}
        self.driver.set_position(sid, goal)

    def move_all(self, angles_deg: dict, speed: Optional[int] = None) -> None:
        """여러 관절을 한 번에. {"shoulder": 30, "elbow": -20} 또는 {2: 30, 3: -20}.
        적지 않은 관절은 그대로 둔다."""
        raw = {_resolve_joint(k): deg_to_pos(v) for k, v in angles_deg.items() if v is not None}
        self._move_positions(raw, speed)

    def gripper(self, percent: float, speed: Optional[int] = None) -> None:
        """그리퍼 0(닫힘) ~ 100(열림)"""
        lim = JOINT_LIMITS.get(6)
        self.move_raw(6, position_from_fraction(max(0.0, min(100.0, percent)) / 100.0, lim), speed)

    def open_gripper(self, speed: Optional[int] = None) -> None:
        self.gripper(100, speed)

    def close_gripper(self, speed: Optional[int] = None) -> None:
        self.gripper(0, speed)

    def center(self, speed: Optional[int] = None) -> None:
        """모든 관절을 가운데(0°)로. 접힌 자세에서는 어깨가 약 100° 움직이니 주변을 비운다"""
        self._move_positions({sid: POS_CENTER for sid in JOINT_IDS if sid != 6}, speed)

    def stop(self) -> None:
        """그 자리에서 멈춘다 (목표 = 지금 위치). 토크가 꺼진 관절은 건드리지 않는다"""
        self._hold_here([sid for sid in JOINT_IDS if self.driver.get_torque(sid)])
        self._goals = {}

    def wait_until_settled(self, tolerance_deg: float = 0.5, timeout: float = 5.0) -> bool:
        """마지막으로 보낸 목표에 모든 관절이 tolerance_deg 안으로 들어오면 True.

        물체를 쥔 그리퍼처럼 목표까지 못 가는 관절은 0.5초 동안 멈춰 있으면 도착으로 본다.
        (컨트롤러 앱 soarm.py 는 '두 번 읽은 값이 비슷하면 도착' 이라, 출발 직후 가속 중이거나
        느린 속도에서는 움직이는 중에 True 가 나왔다 — 실물에서 5° 명령이 0.7° 만 가고 끝남)
        """
        tol = tolerance_deg / 360.0 * 4095
        deadline = time.time() + timeout
        still_since, prev = None, None
        while time.time() < deadline:
            pos = self.positions()
            if all(pos.get(s) is not None and abs(pos[s] - g) <= tol for s, g in self._goals.items()):
                return True
            if prev is not None and all(pos.get(s) is not None and prev.get(s) is not None
                                        and abs(pos[s] - prev[s]) <= 2 for s in self._goals):
                still_since = still_since or time.time()
                if time.time() - still_since >= 0.5:
                    return True
            else:
                still_since = None
            prev = pos
            time.sleep(0.05)
        return False

    def _move_positions(self, raw: dict[int, int], speed: Optional[int]) -> None:
        self._require_locked(list(raw))
        sp = self.speed if speed is None else speed
        for sid in raw:
            self._set_speed(sid, sp)
        goals = {sid: self._clamp(sid, p) for sid, p in raw.items()}
        self._goals = goals
        self.driver.set_all_positions(goals)

    def _require_locked(self, ids) -> None:
        off = [JOINT_NAMES[sid - 1] for sid in ids if not self.driver.get_torque(sid)]
        if off:
            raise RobotError(f"토크가 꺼진 관절이 있습니다 {off}. bot.torque(True) 를 먼저 하세요")

    def _set_speed(self, sid: int, speed: int) -> None:
        """goal_speed 는 바뀔 때만 쓴다 (매번 쓰면 서보 동작이 끊긴다 — 컨트롤러 앱과 같은 방식)"""
        if speed and speed > 0 and self._last_speed.get(sid) != int(speed):
            self.driver.set_speed(sid, int(speed))
            self._last_speed[sid] = int(speed)

    @staticmethod
    def _clamp(sid: int, position: int) -> int:
        """data/calibration.json 의 가동범위 안으로 (컨트롤러 앱 _clamp_position 과 같다)"""
        lim = JOINT_LIMITS.get(sid)
        if not lim:
            return max(POS_MIN, min(POS_MAX, int(position)))
        if lim["max"] <= POS_MAX:
            return max(lim["min"], min(lim["max"], int(position)))
        pos = unwrap_position(int(position), lim)
        if pos > lim["max"]:
            past_max = pos - lim["max"]
            below_min = (lim["min"] + POS_RANGE) - pos
            pos = lim["max"] if past_max <= below_min else lim["min"]
        return wrap_position(pos)

    # ── 자세 (티칭) ──────────────────────────────────────────────────────
    def poses(self) -> dict[str, dict]:
        """저장된 자세 {이름: {"angles": {id: 도}, ...}} — 이 팔 기준 각도로 풀어서 보여 준다"""
        out = {}
        for name, entry in _load_json(POSES_FILE, {}).items():
            raw = _decode(entry.get("positions"), entry.get("format") == POSE_FORMAT)
            out[name] = {**entry, "angles": {sid: pos_to_deg(p) for sid, p in raw.items()}}
        return out

    def save_pose(self, name: str) -> dict:
        """지금 자세를 이름 붙여 저장 (data/poses.json)"""
        if not name:
            raise ValueError("자세 이름이 비었습니다")
        all_poses = _load_json(POSES_FILE, {})
        entry = {"positions": _encode(self.positions()), "format": POSE_FORMAT,
                 "robot": drv.CALIBRATION_LABEL, "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        all_poses[name] = entry
        _save_json(POSES_FILE, all_poses)
        return entry

    def go_pose(self, name: str, speed: Optional[int] = None, wait: bool = True, timeout: float = 10.0,
                gripper: bool = False) -> None:
        """저장한 자세로. 기본은 도착할 때까지 기다린다.

        그리퍼는 기본으로 그대로 둔다 (gripper=False). 자세에 저장된 그리퍼 값까지 되돌리면
        pick 에서 닫은 그리퍼가 above_pick 으로 올라가며 다시 열려 물체를 놓친다.
        그리퍼까지 저장한 대로 하려면 gripper=True. 열고 닫기는 gripper() 로 따로 한다
        """
        entry = _load_json(POSES_FILE, {}).get(name)
        if entry is None:
            raise RobotError(f"자세 '{name}' 이 없습니다. 있는 것: {sorted(_load_json(POSES_FILE, {}))}")
        raw = _decode(entry.get("positions"), entry.get("format") == POSE_FORMAT)
        if not gripper:
            raw.pop(6, None)
        self._move_positions(raw, speed)
        if wait:
            self.wait_until_settled(timeout=timeout)

    def delete_pose(self, name: str) -> bool:
        all_poses = _load_json(POSES_FILE, {})
        if name not in all_poses:
            return False
        del all_poses[name]
        _save_json(POSES_FILE, all_poses)
        return True

    # ── 녹화 · 재생 ──────────────────────────────────────────────────────
    def record(self, seconds: float, interval_ms: int = 100, name: Optional[str] = None,
               on_frame=None) -> list[dict]:
        """seconds 동안 자세를 interval_ms 마다 기록. 보통 토크를 풀고 손으로 움직이며 쓴다.
        name 을 주면 data/recordings/<name>.json 으로 저장"""
        frames, t0 = [], time.time()
        interval = max(20, int(interval_ms)) / 1000.0
        while time.time() - t0 < seconds:
            pos = self.positions()
            frames.append({"time": round(time.time() - t0, 3), "positions": {str(s): pos.get(s) for s in JOINT_IDS}})
            if on_frame:
                on_frame(len(frames), pos)
            time.sleep(interval)
        if name:
            self.save_recording(name, frames)
        return frames

    def save_recording(self, name: str, frames: list[dict]) -> Path:
        safe = "".join(c for c in name if c.isalnum() or c in "-_ ") or "recording"
        path = RECORDINGS_DIR / f"{safe}.json"
        payload = {"format": POSE_FORMAT, "robot": drv.CALIBRATION_LABEL,
                   "frames": [{"time": f["time"], "positions": _encode({int(k): v for k, v in f["positions"].items()})}
                              for f in frames]}
        _save_json(path, payload)
        return path

    def load_recording(self, name: str) -> list[dict]:
        path = RECORDINGS_DIR / name
        if not path.suffix:
            path = path.with_suffix(".json")
        data = _load_json(path, None)
        if data is None:
            raise RobotError(f"녹화 '{name}' 이 없습니다 ({path})")
        if isinstance(data, list):           # 컨트롤러 앱의 옛 형식 (원시값 목록)
            return data
        normalized = data.get("format") == POSE_FORMAT
        return [{"time": f.get("time"), "positions": {str(s): p for s, p in _decode(f.get("positions"), normalized).items()}}
                for f in data.get("frames") or []]

    def play(self, recording: Union[str, list], speed: float = 1.0, start_speed: Optional[int] = None) -> None:
        """녹화 재생. 첫 프레임까지 먼저 가서 멈춘 뒤, 기록된 시간 간격대로 프레임을 따라간다.
        서보 속도는 바꾸지 않으므로(기본 200 틱/초) 손으로 빠르게 움직인 녹화는 그보다 느리게 따라간다."""
        frames = self.load_recording(recording) if isinstance(recording, str) else recording
        if not frames:
            return
        speed = max(0.1, min(5.0, float(speed)))
        first = {int(k): v for k, v in frames[0]["positions"].items() if v is not None}
        self._move_positions(first, start_speed)
        self.wait_until_settled(timeout=10.0)
        for i, f in enumerate(frames):
            pos = {int(k): v for k, v in f["positions"].items() if v is not None}
            self.driver.set_all_positions({sid: self._clamp(sid, p) for sid, p in pos.items()})
            if i + 1 < len(frames):
                time.sleep(max(0.0, (frames[i + 1]["time"] - f["time"]) / speed))


# ── 자세 파일 변환 (컨트롤러 앱 controller.py 의 _encode/_decode_positions 와 같다) ──
def _encode(positions: dict) -> dict:
    return {str(sid): window_fraction(positions.get(sid), JOINT_LIMITS.get(sid)) for sid in JOINT_IDS}


def _decode(positions: Optional[dict], normalized: bool) -> dict[int, int]:
    out = {}
    for key, val in (positions or {}).items():
        if val is None:
            continue
        sid = int(key)
        lim = JOINT_LIMITS.get(sid)
        if lim is None:
            continue
        out[sid] = position_from_fraction(val, lim) if normalized else int(val)
    return out


def _load_json(path: Path, default):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return default


def _save_json(path: Path, data) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
