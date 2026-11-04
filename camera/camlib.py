"""카메라 실습 공통 함수 — 카메라 열기 · 색 덩어리 찾기 · 픽셀 → 책상 좌표 · 손끝 좌표 · 좌표로 팔 움직이기.

카메라는 노트북 화면 위에 걸친 USB 카메라(로지텍)로, 팔과 책상이 한 화면에 다 보이게 둔다.
설정은 camera/camera.json (c1 이 만든다). 사진 · 영상은 camera/out/ 에 저장된다.

    from camlib import load_settings, open_cam, find_blob
    cfg = load_settings()
    cap = open_cam(cfg)
    ok, frame = cap.read()
    blob = find_blob(frame, "red")        # {"u": 312, "v": 240, "area": 1830, ...} 또는 None

필요한 것:  pip install -e ".[camera,theory]"   (opencv · numpy)
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")     # 카메라를 열 때 나오는 WARN 줄을 줄인다

import cv2  # noqa: E402
import numpy as np

from soarm.color import DEFAULT_RANGES, open_camera
from soarm.driver_sdk import JOINT_LIMITS, POS_CENTER, STS3215Driver, position_from_fraction
from soarm.kinematics import LIMITS_DEG, fk, frames

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
SETTINGS_FILE = HERE / "camera.json"
TABLE_FILE = HERE / "table_map.json"

DEFAULT_SETTINGS = {"camera": 1, "width": 640, "height": 480}   # 노트북 내장 = 0, USB 카메라 = 1 인 경우가 많다


# ── 설정 · 저장 ──────────────────────────────────────────────────────
def load_settings() -> dict:
    """camera.json → {"camera": 1, "width": 640, "height": 480}. 명령에 --camera 2 를 주면 그 번호"""
    cfg = dict(DEFAULT_SETTINGS)
    if SETTINGS_FILE.exists():
        cfg.update(json.loads(SETTINGS_FILE.read_text(encoding="utf-8")))
    if "--camera" in sys.argv:
        cfg["camera"] = int(sys.argv[sys.argv.index("--camera") + 1])
    return cfg


def save_settings(cfg: dict) -> Path:
    SETTINGS_FILE.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")
    return SETTINGS_FILE


class LoopFile:
    """영상 파일을 카메라처럼: 제 속도(fps)로 읽고, 끝나면 처음부터 다시.
    카메라가 없을 때 camera.json 의 "camera" 에 "data/color_test.mp4" 처럼 파일 경로를 넣는다
    (영상 만들기: python scripts/make_test_video.py)"""

    def __init__(self, path: str):
        self.cap = cv2.VideoCapture(path)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.dt = 1.0 / (fps if fps > 0 else 10.0)
        self.next = time.time()

    def isOpened(self):
        return self.cap.isOpened()

    def read(self):
        time.sleep(max(0.0, self.next - time.time()))
        self.next = max(self.next + self.dt, time.time())
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return ok, frame

    def get(self, prop):
        return self.cap.get(prop)

    def release(self):
        self.cap.release()


def open_cam(cfg: dict):
    """camera.json 의 번호(또는 영상 파일)로 연다. 못 열면 c1 을 하라는 말과 함께 끝낸다"""
    index = cfg["camera"]
    hint = f"카메라 {index} 를 열 수 없습니다. python camera/c1_find_camera.py 로 번호를 확인하세요 (다른 프로그램이 쓰는 중이어도 안 열림)"
    if isinstance(index, str) and Path(index).is_file():
        cam = LoopFile(index)
        if not cam.isOpened():
            sys.exit(f"영상 파일 {index} 을 열 수 없습니다")
        return cam
    if sys.platform.startswith("linux") and isinstance(index, int) and not Path(f"/dev/video{index}").exists():
        sys.exit(hint)                                   # 리눅스 · Pi: 없는 번호를 열면 OpenCV 경고가 길게 나온다
    try:
        return open_camera(index, cfg["width"], cfg["height"])
    except RuntimeError:
        sys.exit(hint)


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def save_jpg(img, name: str) -> Path:
    """camera/out/<name>.jpg. cv2.imwrite 는 한글 경로에서 실패해서 imencode 로 쓴다"""
    OUT.mkdir(exist_ok=True)
    path = OUT / f"{name}.jpg"
    ok, buf = cv2.imencode(".jpg", img)
    path.write_bytes(buf.tobytes())
    return path


# ── 색 덩어리 ───────────────────────────────────────────────────────
def color_mask(frame, color: str = "red", ranges: dict = DEFAULT_RANGES):
    """HSV 범위에 드는 픽셀 = 255. 작은 점(잡음)은 열기(open)로 지운다"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges[color]:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


def find_blob(frame, color: str = "red", min_area: int = 100, zone=None):
    """화면에서 그 색의 가장 큰 덩어리 → {"u", "v", "area", "box"} (픽셀). 없으면 None

    zone = [x, y, w, h] 를 주면 그 안에서만 찾는다 (팔의 빨간 부품 같은 것을 빼려고)
    min_area: 이보다 작은 덩어리는 잡음으로 본다. 노트북 위 카메라에서 4 cm 종이가 300 ~ 400 픽셀
    """
    mask = color_mask(frame, color)
    if zone is not None:
        x, y, w, h = [int(v) for v in zone]
        keep = np.zeros_like(mask)
        keep[y:y + h, x:x + w] = 255
        mask &= keep
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))          # 0 번은 배경
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    x, y, w, h = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    return {"u": round(float(cent[i][0]), 1), "v": round(float(cent[i][1]), 1), "area": area, "box": [x, y, w, h]}


def draw_blob(img, blob, color=(0, 0, 255), label: str = ""):
    if blob is None:
        return img
    x, y, w, h = blob["box"]
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
    cv2.drawMarker(img, (int(blob["u"]), int(blob["v"])), color, cv2.MARKER_CROSS, 18, 2)
    text = label or f"({blob['u']:.0f}, {blob['v']:.0f}) {blob['area']}px"
    cv2.putText(img, text, (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return img


# ── 픽셀 → 책상 좌표 (호모그래피) ───────────────────────────────────
class TableMap:
    """책상(평면) 위 점의 픽셀 (u, v) ↔ 로봇 좌표 (x, y) [m]. 점 4개 이상으로 3×3 행렬 H 를 구한다"""

    def __init__(self, H, z: float, pixels=None, points=None):
        self.H = np.asarray(H, float)
        self.z = float(z)                    # 책상 높이 (로봇 좌표, m)
        self.pixels = pixels or []
        self.points = points or []

    @classmethod
    def fit(cls, pixels, points_xyz) -> "TableMap":
        src = np.asarray(pixels, np.float64)
        dst = np.asarray([p[:2] for p in points_xyz], np.float64)
        H, _ = cv2.findHomography(src, dst, 0)
        if H is None:
            raise ValueError("점이 한 줄에 놓여 있어 계산할 수 없습니다. 네 모서리처럼 넓게 퍼뜨리세요")
        z = float(np.mean([p[2] for p in points_xyz]))
        return cls(H, z, [list(map(float, p)) for p in pixels], [list(map(float, p)) for p in points_xyz])

    def to_xy(self, u: float, v: float) -> tuple[float, float]:
        p = self.H @ np.array([u, v, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def object_xy(self, blob: dict, half: float = 0.0) -> tuple[float, float]:
        """덩어리 → 물체 가운데 (x, y). 높이가 있는 물체용 (이론 8: 3 cm 윗면을 책상 H 로 바꾸면 5 cm 넘게 빗나감)

        덩어리 아래쪽 가장자리 = 물체가 책상에 닿은 앞쪽 끝 → 책상 높이라 H 가 맞다.
        거기서 카메라 반대쪽(화면 위쪽)으로 half(물체 앞뒤 길이의 반, m)만큼 들어간 곳이 가운데.
        종이처럼 납작하면 half = 0 이고 덩어리 중심을 써도 같다
        """
        x, y, w, h = blob["box"]
        u, v = blob["u"], y + h - 1                         # 아래쪽 가장자리 가운데
        p1 = np.array(self.to_xy(u, v))
        p2 = np.array(self.to_xy(u, v - 20))                # 20 픽셀 위 = 카메라에서 먼 쪽
        d = (p2 - p1) / (np.linalg.norm(p2 - p1) or 1.0)
        return tuple(float(c) for c in p1 + d * half)

    def errors_mm(self) -> list[float]:
        """맞춘 점들을 다시 넣어 본 오차 (mm). 점이 4개면 늘 0 에 가깝다 → 5번째 점으로 시험한다"""
        return [round(math.dist(self.to_xy(*px), pt[:2]) * 1000, 1) for px, pt in zip(self.pixels, self.points)]

    def save(self, path: Path = TABLE_FILE) -> Path:
        path.write_text(json.dumps({"H": self.H.tolist(), "z": self.z, "pixels": self.pixels, "points": self.points},
                                   indent=2), encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: Path = TABLE_FILE) -> "TableMap":
        if not path.exists():
            raise FileNotFoundError(f"{path.name} 이 없습니다. 먼저 python camera/c5_table_map.py")
        d = json.loads(path.read_text(encoding="utf-8"))
        return cls(d["H"], d["z"], d.get("pixels"), d.get("points"))


# ── c9 가 만든 카메라 ↔ 로봇 식 (hand_eye.json) ──────────────────────
HAND_EYE_FILE = HERE / "hand_eye.json"


def load_hand_eye(path: Path = HAND_EYE_FILE) -> dict:
    if not path.exists():
        sys.exit(f"{path.name} 이 없습니다. 먼저 python camera/c9_hand_eye.py (--teach · --ruler 도 됨)")
    return json.loads(path.read_text(encoding="utf-8"))


def hand_eye_xy(model: dict, u: float, v: float) -> list:
    """픽셀 → 로봇 (x, y) m. line(세로축) 은 1차원 원근식 x = (a s + b) / (c s + 1), plane 은 3×3 호모그래피"""
    if model["mode"] == "line":
        s = (np.array([u, v]) - model["mean"]) @ np.array(model["dir"])
        a, b, c = model["abc"]
        return [float((a * s + b) / (c * s + 1)), model.get("y", 0.0)]
    return list(TableMap(model["H"], model["z"]).to_xy(u, v))


# ── 팔: 손끝 좌표 · 좌표로 움직이기 ─────────────────────────────────
def urdf_deg(bot) -> list[float]:
    """지금 관절 5개(받침 ~ 손목 회전)를 URDF 기준 도로 → soarm.kinematics.fk 에 그대로 넣는다"""
    rad = bot.joint_radians()
    return [math.degrees(rad[sid]) for sid in range(1, 6)]


def tip_xyz(bot) -> list[float]:
    """지금 손끝(집게 사이) 위치, 로봇 좌표 m (x 앞, y 왼쪽, z 위)"""
    return [round(float(v), 4) for v in fk(urdf_deg(bot))]


def _tip_and_axis(q, radial):
    """손끝 위치 + '집게가 바깥(앞)으로 얼마나 기울었나' (0 이면 수직)"""
    F = dict(frames(q))
    tip = F["gripper_frame_joint"][:3, 3]
    axis = -F["wrist_roll"][:3, 2]                       # 손목 회전축 = 집게가 향하는 방향
    return np.concatenate([tip, [0.1 * float(axis[:2] @ radial)]]), axis


def ik_down(target, q0, iters: int = 300, tol: float = 0.001):
    """손끝을 target(x, y, z m)에, 집게는 바닥을 향하게. → URDF 각도 5개(도), 닿지 않으면 None

    soarm.kinematics.ik_numeric(DLS, 위치 3개)에 '집게 수직' 조건 하나를 더한 것 (식 4개, 관절 4개).
    q0(시작 자세)에 가까운 답이 나온다. 손목 회전(5번째)은 q0 그대로
    """
    target = np.asarray(target, float)
    radial = target[:2] / (np.linalg.norm(target[:2]) or 1.0)      # 받침에서 목표 쪽 가로 방향
    goal = np.concatenate([target, [0.0]])
    q = np.array(q0[:5], float)
    for _ in range(iters):
        f, axis = _tip_and_axis(q, radial)
        err = goal - f
        if np.linalg.norm(err[:3]) < tol and abs(err[3]) < 0.005 and axis[2] < 0:   # 0.005 ≈ 3°
            return [round(float(v), 2) for v in q]
        J = np.zeros((4, 4))
        for j in range(4):
            dq = np.zeros(5)
            dq[j] = np.degrees(1e-4)
            J[:, j] = (_tip_and_axis(q + dq, radial)[0] - f) / 1e-4
        step = J.T @ np.linalg.solve(J @ J.T + 0.02 ** 2 * np.eye(4), err)
        q[:4] += np.degrees(step)
        q = np.clip(q, LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])
    return None


def _urdf_to_raw(sid: int, deg: float) -> int:
    """URDF 각도(도) → 이 팔의 서보 값. Robot.joint_radians() 의 거꾸로"""
    lim = JOINT_LIMITS.get(sid) or {}
    rad = math.radians(deg)
    if lim.get("invert"):
        rad = -rad
    if "rad_min" in lim and "rad_max" in lim and "min" in lim and "max" in lim:
        t = (rad - lim["rad_min"]) / ((lim["rad_max"] - lim["rad_min"]) or 1)
        return position_from_fraction(t, lim)
    return int(round(POS_CENTER + rad / (2 * math.pi) * 4095))


def move_urdf(bot, q_deg, speed: int = 150, wait: bool = True) -> None:
    """URDF 각도 5개(도)로 팔을 움직인다 (그리퍼는 그대로)"""
    raw = {sid: _urdf_to_raw(sid, q) for sid, q in zip(range(1, 6), q_deg)}
    now = bot.positions()
    far = max(abs(raw[s] - (now[s] if now[s] is not None else raw[s])) for s in raw)   # 가장 많이 도는 관절 (틱)
    bot.move_all({s: STS3215Driver.position_to_degrees(p) for s, p in raw.items()}, speed=speed)
    if wait:
        bot.wait_until_settled(timeout=far / max(1, speed) * 1.3 + 1.0)     # 속도(틱/초)로 걸리는 시간 + 여유
