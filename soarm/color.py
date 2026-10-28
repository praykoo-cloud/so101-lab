"""정해진 자리(ROI)의 색 판별 — 빨강 · 파랑 · 없음. 위치 보정은 하지 않는다.

카메라는 고정, 물체는 늘 같은 자리(검사 자리)에 놓는다. ROI 안에서 색마다 HSV 범위에 드는
픽셀 비율을 세고, 가장 큰 색이 min_ratio 를 넘으면 그 색이다.

    from soarm.color import ColorConfig, classify, open_camera
    cfg = ColorConfig.load()                 # project/color.json
    cap = open_camera(cfg.camera)
    ok, frame = cap.read()
    color, ratios = classify(frame, cfg)     # ("red", {"red": 0.62, "blue": 0.01})

cv2 · numpy 가 필요하다:  pip install -e ".[camera]"
"""
from __future__ import annotations

import json
import platform
from dataclasses import asdict, dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = ROOT / "project" / "color.json"

# HSV 범위 (OpenCV: H 0~180, S · V 0~255). 빨강은 색상환 양 끝이라 두 칸
DEFAULT_RANGES = {
    "red": [[[0, 100, 70], [10, 255, 255]], [[170, 100, 70], [180, 255, 255]]],
    "blue": [[[95, 100, 50], [130, 255, 255]]],
}


@dataclass
class ColorConfig:
    camera: int | str = 0
    width: int = 640
    height: int = 480
    roi: list = field(default_factory=lambda: [270, 190, 100, 100])   # x, y, w, h (픽셀)
    min_ratio: float = 0.30
    ranges: dict = field(default_factory=lambda: json.loads(json.dumps(DEFAULT_RANGES)))

    @classmethod
    def load(cls, path: Path | str = CONFIG_FILE) -> "ColorConfig":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except FileNotFoundError:
            return cls()
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})

    def save(self, path: Path | str = CONFIG_FILE) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=False), encoding="utf-8")
        return path


def open_camera(index, width: int = 640, height: int = 480):
    """Windows 는 DirectShow, 리눅스 · 라즈베리파이는 V4L2 (기본)"""
    import cv2
    if platform.system() == "Windows" and isinstance(index, int):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 {index} 를 열 수 없습니다 (번호 · 다른 프로그램이 쓰는지 확인)")
    for _ in range(5):          # 처음 몇 장은 노출이 맞지 않아 버린다
        cap.read()
    return cap


def ratios_in_roi(frame, cfg: ColorConfig) -> dict[str, float]:
    import cv2
    import numpy as np
    x, y, w, h = [int(v) for v in cfg.roi]
    hsv = cv2.cvtColor(frame[y:y + h, x:x + w], cv2.COLOR_BGR2HSV)
    total = max(1, hsv.shape[0] * hsv.shape[1])
    out = {}
    for name, ranges in cfg.ranges.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        out[name] = round(float(cv2.countNonZero(mask)) / total, 3)
    return out


def classify(frame, cfg: ColorConfig) -> tuple[str, dict[str, float]]:
    """→ ("red" | "blue" | "none", 색마다 비율)"""
    r = ratios_in_roi(frame, cfg)
    best = max(r, key=r.get)
    return (best if r[best] >= cfg.min_ratio else "none"), r


def draw(frame, cfg: ColorConfig, color: str, ratios: dict[str, float]):
    """ROI 상자와 판별 결과를 그린 사본"""
    import cv2
    img = frame.copy()
    x, y, w, h = [int(v) for v in cfg.roi]
    bgr = {"red": (0, 0, 255), "blue": (255, 0, 0)}.get(color, (200, 200, 200))
    cv2.rectangle(img, (x, y), (x + w, y + h), bgr, 2)
    text = f"{color}  " + "  ".join(f"{k} {v:.2f}" for k, v in ratios.items())
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2)
    return img
