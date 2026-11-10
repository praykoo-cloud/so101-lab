"""YOLO 인식: 그리퍼 카메라 사진에서 차 · 상자 · 포스트잇 · 성공한 상자를 찾는다.

모델: mission/models/arm_v1_yolo11n_320.onnx (팀 레포 Team_koo 의 12번 PR, 소리님 학습). OpenCV 만으로 읽는다 (추가 설치 없음).

    det = Detector()                       # 모델 읽기 (없으면 FileNotFoundError)
    found = det.detect(frame)              # frame = 카메라 사진 (BGR). → [Det(name, conf, x, y, w, h), ...]
    box = pick_best(found, BASKET_NAMES)   # 가장 확실한 상자 하나 (없으면 None)

    python -m mission.yolo 사진.png        # 사진 한 장으로 시험 (찾은 것 목록 출력)

단어장
- YOLO  : 사진에서 물체를 찾아 네모(박스)로 알려 주는 AI 모델
- conf  : 확신 정도 0~1 (1 에 가까울수록 확실)
- NMS   : 같은 물체에 네모가 여러 개 겹쳐 나올 때 가장 확실한 하나만 남기는 정리

고칠 숫자: 설정의 yolo_model (파일 이름) · yolo_conf (이 값보다 확신이 낮으면 버림)
주의: 공(ball_*)은 학습 사진이 17장뿐이라 믿지 않는다 → 공은 티칭한 고정 위치로 집는다.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import MISSION_DIR

MODELS_DIR = MISSION_DIR / "models"
DEFAULT_MODEL = "arm_v1_yolo11n_320.onnx"
CLASS_NAMES = ["ball_blue", "ball_red", "box", "car", "postit_blue", "postit_red", "success_blue", "success_red"]
BASKET_NAMES = ("box", "success_red", "success_blue")       # 차 위 바구니 (공이 들어가면 success_* 로 바뀐다)
MAX_COVER = 0.9                                             # 네모가 화면의 이만큼 넘게 덮으면 버린다 (실측: 차가 없는 장면 전체를 car 0.6 으로 잡음)
TRACK_NAMES = BASKET_NAMES + ("car",)                         # 추적할 때 볼 것 (상자가 보이면 상자, 없으면 차)
LABELS = {"ball_blue": "파란 공", "ball_red": "빨간 공", "box": "상자", "car": "차", "postit_blue": "파란 포스트잇",
          "postit_red": "빨간 포스트잇", "success_blue": "파란 공이 든 상자", "success_red": "빨간 공이 든 상자"}


@dataclass
class Det:
    name: str
    conf: float
    x: float            # 왼쪽 위 (사진 픽셀)
    y: float
    w: float
    h: float

    @property
    def cx(self) -> float:
        return self.x + self.w / 2

    @property
    def cy(self) -> float:
        return self.y + self.h / 2

    def to_dict(self) -> dict:
        return {"name": self.name, "label": LABELS.get(self.name, self.name), "conf": round(self.conf, 2),
                "x": round(self.x), "y": round(self.y), "w": round(self.w), "h": round(self.h)}


def letterbox(frame, size: int):
    """비율을 지키며 size×size 로 줄이고 남는 곳은 회색(114)으로 채운다 (학습 때와 같은 방식). → (그림, 배율, 왼쪽 여백, 위 여백)"""
    import cv2
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, round(w * scale)), max(1, round(h * scale))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    left, top = (size - nw) // 2, (size - nh) // 2
    canvas[top:top + nh, left:left + nw] = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return canvas, scale, left, top


def decode(output, scale: float, left: int, top: int, frame_w: int, frame_h: int, conf: float, iou: float,
           max_cover: float = MAX_COVER) -> list[Det]:
    """모델 출력 (1, 4+클래스 수, 후보 수) → Det 목록. 박스 4개 값은 (가운데 x, 가운데 y, 너비, 높이), objectness 없음"""
    import cv2
    rows = np.asarray(output)[0].T                                # (후보 수, 12)
    scores = rows[:, 4:]
    cls = scores.argmax(axis=1)
    best = scores[np.arange(len(rows)), cls]
    keep = best >= conf
    rows, cls, best = rows[keep], cls[keep], best[keep]
    out: list[Det] = []
    for c in sorted(set(cls.tolist())):                           # 클래스마다 겹친 네모 정리
        idx = np.where(cls == c)[0]
        boxes = [[float(r[0] - r[2] / 2), float(r[1] - r[3] / 2), float(r[2]), float(r[3])] for r in rows[idx]]
        picked = cv2.dnn.NMSBoxes(boxes, [float(v) for v in best[idx]], conf, iou)
        for k in np.asarray(picked).reshape(-1):
            bx, by, bw, bh = boxes[int(k)]
            x, y = (bx - left) / scale, (by - top) / scale        # 여백 · 배율을 되돌려 원래 사진 좌표로
            w, h = bw / scale, bh / scale
            x2, y2 = min(frame_w, x + w), min(frame_h, y + h)
            x, y = max(0.0, x), max(0.0, y)
            if (x2 - x) * (y2 - y) > max_cover * frame_w * frame_h:
                continue                                          # 화면을 거의 다 덮는 네모는 오인식으로 본다
            if x2 - x >= 2 and y2 - y >= 2:
                out.append(Det(CLASS_NAMES[c], float(best[idx][int(k)]), x, y, x2 - x, y2 - y))
    out.sort(key=lambda d: -d.conf)
    return out


class Detector:
    def __init__(self, model: str | Path | None = None, conf: float = 0.45, iou: float = 0.5):
        import cv2
        path = Path(model) if model else MODELS_DIR / DEFAULT_MODEL
        if not path.is_absolute() and not path.exists():
            path = MODELS_DIR / path
        if not path.exists():
            raise FileNotFoundError(f"YOLO 모델 파일이 없어요: {path} (팀 레포 Self_Driving/perception/models 에서 복사)")
        self.path, self.conf, self.iou = path, float(conf), float(iou)
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.size = 640 if "640" in path.stem else 320            # 파일 이름의 입력 크기

    def detect(self, frame) -> list[Det]:
        import cv2
        if frame is None:
            return []
        img, scale, left, top = letterbox(frame, self.size)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (self.size, self.size), swapRB=True, crop=False)
        self.net.setInput(blob)
        h, w = frame.shape[:2]
        return decode(self.net.forward(), scale, left, top, w, h, self.conf, self.iou)


def pick_best(dets: list[Det], names) -> Det | None:
    """names 에 드는 것 중 가장 확실한 하나"""
    hits = [d for d in dets if d.name in names]
    return max(hits, key=lambda d: d.conf) if hits else None


def track_target(dets: list[Det]) -> Det | None:
    """추적할 대상: 상자가 보이면 상자, 아니면 차"""
    return pick_best(dets, BASKET_NAMES) or pick_best(dets, ("car",))


def draw(frame, dets: list[Det]):
    """찾은 것을 사진 위에 네모로 그린 새 사진 (화면 확인용, 글자는 영어 이름)"""
    import cv2
    out = frame.copy()
    colors = {"box": (200, 200, 200), "car": (0, 200, 255), "postit_red": (0, 0, 255), "postit_blue": (255, 80, 0),
              "success_red": (0, 0, 180), "success_blue": (180, 60, 0), "ball_red": (80, 80, 255), "ball_blue": (255, 160, 80)}
    for d in dets:
        c = colors.get(d.name, (0, 255, 0))
        p1, p2 = (int(d.x), int(d.y)), (int(d.x + d.w), int(d.y + d.h))
        cv2.rectangle(out, p1, p2, c, 2)
        cv2.putText(out, f"{d.name} {d.conf:.2f}", (p1[0], max(14, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    return out


def main(argv=None) -> int:
    import cv2
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print(__doc__)
        return 1
    frame = cv2.imdecode(np.fromfile(args[0], dtype=np.uint8), cv2.IMREAD_COLOR)      # 한글 경로도 읽히게
    if frame is None:
        print(f"사진을 읽지 못했어요: {args[0]}")
        return 1
    found = Detector().detect(frame)
    if not found:
        print("아무것도 찾지 못했어요.")
    for d in found:
        print(f"  {LABELS[d.name]:12s} 확신 {d.conf:.2f}  위치 x={d.x:.0f} y={d.y:.0f} 크기 {d.w:.0f}×{d.h:.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
