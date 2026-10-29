"""색 분류 시험 영상 만들기 (카메라 · 색 종이 없이 p4 · sort.launch 를 돌려 볼 때).

    python scripts/make_test_video.py     → data/color_test.mp4 (빈 자리 → 빨강 → 빈 자리 → 파랑 → 빈 자리, 23초)

검사 자리(ROI)는 project/color.json 을 따른다 (없으면 기본값). 영상 파일을 카메라 자리에 준다:

    ros2 launch so101_ros sort.launch.py port:=fake camera:=data/color_test.mp4
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

from soarm.color import ColorConfig  # noqa: E402
from soarm.robot import ROOT  # noqa: E402

OUT = ROOT / "data" / "color_test.mp4"
FPS = 10
SCENES = [(3, None), (4, (30, 30, 200)), (6, None), (4, (200, 80, 20)), (6, None)]   # (초, BGR 색 또는 빈 자리)

cfg = ColorConfig.load()
x, y, w, h = cfg.roi
OUT.parent.mkdir(parents=True, exist_ok=True)
out = cv2.VideoWriter(str(OUT), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (cfg.width, cfg.height))
for sec, bgr in SCENES:
    for _ in range(sec * FPS):
        img = np.full((cfg.height, cfg.width, 3), 120, np.uint8)     # 회색 바닥
        if bgr:
            img[y:y + h, x:x + w] = bgr
        out.write(img)
out.release()
print(f"{OUT}  ({sum(s for s, _ in SCENES)}초, ROI {cfg.roi})")
