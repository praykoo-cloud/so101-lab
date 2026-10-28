"""프로젝트 1 — 카메라 확인 · 검사 자리(ROI) 정하기.

    python project/p1_camera.py                  창에서 마우스로 ROI 를 끌고 Enter (PC)
    python project/p1_camera.py --roi 270 190 100 100   숫자로 정하기 (화면 없는 라즈베리파이)
    python project/p1_camera.py --snapshot       지금 화면을 project/snapshot.jpg 로 (ROI 상자 포함)

할 일
  1. 카메라를 검사 자리(물체를 놓을 곳) 위에 고정한다. 팔이 물체를 집으러 와도 가리지 않는 각도로
  2. 검사 자리에 빨강 종이를 놓고 실행 → 종이 안쪽만 들어가게 ROI 를 끈다
  3. project/color.json 에 저장된다. CAMERA 번호가 틀리면 0 · 1 · 2 로 바꿔 본다
"""
import sys
from pathlib import Path

import cv2

from soarm.color import ColorConfig, classify, draw, open_camera

CAMERA = 0

cfg = ColorConfig.load()
cfg.camera = int(sys.argv[sys.argv.index("--camera") + 1]) if "--camera" in sys.argv else CAMERA
cap = open_camera(cfg.camera, cfg.width, cfg.height)
ok, frame = cap.read()
if not ok:
    sys.exit("카메라에서 영상을 받지 못했습니다")
print(f"카메라 {cfg.camera}: {frame.shape[1]}x{frame.shape[0]}")

if "--roi" in sys.argv:
    i = sys.argv.index("--roi")
    cfg.roi = [int(v) for v in sys.argv[i + 1:i + 5]]
elif "--snapshot" not in sys.argv:
    r = cv2.selectROI("ROI: 검사 자리를 끌고 Enter", frame, showCrosshair=True)
    cv2.destroyAllWindows()
    if r[2] > 0 and r[3] > 0:
        cfg.roi = [int(v) for v in r]

color, ratios = classify(frame, cfg)
print(f"ROI {cfg.roi} → {color}  {ratios}")
print("저장:", cfg.save())

if "--snapshot" in sys.argv:
    ok, buf = cv2.imencode(".jpg", draw(frame, cfg, color, ratios))   # imwrite 는 한글 경로에서 실패한다
    out = Path(__file__).with_name("snapshot.jpg")
    out.write_bytes(buf.tobytes())
    print("사진:", out)
cap.release()
