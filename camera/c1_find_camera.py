"""카메라 1 — 카메라 번호 찾기. 번호마다 사진을 한 장씩 찍어 camera/out/ 에 둔다.

    python camera/c1_find_camera.py              0 ~ 3 번을 차례로 열어 본다
    python camera/c1_find_camera.py --use 1      1 번을 쓰기로 camera/camera.json 에 저장

할 일
  1. 로지텍 카메라를 노트북 화면 위에 걸치고, 팔과 책상이 다 보이게 각도를 맞춘다
  2. 실행 → camera/out/cam0.jpg, cam1.jpg ... 를 열어 팔이 보이는 번호를 찾는다
     (노트북 내장 카메라가 0, USB 카메라가 1 인 경우가 많다. PC 마다 다르다)
  3. --use 번호 로 저장한다. 다음 실습(c2 ~ c8)은 이 번호를 읽는다
"""
import os
import platform
import sys
from pathlib import Path

os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")       # 없는 번호를 열 때 나오는 긴 경고를 끈다

import cv2  # noqa: E402

from camlib import DEFAULT_SETTINGS, OUT, load_settings, save_jpg, save_settings  # noqa: E402
from soarm.color import open_camera  # noqa: E402

if "--use" in sys.argv:
    cfg = load_settings()
    cfg["camera"] = int(sys.argv[sys.argv.index("--use") + 1])
    print("저장:", save_settings(cfg), cfg)
    sys.exit()

found = []
for index in range(4):
    if platform.system() == "Linux" and not Path(f"/dev/video{index}").exists():
        print(f"카메라 {index}: 없음")
        continue
    try:
        cap = open_camera(index, DEFAULT_SETTINGS["width"], DEFAULT_SETTINGS["height"])
    except RuntimeError:
        print(f"카메라 {index}: 없음")
        continue
    ok, frame = cap.read()
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    if not ok:
        print(f"카메라 {index}: 열렸지만 영상이 오지 않음 (다른 프로그램이 쓰는 중?)")
        continue
    path = save_jpg(frame, f"cam{index}")
    found.append(index)
    print(f"카메라 {index}: {frame.shape[1]}x{frame.shape[0]}{f', {fps:.0f} fps' if fps > 0 else ''} → {path.name}")

if not found:
    sys.exit("카메라를 찾지 못했습니다. USB 를 다시 꽂고, Zoom · 카메라 앱 · ArmStudio 를 닫은 뒤 다시 하세요")
print(f"\n사진 폴더: {OUT}")
print(f"팔이 보이는 번호를 골라:  python camera/c1_find_camera.py --use {found[-1]}")
