"""카메라 사진 찍기 (Pi 에서 사진을 찍어 PC 로 가져가 구역 · 색을 티칭할 때 쓴다).

Pi 에는 화면이 없어서 구역(ROI)을 마우스로 그리는 창을 띄울 수 없다.
그래서 Pi 에서 사진을 찍고 → PC 로 복사해서 → PC 에서 m0_teach_rois / m0_teach_colors 의 --image 로 티칭 → 결과 파일을 Pi 로 복사한다.

실행 (Pi, so101-lab 폴더에서):
    .venv/bin/python -m mission.m0_capture_frames --scan                    # 카메라 번호 찾기 (열리는 번호 + 사진 1장씩)
    .venv/bin/python -m mission.m0_capture_frames --label wait              # config 의 카메라로 5장 (mission/frames/wait_*.png)
    .venv/bin/python -m mission.m0_capture_frames --camera 2 --label basket --count 3

찍는 요령
- 구역마다 팔 자세가 다르니, 자세로 보낸 뒤(예: m0_teach_poses 의 go wait) 그 자세에서 --label 이름을 붙여 찍는다.
    wait          → arrival, note 구역용 (자동차 · 포스트잇이 보이게 자동차를 세워 두고)
    stand_red     → stand_red (빨간 공 자리). stand_blue, stand_area, basket, outside, gripper 도 같은 방식
- 조명은 실제 경기장과 같게. 색 티칭용 사진은 빨강 · 파랑 포스트잇 / 공을 각각 따로 찍는다 (label 예: note_red).

사진은 mission/frames/ 에 저장된다 (git 에 올리지 않는다).
"""
from __future__ import annotations

import argparse
import platform
import time
from pathlib import Path

import cv2

from .config import MISSION_DIR, MissionConfig

FRAMES_DIR = MISSION_DIR / "frames"


def _open(index, width, height):
    if platform.system() == "Windows" and isinstance(index, int):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def scan(width, height) -> int:
    found = 0
    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    for i in range(10):
        cap = _open(i, width, height)
        ok = cap.isOpened()
        frame = None
        if ok:
            for _ in range(8):                      # 처음 몇 장은 노출이 안 맞아 버린다
                ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            path = FRAMES_DIR / f"scan_camera{i}.png"
            cv2.imwrite(str(path), frame)
            print(f"  카메라 {i}: 열림 {frame.shape[1]}x{frame.shape[0]} → {path}")
            found += 1
        else:
            print(f"  카메라 {i}: 안 열림")
    if not found:
        print("열리는 카메라가 없습니다. USB 연결, 다른 프로그램이 쓰는지, 권한(video 그룹)을 확인하세요.")
        return 1
    print("scan_camera*.png 를 열어 어느 번호가 그리퍼(손목) 카메라인지 보고, mission/config.json 의 camera 에 적으세요.")
    return 0


def main(argv=None) -> int:
    cfg = MissionConfig.load()
    ap = argparse.ArgumentParser(description="카메라 사진 찍기")
    ap.add_argument("--camera", default=None, help="카메라 번호 (기본: config 의 camera)")
    ap.add_argument("--label", default="shot", help="파일 이름 앞부분 (예: wait, note_red)")
    ap.add_argument("--count", type=int, default=5, help="몇 장 (기본 5, 0.5초 간격)")
    ap.add_argument("--scan", action="store_true", help="카메라 번호 0~9 를 열어 보고 사진 1장씩 저장")
    args = ap.parse_args(argv)

    if args.scan:
        return scan(cfg.width, cfg.height)

    index = cfg.camera if args.camera is None else (int(args.camera) if str(args.camera).isdigit() else args.camera)
    cap = _open(index, cfg.width, cfg.height)
    if not cap.isOpened():
        print(f"카메라 {index} 를 열 수 없습니다. --scan 으로 번호를 먼저 찾으세요.")
        return 1
    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%H%M%S")
    saved = 0
    try:
        for _ in range(8):
            cap.read()
        for i in range(max(1, args.count)):
            ok, frame = cap.read()
            if not ok or frame is None:
                print("프레임을 읽지 못했습니다")
                continue
            path = FRAMES_DIR / f"{args.label}_{stamp}_{i}.png"
            cv2.imwrite(str(path), frame)
            saved += 1
            print(f"저장: {path}")
            time.sleep(0.5)
    finally:
        cap.release()
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())
