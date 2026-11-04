"""카메라 3 — 영상과 관절 각도를 같이 기록하기 (LeRobot 데이터의 모양: 장면 + 관절).

    python camera/c3_record.py --seconds 10                   토크를 끄고, 손으로 팔을 움직이는 10초
    python camera/c3_record.py --poses home above_pick home   저장한 자세를 차례로 가며 기록

camera/out/rec_<시각>.mp4 (영상)과 rec_<시각>.csv (장마다 시각 · 관절 6개 · 손끝 x y z)가 생긴다.
포트는 SOARM_PORT (없으면 꽂힌 보드 하나). 팔 없이: SOARM_PORT=fake

할 일
  1. 손으로 기록 → csv 를 열어 장 수 · 시간 간격을 본다 (카메라 30 fps 면 약 0.033 초)
  2. --poses 로 기록 → 영상과 csv 의 손끝 z 가 같이 내려가고 올라가는지 본다
  3. 생각해 보기: 3일차 VLA 는 이 '장면 + 관절' 쌍을 수백 개 모아 학습한다. 무엇이 입력이고 무엇이 답일까?
"""
import csv
import shutil
import sys
import tempfile
import time
from pathlib import Path

import cv2

from camlib import OUT, load_settings, open_cam, stamp, tip_xyz
from soarm import Robot


def arg(name, default=None, many=False):
    if name not in sys.argv:
        return default
    i = sys.argv.index(name)
    if many:
        return [a for a in sys.argv[i + 1:] if not a.startswith("--")]
    return sys.argv[i + 1]


seconds = float(arg("--seconds", 10))
poses = arg("--poses", [], many=True)
HOLD = 3.0                                      # 자세마다 기록하는 시간 (초)

cfg = load_settings()
cap = open_cam(cfg)
ok, frame = cap.read()
if not ok:
    sys.exit("영상이 오지 않습니다")
h, w = frame.shape[:2]
fps = cap.get(cv2.CAP_PROP_FPS)
fps = fps if fps > 0 else 30                    # Windows(DirectShow)는 -1 을 주기도 한다

name = f"rec_{stamp()}"
OUT.mkdir(exist_ok=True)
tmp = Path(tempfile.gettempdir()) / f"{name}.mp4"         # VideoWriter 는 한글 경로에 쓰지 못한다 → 임시로 쓰고 옮긴다
video = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
rows = []

bot = Robot()
print(bot)
if poses:
    bot.torque(True)
    seconds = HOLD * len(poses)
else:
    bot.torque(False)
    print("토크 꺼짐: 팔을 손으로 움직이세요 (한 손으로 받치고)")

t0, current = time.time(), None
try:
    while time.time() - t0 < seconds:
        if poses:
            p = poses[min(int((time.time() - t0) / HOLD), len(poses) - 1)]   # 지금 가야 할 자세
            if p != current:
                current = p
                print(f"{time.time() - t0:5.1f} s  → {current}")
                bot.go_pose(current, speed=200, wait=False)
        ok, frame = cap.read()
        if not ok:
            break
        ang = bot.angles()
        rows.append([round(time.time() - t0, 3)] + [round(ang[s], 1) for s in range(1, 7)] + tip_xyz(bot))
        video.write(frame)
finally:
    video.release()
    cap.release()
    if poses:
        bot.wait_until_settled(timeout=5)
    bot.close()

mp4 = OUT / f"{name}.mp4"
shutil.move(str(tmp), mp4)
with open(OUT / f"{name}.csv", "w", newline="", encoding="utf-8") as f:
    wr = csv.writer(f)
    wr.writerow(["t", "j1", "j2", "j3", "j4", "j5", "j6", "x", "y", "z"])
    wr.writerows(rows)
dt = (rows[-1][0] - rows[0][0]) / max(1, len(rows) - 1) if len(rows) > 1 else 0
print(f"\n{len(rows)} 장, {rows[-1][0]:.1f} 초, 장 간격 평균 {dt:.3f} s ({1 / dt if dt else 0:.1f} 장/초)")
print(f"손끝 z: {min(r[9] for r in rows):.3f} ~ {max(r[9] for r in rows):.3f} m")
print("영상:", mp4)
print("관절:", OUT / f"{name}.csv")
