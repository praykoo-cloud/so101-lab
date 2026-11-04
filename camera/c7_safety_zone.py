"""카메라 7 — 안전 구역: 화면의 정해 둔 곳에 손이 들어오면 팔을 멈추고, 비면 다시 움직인다.

    python camera/c7_safety_zone.py                    창에서 안전 구역을 마우스로 끌고 Enter
    python camera/c7_safety_zone.py --zone 0 300 640 180   숫자로 (x y w h 픽셀, 라즈베리파이)
    python camera/c7_safety_zone.py --no-arm           팔 없이 감지만 시험

팔은 자세 home → above_pick → home 을 계속 돈다 (1일차에 저장한 자세).
감지 = 지금 장과 직전 장의 차이 (움직인 픽셀 비율). 팔도 움직이니까 구역은 팔이 지나가지 않는 곳(책상 앞쪽 가장자리)에.

할 일
  1. 구역에 손을 넣었다 빼며 '멈춤' · '다시 시작' 이 바로 나오는지 본다
  2. 그늘 · 형광등 깜빡임에 멈추면 THRESHOLD 를 올린다. 손을 넣어도 안 멈추면 내린다
  3. 1일차 '로봇 실습' 의 긴급정지와 비교: 버튼은 사람이 눌러야 하고, 카메라는 알아서 본다. 둘 다 있으면?
"""
import sys
import time

import cv2
import numpy as np

from camlib import load_settings, open_cam
from soarm import Robot

THRESHOLD = 0.02              # 구역 픽셀 중 2 % 넘게 바뀌면 '누가 있다'
QUIET = 1.5                   # 이만큼(초) 조용하면 다시 움직인다
POSES = ["home", "above_pick", "home"]

window = "--no-window" not in sys.argv
cfg = load_settings()
cap = open_cam(cfg)
ok, frame = cap.read()
if "--zone" in sys.argv:
    zone = [int(v) for v in sys.argv[sys.argv.index("--zone") + 1:][:4]]
elif window:
    zone = [int(v) for v in cv2.selectROI("safety zone: drag + Enter", frame, showCrosshair=False)]
    cv2.destroyAllWindows()
else:
    h, w = frame.shape[:2]
    zone = [0, int(h * 0.7), w, h - int(h * 0.7)]          # 기본: 화면 아래 30 % (책상 앞쪽)
print("안전 구역 (x y w h):", zone)

bot = None if "--no-arm" in sys.argv else Robot()
if bot:
    bot.torque(True)


def gray_zone(img):
    x, y, w, h = zone
    g = cv2.cvtColor(img[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(g, (9, 9), 0)


prev, stopped, quiet_since, step = gray_zone(frame), False, time.time(), 0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cur = gray_zone(frame)
        moved = float(np.mean(cv2.absdiff(cur, prev) > 25))      # 바뀐 픽셀 비율
        prev = cur
        someone = moved > THRESHOLD
        if someone:
            quiet_since = time.time()
            if not stopped:
                stopped = True
                print(f"{time.strftime('%H:%M:%S')}  멈춤 (구역 {moved * 100:.1f} % 바뀜)")
                if bot:
                    bot.stop()
        elif stopped and time.time() - quiet_since > QUIET:
            stopped = False
            print(f"{time.strftime('%H:%M:%S')}  다시 시작")
        if bot and not stopped and bot.wait_until_settled(timeout=0.01):
            bot.go_pose(POSES[step % len(POSES)], speed=150, wait=False)
            step += 1
        if window:
            x, y, w, h = zone
            show = frame.copy()
            cv2.rectangle(show, (x, y), (x + w, y + h), (0, 0, 255) if stopped else (0, 200, 0), 3)
            cv2.putText(show, f"{'STOP' if stopped else 'RUN'}  {moved * 100:.1f}%", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255) if stopped else (0, 200, 0), 2)
            cv2.imshow("safety (q: quit)", show)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
except KeyboardInterrupt:
    pass
finally:
    if bot:
        bot.stop()
        bot.close()
    cap.release()
    cv2.destroyAllWindows()
    print("끝")
