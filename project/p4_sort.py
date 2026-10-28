"""프로젝트 4 — 색 분류: 검사 자리의 물체를 색에 따라 빨강 · 파랑 상자로.

    python project/p4_sort.py               창 없이 글자만
    python project/p4_sort.py --window      카메라 화면도 띄운다 (PC)

흐름 (상태 기계)
  WAIT   검사 자리에 같은 색이 STABLE 장 연속 → PICK
  PICK   home → 열기 → above_pick → pick → 닫기 → above_pick
  PLACE  above_<색> → <색> → 열기 → above_<색> → home
  CLEAR  검사 자리가 none 이 될 때까지 (다음 물체를 기다림) → WAIT
Ctrl+C: 그 자리에서 멈춘다.
"""
import os
import sys
import time

from soarm import Robot
from soarm.color import ColorConfig, classify, draw, open_camera

PORT = os.environ.get("SOARM_PORT", "COM4")
OPEN, CLOSE = 60, 5          # 실습 05 에서 찾은 값
FAST, SLOW = 200, 80
STABLE = 5
window = "--window" in sys.argv


def say(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}")


def grip(bot, pct):
    bot.gripper(pct, speed=300)
    bot.wait_until_settled(timeout=6)
    time.sleep(0.3)


def pick(bot):
    bot.go_pose("home", FAST)
    grip(bot, OPEN)
    bot.go_pose("above_pick", FAST)
    bot.go_pose("pick", SLOW)
    grip(bot, CLOSE)
    bot.go_pose("above_pick", SLOW)


def place(bot, color):
    bot.go_pose(f"above_{color}", FAST)
    bot.go_pose(color, SLOW)
    grip(bot, OPEN)
    bot.go_pose(f"above_{color}", SLOW)
    bot.go_pose("home", FAST)


cfg = ColorConfig.load()
cap = open_camera(cfg.camera, cfg.width, cfg.height)
if window:
    import cv2
bot = Robot(PORT)
bot.torque(True)
bot.go_pose("home", FAST)

state, last, count, done = "WAIT", None, 0, {"red": 0, "blue": 0}
say("WAIT: 검사 자리에 물체를 놓으세요")
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            say("카메라 영상 없음")
            break
        color, ratios = classify(frame, cfg)
        count = count + 1 if color == last else 1
        last = color
        if window:
            cv2.imshow("sort", draw(frame, cfg, color, ratios))
            cv2.waitKey(1)

        if state == "WAIT" and count >= STABLE and color in ("red", "blue"):
            say(f"PICK: {color} {ratios[color]:.2f}")
            pick(bot)
            say(f"PLACE: {color} 상자")
            place(bot, color)
            done[color] += 1
            say(f"끝: 빨강 {done['red']} · 파랑 {done['blue']}")
            state, count = "CLEAR", 0
        elif state == "CLEAR" and count >= STABLE and color == "none":
            say("WAIT: 다음 물체를 놓으세요")
            state = "WAIT"
except KeyboardInterrupt:
    bot.stop()
    say("멈춤")
finally:
    cap.release()
    bot.close()
