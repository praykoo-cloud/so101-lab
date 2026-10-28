"""실습 08 — 티칭 5: 손으로 움직인 동작을 녹화하고 재생하기.

    python lessons/08_record_play.py               녹화 SECONDS 초 → 재생
    python lessons/08_record_play.py --play take1  저장된 녹화만 재생

할 일
  1. 실행하면 3초 뒤 녹화가 시작된다. 토크가 풀린 팔을 손으로 천천히 움직인다
  2. 녹화가 끝나면 팔을 놓는다. 팔이 녹화 첫 자세로 간 뒤 같은 동작을 따라 한다
  3. SPEED 를 0.5 · 2.0 으로 바꿔 재생 속도를 비교한다 (data/recordings/take1.json)
"""
import os
import sys
import time

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
SECONDS = 8
SPEED = 1.0
NAME = "take1"

bot = Robot(PORT)
if "--play" not in sys.argv:
    bot.torque(False)
    for k in (3, 2, 1):
        print(f"녹화 {k}..")
        time.sleep(1)
    print(f"녹화 중 ({SECONDS}초) — 팔을 손으로 움직이세요")
    frames = bot.record(SECONDS, interval_ms=100, name=NAME)
    print(f"녹화 끝: {len(frames)} 프레임 → data/recordings/{NAME}.json")
    print("팔을 놓으세요. 2초 뒤 재생합니다")
    time.sleep(2)
else:
    NAME = sys.argv[sys.argv.index("--play") + 1]

bot.torque(True)
bot.play(NAME, speed=SPEED)
print("재생 끝")
bot.close()
