"""실습 09 — 나만의 동작: 짧은 동작을 함수로 만들고 이어 붙이기.

    python lessons/09_my_motion.py fake      가상 팔로 먼저
    python lessons/09_my_motion.py           실물 (SOARM_PORT, 없으면 COM4)

할 일
  1. 가상 팔(fake)로 먼저 돌려 순서 · 걸리는 시간을 본다
  2. 실물에서는 주변 50 cm 를 비우고, 처음에는 SPEED 를 80 으로
  3. 동작 함수를 하나 더 만들어 SHOW 에 넣는다. 재료: pose(관절) · bot.go_pose(저장한 자세, 06) · bot.gripper
"""
import os
import sys
import time

from soarm import Robot

PORT = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SOARM_PORT", "COM4")
SPEED = 150                  # 서보 속도 (틱/초). 200 ≈ 17.6°/s
READY = {"base": 0, "shoulder": 0, "elbow": 0, "wrist": 0, "roll": 0}   # 0 자세: 윗팔 위로, 아랫팔 · 손목 앞으로


def pose(bot, speed=SPEED, **change):
    """READY 에서 change 만 바꾼 자세로 가고, 도착할 때까지 기다린다"""
    bot.move_all({**READY, **change}, speed=speed)
    bot.wait_until_settled(tolerance_deg=2, timeout=8)


def hello(bot):
    """손목을 좌우로 돌리며 집게를 벌렸다 닫는다"""
    for _ in range(2):
        bot.gripper(60)
        pose(bot, roll=40)
        bot.gripper(10)
        pose(bot, roll=-40)
    pose(bot)


def nod(bot):
    """손목을 위아래로 (끄덕이기)"""
    for _ in range(2):
        pose(bot, wrist=20)
        pose(bot, wrist=-20)
    pose(bot)


def no(bot):
    """몸통을 좌우로 (도리도리)"""
    for _ in range(2):
        pose(bot, base=25)
        pose(bot, base=-25)
    pose(bot)


SHOW = [hello, nod, no]

bot = Robot(PORT)
bot.torque(True)
t0 = time.time()
try:
    print("READY 로 (접힌 자세에서 어깨가 크게 움직인다)")
    pose(bot, speed=100)
    for act in SHOW:
        print(f"{time.time() - t0:5.1f} s  {act.__name__}: {act.__doc__}")
        act(bot)
    print(f"{time.time() - t0:5.1f} s  끝")
except KeyboardInterrupt:
    bot.stop()                       # Ctrl+C: 그 자리에서 멈춘다 (토크는 켜진 채)
    print("멈춤")
finally:
    bot.close()
