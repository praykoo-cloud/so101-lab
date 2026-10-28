"""실습 05 — 그리퍼: 열기 · 닫기, 내 물체에 맞는 '닫기' 값 찾기.

    python lessons/05_gripper.py

할 일
  1. 실행하면 그리퍼가 OPEN% → CLOSE% → OPEN% 로 움직인다 (0 = 닫힘, 100 = 열림)
  2. 열린 그리퍼 사이에 큐브(또는 종이 상자)를 끼우고 CLOSE 를 바꿔 가며 실행한다
  3. 물체가 빠지지 않으면서 서보가 너무 세게 누르지 않는 값을 적는다 (부하 숫자도 본다)
"""
import os

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
OPEN = 60           # %
CLOSE = 5           # % ← 내 물체에 맞게

bot = Robot(PORT, speed=300)
bot.torque(True)

for label, pct in [("열기", OPEN), ("닫기", CLOSE), ("열기", OPEN)]:
    bot.gripper(pct)
    bot.wait_until_settled(timeout=6)      # 물체에 막히면 0.5초 멈춘 뒤 끝난다
    load = bot.driver.get_load(6)
    print(f"{label} {pct}% → 지금 {bot.gripper_percent()}%   부하 {load}")

bot.close()
