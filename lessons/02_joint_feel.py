"""실습 02 — 관절 하나씩 움직여 보기.

    python lessons/02_joint_feel.py

할 일
  1. 실행하면 1번 관절(base)이 지금 자리에서 +10° 갔다가 돌아온다
  2. JOINT 를 "shoulder" · "elbow" · "wrist_pitch" · "wrist_roll" 로 바꿔 가며 실행한다
  3. 관절마다 + 방향이 어느 쪽인지 적는다 (예: base + = 위에서 봤을 때 시계 방향)

주의: 처음엔 STEP 을 10° 이하로. 접힌 자세에서 shoulder · wrist_pitch 는 - 쪽이 끝이라 + 로만 움직인다.
"""
import os

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
JOINT = "base"      # base · shoulder · elbow · wrist_pitch · wrist_roll
STEP = 10           # 도

bot = Robot(PORT, speed=150)
bot.torque(True)                     # 잠그기: 지금 자세 그대로 힘이 들어간다

start = bot.angles()
sid = {"base": 1, "shoulder": 2, "elbow": 3, "wrist_pitch": 4, "wrist_roll": 5}[JOINT]
print(f"{JOINT}: 지금 {start[sid]:.1f}°")

bot.move(JOINT, start[sid] + STEP)
bot.wait_until_settled()
print(f"  +{STEP}° → {bot.angles()[sid]:.1f}°")

bot.move(JOINT, start[sid])
bot.wait_until_settled()
print(f"  제자리 → {bot.angles()[sid]:.1f}°")

bot.close()                          # 토크는 켜진 채로 둔다 (팔이 자세를 유지)
