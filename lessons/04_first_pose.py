"""실습 04 — 티칭 2: 적어 둔 자세로 가게 하기.

    python lessons/04_first_pose.py

할 일
  1. 03 에서 찍힌 POSE_0, POSE_1 줄을 아래에 붙여 넣는다 (지금 값은 예시)
  2. 실행하면 POSE_0 → POSE_1 → POSE_0 순서로 간다
  3. SPEED 를 100 · 300 으로 바꿔 차이를 본다 (서보 속도, 틱/초. 200 ≈ 17.6°/s)
"""
import os

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
SPEED = 150

# ▼ 03 에서 찍힌 줄로 바꾼다
POSE_0 = {"base": -4.0, "shoulder": -95.0, "elbow": 85.0, "wrist_pitch": -90.0, "wrist_roll": -5.0}
POSE_1 = {"base": 6.0, "shoulder": -95.0, "elbow": 85.0, "wrist_pitch": -85.0, "wrist_roll": -5.0}

bot = Robot(PORT, speed=SPEED)
bot.torque(True)

for name, pose in [("POSE_0", POSE_0), ("POSE_1", POSE_1), ("POSE_0", POSE_0)]:
    bot.move_all(pose)
    ok = bot.wait_until_settled(timeout=10)
    a = bot.angles()
    print(name, "도착" if ok else "시간 초과", [a[i] for i in range(1, 6)])

bot.close()
