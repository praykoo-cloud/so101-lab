"""실습 03 — 티칭 1: 손으로 자세를 잡고 각도 읽기.

    python lessons/03_teach_read.py

할 일
  1. 실행하면 토크가 풀린다. 팔을 한 손으로 받친다 (힘이 빠져 처질 수 있다)
  2. 원하는 자세로 옮기고 Enter → 그 자세가 코드 한 줄로 찍힌다
  3. 여러 번 반복한 뒤 q + Enter. 찍힌 줄을 04_first_pose.py 에 붙여 넣는다
"""
import os

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
NAMES = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]

bot = Robot(PORT)
bot.torque(False)
print("토크를 풀었습니다. 팔을 받치고 자세를 잡으세요.")

n = 0
while True:
    cmd = input(f"[{n}] 자세를 잡고 Enter (끝내려면 q): ").strip().lower()
    if cmd == "q":
        break
    a = bot.angles()
    pose = ", ".join(f'"{name}": {a[i + 1]:.1f}' for i, name in enumerate(NAMES))
    print(f"POSE_{n} = {{{pose}}}")
    print(f"   그리퍼 {bot.gripper_percent()}% 열림")
    n += 1

bot.torque(True)                     # 다시 잠그기 (마지막 자세에서)
print("잠갔습니다.")
bot.close()
