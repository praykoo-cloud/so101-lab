"""실습 01 — 팔과 연결하고 상태 읽기 (팔은 움직이지 않는다).

    python lessons/01_hello.py

할 일
  1. PORT 를 내 팔의 포트로 바꾼다 (scripts/find_port.py 로 확인)
  2. 실행해서 관절 6개의 각도 · 전압 · 온도가 나오는지 본다
  3. 토크가 꺼진 상태에서 팔을 손으로 조금 움직이고 다시 실행 → 각도가 바뀌는지 본다
"""
import os

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")     # 리눅스 · 라즈베리파이: "/dev/ttyACM0"

bot = Robot(PORT)
print("연결:", bot)
print("실물 연결:", bot.real)          # False 면 가상 팔 (SOARM_PORT=fake)

for sid, s in bot.status().items():
    print(f"  {sid}. {s['name']:<12} {s['angle_deg']:7.1f}°   전압 {s['voltage']:.1f}V   "
          f"온도 {s['temperature']}℃   토크 {'켜짐' if s['torque_enabled'] else '꺼짐'}")
print(f"그리퍼: {bot.gripper_percent()}% 열림")

bot.close()
