"""실습 06 — 티칭 3: 자세를 이름으로 저장하기 (data/poses.json).

    python lessons/06_save_poses.py                 자세마다 Enter
    python lessons/06_save_poses.py --countdown 5   자세마다 5초 세고 자동 저장 (두 손으로 잡을 때)

할 일
  1. 토크가 풀리면 안내에 나오는 자세를 손으로 잡고 Enter
     home(접힌 자세) · above_pick(물체 위 5cm) · pick(물체를 감싸는 높이) · above_place · place
  2. 그리퍼 값도 같이 저장되지만 go_pose 는 그리퍼를 움직이지 않는다. 열고 닫기는 07 에서 gripper() 로
  3. 다 하면 data/poses.json 이 생긴다. 다시 잡고 싶은 자세만 NAMES 에 남기고 다시 실행해도 된다
"""
import os
import sys
import time

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
NAMES = ["home", "above_pick", "pick", "above_place", "place"]

countdown = int(sys.argv[sys.argv.index("--countdown") + 1]) if "--countdown" in sys.argv else 0

bot = Robot(PORT)
bot.torque(False)
print("토크를 풀었습니다. 팔을 받치고 자세를 잡으세요.\n")

for name in NAMES:
    if countdown:
        print(f"'{name}' 자세를 잡으세요 ", end="", flush=True)
        for k in range(countdown, 0, -1):
            print(f"{k}.. ", end="", flush=True)
            time.sleep(1)
        print()
    else:
        input(f"'{name}' 자세를 잡고 Enter: ")
    bot.save_pose(name)
    a = bot.poses()[name]["angles"]
    print(f"  저장: {name:12s} " + "  ".join(f"{a[i]:6.1f}" for i in range(1, 6)) + f"   그리퍼 {bot.gripper_percent()}%")

bot.torque(True)
print("\n잠갔습니다. 저장한 자세:", list(bot.poses()))
bot.close()
