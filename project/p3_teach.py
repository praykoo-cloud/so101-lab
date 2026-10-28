"""프로젝트 3 — 분류에 쓸 자세 티칭.

    python project/p3_teach.py                  자세마다 Enter
    python project/p3_teach.py --countdown 5    자세마다 5초 세고 저장
    python project/p3_teach.py pick red         고치고 싶은 자세만 다시

저장하는 자세 (data/poses.json)
  home        접힌 자세. 카메라를 가리지 않는 곳
  above_pick  검사 자리 위 5cm      pick   물체를 감싸는 높이 (그리퍼는 벌린 채)
  above_red   빨강 상자 위           red    빨강 상자에 내려놓는 높이
  above_blue  파랑 상자 위           blue   파랑 상자에 내려놓는 높이
"""
import os
import sys
import time

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
ALL = ["home", "above_pick", "pick", "above_red", "red", "above_blue", "blue"]

args = [a for a in sys.argv[1:] if not a.startswith("--") and not a.isdigit()]
names = [n for n in args if n in ALL] or ALL
countdown = int(sys.argv[sys.argv.index("--countdown") + 1]) if "--countdown" in sys.argv else 0

bot = Robot(PORT)
bot.torque(False)
print("토크를 풀었습니다. 팔을 받치고 자세를 잡으세요.\n")
for name in names:
    if countdown:
        print(f"'{name}' ", end="", flush=True)
        for k in range(countdown, 0, -1):
            print(f"{k}.. ", end="", flush=True)
            time.sleep(1)
        print()
    else:
        input(f"'{name}' 자세를 잡고 Enter: ")
    bot.save_pose(name)
    a = bot.poses()[name]["angles"]
    print(f"  저장 {name:11s} " + " ".join(f"{a[i]:6.1f}" for i in range(1, 6)))
bot.torque(True)
missing = [n for n in ALL if n not in bot.poses()]
print("\n잠갔습니다." + (f" 아직 없는 자세: {missing}" if missing else " 자세 7개가 모두 있습니다."))
bot.close()
