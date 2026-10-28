"""실습 07 — 티칭 4: 저장한 자세를 순서대로 → 픽 앤 플레이스.

    python lessons/07_pick_place.py

전제: 06 으로 home · above_pick · pick · above_place · place 를 저장했다.

할 일
  1. 물체를 pick 자리에 놓고 실행한다
  2. 순서: home → 열기 → above_pick → pick(천천히) → 닫기 → above_pick → above_place → place → 열기 → above_place → home
  3. 물체를 놓친다면: pick 자세를 다시 저장(06) 하거나, CLOSE 를 05 에서 찾은 값으로
"""
import os
import time

from soarm import Robot

PORT = os.environ.get("SOARM_PORT", "COM4")
OPEN, CLOSE = 60, 5          # 05 에서 찾은 값
FAST, SLOW = 200, 80         # 서보 속도 (틱/초)


def go(bot, name, speed=FAST):
    bot.go_pose(name, speed=speed)
    print(f"  → {name}")


def grip(bot, pct):
    bot.gripper(pct, speed=300)
    bot.wait_until_settled(timeout=6)      # 물체를 쥐면 그 자리에서 멈춘 뒤 끝난다
    time.sleep(0.3)
    print(f"  그리퍼 {pct}%  (부하 {bot.driver.get_load(6)})")


bot = Robot(PORT)
bot.torque(True)
try:
    go(bot, "home")
    grip(bot, OPEN)
    go(bot, "above_pick")
    go(bot, "pick", SLOW)
    grip(bot, CLOSE)
    go(bot, "above_pick", SLOW)
    go(bot, "above_place")
    go(bot, "place", SLOW)
    grip(bot, OPEN)
    go(bot, "above_place", SLOW)
    go(bot, "home")
    print("끝")
except KeyboardInterrupt:
    bot.stop()                       # Ctrl+C: 그 자리에서 멈춘다 (토크는 켜진 채)
    print("멈춤")
finally:
    bot.close()
