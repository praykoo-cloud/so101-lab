"""실물 팔로워로 SDK 를 작게 움직여 보는 확인 (사람이 옆에 있을 때만).

    python tests/real_smoke.py COM4

1번 관절 · 손목 · 그리퍼를 몇 도씩만 움직이고, 자세 저장 → 이동 → 저장한 자세로 복귀를 본다.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from soarm import Robot  # noqa: E402


def show(bot, label):
    a = bot.angles()
    p = bot.positions()
    print(f"  {label:22s} 도 {[round(a[s], 1) for s in range(1, 7)]}  틱 {[p[s] for s in range(1, 7)]}")


port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
bot = Robot(port, speed=100)
try:
    show(bot, "시작 (토크 꺼짐)")
    bot.torque(True)
    time.sleep(1.0)
    show(bot, "토크 켬 (그대로여야)")

    a = bot.angles()
    bot.move("base", a[1] + 5)
    bot.wait_until_settled(timeout=4)
    show(bot, "1번 +5°")
    bot.move("base", a[1])
    bot.wait_until_settled(timeout=4)
    show(bot, "1번 복귀")

    bot.move("wrist_pitch", a[4] + 5)
    bot.wait_until_settled(timeout=4)
    show(bot, "손목 +5° (위로)")
    bot.move("wrist_pitch", a[4])
    bot.wait_until_settled(timeout=4)
    show(bot, "손목 복귀")

    g0 = bot.gripper_percent()
    bot.gripper(g0 + 12)
    time.sleep(1.5)
    print(f"  그리퍼 {g0}% → {bot.gripper_percent()}%")
    bot.gripper(g0)
    time.sleep(1.5)
    print(f"  그리퍼 복귀 → {bot.gripper_percent()}%")

    bot.save_pose("smoke_start")
    bot.move("base", a[1] + 5)
    bot.wait_until_settled(timeout=4)
    show(bot, "1번 +5° (저장 뒤)")
    bot.go_pose("smoke_start", timeout=5)
    show(bot, "go_pose(smoke_start)")
finally:
    bot.stop()
    bot.torque(False)
    time.sleep(0.5)
    show(bot, "토크 끔")
    print("  토크 상태:", {s: bot.driver.get_torque(s) for s in range(1, 7)})
    bot.delete_pose("smoke_start")
    bot.close()
