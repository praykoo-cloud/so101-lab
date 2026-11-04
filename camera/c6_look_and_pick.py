"""카메라 6 — 보고 집기: 카메라로 빨강 물체를 찾아 그 자리로 팔을 보내 집고, 정해 둔 곳에 놓는다.

    python camera/c6_look_and_pick.py --dry      계산만 (팔을 움직이지 않는다). 먼저 이걸로
    python camera/c6_look_and_pick.py            한 번 집어서 놓기
    python camera/c6_look_and_pick.py --loop     물체가 보일 때마다 계속 (Ctrl+C 로 끝)

순서: home 자세 → 사진 → 빨강 덩어리 → table_map.json 으로 물체 가운데 (x, y) → 역기구학(집게 수직)
      → 위 6 cm → 내려가 집기 → 올리기 → 놓는 곳 → 놓기 → home
먼저 할 것: c5 (table_map.json), 자세 'home' 저장 (1일차 lessons/06)

할 일
  1. --dry 로 물체를 옮겨 가며 (x, y) 와 각도가 그럴듯한지 본다. '닿지 않음' 이 나오는 자리를 찾아 본다
  2. 실제로 집어 본다. 손끝이 물체 앞이나 뒤를 짚으면 HALF 를, 늘 한쪽으로 빗나가면 OFFSET 을 고친다
  3. PLACE 를 바꿔 놓는 자리를 옮긴다. --loop 로 물체를 놓을 때마다 치우게 해 본다
"""
import sys
import time

from camlib import TableMap, find_blob, ik_down, load_settings, move_urdf, open_cam, urdf_deg
from soarm import Robot

ABOVE = 0.06                  # 물체 위 몇 m 에서 내려갈지
GRIP_Z = 0.015                # 집는 높이 (책상 위 m). 물체가 크면 올린다
HALF = 0.0125                 # 물체 앞뒤 길이의 반 (m). 2.5 cm 블록 → 0.0125, 납작한 종이 → 0
PLACE = (0.18, -0.15)         # 놓는 곳 (x, y m). 오른쪽 앞
OFFSET = (0.0, 0.0)           # 늘 한쪽으로 빗나가면 여기서 고친다 (m)
SPEED = 150

dry = "--dry" in sys.argv
loop = "--loop" in sys.argv
tm = TableMap.load()
cfg = load_settings()
cap = open_cam(cfg)
bot = Robot()
print(bot, f"책상 높이 z = {tm.z:.3f} m", "(계산만)" if dry else "")


def look():
    for _ in range(5):                            # 팔이 움직이는 동안 쌓인 장을 버린다
        cap.read()
    ok, frame = cap.read()
    return find_blob(frame, "red") if ok else None


def solve(x, y, z, seed):
    q = ik_down((x, y, z), seed)
    if q is None:
        print(f"   ({x:.3f}, {y:.3f}, {z:.3f}) 닿지 않음")
    return q


def pick_once():
    blob = look()
    if blob is None:
        print("빨강 물체가 안 보입니다")
        return False
    x, y = tm.object_xy(blob, HALF)                # 덩어리 아래 가장자리(책상에 닿은 곳) + HALF
    x, y = x + OFFSET[0], y + OFFSET[1]
    print(f"덩어리 중심 ({blob['u']:.0f}, {blob['v']:.0f}) → 물체 가운데 x={x:.3f} y={y:.3f} m")
    seed = urdf_deg(bot)
    q_up = solve(x, y, tm.z + ABOVE, seed)
    q_down = q_up and solve(x, y, tm.z + GRIP_Z, q_up)
    q_place = q_up and solve(PLACE[0], PLACE[1], tm.z + ABOVE, q_up)
    if not (q_up and q_down and q_place):
        return False
    for name, q in (("위", q_up), ("집기", q_down), ("놓는 곳", q_place)):
        print(f"   {name:5s} 관절(URDF °) {q}")
    if dry:
        return True
    bot.open_gripper()
    move_urdf(bot, q_up, SPEED)
    move_urdf(bot, q_down, SPEED // 2)
    bot.close_gripper()
    time.sleep(1.0)
    move_urdf(bot, q_up, SPEED)
    move_urdf(bot, q_place, SPEED)
    bot.open_gripper()
    time.sleep(0.8)
    bot.go_pose("home", speed=SPEED)
    print("   놓았습니다")
    return True


try:
    if not dry:
        bot.torque(True)
        bot.go_pose("home", speed=SPEED)
    while True:
        pick_once()
        if not loop:
            break
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\n멈춤")
    if not dry:
        bot.stop()
finally:
    cap.release()
    bot.close()
