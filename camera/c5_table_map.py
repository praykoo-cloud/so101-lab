"""카메라 5 — 픽셀을 로봇 좌표로: 팔 손끝으로 책상의 네 점을 짚어 맞추기 (호모그래피).

    python camera/c5_table_map.py                점 4개 (기본)
    python camera/c5_table_map.py --points 5     5번째 점으로 정확도까지 잰다 (권장)

점마다 두 번 Enter:
  ① 빨강 종이 조각(지름 4 cm 쯤, 멀리서도 보이게)을 책상 위 k 번 자리에 놓고, 팔은 화면에서 비킨 채 Enter → 카메라가 종이 중심 픽셀 (u, v) 를 찾는다
  ② 토크가 꺼진 팔의 손끝(집게 사이)을 종이 가운데에 대고 Enter → 정기구학으로 손끝 (x, y, z) 를 계산한다
결과: camera/table_map.json (3×3 행렬 H, 책상 높이 z) · camera/out/table_map.jpg

할 일
  1. 네 점은 책상의 네 모서리처럼 넓게. 한 줄에 놓으면 계산이 안 된다
  2. 5번째 점은 가운데쯤. 출력의 '시험 점 오차' 가 10 mm 안쪽이면 c6 으로 집을 수 있다
  3. 카메라를 건드리면 처음부터 다시 (H 는 카메라 자리에 묶여 있다)
"""
import sys

from camlib import TableMap, draw_blob, find_blob, load_settings, open_cam, save_jpg, tip_xyz
from soarm import Robot

N = int(sys.argv[sys.argv.index("--points") + 1]) if "--points" in sys.argv else 4
FIT = min(N, 4) if N >= 5 else N                 # 5개 이상이면 앞 4개로 맞추고 나머지로 시험


def ask(msg):
    try:
        input(msg)
    except (KeyboardInterrupt, EOFError):
        sys.exit("\n중단")


cfg = load_settings()
cap = open_cam(cfg)
bot = Robot()
bot.torque(False)
print(bot, "— 토크 꺼짐. 팔을 손으로 움직일 수 있습니다 (한 손으로 받치기)\n")

pixels, points, last = [], [], None
for k in range(1, N + 1):
    while True:
        ask(f"[{k}/{N}] ① 빨강 종이를 {k}번 자리에 놓고 팔을 화면에서 비킨 뒤 Enter ")
        for _ in range(3):                       # 오래된 장을 버린다
            cap.read()
        ok, frame = cap.read()
        blob = find_blob(frame, "red") if ok else None
        if blob:
            break
        print("   종이를 찾지 못했습니다. 화면 안인지, 빛이 번쩍이지 않는지 보고 다시")
    print(f"   픽셀 (u, v) = ({blob['u']:.1f}, {blob['v']:.1f})")
    last = draw_blob(frame.copy() if last is None else last, blob, label=str(k))
    ask("   ② 손끝을 종이 가운데에 대고 Enter ")
    xyz = tip_xyz(bot)
    print(f"   손끝 (x, y, z) = ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m\n")
    pixels.append([blob["u"], blob["v"]])
    points.append(xyz)
cap.release()
bot.close()

tm = TableMap.fit(pixels[:FIT], points[:FIT])
print("H =")
for row in tm.H:
    print("   ", "  ".join(f"{v:12.6f}" for v in row))
print(f"책상 높이 z = {tm.z:.3f} m (점 {FIT}개 평균)")
for px, pt in zip(pixels[FIT:], points[FIT:]):
    x, y = tm.to_xy(*px)
    d = ((x - pt[0]) ** 2 + (y - pt[1]) ** 2) ** 0.5 * 1000
    print(f"시험 점 오차: 카메라 ({x:.3f}, {y:.3f}) ↔ 손끝 ({pt[0]:.3f}, {pt[1]:.3f}) → {d:.1f} mm")
print("저장:", tm.save())
print("사진:", save_jpg(last, "table_map"))
