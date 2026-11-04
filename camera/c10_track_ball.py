"""카메라 10 — 공 따라가기 (앞뒤만): 바닥 위 빨간 공을 사람이 앞뒤로 슬쩍 밀면, 팔이 공 바로 위를 따라간다.

    python camera/c10_track_ball.py               팔로 (SOARM_PORT). 끝은 Ctrl+C → 그 자리에 멈춤
    python camera/c10_track_ball.py --dry         팔 없이: 공의 x · 팔이 갈 x · 관절 각도만 찍는다
    python camera/c10_track_ball.py --window      카메라 창에 공 · x 를 그린다 (q 끝)
먼저: c9 로 camera/hand_eye.json (자동 · --teach · --ruler 아무거나)

SO-101 은 받침을 돌리지 않으면 앞뒤로 잘 움직인다 → 받침 0°, 집게를 세운 채 x 만 바꾼다.
매 장 역기구학을 풀지 않는다. 시작할 때 x 를 5 mm 간격으로 미리 풀어 '각도 표' 를 만들고 그 사이를 이어 쓴다:
  - 빠르다 (장마다 표 찾기)
  - 이웃 칸이 이어진 답이라 팔이 다른 자세로 튀지 않는다 (ArmStudio 0.2.13 에서 겪은 문제)
  - 표 밖(닿지 않는 x)은 끝 칸에서 멈춘다
흔들림 막기: 공 x 를 부드럽게(이동 평균) · 움직이는 중에는 8 mm 넘게 바뀔 때만 · 1초에 5번까지 보낸다.
공이 0.5초 멈추면 2 mm 차이도 보내 정확히 그 위에 선다. 공을 1초 넘게 놓치면 그 자리에 선다.

할 일
  1. --dry 로 공을 앞뒤로 밀며 '공 x' 와 '팔 목표' 가 같이 바뀌는지, 끝에서 멈추는지 본다
  2. 팔로: SPEED · ALPHA · DEADBAND 를 바꿔 '빠르지만 떨지 않는' 값을 찾는다
  3. 팔이 공을 가려 카메라가 공을 놓칠 때 무슨 일이 생기는지 본다 (ABOVE 를 높이거나 카메라를 옆으로)
"""
import sys
import time

import cv2
import numpy as np

from camlib import draw_blob, find_blob, hand_eye_xy, ik_down, load_hand_eye, load_settings, move_urdf, open_cam, tip_xyz, urdf_deg
from soarm import Robot

RADIUS = 0.03        # 공 반지름 (m) — c9 와 같게
ABOVE = 0.01         # 공 꼭대기 위로 집게 끝을 이만큼 (m). 집게를 세운 채로는 높이 갈수록 앞뒤로 덜 닿는다:
                     #   집게 끝 높이 6 cm → x 0.12~0.28 · 7 cm → 0.13~0.26 · 8 cm → 0.15~0.24 · 10 cm 부터 안 닿음
X_MIN, X_MAX = 0.12, 0.30
STEP = 0.005         # 각도 표 간격 (m)
ALPHA = 0.5          # 부드럽게: 새 값의 몫 (1 = 그대로, 작을수록 느긋)
DEADBAND = 0.008     # 움직이는 중에는 이보다 덜 바뀌면 보내지 않는다 (m) → 떨지 않게
SETTLE = 0.5         # 공이 이만큼(초) 멈춰 있으면 2 mm 차이도 한 번 더 보낸다 → 멈추면 정확히 그 위
FINE = 0.002
CMD_HZ = 5           # 1초에 보내는 명령 수 상한
LOST = 1.0           # 공을 이만큼(초) 못 보면 그 자리에 선다
SPEED = 300          # 서보 속도 (틱/초)

dry = "--dry" in sys.argv
window = "--window" in sys.argv
model = load_hand_eye()
z_tip = model["z"] + RADIUS + ABOVE                    # 공 중심 평면 + 반지름 = 공 꼭대기, 그 위로 ABOVE


def build_table():
    """x → 관절 각도 표. 가운데서 시작해 양쪽으로, 앞 칸의 답에서 다음 칸을 푼다 (이어진 자세)"""
    t0 = time.time()
    xs = np.round(np.arange(X_MIN, X_MAX + 1e-9, STEP), 4)
    mid = len(xs) // 2
    seed0 = [0, 0, 30, 60, 0]
    table = {}
    for order in (range(mid, len(xs)), range(mid - 1, -1, -1)):
        seed = table.get(xs[mid], seed0)
        for i in order:
            q = ik_down((xs[i], 0.0, z_tip), seed) or ik_down((xs[i], 0.0, z_tip), seed0)
            if q is None:
                break                                  # 여기부터 바깥은 닿지 않음
            table[xs[i]] = q
            seed = q
    keys = sorted(table)
    if not keys:
        sys.exit(f"높이 {z_tip:.3f} m 에서 닿는 x 가 없습니다 (ABOVE 를 낮추기)")
    print(f"각도 표: x {keys[0]:.3f} ~ {keys[-1]:.3f} m, {len(keys)} 칸, 집게 끝 높이 {z_tip:.3f} m ({time.time() - t0:.1f} 초)")
    return np.array(keys), np.array([table[k] for k in keys])


def q_for(x, keys, qs):
    """표 사이를 이어 쓴다 (관절마다 선형)"""
    return [float(np.interp(x, keys, qs[:, j])) for j in range(5)]


keys, qs = build_table()
lo, hi = float(keys[0]), float(keys[-1])
cap = open_cam(load_settings())
bot = None if dry else Robot()
if bot:
    bot.torque(True)
    print(bot, "→ 가운데로 천천히")
    move_urdf(bot, q_for((lo + hi) / 2, keys, qs), 120)

x_f, x_sent, t_sent, t_seen, t_print, stopped = None, None, 0.0, time.time(), 0.0, False
x_still, t_still = None, time.time()
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        b = find_blob(frame, "red")
        now = time.time()
        if b is None:
            if now - t_seen > LOST and not stopped:
                print(f"{time.strftime('%H:%M:%S')}  공 놓침 → 그 자리에 정지")
                if bot:
                    bot.stop()
                stopped = True
        else:
            t_seen, stopped = now, False
            x_raw = hand_eye_xy(model, b["u"], b["v"])[0]
            x_f = x_raw if x_f is None else ALPHA * x_raw + (1 - ALPHA) * x_f
            x_t = float(np.clip(x_f, lo, hi))
            if x_still is None or abs(x_t - x_still) > FINE:
                x_still, t_still = x_t, now                    # 공이 움직였다 → 멈춤 시계 다시
            settled = now - t_still >= SETTLE
            need = x_sent is None or abs(x_t - x_sent) >= (FINE if settled else DEADBAND)
            if need and now - t_sent >= 1 / CMD_HZ:
                q = q_for(x_t, keys, qs)
                if bot:
                    move_urdf(bot, q, SPEED, wait=False)
                x_sent, t_sent = x_t, now
            if now - t_print >= 0.5:
                t_print = now
                edge = "  (끝: 더 못 감)" if x_f < lo or x_f > hi else ""
                tip = f" · 손끝 {tip_xyz(bot)[0]:.3f}" if bot else f" · 관절 {[round(v) for v in q_for(x_t, keys, qs)[:4]]}"
                print(f"{time.strftime('%H:%M:%S')}  공 x {x_raw:.3f} → 팔 목표 {x_t:.3f} m{tip}{edge}")
        if window:
            show = draw_blob(frame.copy(), b, label=f"x {x_f:.3f} m" if (b and x_f is not None) else "")
            cv2.imshow("track (q: quit)", show)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
except KeyboardInterrupt:
    pass
finally:
    if bot:
        bot.stop()
        bot.close()
    cap.release()
    cv2.destroyAllWindows()
    print("끝 (팔은 그 자리에 힘을 준 채 멈춤. 내려놓을 때는 받치고 토크 끄기)")
