"""이론 5 — 관절 하나를 목표 각도로: 뱅뱅 제어 vs 비례(P) 제어 vs PD.

    python theory/t5_bangbang_vs_p.py      # 시뮬레이션 (팔을 움직이지 않는다)

관절 = 관성 J 인 막대 + 마찰. 모터가 낼 수 있는 힘은 ±U_MAX 까지
  뱅뱅 : 목표보다 작으면 +최대, 크면 -최대
  P    : 힘 = Kp × (목표 - 지금)
  PD   : 힘 = Kp × 오차 - Kd × 속도      ← STS3215 서보 안에서 도는 것도 이런 위치 제어

할 일
  1. 세 방법으로 0° → 30° 를 시켜 도착 시간 · 넘침 · 떨림을 비교한다
  2. Kp 를 키우면 도착 시간과 넘침이 어떻게 바뀌는지 본다 (모터 힘 한계 ±U_MAX 를 생각하며)
  3. Kd 를 넣으면 넘침이 줄어드는 것을 본다
"""
import numpy as np

J, FRICTION, U_MAX = 0.002, 0.03, 0.5      # kg·m², N·m·s/rad, N·m
DT, T_END, GOAL = 0.001, 3.0, np.radians(30)


def run(ctrl):
    th, w, log = 0.0, 0.0, []
    for k in range(int(T_END / DT)):
        u = float(np.clip(ctrl(GOAL - th, w), -U_MAX, U_MAX))
        w += (u - FRICTION * w) / J * DT
        th += w * DT
        log.append(np.degrees(th))
    return np.array(log)


def report(name, log):
    goal = np.degrees(GOAL)
    inside = np.abs(log - goal) <= 1.0
    settle = next((i for i in range(len(log)) if inside[i:].all()), None)
    tail = log[-500:]                                     # 마지막 0.5 초
    step = np.diff(np.round(tail, 2))
    flips = int(np.sum(np.diff(np.sign(step[step != 0])) != 0))   # 0.01° 넘게 움직인 방향이 바뀐 횟수
    t = f"{settle * DT * 1000:5.0f} ms" if settle is not None else "  못 멈춤"
    print(f"{name:16s} 1° 안에 머무름 {t}   넘침 {max(0, log.max() - goal):5.1f}°   "
          f"마지막 0.5 s: 떨림 ±{(tail.max() - tail.min()) / 2:4.2f}°, 방향 바뀜 {flips}번")


report("뱅뱅", run(lambda e, w: U_MAX * np.sign(e)))
for kp in (1, 3, 10):
    report(f"P  Kp={kp}", run(lambda e, w, kp=kp: kp * e))
for kd in (0.05, 0.15):
    report(f"PD Kp=10 Kd={kd}", run(lambda e, w, kd=kd: 10 * e - kd * w))
