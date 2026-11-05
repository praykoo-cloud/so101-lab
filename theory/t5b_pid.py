"""이론 5b — PID: 중력이 있으면 P · PD 만으로는 목표에 못 닿는다 → I 로 남은 오차를 없앤다.

    python theory/t5b_pid.py      # 시뮬레이션 (팔을 움직이지 않는다)

t5 와 같은 관절(관성 J 막대 + 마찰, 모터 힘 ±U_MAX)에 중력 G·cos(θ) 를 더했다 (어깨처럼 팔 무게를 드는 관절).
  P   : 힘 = Kp × e
  PD  : 힘 = Kp × e - Kd × 속도
  PID : 힘 = Kp × e + Ki × (e 를 쌓은 값) - Kd × 속도      e = 목표 - 지금

할 일
  1. P · PD 가 목표 30° 바로 아래에서 멈추는 것을 본다 (남은 오차 = 중력 ÷ Kp)
  2. I 를 넣으면 남은 오차가 0 으로 가는 것을 본다. Ki 를 키우면 빨라지지만 넘침이 커진다
  3. 적분 한계(anti-windup)를 빼면 처음에 너무 많이 쌓여 크게 넘친다
"""
import numpy as np

J, FRICTION, U_MAX = 0.002, 0.03, 0.5      # t5 와 같음: kg·m², N·m·s/rad, N·m
G = 0.15                                   # 중력 토크 (N·m, 팔이 수평일 때). 30° 에서 0.13 N·m
DT, T_END, GOAL = 0.001, 3.0, np.radians(30)
KP, KD = 10, 0.15                          # t5 의 PD 와 같음


def run(kp, kd=0.0, ki=0.0, windup=True):
    th, w, acc, log = 0.0, 0.0, 0.0, []
    for _ in range(int(T_END / DT)):
        e = GOAL - th
        acc += e * DT
        if windup:                                        # 적분이 혼자 모터 힘 한계를 넘지 않게
            acc = float(np.clip(acc, -U_MAX / max(ki, 1e-9), U_MAX / max(ki, 1e-9)))
        u = float(np.clip(kp * e + ki * acc - kd * w, -U_MAX, U_MAX))
        w += (u - FRICTION * w - G * np.cos(th)) / J * DT
        th += w * DT
        log.append(np.degrees(th))
    return np.array(log)


def report(name, log):
    goal = np.degrees(GOAL)
    near = np.abs(log - goal) <= 0.1
    settle = next((i for i in range(len(log)) if near[i:].all()), None)
    t = f"{settle * DT * 1000:5.0f} ms" if settle is not None else "  못 들어옴"
    print(f"{name:22s} 넘침 {max(0, log.max() - goal):4.1f}°   0.1° 안 {t}   "
          f"남은 오차 (마지막 0.5 s 평균) {goal - log[-500:].mean():5.2f}°")


print(f"중력 G = {G} N·m → 30° 를 버티는 데 {G * np.cos(GOAL):.3f} N·m. P 만이면 남는 오차 약 {np.degrees(G * np.cos(GOAL) / KP):.2f}°\n")
report(f"P   Kp={KP}", run(KP))
report(f"PD  Kp={KP} Kd={KD}", run(KP, KD))
for ki in (10, 30, 100):
    report(f"PID Ki={ki}", run(KP, KD, ki))
report("PID Ki=30 적분 한계 없음", run(KP, KD, 30, windup=False))
