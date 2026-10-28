"""이론 3 — 역기구학을 반복해서 풀기: CCD 와 감쇠 최소제곱(DLS).

    python theory/t3_ik_iterative.py

SO-101 은 식으로 한 번에 풀기엔 링크가 비스듬하고 관절 제한이 있다. 그래서 "조금 고치기"를 되풀이한다.
  CCD : 관절을 손끝 쪽부터 하나씩, 손끝이 목표를 보도록 돌린다
  DLS : 야코비안(관절을 조금 돌리면 손끝이 얼마나 움직이나)으로 모든 관절을 한 번에 고친다

할 일
  1. 같은 목표를 두 방법으로 풀어 반복 횟수와 남은 오차를 비교한다
  2. 닿지 않는 목표에서 두 방법이 어떻게 끝나는지 본다
  3. 시작 자세를 바꾸면 다른 답(다른 자세)이 나오는 것을 본다
"""
import numpy as np

from soarm.kinematics import LIMITS_DEG, fk, frames, ik_numeric

USE = [0, 1, 2, 3]          # pan · lift · elbow · wrist_flex (손목 회전은 손끝 위치를 거의 안 바꾼다)
TOL = 0.001                 # 1 mm


def ccd(target, q0, sweeps=200):
    """반환: (각도, 한 바퀴 횟수, 남은 오차 m)"""
    q = np.array(q0, float)
    for k in range(sweeps):
        if np.linalg.norm(target - fk(q)) < TOL:
            return q, k, float(np.linalg.norm(target - fk(q)))
        for i in reversed(USE):
            T = frames(q)
            joint = [t for name, t in T if name not in ("gripper_frame_joint",)][i]
            p, axis = joint[:3, 3], joint[:3, 2]
            tip = T[-1][1][:3, 3]
            a, b = tip - p, target - p
            a -= axis * (a @ axis)          # 관절 축에 수직인 평면에 비춘 두 벡터
            b -= axis * (b @ axis)
            if np.linalg.norm(a) < 1e-9 or np.linalg.norm(b) < 1e-9:
                continue
            ang = np.arctan2(axis @ np.cross(a, b), a @ b)
            q[i] = np.clip(q[i] + np.degrees(ang), *LIMITS_DEG[i])
    return q, sweeps, float(np.linalg.norm(target - fk(q)))


def dls(target, q0):
    """반환: (각도, 반복 횟수, 남은 오차 m). 200번 안에 1 mm 안으로 못 들어가면 반복 횟수 200"""
    return ik_numeric(target, q0=q0, tol=TOL, iters=200, strict=False)


start = [0, 0, 0, 0, 0]
targets = {"앞 아래 (250, 0, 100)": [0.25, 0.0, 0.10], "왼쪽 (150, 150, 80)": [0.15, 0.15, 0.08],
           "가까이 (120, -60, 40)": [0.12, -0.06, 0.04], "너무 멀리 (500, 0, 100)": [0.50, 0.0, 0.10]}

print(f"{'목표':24s} {'CCD 반복':>9s} {'CCD 오차':>9s} {'DLS 반복':>9s} {'DLS 오차':>9s}")
for name, t in targets.items():
    t = np.array(t)
    qc, kc, ec = ccd(t, start)
    qd, kd, ed = dls(t, start)
    print(f"{name:24s} {kc:>9} {ec * 1000:>7.2f}mm {kd:>9} {ed * 1000:>7.2f}mm")

print("(CCD 1번 = 관절 4개를 하나씩 돌린 한 바퀴, DLS 1번 = 관절 4개를 한꺼번에 고친 것)")

print("\n시작 자세에 따라 답이 달라진다 (목표 (200, 0, 150))")
t = np.array([0.20, 0.0, 0.15])
for s in ([0, 0, 0, 0, 0], [0, -60, 90, 0, 0], [0, 60, -60, 60, 0]):
    q, k, e = ik_numeric(t, q0=s, tol=TOL)
    print(f"  시작 {s} → 답 {np.round(q[:4], 1).tolist()}  ({k}번, 오차 {e * 1000:.2f} mm)")
