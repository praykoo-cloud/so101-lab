"""이론 1 — 정기구학: 관절 각도 → 손끝 위치.

    python theory/t1_fk.py            # 계산만
    python theory/t1_fk.py COM4       # 실물 팔의 지금 각도로도 계산 (토크는 건드리지 않는다)

할 일
  1. 2링크 팔을 손으로 계산한 값과 코드 값이 같은지 본다
  2. SO-101 의 DH 표(숫자 16개)로 계산한 손끝이 URDF 계산과 몇 mm 차이 나는지 본다
  3. 실물 팔을 손으로 움직여 놓고 실행해 손끝 위치를 자로 재 본다

각도는 URDF 기준 (모두 0° = 팔을 앞으로 쭉 편 자세). 위치는 mm.
"""
import sys

import numpy as np

from soarm.kinematics import fk

# ── 1. 2링크 팔 (평면) ───────────────────────────────────────────
# 어깨에서 L1, 팔꿈치에서 L2. θ1 은 바닥에서, θ2 는 윗팔에서 잰 각도
L1, L2 = 116.0, 135.0


def fk_2link(t1, t2):
    t1, t2 = np.radians(t1), np.radians(t2)
    x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
    z = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
    return x, z


print("[2링크] θ1=90°, θ2=-90° → 손 계산: x = 0 + 135 = 135, z = 116 + 0 = 116")
print("        코드:", [round(float(v), 1) for v in fk_2link(90, -90)])

# ── 2. SO-101 의 DH 표 ────────────────────────────────────────────
# 관절마다 좌표계를 하나씩 붙이고, 다음 좌표계까지를 숫자 4개로 적는다
#   a: 다음 축까지 거리,  α: 다음 축이 기운 각도,  d: 축을 따라 올라간 거리,  θ: 축 둘레 회전
# SO-101 은 어깨 · 팔꿈치 · 손목 축이 모두 평행 → 2 ~ 4 줄은 α = 0, d = 0 인 평면 3링크.
# θ 의 숫자는 0° 자세에서 링크가 기운 각도 (URDF 에서 잰 값). 관절이 + 로 돌면 팔이 아래로 숙인다
#        a(mm)   α(°)   d(mm)   θ(°)
DH = [(30.39, 90.0, 116.60, lambda q: -q[0]),          # 1 shoulder_pan (위에서 볼 때 + = 시계 방향)
      (116.00, 0.0, 0.00, lambda q: 76.03 - q[1]),     # 2 shoulder_lift  어깨 → 팔꿈치
      (135.00, 0.0, 0.00, lambda q: -73.82 - q[2]),    # 3 elbow_flex     팔꿈치 → 손목
      (159.40, 0.0, 0.00, lambda q: -5.05 - q[3])]     # 4 wrist_flex     손목 → 손끝
PAN_X = 38.84          # base_link 원점에서 회전축까지 (앞으로)


def dh(a, alpha, d, theta):
    """DH 한 줄 → 4×4 변환: z 축으로 θ 돌고 d 올라간 뒤, x 축으로 a 가고 α 기울기"""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def fk_dh(q):
    """q = [pan, lift, elbow, wrist_flex, wrist_roll] (도) → 손끝 (mm). 행렬 4개를 곱한다"""
    T = np.eye(4)
    T[0, 3] = PAN_X
    for a, alpha, d, theta in DH:
        T = T @ dh(a, np.radians(alpha), d, np.radians(theta(q)))
    return T[:3, 3]


print("\n[SO-101] 각도 → DH 표 / URDF (mm)")
for q in ([0, 0, 0, 0, 0], [0, 0, 90, 0, 0], [30, -20, 40, 10, 0], [-45, 30, 30, 60, 0]):
    a, b = fk_dh(q), fk(q) * 1000
    print(f"  {q} → DH {a.round(1)}  URDF {b.round(1)}  차이 {np.linalg.norm(a - b):.2f}")

rng = np.random.default_rng(0)
qs = rng.uniform([-110, -100, -96, -95, 0], [110, 100, 96, 95, 0], size=(2000, 5))
err = [np.linalg.norm(fk_dh(q) - fk(q) * 1000) for q in qs]
print(f"  무작위 2000 자세: 평균 차이 {np.mean(err):.2f} mm, 최대 {np.max(err):.2f} mm")

# ── 3. 실물 팔 ───────────────────────────────────────────────────
if len(sys.argv) > 1:
    from soarm import Robot

    with Robot(sys.argv[1]) as bot:
        rad = bot.joint_radians()
        q = [np.degrees(rad[i]) for i in range(1, 6)]
        print("\n[실물] 지금 각도", [round(v, 1) for v in q])
        print("       손끝 (mm)", (fk(q) * 1000).round(1), "← base_link 원점: 회전축 뒤 38.8 mm, 바닥면")
