"""이론 7 — 야코비안과 특이점: 팔을 쭉 펴면 왜 관절이 크게 도나.

    python theory/t7_singularity.py      # 계산만

이론 2 의 2링크 팔 (L1 = 116, L2 = 135 mm, 어깨 θ1 · 팔꿈치 θ2) 로 본다.
야코비안 J = '관절을 조금 돌리면 손끝이 얼마나, 어느 쪽으로 움직이나' (2 × 2 표).
손끝을 v 만큼 움직이려면 관절은 dq = J⁻¹ v. 그런데 det J = L1 · L2 · sin θ2 라서
팔꿈치를 쭉 펴면(θ2 → 0) det J → 0 = 특이점. 바깥으로 1 cm/s 를 내려고 관절이 끝없이 빨라진다.
DLS(감쇠, 이론 3)는 관절 속도를 막는 대신 손끝이 덜 간다.

할 일
  1. 손끝을 바깥으로 뻗으며 '바깥으로 1 cm/s' 에 필요한 관절 속도가 어떻게 커지는지 본다 (서보 한계 약 17.6 °/s)
  2. 같은 자리에서 '옆(위)으로 1 cm/s' 는 왜 괜찮은지 본다
  3. 감쇠를 넣으면 관절 속도와 손끝 속도가 어떻게 바뀌는지 본다
     (ArmStudio 손끝 이동이 팔 끝에서 관절을 크게 튀게 하던 까닭 → 0.2.13 에서 한 번에 도는 각도를 막았다)
"""
import numpy as np

L1, L2 = 0.116, 0.135
V = 0.01                          # 손끝 1 cm/s
LAMBDA = 0.01                     # DLS 감쇠
SERVO = 17.6                      # 서보 기본 속도 °/s


def ik(r):
    """손끝을 어깨에서 거리 r, 수평(높이 0) 에 → θ1, θ2 (팔꿈치 한쪽 답)"""
    c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    t2 = -np.arccos(np.clip(c2, -1, 1))
    t1 = -np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
    return t1, t2


def jac(t1, t2):
    s1, c1, s12, c12 = np.sin(t1), np.cos(t1), np.sin(t1 + t2), np.cos(t1 + t2)
    return np.array([[-L1 * s1 - L2 * s12, -L2 * s12],
                     [L1 * c1 + L2 * c12, L2 * c12]])     # (x 바깥, z 위) ← (θ1, θ2)


def joint_speed(J, v, lam=0.0):
    dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(2), v)
    return np.degrees(np.max(np.abs(dq))), np.linalg.norm(J @ dq) * 1000


print(f"2링크 팔 L1 = {L1 * 1000:.0f}, L2 = {L2 * 1000:.0f} mm, 최대 {(L1 + L2) * 1000:.0f} mm. 손끝 {V * 100:.0f} cm/s 에 필요한 가장 빠른 관절\n")
print(f"{'거리':>6} {'팔꿈치':>7} {'|det J|':>9}  {'바깥으로':>10} {'위로':>9}   {'DLS 바깥으로: 관절 · 손끝':>24}")
for r in [0.150, 0.200, 0.230, 0.240, 0.245, 0.248, 0.250, 0.2505]:
    t1, t2 = ik(r)
    J = jac(t1, t2)
    out, _ = joint_speed(J, np.array([V, 0]))
    up, _ = joint_speed(J, np.array([0, V]))
    dq_d, v_d = joint_speed(J, np.array([V, 0]), LAMBDA)
    warn = "  ← 서보보다 빠름" if out > SERVO else ""
    print(f"{r * 1000:5.1f}mm {np.degrees(t2):6.1f}° {abs(np.linalg.det(J)) * 1e4:6.1f} cm²  {out:7.1f}°/s {up:6.1f}°/s"
          f"   {dq_d:7.1f}°/s · {v_d:4.1f} mm/s{warn}")
