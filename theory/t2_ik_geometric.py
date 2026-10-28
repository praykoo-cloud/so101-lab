"""이론 2 — 역기구학을 식으로 풀기 (기하학 해).

    python theory/t2_ik_geometric.py

할 일
  1. 2링크 팔: 손끝 (x, z) 를 주면 코사인 법칙으로 θ1 · θ2 를 구한다
  2. 답이 2개(팔꿈치 위 · 아래)인 것과, 닿지 않으면 답이 없는 것을 확인한다
  3. 구한 각도를 정기구학(t1)에 다시 넣어 같은 점으로 가는지 본다
"""
import numpy as np

L1, L2 = 116.0, 135.0


def fk_2link(t1, t2):
    t1, t2 = np.radians(t1), np.radians(t2)
    return np.array([L1 * np.cos(t1) + L2 * np.cos(t1 + t2), L1 * np.sin(t1) + L2 * np.sin(t1 + t2)])


def ik_2link(x, z):
    """(x, z) → [(θ1, θ2), ...] 도. 닿지 않으면 빈 목록"""
    d2 = x * x + z * z
    c2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)      # 코사인 법칙: cos θ2
    if abs(c2) > 1:
        return []
    out = []
    for s2 in (np.sqrt(1 - c2 * c2), -np.sqrt(1 - c2 * c2)):   # 팔꿈치 아래 · 위
        t2 = np.arctan2(s2, c2)
        t1 = np.arctan2(z, x) - np.arctan2(L2 * s2, L1 + L2 * c2)
        out.append((np.degrees(t1), np.degrees(t2)))
    return out


for target in ((150, 100), (251, 0), (50, 20), (300, 0)):
    sols = ik_2link(*target)
    print(f"목표 {target}: 해 {len(sols)}개")
    for t1, t2 in sols:
        back = fk_2link(t1, t2)
        print(f"   θ1={t1:7.2f}°  θ2={t2:7.2f}°  → 정기구학 {back.round(3)}")

print(f"\n닿는 거리: {abs(L1 - L2):.0f} mm ~ {L1 + L2:.0f} mm (그 밖은 해 없음)")
