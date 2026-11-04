"""이론 6 — 궤적: A 에서 B 로 '어떻게' 가나 (관절 따로 · 관절 같이 · 직선).

    python theory/t6_trajectory.py      # 계산만 (팔을 움직이지 않는다)

Robot.move · go_pose 는 관절마다 같은 속도(서보 속도, 틱/초)로 목표까지 간다.
덜 도는 관절이 먼저 도착해 멈추니까, 손끝은 직선이 아니라 휘어진 길로 간다.
  관절 따로 : 관절마다 같은 속도, 먼저 끝난 관절은 멈춤        ← 지금 soarm 이 하는 것
  관절 같이 : 모든 관절이 같은 순간에 도착하게 속도를 나눔      (가장 많이 도는 관절이 시간을 정함)
  직선      : 손끝 길을 점 60개로 나눠 점마다 역기구학          (관절 속도 한계를 넘지 않게 시간을 늘림)

할 일
  1. 세 방법이 직선에서 얼마나 벗어나는지(최대 mm), 가장 낮은 높이를 비교한다
  2. 직선으로 가면 무엇을 내주나 (걸리는 시간)
  3. 물체를 집은 뒤 '똑바로 위로' 들어야 하는 때는 어떤 방법이 필요할까
"""
import numpy as np

from soarm.kinematics import fk, ik_numeric

SPEED = 200 / 4096 * 360          # 서보 속도 200 틱/초 = 17.6 °/s (Robot 기본값)
DT = 0.01
A = [0.15, 0.10, 0.03]            # 왼쪽 앞, 낮게 (물체를 집은 자리)
B = [0.28, -0.05, 0.03]           # 오른쪽 멀리, 같은 높이 (놓을 자리)


def solve(target, q0):
    r = ik_numeric(target, q0)
    if r is None:
        raise SystemExit(f"{target} 닿지 않음")
    return r[0]


def separate(qa, qb):
    """관절마다 같은 속도로, 먼저 끝나면 멈춤"""
    d = qb - qa
    T = np.max(np.abs(d)) / SPEED
    return [qa + np.sign(d) * np.minimum(np.abs(d), SPEED * t) for t in np.arange(0, T + DT, DT)], T


def together(qa, qb):
    """모두 같은 순간에 도착 (가장 많이 도는 관절의 시간)"""
    d = qb - qa
    T = np.max(np.abs(d)) / SPEED
    return [qa + d * min(1.0, t / T) for t in np.arange(0, T + DT, DT)], T


def straight(qa, pa, pb, n=60):
    """손끝 직선 위 점마다 역기구학. 이웃 점 사이 관절 변화가 속도 한계를 넘지 않게 시간을 잡는다"""
    qs, q = [qa], qa
    for s in np.linspace(0, 1, n)[1:]:
        q = solve(pa + s * (pb - pa), q)
        qs.append(q)
    steps = [np.max(np.abs(b - a)) / SPEED for a, b in zip(qs, qs[1:])]
    return qs, float(sum(steps))


def report(name, qs, T, pa, pb):
    tips = np.array([fk(q) for q in qs])
    ab = (pb - pa) / np.linalg.norm(pb - pa)
    rel = tips - pa
    off = np.linalg.norm(rel - np.outer(rel @ ab, ab), axis=1)          # 직선 AB 에서 떨어진 거리
    length = np.sum(np.linalg.norm(np.diff(tips, axis=0), axis=1))
    print(f"{name:8s} 시간 {T:4.1f} s   손끝 길이 {length * 1000:5.0f} mm   "
          f"직선에서 최대 {off.max() * 1000:5.1f} mm   가장 낮은 z {tips[:, 2].min() * 1000:5.1f} mm")


pa, pb = np.array(A), np.array(B)
qa = solve(A, [0, 0, 30, 60, 0])
qb = solve(B, qa)
print(f"A {A} → B {B}, 직선 거리 {np.linalg.norm(pb - pa) * 1000:.0f} mm, 서보 속도 {SPEED:.1f} °/s")
print(f"관절 변화 (°): {np.round(qb - qa, 1)[:4]}  (받침 · 어깨 · 팔꿈치 · 손목)\n")
report("관절 따로", *separate(qa, qb), pa, pb)
report("관절 같이", *together(qa, qb), pa, pb)
report("직선", *straight(qa, pa, pb), pa, pb)

print("\n물체를 집고 똑바로 5 cm 들어 올리기 (A → A + z 5 cm)")
up = pa + [0, 0, 0.05]
qu = solve(up, qa)
report("관절 따로", *separate(qa, qu), pa, up)
report("직선", *straight(qa, pa, up), pa, up)
