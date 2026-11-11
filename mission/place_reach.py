"""허브 4번 'YOLO 넣기' 에 쓰는 손끝 좌표 도구: 모터 각도 ↔ 기구학 각도, 집게가 아래를 보는 IK, 손끝 옮기기.

쓰는 곳: mission/place_flow.py (5 바구니 위 맞추기 · 6 내려가기 · 8 들어올리기 · 카메라 방향 재기)
집게 기울기: 수직으로 세우지 않고 '학습한 5 자세의 기울기' 를 그대로 유지한다 (2026-11-11: 수직으로 세우면 손목이 한계를 넘었음)
바탕: soarm/kinematics.py (URDF 로 FK · 야코비안) + camera/camlib.py 의 ik_down 과 같은 방식 (수업 파일은 고치지 않고 같은 식을 여기에 둔다)

좌표: 로봇 받침 기준, m 단위 (x 앞, y 왼쪽, z 위). 손끝 = 집게 사이 (gripper_frame_link)

단어장
- FK (정기구학)  : 관절 각도 → 손끝 위치
- IK (역기구학)  : 손끝 위치 → 관절 각도 (여기서는 '집게가 바닥을 향하게' 조건을 하나 더 건다)
- 모터 각도      : soarm Robot.angles() 의 값 (서보 가운데 2048 = 0도). 허브가 저장하는 값
- URDF 각도      : 기구학 모델의 관절 각도. 캘리브레이션 범위로 모터 각도와 바꾼다 (Robot.joint_radians 와 같은 식)
"""
from __future__ import annotations

import math

import numpy as np

from soarm.kinematics import LIMITS_DEG, fk, frames
from soarm.robot import JOINT_LIMITS, POS_CENTER, deg_to_pos, pos_to_deg, position_from_fraction, window_fraction

JOINTS = ("1", "2", "3", "4", "5")


class ReachError(RuntimeError):
    """팔이 닿지 않거나 안전 범위를 넘음. 메시지는 화면에 그대로 보여 준다"""


# ── 모터 각도 ↔ URDF 각도 ──────────────────────────────────────────────
def urdf_from_angles(angles: dict) -> list[float]:
    """모터 각도 {"1": 도, …} → URDF 각도 5개(도). Robot.joint_radians() 와 같은 식"""
    out = []
    for j in JOINTS:
        sid = int(j)
        pos = deg_to_pos(float(angles[j]))
        lim = JOINT_LIMITS.get(sid) or {}
        t = window_fraction(pos, lim)
        if t is not None and "rad_min" in lim and "rad_max" in lim:
            rad = lim["rad_min"] + t * (lim["rad_max"] - lim["rad_min"])
        else:
            rad = (pos - POS_CENTER) / 4095 * 2 * math.pi
        if lim.get("invert"):
            rad = -rad
        out.append(math.degrees(rad))
    return out


def angles_from_urdf(q) -> dict:
    """URDF 각도 5개(도) → 모터 각도 {"1": 도, …}. 위 함수의 거꾸로"""
    out = {}
    for j, deg in zip(JOINTS, q):
        sid = int(j)
        lim = JOINT_LIMITS.get(sid) or {}
        rad = math.radians(float(deg))
        if lim.get("invert"):
            rad = -rad
        if "rad_min" in lim and "rad_max" in lim and "min" in lim and "max" in lim:
            t = (rad - lim["rad_min"]) / ((lim["rad_max"] - lim["rad_min"]) or 1)
            pos = position_from_fraction(t, lim)
        else:
            pos = int(round(POS_CENTER + rad / (2 * math.pi) * 4095))
        out[j] = round(float(pos_to_deg(pos)), 2)
    return out


def tip_of(angles: dict) -> np.ndarray:
    """모터 각도 → 손끝 위치 (m)"""
    return np.asarray(fk(urdf_from_angles(angles)), float)


# ── 집게가 아래를 보는 IK (camera/camlib.py ik_down 과 같은 식) ────────
def _tip_and_axis(q, radial):
    F = dict(frames(q))
    tip = F["gripper_frame_joint"][:3, 3]
    axis = -F["wrist_roll"][:3, 2]                        # 손목 회전축 = 집게가 향하는 방향
    return np.concatenate([tip, [0.1 * float(axis[:2] @ radial)]]), axis


def lean_of(q) -> float:
    """집게가 받침 바깥(앞)으로 기운 정도 (0 = 수직 아래, 0.26 ≈ 15°). IK 가 이 값을 유지한다"""
    tip = np.asarray(fk(q), float)
    radial = tip[:2] / (np.linalg.norm(tip[:2]) or 1.0)
    return float(_tip_and_axis(q, radial)[0][3] / 0.1)


def ik_down(target, q0, iters: int = 300, tol: float = 0.001, lean: float = 0.0):
    """손끝을 target(x, y, z m)에, 집게 기울기는 lean(0 = 수직 아래) 으로 → URDF 각도 5개(도). 닿지 않으면 None.
    q0(시작 자세)에 가까운 답이 나온다. 손목 회전(5번째)은 q0 그대로"""
    target = np.asarray(target, float)
    radial = target[:2] / (np.linalg.norm(target[:2]) or 1.0)
    goal = np.concatenate([target, [0.1 * lean]])
    q = np.array(q0[:5], float)
    for _ in range(iters):
        f, axis = _tip_and_axis(q, radial)
        err = goal - f
        if np.linalg.norm(err[:3]) < tol and abs(err[3]) < 0.005 and axis[2] < 0:   # 0.005 ≈ 3°
            return [round(float(v), 2) for v in q]
        J = np.zeros((4, 4))
        for j in range(4):
            dq = np.zeros(5)
            dq[j] = np.degrees(1e-4)
            J[:, j] = (_tip_and_axis(q + dq, radial)[0] - f) / 1e-4
        step = J.T @ np.linalg.solve(J @ J.T + 0.02 ** 2 * np.eye(4), err)
        q[:4] += np.degrees(step)
        q = np.clip(q, LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])
    return None


class Reach:
    """ops(ArmOps 또는 시험용 가짜 팔) 로 손끝을 좌표로 움직인다. 집게 기울기는 lean 으로 고정 (학습한 5 자세에서 잼)"""

    def __init__(self, ops, lean: float = 0.0):
        self.ops, self.lean = ops, lean

    def now(self) -> tuple[np.ndarray, list[float]]:
        """(지금 손끝 위치 m, 지금 URDF 각도)"""
        q = urdf_from_angles(self.ops.angles())
        return np.asarray(fk(q), float), q

    def go(self, xyz, seed=None) -> list[float]:
        """손끝을 xyz 로 (집게는 아래). 닿지 않으면 ReachError. 반환: 이동한 URDF 각도"""
        if seed is None:
            seed = self.now()[1]
        q = ik_down(xyz, seed, lean=self.lean)
        if q is None:
            x, y, z = (float(v) for v in xyz)
            raise ReachError(f"팔이 거기까지 닿지 않아요 (받침에서 앞으로 {x * 1000:.0f} mm · 옆으로 {y * 1000:.0f} mm · 높이 {z * 1000:.0f} mm). "
                             "관절(주로 손목)이 더 꺾이지 않는 자리예요")
        self.ops.move_to(angles_from_urdf(q))
        return q
