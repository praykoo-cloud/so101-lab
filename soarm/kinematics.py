"""SO-101 기구학: URDF(model/so101.urdf) 에서 읽은 관절 사슬로 정기구학 · 역기구학.

    from soarm.kinematics import fk, ik_numeric
    fk([0, 0, 0, 0, 0])            # 관절 각도(도, URDF 기준) → 손끝 위치 (m)
    ik_numeric([0.25, 0.0, 0.10])  # 손끝 위치 → 관절 각도. 닿지 않으면 None

각도는 URDF 기준 (Robot.joint_radians() 와 같은 0점). 손끝은 gripper_frame_link (집게 사이).
numpy 만 쓴다.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = Path(__file__).resolve().parent.parent / "model" / "so101.urdf"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
TIP = "gripper_frame_link"


def _rot(r, p, y):
    """URDF rpy → 회전 행렬 (Rz · Ry · Rx)"""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rot(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis, a):
    """축 axis 둘레로 a 라디안 (로드리게스)"""
    k = np.asarray(axis, float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    return T


def load_chain(path=URDF):
    """base_link 에서 손끝까지 관절 목록: (이름, 고정 변환, 축, 하한, 상한). 고정 관절은 축 None"""
    root = ET.parse(path).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain, link = [], TIP
    while link in by_child:
        j = by_child[link]
        o = j.find("origin")
        T = _tf([float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()])
        lim = j.find("limit")
        axis = [float(v) for v in j.find("axis").get("xyz").split()] if j.get("type") == "revolute" else None
        lo, hi = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else (0.0, 0.0)
        chain.append((j.get("name"), T, axis, lo, hi))
        link = j.find("parent").get("link")
    return chain[::-1]


CHAIN = load_chain()
LIMITS_DEG = np.degrees([[lo, hi] for name, _, axis, lo, hi in CHAIN if axis is not None])


def frames(q_deg):
    """관절마다 base_link 기준 4×4 변환 (마지막이 손끝)"""
    q = iter(np.radians(q_deg))
    T, out = np.eye(4), []
    for name, T0, axis, *_ in CHAIN:
        T = T @ T0
        if axis is not None:
            T = T @ _axis_rot(axis, next(q))
        out.append((name, T.copy()))
    return out


def fk(q_deg):
    """정기구학: 관절 각도 5개(도) → 손끝 위치 (m)"""
    return frames(q_deg)[-1][1][:3, 3]


def jacobian(q_deg, h=1e-6):
    """손끝 위치를 각 관절로 미분한 3×5 행렬 (수치 미분, m/rad)"""
    q = np.asarray(q_deg, float)
    p0 = fk(q)
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += np.degrees(h)
        J[:, i] = (fk(dq) - p0) / h
    return J


def ik_numeric(target, q0=None, iters=200, tol=1e-4, damping=0.01, use=(0, 1, 2, 3), strict=True):
    """수치 역기구학 (감쇠 최소제곱). 손끝 위치만 맞춘다. 닿지 않으면 None

    use: 움직일 관절 번호. 기본은 손목 회전(4)을 빼고 4개
    strict=False 면 닿지 않아도 마지막 자세를 돌려준다
    반환: (관절 각도 5개(도), 반복 횟수, 남은 오차 m)
    """
    target = np.asarray(target, float)
    q = np.zeros(5) if q0 is None else np.asarray(q0, float).copy()
    use = list(use)
    for k in range(iters):
        err = target - fk(q)
        if np.linalg.norm(err) < tol:
            return q, k, float(np.linalg.norm(err))
        J = jacobian(q)[:, use]
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(3), err)
        q[use] += np.degrees(dq)
        q = np.clip(q, LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])
    err = float(np.linalg.norm(target - fk(q)))
    return (q, iters, err) if err < tol or not strict else None
