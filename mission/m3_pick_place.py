"""집기 · 넣기 동작 (티칭한 자세만 사용). 판정은 하지 않고 "무엇을 했고 그리퍼가 어땠는지" 만 돌려준다.

자세 이름 (m0_teach_poses.py 로 티칭): wait, above_red, red, above_blue, blue, above_basket, basket, retreat_mid

순서
  pick : above_X (그리퍼 열기) → X (내려감) → 그리퍼 닫기 → 닫힌 정도 읽기 → above_X (들기) → 잠깐 유지 → 다시 읽기
  place: above_basket → basket → 그리퍼 열기 → above_basket → retreat_mid (바구니에서 빠짐)

그리퍼 % 는 0(닫힘) ~ 100(열림). 공(3 cm 스펀지)을 쥐면 닫힘 값이 공 크기에서 멈춘다 → 그 범위는
티칭 도구(m0_teach_poses 의 ballgrip)로 cfg.grip_ball_min_pct ~ max_pct 에 저장한다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from .arm_io import ArmIO
from .config import MissionConfig

NEEDED_POSES = ["wait", "above_red", "red", "above_blue", "blue", "above_basket", "basket", "retreat_mid",
                "recovery_wait"]


@dataclass
class PickReport:
    closed_pct: float | None        # 닫고 난 직후 그리퍼 %
    lifted_pct: float | None        # 들어 올려 잠깐 유지한 뒤 %
    in_band_after_close: bool       # 공 크기 범위 안인가
    in_band_after_lift: bool
    suspect: bool                   # 집기 실패 "의심" (다 닫힘, 범위 밖). 확정은 영상 증거가 함께 있어야 한다


def band_ok(cfg: MissionConfig, pct: float | None) -> bool:
    if pct is None or cfg.grip_ball_max_pct <= 0:       # 티칭 전이면 "공을 쥠" 으로 인정하지 않는다
        return False
    return cfg.grip_ball_min_pct <= pct <= cfg.grip_ball_max_pct


def pick(arm: ArmIO, color: str, cfg: MissionConfig) -> PickReport:
    above, low = f"above_{color}", color
    arm.grip(cfg.grip_open_pct)
    arm.go(above)
    arm.go(low)
    arm.grip(0.0)                                   # 닫기 (공이 있으면 공 크기에서 멈춘다)
    closed = arm.grip_pct()
    arm.go(above)                                   # 들기 (그리퍼 값은 그대로)
    time.sleep(0.6)                                 # 잠깐 유지: 미끄러지면 값이 변한다
    lifted = arm.grip_pct()
    ok_close, ok_lift = band_ok(cfg, closed), band_ok(cfg, lifted)
    return PickReport(closed, lifted, ok_close, ok_lift, suspect=not (ok_close and ok_lift))


def place(arm: ArmIO, cfg: MissionConfig) -> None:
    arm.go("above_basket")
    arm.go("basket")
    arm.grip(cfg.grip_open_pct)                     # 놓기
    time.sleep(0.4)
    arm.go("above_basket")                          # 위로 후퇴
    arm.go("retreat_mid")                           # 바구니에서 완전히 빠진 중간 자세


def go_wait(arm: ArmIO) -> bool:
    return arm.go("wait")
