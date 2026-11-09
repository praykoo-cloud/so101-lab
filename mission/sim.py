"""가상 세계: 실물 팔 · 카메라 · ROS2 없이 미션 전체 흐름을 돌려 보는 연습용 (시험도 이걸 쓴다).

    python -m mission.mission_main --sim fail_then_ok

가상 세계는 공이 어디 있는지(origin 받침대 / gripper 그리퍼 / basket 바구니 안 / outside 바깥)를 들고 있고,
카메라 대신 "지금 팔 자세에서 무엇이 보이는지" 를 그 값으로 흉내 낸다.
시간은 가짜 시계라서 120 초짜리 대기도 바로 끝난다.
"""
from __future__ import annotations

from .config import MissionConfig
from .link import FakeLink
from .m3_pick_place import PickReport
from .mission_main import Mission
from .vision import ColorVote

# 시나리오: 시도마다 {"pick": ok|miss, "place": ok|drop|hide|stuck}
SCENARIOS = {
    "ok": "한 번에 성공",
    "blue": "파란 주문, 한 번에 성공",
    "fail_then_ok": "1차 집기 실패(공이 그대로) → 2차 성공",
    "drop_then_restore": "1차에 공이 바구니 밖으로 떨어짐 → 팀원이 원위치 복구 → 2차 성공",
    "stuck_then_ok": "1차에 공이 그리퍼에 붙어 안 놓임 → 다시 넣기(2차) 성공",
    "fail3": "3번 모두 집기 실패 → 실패 통보 (공 없이 출발)",
    "hidden_then_seen": "공은 들어갔는데 처음엔 안 보임 → 재관측으로 성공 확인",
    "hidden_forever": "공이 바구니에 들어갔는데 끝까지 안 보임 → 불명 = 실패 → 복구 대기 → 시간 초과 안전 중단",
    "restore_timeout": "공이 떨어졌는데 아무도 복구하지 않음 → 안전 중단",
}


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += max(0.0, dt)


class SimWorld:
    def __init__(self, order: str, script: list[dict], restore_after: float | None, hide_checks: int):
        self.order = order
        self.script = list(script)
        self.restore_after = restore_after
        self.ball = "origin"
        self.hidden = False
        self.hide_checks = hide_checks          # 이 횟수만큼 관측해도 안 보이다가 그 뒤 보임 (-1 = 끝까지 안 보임)
        self.recovery_since: float | None = None
        self.arrival_calls = 0
        self.left_calls = 0


class SimArm:
    def __init__(self, world: SimWorld, clock: SimClock):
        self.world, self.clock = world, clock
        self.pose = "rest"
        self.grip_value = 100.0
        self.torque = False

    def preflight(self, needed):
        return []

    def torque_on(self):
        self.torque = True

    def torque_off(self):
        self.torque = False

    def hold(self):
        pass

    def close(self):
        pass

    def go(self, pose, speed=None, wait=True, timeout=12.0):
        self.pose = pose
        self.clock.sleep(1.0)
        if pose == "recovery_wait" and self.world.recovery_since is None:
            self.world.recovery_since = self.clock.now()
        return True

    def grip(self, pct, wait=True):
        self.grip_value = pct

    def grip_pct(self):
        return self.grip_value

    def at_pose(self, pose, tol_deg=None):
        return self.pose == pose

    def nearest_pose(self, candidates, tol_deg=None):
        return self.pose if self.pose in candidates else None

    def too_hot(self):
        return False

    def warm(self):
        return False

    def max_temperature(self):
        return 35.0


class SimMotion:
    """m3_pick_place 대신: 시나리오 대본대로 공을 옮긴다"""

    def __init__(self, world: SimWorld):
        self.w = world
        self.cur: dict | None = None

    def _next(self) -> dict:
        return self.w.script.pop(0) if self.w.script else {"pick": "ok", "place": "ok"}

    def pick(self, arm, color, cfg):
        self.cur = self._next()
        arm.go(f"above_{color}")
        arm.go(color)
        arm.go(f"above_{color}")
        if self.cur["pick"] == "ok":
            self.w.ball = "gripper"
            arm.grip_value = 30.0
            return PickReport(30.0, 30.0, True, True, False)
        arm.grip_value = 2.0                    # 공이 없어서 끝까지 닫힘 (의심)
        return PickReport(2.0, 2.0, False, False, True)

    def place(self, arm, cfg):
        if self.cur is None:                    # 다시 넣기(replace): 새 시도의 대본
            self.cur = self._next()
        behavior = self.cur["place"]
        arm.go("above_basket")
        arm.go("basket")
        if behavior == "ok":
            self.w.ball = "basket"
        elif behavior == "hide":
            self.w.ball, self.w.hidden = "basket", True
        elif behavior == "drop":
            self.w.ball = "outside"
        elif behavior == "stuck":
            self.w.ball = "gripper"
        arm.grip_value = 100.0 if behavior != "stuck" else 30.0
        arm.go("above_basket")
        arm.go("retreat_mid")
        self.cur = None


class SimVision:
    def __init__(self, world: SimWorld, arm: SimArm, clock: SimClock, link: FakeLink):
        self.w, self.arm, self.clock, self.link = world, arm, clock, link

    def reset_arrival(self):
        self.w.arrival_calls = 0

    def arrival_step(self):
        self.w.arrival_calls += 1
        return self.w.arrival_calls >= 3

    def read_color(self):
        return ColorVote(self.w.order, {"red": 15, "blue": 0} if self.w.order == "red" else {"red": 0, "blue": 15},
                         {"red": 0.05, "blue": 0.5}, False, 15)

    def _tick_restore(self):
        w = self.w
        if (w.restore_after is not None and w.ball == "outside" and w.recovery_since is not None
                and self.clock.now() - w.recovery_since >= w.restore_after):
            w.ball = "origin"                   # 팀원이 공을 받침대 원위치에 놓음

    def ball_at_origin(self, color):
        self._tick_restore()
        return self.w.ball == "origin" and self.arm.pose in (f"above_{color}", "recovery_wait")

    def ball_in_gripper(self, color):
        return self.w.ball == "gripper"

    def ball_outside(self, color):
        return self.w.ball == "outside" and self.arm.pose == "retreat_mid"

    def check_success(self, color):
        w = self.w
        if self.ball_in_gripper(color) or self.ball_outside(color):
            return "fail"
        if self.arm.pose == "retreat_mid" and w.ball == "basket":
            if w.hidden and w.hide_checks != 0:
                if w.hide_checks > 0:
                    w.hide_checks -= 1
                return "unknown"
            return "success"
        return "unknown"

    def area_quiet(self, seconds, threshold=4.0):
        self.clock.sleep(seconds)
        return True

    def car_left_step(self):
        if self.link.state().get("clear"):
            self.w.left_calls += 1
        return self.w.left_calls >= 3


def build_sim(name: str, store) -> Mission:
    ok = {"pick": "ok", "place": "ok"}
    script, restore, hide = [ok], None, 0
    order = "red"
    if name == "blue":
        order = "blue"
    elif name == "fail_then_ok":
        script = [{"pick": "miss", "place": "ok"}, ok]
    elif name == "drop_then_restore":
        script, restore = [{"pick": "ok", "place": "drop"}, ok], 20.0
    elif name == "stuck_then_ok":
        script = [{"pick": "ok", "place": "stuck"}, {"pick": "ok", "place": "ok"}]
    elif name == "fail3":
        script = [{"pick": "miss", "place": "ok"}] * 3
    elif name == "hidden_then_seen":
        script, hide = [{"pick": "ok", "place": "hide"}], 2
    elif name == "hidden_forever":
        script, hide = [{"pick": "ok", "place": "hide"}], -1
    elif name == "restore_timeout":
        script, restore = [{"pick": "ok", "place": "drop"}], None
    elif name != "ok":
        raise SystemExit(f"모르는 시나리오 '{name}'. 목록: --sim --list")
    clock = SimClock()
    world = SimWorld(order, script, restore, hide)
    cfg = MissionConfig()
    for roi in cfg.rois:
        cfg.rois[roi] = [10, 10, 100, 100]
    cfg.grip_ball_min_pct, cfg.grip_ball_max_pct = 15.0, 60.0
    arm = SimArm(world, clock)
    link = FakeLink()
    vision = SimVision(world, arm, clock, link)
    return Mission(cfg, store, arm, vision, link, SimMotion(world), clock=clock.now, sleep=clock.sleep)
