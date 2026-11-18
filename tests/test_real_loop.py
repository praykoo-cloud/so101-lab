"""허브 4번 '실전 반복' (경연용): 한 사이클이 끝나면 차가 떠나기를 기다렸다가 다시 2 부터 (가짜 팔 · 가짜 카메라)"""
import time
import unittest

from mission.place_learn import PlaceError
from tests import test_mission_place as base
from tests.test_mission_place import steps


def until(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


class RealLoopTest(unittest.TestCase):
    setUp = base.SessionTest.setUp

    def prepare(self):
        self.cfg.t_left = 0.05                       # 차가 이만큼 안 보이면 떠남 (시험은 짧게)
        self.cfg.load_leave_timeout = 5.0
        self.cfg.load_retry_wait_seconds = 0.01
        self.book.add_take(self.sid, steps(pan=10.0, cx=320.0), {}, success=True)
        self.book.choose_set(self.sid)
        original = self.ops.grip

        def grip(pct):                               # 집게를 열면 공이 바구니에 들어간 것으로
            original(pct)
            if self.ball_goes_in:
                self.eye.loaded = pct > 50
        self.ops.grip = grip
        self.ball_goes_in = True

    def car_leaves(self):
        self.eye.car = False
        self.eye.loaded = False

    def test_two_cars_in_a_row_then_stop(self):
        self.prepare()
        msg = self.s.start("real", self.sid, self.link)
        self.assertIn("실전 반복", msg)
        self.assertTrue(until(lambda: self.link.states().count("SUCCESS") == 1), self.s.status)
        self.car_leaves()
        self.assertTrue(until(lambda: self.s.status["cycle"] == 2), self.s.status)   # 떠난 걸 보고 다음 차를 기다림
        self.assertTrue(self.s.busy())
        self.eye.car = True                                                          # 다음 차가 옴
        self.assertTrue(until(lambda: self.link.states().count("SUCCESS") == 2), self.s.status)
        self.car_leaves()
        self.assertTrue(until(lambda: self.s.status["cycle"] == 3), self.s.status)
        self.s.handle("stop", {})
        self.s._thread.join(5)
        self.assertFalse(self.s.busy())
        self.assertEqual(self.s.status["tally"], {"ok": 2, "fail": 0})
        states = self.link.states()
        self.assertEqual(states[:4], ["WAITING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS"])
        self.assertEqual(states[4:8], ["WAITING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS"])   # 두 번째 차도 처음부터
        self.assertEqual(self.ops.picked, ["red", "red"])
        self.assertEqual(self.s.state()["mode"], "real")

    def test_does_not_take_the_same_car_twice(self):
        self.prepare()
        self.s.start("real", self.sid, self.link)
        self.assertTrue(until(lambda: self.link.states().count("SUCCESS") == 1), self.s.status)
        time.sleep(0.3)                                                              # 차가 아직 서 있다
        self.assertEqual(self.link.states().count("STOP_REQUEST"), 1)                # 같은 차에 다시 정지 신호를 내지 않음
        self.assertEqual(self.s.status["cycle"], 1)

    def test_failure_retries_after_a_timed_wait_without_the_button(self):
        self.prepare()
        self.ball_goes_in = False                                                    # 계속 넣기 실패
        seen_ball_wait = []
        self.s.start("real", self.sid, self.link)
        self.assertTrue(until(lambda: (seen_ball_wait.append(self.s.status["waiting"]) or
                                       "FAILED" in self.link.states())), self.s.status)
        self.assertNotIn("ball", seen_ball_wait)                                     # [공 놓았어요] 를 기다리지 않음
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2, 3])
        self.assertTrue(until(lambda: self.s.status["tally"]["fail"] == 1), self.s.status)
        self.assertTrue(self.s.busy())                                               # 실패해도 다음 차를 기다린다
        self.s.handle("stop", {})
        self.s._thread.join(5)

    def test_real_ignores_start_and_color_and_needs_learned_takes(self):
        with self.assertRaises(PlaceError):
            self.s.start("real", self.sid, self.link)                                # ✔ 회차가 없음
        self.prepare()
        self.s.start("real", self.sid, self.link, 9, "blue", True)                   # 고른 단계 · 색 · 멈추기는 무시
        self.assertTrue(until(lambda: self.ops.picked), self.s.status)
        self.assertEqual(self.ops.picked[0], "red")                                  # 포스트잇(빨강)을 읽음
        self.assertEqual(self.link.states()[0], "WAITING")                           # 2 부터
        self.s.handle("stop", {})
        self.s._thread.join(5)

    def test_test_mode_still_waits_for_the_ball_button(self):
        self.prepare()
        self.ball_goes_in = False
        self.s.start("test", self.sid, self.link, 2, "auto", False)
        self.wait_for("ball")                                                        # 테스트는 예전처럼 버튼
        self.s.handle("stop", {})
        self.s._thread.join(5)

    wait_for = base.SessionTest.wait_for


class WaitCarGoneTest(unittest.TestCase):
    """PlaceRunner.wait_car_gone: 서 있던 차가 DRIVING 을 알리거나, 차 · 바구니가 t_left 초 동안 안 보이면 떠남"""

    def make(self, car_state, car_visible=True):
        from mission.config import MissionConfig
        from mission.load_link import FakeLoadLink
        from mission.place_flow import PlaceRunner
        from tests.test_mission_place import Clock, FakeEye, FakeOps
        self.clock = Clock()
        ops = FakeOps()
        self.eye = FakeEye(ops, car=car_visible)
        self.link = FakeLoadLink()
        self.link.car_state = car_state
        cfg = MissionConfig(t_left=1.0, load_leave_timeout=10.0)
        return PlaceRunner(cfg, ops, self.eye, self.link, None, clock=self.clock, sleep=self.clock.sleep,
                           log=lambda *a, **k: None)

    def test_stopped_car_reports_driving(self):
        r = self.make("LOADING")
        real_sleep = r.sleep

        def sleep(dt):
            real_sleep(dt)
            if self.clock.t > 2.0:
                self.link.car_state = "DRIVING"
        r.sleep = sleep
        self.assertTrue(r.wait_car_gone())
        self.assertLess(self.clock.t, 3.0)

    def test_driving_from_the_start_is_not_trusted_but_vision_is(self):
        r = self.make("DRIVING")                                                    # 서지 않고 지나가는 중이던 차
        real_sleep = r.sleep

        def sleep(dt):
            real_sleep(dt)
            if self.clock.t > 4.0:
                self.eye.car = False
        r.sleep = sleep
        self.assertTrue(r.wait_car_gone())
        self.assertGreater(self.clock.t, 4.9)                                       # 안 보이고 t_left 뒤

    def test_timeout_returns_false(self):
        r = self.make(None)
        self.assertFalse(r.wait_car_gone())                                         # 계속 보임 → 시간 초과
        self.assertGreaterEqual(self.clock.t, 10.0)

    def test_stop_button_while_waiting(self):
        from mission.place_flow import FlowStopped
        r = self.make(None)
        r.stop_event.set()
        with self.assertRaises(FlowStopped):
            r.wait_car_gone()


if __name__ == "__main__":
    unittest.main()
