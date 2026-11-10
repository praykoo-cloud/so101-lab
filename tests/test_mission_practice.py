"""집기 연습(허브 15번) 시험 — 가짜 팔 · 가짜 관측으로 흐름만 본다 (실물 팔 · 카메라 · 실제 기록 파일은 쓰지 않는다).

    python -m unittest tests.test_mission_practice
"""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from mission.config import MissionConfig
from mission.practice import Options, Practice, append_log, read_log, suggest_grip, summarize
from mission.vision import ColorVote


class FakeArm:
    def __init__(self):
        self.history, self.grips = [], []
        self.torque = None
        self.hot = False
        self.grip_value = 100.0
        self.closed_value = 30.0              # 공을 쥐면 멈추는 값
        self.missing = []
        self.fail_pose = None

    def preflight(self, needed):
        return [f"자세 '{n}' 이 티칭되지 않았습니다" for n in needed if n in self.missing]

    def torque_on(self):
        self.torque = True

    def torque_off(self):
        self.torque = False

    def too_hot(self):
        return self.hot

    def go(self, pose, **kw):
        self.history.append(pose)
        return pose != self.fail_pose

    def grip(self, pct, wait=True):
        self.grips.append(pct)
        self.grip_value = self.closed_value if pct == 0.0 else pct

    def grip_pct(self):
        return self.grip_value


class FakeRefs:
    refs = {"note_red": {}, "note_blue": {}, "ball_red": {}, "ball_blue": {}}


class FakeVision:
    def __init__(self):
        self.votes = []                       # read_color 가 차례로 돌려줄 값 (None = 포스트잇 안 보임)
        self.outcome = "success"
        self.at_origin = True
        self.in_gripper = True

    def read_color(self):
        v = self.votes.pop(0) if self.votes else "red"
        return None if v is None else ColorVote(v, {"red": 5, "blue": 0}, {"red": 0.1, "blue": 0.6})

    def check_success(self, color):
        return self.outcome

    def ball_at_origin(self, color):
        return self.at_origin

    def ball_in_gripper(self, color):
        return self.in_gripper


class PracticeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = Path(self.tmp.name) / "log.jsonl"
        self.cfg = MissionConfig()
        self.cfg.rois["stand_red"] = [0, 0, 10, 10]
        self.cfg.rois["stand_blue"] = [0, 0, 10, 10]
        self.cfg.rois["gripper"] = [0, 0, 10, 10]
        self.cfg.rois["note"] = [0, 0, 10, 10]
        self.cfg.grip_ball_min_pct, self.cfg.grip_ball_max_pct = 20.0, 40.0
        self.arm, self.vision = FakeArm(), FakeVision()
        self.p = Practice(self.arm, self.vision, self.cfg, FakeRefs(), self.log, sleep=lambda s: None)

    def run_until_done(self, opts, helper=None, timeout=5):
        self.p.start(opts)
        end = time.time() + timeout
        while self.p.running() and time.time() < end:
            if helper:
                helper()
            time.sleep(0.01)
        self.assertFalse(self.p.running(), "연습이 끝나지 않았어요")

    def test_full_cycles_alternate_colors_and_finish_safely(self):
        self.run_until_done(Options(color_mode="alternate", stop_after=5, max_cycles=2), helper=self.p.resume)
        recs = read_log(self.log)
        self.assertEqual([r["color"] for r in recs], ["red", "blue"])
        self.assertTrue(all(r["steps_done"] == 5 and r["outcome"] == "success" for r in recs))
        self.assertEqual(self.arm.history[0], "wait")
        self.assertEqual(self.arm.history[-1], "wait")                      # 마지막은 대기 자세
        self.assertIs(self.arm.torque, False)                               # 대기 자세 다음 토크 끄기
        self.assertIn("above_basket", self.arm.history)

    def test_stop_after_lift_puts_ball_back_and_keeps_going_without_helper(self):
        """3단계까지만: 공을 받침대로 되돌려 놓고, 공이 보이면 사람 도움 없이 다음 시도"""
        self.run_until_done(Options(color_mode="red", stop_after=3, max_cycles=3))
        recs = read_log(self.log)
        self.assertEqual(len(recs), 3)
        self.assertTrue(all(r["outcome"] == "partial" and r["steps_done"] == 3 for r in recs))
        self.assertNotIn("above_basket", self.arm.history)                  # 바구니로는 안 감
        self.assertEqual(self.arm.grips.count(100.0) >= 6, True)            # 매번 열기 + 되돌려 놓기
        self.assertTrue(all(r["good_hold"] for r in recs))                  # 들어올렸을 때 공이 그리퍼에 보임

    def test_waits_for_person_when_ball_is_not_seen_again(self):
        self.vision.at_origin = False
        self.p.start(Options(color_mode="red", stop_after=2, max_cycles=2))
        deadline = time.time() + 3
        while self.p.state != "waiting" and time.time() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.p.state, "waiting")
        self.assertIn("공 올려놨어요", self.p.message)
        self.assertEqual(self.p.cycle, 1)
        self.vision.at_origin = True
        self.p.resume()
        self.p.join(3)
        self.assertEqual(len(read_log(self.log)), 2)

    def test_basket_mode_reads_postit_color_and_retries_when_not_visible(self):
        self.vision.votes = [None, "blue"]                                  # 처음엔 안 보임 → 사람이 계속 → 파랑
        self.p.start(Options(color_mode="basket", stop_after=3, max_cycles=1, expected_color="blue"))
        deadline = time.time() + 3
        while self.p.state != "waiting" and time.time() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.p.state, "waiting")
        self.assertIn("포스트잇이 안 보여요", self.p.message)
        self.p.resume()
        self.p.join(3)
        rec = read_log(self.log)[0]
        self.assertEqual((rec["color"], rec["decided_by"], rec["expected"]), ("blue", "post-it", "blue"))
        self.assertIn("above_blue", self.arm.history)                       # 읽은 색의 공을 집으러 감
        self.assertEqual(summarize([rec])["reading"], {"total": 1, "correct": 1, "wrong": 0})

    def test_confirm_each_step_waits_before_every_step(self):
        self.p.start(Options(color_mode="red", stop_after=2, confirm_each_step=True, max_cycles=1))
        seen = []
        end = time.time() + 5
        while self.p.running() and time.time() < end:
            if self.p.state == "waiting":
                seen.append(self.p.step)
                self.p.resume()
            time.sleep(0.01)
        self.assertEqual(seen, [1, 2])

    def test_stop_button_ends_a_wait_and_closes_the_arm_safely(self):
        self.p.start(Options(color_mode="red", stop_after=5))               # 무한 반복 + 5단계 → 공 올려놓기를 기다림
        deadline = time.time() + 3
        while self.p.state != "waiting" and time.time() < deadline:
            time.sleep(0.01)
        self.p.stop()
        self.p.join(3)
        self.assertFalse(self.p.running())
        self.assertIs(self.arm.torque, False)
        self.assertEqual(self.arm.history[-1], "wait")
        self.assertEqual(self.p.state, "idle")

    def test_overheat_and_unreachable_pose_halt_with_reason(self):
        self.arm.fail_pose = "above_red"
        self.run_until_done(Options(color_mode="red", stop_after=1, max_cycles=1))
        self.assertIn("above_red", self.p.error)
        self.assertEqual(read_log(self.log), [])                            # 못 간 시도는 기록하지 않는다
        self.assertIs(self.arm.torque, False)

    def test_start_is_refused_with_reasons_when_not_ready(self):
        self.arm.missing = ["above_blue"]
        with self.assertRaises(ValueError) as ctx:
            self.p.start(Options(color_mode="alternate"))
        self.assertIn("above_blue", str(ctx.exception))
        self.assertFalse(self.p.running())
        self.cfg.rois["note"] = [0, 0, 0, 0]
        self.arm.missing = []
        with self.assertRaises(ValueError) as ctx:
            self.p.start(Options(color_mode="basket"))
        self.assertIn("포스트잇 구역", str(ctx.exception))

    def test_options_are_validated(self):
        for bad in ({"color_mode": "green"}, {"stop_after": 0}, {"stop_after": 6}, {"max_cycles": 0}, {"expected_color": "pink"}):
            with self.assertRaises(ValueError):
                Options.from_dict(bad)
        self.assertIsNone(Options.from_dict({"max_cycles": ""}).max_cycles)    # 비우면 무한

    def test_suggestion_needs_enough_good_samples_and_warns_on_overlap(self):
        rec = lambda v, good=True, bad=False: {"closed_pct": v, "lifted_pct": v, "good_hold": good, "bad_hold": bad}
        few = suggest_grip([rec(30.0)] * 2)
        self.assertFalse(few["ok"])
        recs = [rec(v) for v in (28, 30, 31, 32)] + [rec(33)]
        s = suggest_grip(recs)
        self.assertTrue(s["ok"])
        self.assertEqual((s["min"], s["max"]), (26.0, 35.0))                # 28~33 ± 2
        self.assertEqual(s["overlap"], 0)
        s2 = suggest_grip(recs + [rec(31, good=False, bad=True)])
        self.assertGreater(s2["overlap"], 0)
        self.assertIn("⚠", s2["message"])

    def test_log_is_append_only_and_bad_lines_are_skipped(self):
        append_log({"n": 1}, self.log)
        self.log.write_text(self.log.read_text(encoding="utf-8") + "깨진 줄\n", encoding="utf-8")
        append_log({"n": 2}, self.log)
        self.assertEqual([r["n"] for r in read_log(self.log)], [1, 2])


if __name__ == "__main__":
    unittest.main()
