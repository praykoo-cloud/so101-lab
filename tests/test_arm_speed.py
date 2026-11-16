"""팔 최고 속도 · 동작 사이 멈칫 없애기 설정 시험 (2026-11-16) — 가짜 팔(soarm.fake)만 쓴다.

    python -m unittest tests.test_arm_speed
"""
import unittest

from mission.arm_io import (ARM_MAX_SPEED, DEFAULT_ACCEL, NO_PAUSE_STILL_S, NO_PAUSE_TOL_DEG, apply_accel,
                            play_options, run_speed, settle)
from mission.config import MissionConfig
from mission.motions import capture_point, play
from soarm import Robot


class SpyRobot:
    """기다리기 · 가속도 쓰기를 기록만 하는 팔"""

    def __init__(self):
        self.waits, self.accels = [], []
        self.driver = self

    def wait_until_settled(self, **kw):
        self.waits.append(kw)
        return True

    def set_acceleration(self, sid, acc):
        self.accels.append((sid, acc))


class SettingsTest(unittest.TestCase):
    def test_defaults(self):
        cfg = MissionConfig()
        self.assertEqual(cfg.load_stop_confirm_seconds, 0.5)                # 정차 확인 0.5초
        self.assertTrue(cfg.arm_fast)
        self.assertTrue(cfg.arm_no_pause)

    def test_fast_uses_the_servo_top_speed_and_off_uses_speed_slow(self):
        self.assertEqual(run_speed(MissionConfig(speed_slow=300)), ARM_MAX_SPEED)
        self.assertEqual(run_speed(MissionConfig(speed_slow=300, arm_fast=False)), 300)

    def test_acceleration_is_written_once_per_change(self):
        bot = SpyRobot()
        cfg = MissionConfig(arm_accel=254)
        apply_accel(bot, cfg)
        apply_accel(bot, cfg)                                                  # 같은 값이면 다시 안 쓴다 (버스 아끼기)
        self.assertEqual({a for _, a in bot.accels}, {254})
        self.assertEqual(len(bot.accels), 6)
        cfg.arm_fast = False
        apply_accel(bot, cfg)                                                  # 끄면 예전 가속도로
        self.assertEqual(bot.accels[-1][1], DEFAULT_ACCEL)

    def test_fake_arm_without_driver_is_skipped(self):
        apply_accel(object(), MissionConfig())

    def test_no_pause_waits_only_until_close_enough(self):
        bot = SpyRobot()
        settle(bot, MissionConfig(), 5.0)
        settle(bot, MissionConfig(arm_no_pause=False), 5.0)
        self.assertEqual(bot.waits[0], {"timeout": 5.0, "tolerance_deg": NO_PAUSE_TOL_DEG, "still_seconds": NO_PAUSE_STILL_S})
        self.assertEqual(bot.waits[1], {"timeout": 5.0})                     # 끄면 예전처럼 (0.5° · 0.5초)


class PlayNoPauseTest(unittest.TestCase):
    def setUp(self):
        self.bot = Robot("fake", speed=2000)
        self.addCleanup(self.bot.close)
        self.bot.torque(True)

    def points(self):
        a = capture_point(self.bot)
        b = capture_point(self.bot)
        b["angles"]["1"] += 8
        b["grip"] = (b["grip"] or 0) + 20                                      # 집게도 움직이는 지점
        return [a, b]

    def test_no_pause_skips_the_rests_between_points(self):
        sleeps = []
        rep = play(self.bot, self.points(), sleep=sleeps.append, **play_options(MissionConfig()))
        self.assertTrue(rep["reached"], rep)
        self.assertEqual(sleeps, [])                                           # 지점마다 쉬기 · 집게 뒤 0.3초 없음

    def test_off_keeps_the_old_rests(self):
        sleeps = []
        rep = play(self.bot, self.points(), sleep=sleeps.append, **play_options(MissionConfig(arm_no_pause=False)))
        self.assertTrue(rep["reached"], rep)
        self.assertIn(0.3, sleeps)
        self.assertGreaterEqual(len(sleeps), 3)                                # 지점 2개 쉬기 + 집게 뒤 0.3초


if __name__ == "__main__":
    unittest.main()
