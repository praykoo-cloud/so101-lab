"""차 추적 테스트(허브 4번) · 대기 자세 저장(허브 1번) 부품 시험 — 가짜 팔 · 가짜 세계만 쓴다 (실물 · 카메라 없음).

    python -m unittest tests.test_mission_tracktest
"""
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from mission.config import MissionConfig
from mission.load_link import FakeLoadLink
from mission.motions import MotionError
from mission.track_test import TrackAbort, TrackTest, idle_status
from mission.watch_page import WatchDesk
from soarm import Robot
from tests.test_mission_loading import FakeArm, FakeEye, World


class HoldArm(FakeArm):
    """FakeArm + hold() (추적이 끝날 때 그 자리에서 멈추는 동작을 기록)"""

    def __init__(self, world, **kw):
        super().__init__(world, **kw)
        self.held = False

    def hold(self):
        self.held = True


def make(start=-10.0, speed=2.0, sign=None, **cfg_kw):
    cfg = MissionConfig(track_period=0.2, track_lost_seconds=1.0, **cfg_kw)
    link = FakeLoadLink()
    world = World(link, start=start, speed=speed)
    arm = HoldArm(world)
    eye = FakeEye(world, arm)
    test = TrackTest(cfg, arm, eye, sign=sign, clock=world.clock, sleep=world.sleep, log=lambda *a, **k: None)
    test.prepare()
    return cfg, link, world, arm, test


def run_ticks(test, world, cfg, n):
    for _ in range(n):
        test.tick()
        world.sleep(cfg.track_period)


class TrackTestCase(unittest.TestCase):
    def test_follows_the_car_without_any_signal_or_pose_move(self):
        cfg, link, world, arm, test = make()
        run_ticks(test, world, cfg, 60)
        st = test.status()
        self.assertGreater(st["found_frames"], 10)
        self.assertTrue(abs(arm.pan - world.car) < 3.0, (arm.pan, world.car))          # 팔이 차를 따라왔다
        self.assertEqual(link.history, [])                                              # 차에게 아무 신호도 안 보냄
        self.assertEqual(arm.went, [])                                                  # 자세 이동도 없음 (받침만)
        self.assertEqual(st["lost_events"], 0)

    def test_pan_never_leaves_the_limit(self):
        cfg, link, world, arm, test = make(start=14.0, speed=0.0, track_pan_limit_deg=5.0)
        run_ticks(test, world, cfg, 30)
        self.assertLessEqual(abs(arm.pan - test.center), 5.0 + 1e-6)
        st = test.status()
        self.assertTrue(st["at_limit"])
        self.assertIn("한계", st["hint"])

    def test_lost_car_returns_to_start(self):
        cfg, link, world, arm, test = make(start=-10.0, speed=6.0)
        run_ticks(test, world, cfg, 150)
        st = test.status()
        self.assertEqual(st["lost_events"], 1)
        self.assertFalse(st["found"])
        self.assertLess(abs(arm.pan - test.center), 1.0)                                # 놓친 뒤 처음 자리로 돌아옴

    def test_wrong_direction_gets_a_hint(self):
        cfg, link, world, arm, test = make(start=5.0, speed=0.0, sign=-1, track_max_step_deg=1.0)
        run_ticks(test, world, cfg, 8)
        self.assertIn("방향", test.status()["hint"])

    def test_right_direction_gets_no_hint(self):
        cfg, link, world, arm, test = make(start=5.0, speed=0.0, sign=1)
        run_ticks(test, world, cfg, 20)
        st = test.status()
        self.assertEqual(st["hint"], "")
        self.assertTrue(st["centered"])
        self.assertLessEqual(abs(st["error_px"]), cfg.track_deadband_px)

    def test_camera_gap_is_reported(self):
        cfg, link, world, arm, test = make()
        test.eye = mock.Mock(look=lambda: None)
        test.tick()
        st = test.status()
        self.assertFalse(st["camera_ok"])
        self.assertIn("영상", st["message"])

    def test_overheat_aborts(self):
        cfg, link, world, arm, test = make()
        arm.hot = True
        with self.assertRaises(TrackAbort):
            test.tick()

    def test_thread_runs_stops_and_holds_arm(self):
        cfg, link, world, arm, test = make(start=2.0, speed=0.0)
        test.sleep = lambda dt: time.sleep(0.002)
        test.clock = time.monotonic
        test.start()
        deadline = time.monotonic() + 5
        while test.status()["frames"] < 5 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(test.status()["state"], "running")
        with self.assertRaises(RuntimeError):
            test.start()                                                                # 이미 추적 중
        test.stop()
        test.join(5)
        self.assertFalse(test.running())
        self.assertEqual(test.status()["state"], "done")
        self.assertTrue(arm.held)                                                       # 그 자리에서 멈춤 (토크는 끄지 않음)

    def test_thread_error_becomes_message_and_holds(self):
        cfg, link, world, arm, test = make()
        arm.hot = True
        test.sleep = lambda dt: time.sleep(0.002)
        test.start()
        test.join(5)
        st = test.status()
        self.assertEqual(st["state"], "error")
        self.assertIn("과열", st["message"])
        self.assertTrue(arm.held)

    def test_go_start_moves_to_wait_pose_first(self):
        cfg = MissionConfig(track_period=0.2)
        world = World(FakeLoadLink(), start=0.0, speed=0.0)
        arm = HoldArm(world, watch_pan=-7.0)
        test = TrackTest(cfg, arm, FakeEye(world, arm), go_start=True, log=lambda *a, **k: None)
        test.prepare()
        self.assertEqual(arm.went, ["watch"])
        self.assertEqual(test.center, -7.0)                                             # 처음 각도 = 대기 자세의 받침 각도

    def test_idle_status_matches_config(self):
        st = idle_status(MissionConfig(track_pan_sign=-1, track_pan_limit_deg=12.0))
        self.assertEqual((st["state"], st["sign"], st["limit"]), ("idle", -1, 12.0))


class WatchDeskTest(unittest.TestCase):
    """대기 자세 저장: 임시 폴더 · 가짜 팔 (실제 data/poses.json 은 건드리지 않는다)"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        poses = Path(self.tmp.name) / "poses.json"
        patcher = mock.patch("soarm.robot.POSES_FILE", poses)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.poses = poses
        self.bot = Robot("fake", speed=2000)
        self.addCleanup(self.bot.close)
        self.desk = WatchDesk(self.bot, MissionConfig(speed_slow=2000), Path(self.tmp.name) / "backup")

    def test_save_then_state_shows_saved_angles_and_backs_up_on_overwrite(self):
        self.desk.set_torque(True)
        self.assertIsNone(self.desk.state()["saved"])
        self.assertIn("저장", self.desk.save())
        saved = self.desk.state()["saved"]
        self.assertIsNotNone(saved)
        self.assertEqual(sorted(saved["angles"]), [1, 2, 3, 4, 5, 6])
        self.assertEqual(list((Path(self.tmp.name) / "backup").glob("*")), [])           # 처음 저장은 백업할 원본이 없다
        self.bot.move(1, self.bot.angles()[1] + 8)
        self.bot.wait_until_settled()
        self.desk.save()
        self.assertEqual(len(list((Path(self.tmp.name) / "backup").glob("poses-*.json"))), 1)   # 덮어쓰기 전에 원본 복사

    def test_go_needs_a_saved_pose_then_goes(self):
        with self.assertRaises(MotionError):
            self.desk.go()
        self.desk.set_torque(True)
        self.desk.save()
        self.bot.move(1, self.bot.angles()[1] + 12)
        self.bot.wait_until_settled()
        self.desk.go()
        self.desk.join(10)
        self.assertFalse(self.desk.busy())
        self.assertIn("도착", self.desk.message)
        self.assertAlmostEqual(self.bot.angles()[1], self.desk.state()["saved"]["angles"][1], delta=1.0)

    def test_buttons_are_refused_while_moving_and_unknown_action(self):
        self.desk.set_torque(True)
        self.desk.save()
        self.desk._thread = threading.Thread(target=time.sleep, args=(0.3,))
        self.desk._thread.start()
        with self.assertRaises(MotionError):
            self.desk.save()
        self.assertEqual(self.desk.state()["now"], {})                                   # 이동 중에는 각도를 읽지 않는다
        self.desk.join(5)
        with self.assertRaises(MotionError):
            self.desk.handle("모르는버튼", {})


if __name__ == "__main__":
    unittest.main()
