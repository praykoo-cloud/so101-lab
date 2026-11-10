"""동작 티칭 저장소 · 재생 시험 — 가짜 팔과 임시 폴더만 쓴다 (실물 팔 · 실제 data/motions.json 은 건드리지 않는다).

    python -m unittest tests.test_mission_motions
"""
import json
import tempfile
import threading
import unittest
from pathlib import Path

from mission.motions import DEFAULT_MOTIONS, MotionBook, MotionError, capture_point, check_point, play
from soarm import Robot


def point(base=0.0, grip=50.0):
    return {"angles": {j: base + j for j in (1, 2, 3, 4, 5)}, "grip": grip, "pause": 0.0}


class BookTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "motions.json"
        self.book = MotionBook(self.path, Path(self.tmp.name) / "backup")

    def test_default_slots_red_and_blue_are_separate(self):
        self.assertEqual(list(self.book.motions), list(DEFAULT_MOTIONS))
        self.assertFalse(self.book.motion("빨간공_집기")["relative"])
        self.assertFalse(self.book.motion("파란공_집기")["relative"])
        self.assertTrue(self.book.motion("바구니_넣기")["relative"])

    def test_takes_accumulate_and_survive_reload(self):
        a = self.book.add_take("빨간공_집기", [point(1), point(2)])
        b = self.book.add_take("빨간공_집기", [point(3)], label="천천히")
        self.assertEqual((a, b), ("t1", "t2"))
        again = MotionBook(self.path)
        self.assertEqual(list(again.motion("빨간공_집기")["takes"]), ["t1", "t2"])
        self.assertEqual(again.take("빨간공_집기", "t2")["label"], "천천히")
        self.assertEqual(again.chosen("빨간공_집기")[0], "t1")            # 첫 회차가 기본 선택
        self.assertEqual(again.motion("파란공_집기")["takes"], {})       # 파랑은 따로

    def test_create_rename_delete_motion(self):
        self.book.create("차_쳐다보기")
        with self.assertRaises(MotionError):
            self.book.create("차_쳐다보기")
        self.book.add_take("차_쳐다보기", [point()])
        self.book.rename("차_쳐다보기", "차 보기")
        self.assertIn("차 보기", self.book.motions)
        self.assertNotIn("차_쳐다보기", self.book.motions)
        saved = self.book.delete_motion("차 보기")
        self.assertTrue(saved.exists())                                   # 지우기 전 백업
        self.assertIn("차 보기", json.loads(saved.read_text(encoding="utf-8"))["motions"])
        self.assertNotIn("차 보기", MotionBook(self.path).motions)

    def test_bad_names_rejected(self):
        for bad in ("", "   ", "a/b", "x" * 31, "<b>"):
            with self.assertRaises(MotionError):
                self.book.create(bad)

    def test_take_ids_never_reused_after_delete(self):
        self.book.add_take("파란공_집기", [point()])
        self.book.add_take("파란공_집기", [point()])
        self.book.delete_take("파란공_집기", "t2")
        self.assertEqual(self.book.add_take("파란공_집기", [point()]), "t3")

    def test_choose_and_delete_chosen_falls_back(self):
        self.book.add_take("빨간공_집기", [point()])
        self.book.add_take("빨간공_집기", [point()])
        self.book.choose("빨간공_집기", "t2")
        self.assertEqual(self.book.chosen("빨간공_집기")[0], "t2")
        self.book.delete_take("빨간공_집기", "t2")
        self.assertEqual(self.book.chosen("빨간공_집기")[0], "t1")
        self.book.delete_take("빨간공_집기", "t1")
        with self.assertRaises(MotionError):
            self.book.chosen("빨간공_집기")

    def test_recommend_prefers_marked_ok_then_small_error(self):
        name = "빨간공_집기"
        for _ in range(3):
            self.book.add_take(name, [point()])
        self.assertIsNone(self.book.recommend(name))                      # 재생한 적이 없으면 추천 없음
        self.book.add_run(name, "t1", {"reached": True, "max_err": 0.2})
        self.book.mark_run(name, "t1", False)
        self.book.add_run(name, "t2", {"reached": True, "max_err": 1.5})
        self.book.mark_run(name, "t2", True)
        self.book.add_run(name, "t3", {"reached": True, "max_err": 0.4})
        self.book.mark_run(name, "t3", True)
        self.assertEqual(self.book.recommend(name), "t3")                 # 둘 다 잘 됨 → 오차가 작은 쪽
        stats = self.book.stats(name, "t1")
        self.assertEqual((stats["plays"], stats["ok"], stats["bad"]), (1, 0, 1))

    def test_stopped_runs_do_not_count(self):
        name = "빨간공_집기"
        self.book.add_take(name, [point()])
        self.book.add_run(name, "t1", {"reached": False, "stopped": True, "max_err": 9})
        self.assertEqual(self.book.stats(name, "t1")["plays"], 0)

    def test_mark_without_run_is_error(self):
        self.book.add_take("빨간공_집기", [point()])
        with self.assertRaises(MotionError):
            self.book.mark_run("빨간공_집기", "t1", True)

    def test_empty_or_broken_points_rejected(self):
        with self.assertRaises(MotionError):
            self.book.add_take("빨간공_집기", [])
        with self.assertRaises(MotionError):
            self.book.add_take("빨간공_집기", [{"angles": {1: 0}}])
        with self.assertRaises(MotionError):
            check_point({"grip": 3})

    def test_car_ref(self):
        self.assertIsNone(self.book.car_ref_pan())
        self.book.set_car_ref(12.345)
        self.assertEqual(MotionBook(self.path).car_ref_pan(), 12.35)


class PlayTest(unittest.TestCase):
    """가짜 팔(soarm.fake)로 실제 재생 함수를 돌린다"""

    def setUp(self):
        self.bot = Robot("fake", speed=2000)
        self.addCleanup(self.bot.close)
        self.bot.torque(True)

    def here(self, **delta):
        p = capture_point(self.bot)
        for j, d in delta.items():
            p["angles"][j[1:]] += d
        p["pause"] = 0.0
        return p

    def test_capture_has_five_joints_and_grip(self):
        p = capture_point(self.bot)
        self.assertEqual(sorted(p["angles"]), ["1", "2", "3", "4", "5"])
        self.assertIsNotNone(p["grip"])

    def test_play_reaches_points_in_order(self):
        seen = []
        pts = [self.here(j1=8), self.here(j1=-8, j2=5)]
        rep = play(self.bot, pts, speed=2000, on_step=lambda i, n: seen.append((i, n)), sleep=lambda s: None)
        self.assertEqual(seen, [(0, 2), (1, 2)])
        self.assertTrue(rep["reached"], rep)
        self.assertEqual(rep["done"], 2)
        self.assertAlmostEqual(self.bot.angles()[1], pts[1]["angles"]["1"], delta=1.0)

    def test_pan_offset_only_turns_the_base(self):
        pts = [self.here()]
        rep = play(self.bot, pts, speed=2000, pan_offset=6.0, sleep=lambda s: None)
        self.assertTrue(rep["reached"], rep)
        now = self.bot.angles()
        self.assertAlmostEqual(now[1], pts[0]["angles"]["1"] + 6.0, delta=1.0)
        self.assertAlmostEqual(now[2], pts[0]["angles"]["2"], delta=1.0)

    def test_stop_event_stops_before_moving(self):
        stop = threading.Event()
        stop.set()
        before = self.bot.angles()[1]
        rep = play(self.bot, [self.here(j1=10)], speed=2000, stop=stop, sleep=lambda s: None)
        self.assertTrue(rep["stopped"])
        self.assertFalse(rep["reached"])
        self.assertAlmostEqual(self.bot.angles()[1], before, delta=0.5)

    def test_too_hot_stops(self):
        rep = play(self.bot, [self.here(j1=10)], speed=2000, too_hot=lambda: True, sleep=lambda s: None)
        self.assertEqual(rep["why"], "모터 과열")
        self.assertEqual(rep["done"], 0)

    def test_torque_off_refuses_to_move(self):
        from soarm import RobotError
        self.bot.torque(False)
        with self.assertRaises(RobotError):
            play(self.bot, [self.here(j1=5)], speed=2000, sleep=lambda s: None)


if __name__ == "__main__":
    unittest.main()
