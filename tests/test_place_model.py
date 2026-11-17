"""허브 4번 '정차 사진 → 5 바구니 위 자세' 2차식 시험 — 가짜 팔 · 가짜 카메라만 쓴다.

    python -m unittest tests.test_place_model
"""
import tempfile
import unittest
from pathlib import Path

from mission.config import MissionConfig
from mission.load_link import FakeLoadLink
from mission.motions import MotionBook
from mission.place_flow import PlaceRunner
from mission.place_learn import PlaceBook, make_snap
from mission.place_model import fit, predict, samples_from_takes
from tests.test_mission_place import Clock, FakeEye, FakeOps


def truth(cx, cy):
    """가짜 '정답': 상자 위치에 따라 5 자세가 정해진다 (휘어진 관계 포함)"""
    x, y = (cx - 320) / 320, (cy - 240) / 240
    return {"1": -10 + 15 * x, "2": -15 - 20 * y + 6 * y * y, "3": 10 + 25 * y - 3 * x * y, "4": 60 + 8 * x * x,
            "5": -93.7}


GRID = [(cx, cy) for cx in (120, 220, 320, 420, 520) for cy in (100, 200, 300, 380)]


def take_steps(cx, cy):
    box = {"name": "box", "conf": 0.9, "cx": cx, "cy": cy, "w": 250.0, "h": 220.0}
    return {"stop": make_snap({"1": -7, "2": -77, "3": 30, "4": 85, "5": -93.7}, 48.0, box, None),
            "above": make_snap(truth(cx, cy), 48.0, None, None),
            "check": make_snap({"1": -2, "2": -70, "3": 5, "4": 100, "5": -93.7}, 48.0, None, None)}


class FitTest(unittest.TestCase):
    def setUp(self):
        self.model = fit([({"cx": cx, "cy": cy}, truth(cx, cy)) for cx, cy in GRID])

    def test_learns_the_curve_and_predicts_a_new_stop(self):
        self.assertEqual(self.model["n"], len(GRID))
        pose, why = predict(self.model, {"cx": 370, "cy": 250})
        want = truth(370, 250)
        for j in "1234":
            self.assertAlmostEqual(pose[j], want[j], delta=0.2, msg=j)
        self.assertEqual(pose["5"], -93.7)                                       # 5 손목 돌리기 = 평균

    def test_outside_the_learned_range_is_not_guessed(self):
        pose, why = predict(self.model, {"cx": 20, "cy": 250}, margin_px=40)
        self.assertIsNone(pose)
        self.assertIn("범위", why)
        self.assertIsNotNone(predict(self.model, {"cx": 90, "cy": 250}, margin_px=40)[0])   # 여유 안은 괜찮다

    def test_too_few_takes_or_no_stop_photo_means_no_model(self):
        self.assertIsNone(fit([({"cx": cx, "cy": cy}, truth(cx, cy)) for cx, cy in GRID[:5]]))
        self.assertIsNone(predict(None, {"cx": 320, "cy": 240})[0])
        self.assertIsNone(predict(self.model, None)[0])

    def test_all_stops_in_one_place_cannot_be_solved(self):
        self.assertIsNone(fit([({"cx": 300, "cy": 200}, truth(300, 200))] * 12))

    def test_prediction_is_kept_inside_the_taught_angles(self):
        m = dict(self.model, angle={j: [0.0, 1.0] for j in "1234"})
        pose, _ = predict(m, {"cx": 320, "cy": 240}, margin_deg=5.0)
        for j in "1234":
            self.assertTrue(-5.0 <= pose[j] <= 6.0, (j, pose[j]))

    def test_only_used_takes_with_both_photos_count(self):
        takes = [{"steps": take_steps(120, 100)}, {"steps": {"above": take_steps(1, 1)["above"]}}]
        self.assertEqual(len(samples_from_takes(takes)), 1)


class RunnerModelTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = PlaceBook(root / "place.json", root / "img", root / "backup")
        sid = self.book.create_set("a")
        for cx, cy in GRID:
            self.book.add_take(sid, take_steps(cx, cy), {}, success=True)
        self.book.add_take(sid, take_steps(900, 900), {}, success=False)       # ✘ 실패 회차는 안 쓴다
        self.learned = self.book.learned(sid)
        self.cfg = MissionConfig(track_period=0.1)
        self.ops = FakeOps()
        self.clock = Clock()
        self.runner = PlaceRunner(self.cfg, self.ops, FakeEye(self.ops), FakeLoadLink(), MotionBook(root / "m.json", root / "b"),
                                  clock=self.clock, sleep=self.clock.sleep, log=lambda *a, **k: None)
        self.runner.max_wait = 30

    def test_learned_has_the_model(self):
        self.assertEqual(self.learned["model"]["n"], len(GRID))

    def test_full_stop_remembers_the_box(self):
        self.assertTrue(self.runner.wait_car_and_stop())
        box = self.runner.stop_box
        self.assertIsNotNone(box)
        self.assertEqual(set(box), {"cx", "cy", "w", "h"})

    def test_step_5_goes_to_the_computed_pose_and_7_returns_there(self):
        self.runner.stop_box = {"cx": 370.0, "cy": 250.0, "w": 250.0, "h": 220.0}
        corr, why = self.runner.step_above(self.learned)
        self.assertIsNotNone(corr, why)
        self.assertIn("계산", why)
        want = truth(370, 250)
        for j in "1234":
            self.assertAlmostEqual(self.ops.a[j], want[j], delta=0.2)
        self.ops.go_wait()
        self.runner.back_above(self.learned, corr)                              # 7 바구니 위 = 5 에서 간 자세
        for j in "1234":
            self.assertAlmostEqual(self.ops.a[j], want[j], delta=0.2)

    def test_without_a_stop_photo_it_uses_the_mean_pose(self):
        self.runner.stop_box = None
        corr, why = self.runner.step_above(self.learned)
        self.assertIn("평균 자세", why)
        self.assertAlmostEqual(self.ops.a["4"], self.learned["poses"]["above"]["4"], delta=0.01)

    def test_model_can_be_switched_off(self):
        self.cfg.place_model = False
        self.runner.stop_box = {"cx": 370.0, "cy": 250.0, "w": 250.0, "h": 220.0}
        self.runner.step_above(self.learned)
        self.assertAlmostEqual(self.ops.a["2"], self.learned["poses"]["above"]["2"], delta=0.01)


if __name__ == "__main__":
    unittest.main()
