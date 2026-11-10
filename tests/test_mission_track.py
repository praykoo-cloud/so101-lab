"""차 추적 시험 — 빈 도로 기준 사진과 비교해 차 찾기, 받침만 돌려 따라가기, 정차하면 wait 자세로.

    python -m unittest tests.test_mission_track
"""
import unittest

import numpy as np

from mission.config import MissionConfig
from mission.errors import SafetyAbort
from mission.vision import find_car
from tests.test_mission_flow import make


def road(value=120):
    return np.full((480, 640, 3), value, np.uint8)


class FindCarTest(unittest.TestCase):
    def setUp(self):
        self.cfg = MissionConfig()
        self.base = road()

    def with_car(self, rect=(200, 200, 120, 80), color=(30, 30, 30)):
        f = self.base.copy()
        x, y, w, h = rect
        f[y:y + h, x:x + w] = color
        return f

    def test_finds_changed_blob_and_its_center(self):
        car = find_car(self.with_car(), self.base, None, self.cfg)
        self.assertIsNotNone(car)
        self.assertAlmostEqual(car.centroid[0], 260, delta=3)
        self.assertAlmostEqual(car.centroid[1], 240, delta=3)
        self.assertGreater(car.area, 8000)

    def test_empty_road_has_no_car(self):
        self.assertIsNone(find_car(self.base.copy(), self.base, None, self.cfg))

    def test_small_blob_and_whole_screen_light_change_are_ignored(self):
        self.assertIsNone(find_car(self.with_car((100, 100, 20, 20)), self.base, None, self.cfg))     # 너무 작음
        self.assertIsNone(find_car(road(170), self.base, None, self.cfg))                               # 조명이 화면 전체에서 바뀜

    def test_only_inside_the_road_zone(self):
        frame = self.with_car((500, 300, 100, 80))
        self.assertIsNone(find_car(frame, self.base, [0, 0, 300, 480], self.cfg))                       # 차는 구역 밖
        self.assertIsNotNone(find_car(frame, self.base, [400, 200, 240, 280], self.cfg))

    def test_no_baseline_or_size_mismatch(self):
        self.assertIsNone(find_car(self.with_car(), None, None, self.cfg))
        self.assertIsNone(find_car(self.with_car(), np.zeros((100, 100, 3), np.uint8), None, self.cfg))


class PanAwareBaselineTest(unittest.TestCase):
    """받침이 돌면 배경이 밀린다 → 지금 각도와 맞는 기준 사진과 비교해야 한다"""

    def setUp(self):
        from mission.vision import ListSource, Vision, ColorRefs
        self.cfg = MissionConfig()
        shifted = road(120)
        shifted[:, ::40] = 60                      # 세로 줄무늬 배경 (받침이 돌면 이 줄무늬가 옆으로 밀린다)
        self.centered = road(120)
        self.centered[:, ::40] = 60
        self.panned = np.roll(self.centered, 17, axis=1)       # 10도 돌렸을 때의 빈 도로 (17픽셀 밀림)
        self.Vision, self.ListSource, self.ColorRefs = Vision, ListSource, ColorRefs

    def vision(self, frame):
        v = self.Vision(self.ListSource([frame] * 5), self.cfg, self.ColorRefs())
        v.baselines = {0: self.centered, 10: self.panned}
        return v

    def car_on(self, base):
        frame = base.copy()
        frame[200:280, 250:350] = 20
        return frame

    def test_uses_the_baseline_that_matches_the_current_pan_angle(self):
        seen = self.vision(self.car_on(self.panned)).car_step(10.0)
        self.assertIsNotNone(seen.car)                                         # 10도 사진과 비교 → 차만 보인다
        self.assertAlmostEqual(seen.car.centroid[0], 300, delta=5)

    def test_comparing_with_the_wrong_angle_would_see_phantom_changes_but_we_do_not(self):
        v = self.vision(self.panned)                                           # 빈 도로, 받침 10도
        self.assertIsNone(v.car_step(10.0).car)                                # 맞는 사진: 아무것도 없음
        wrong = find_car(self.panned, self.centered, None, self.cfg)           # 한 장(0도)으로 비교하면 배경 밀림이 변화로 잡힌다
        self.assertTrue(wrong is None or wrong.area > 0)                       # (참고용: 줄무늬 가장자리 변화)
        self.assertFalse(np.array_equal(self.panned, self.centered))

    def test_no_baseline_close_to_the_angle_means_not_found(self):
        seen = self.vision(self.car_on(self.panned)).car_step(20.0)             # 20도 사진은 없다 (허용 1.5도)
        self.assertIsNone(seen.car)                                            # 틀린 사진으로 비교하지 않고 '못 찾음'

    def test_has_baseline_needs_the_center_photo(self):
        v = self.Vision(self.ListSource([road()]), self.cfg, self.ColorRefs())
        v.baselines = {10: self.panned}
        self.assertFalse(v.has_baseline())
        v.baselines = {0: self.centered}
        self.assertTrue(v.has_baseline())


class CarLookTest(unittest.TestCase):
    """등록한 우리 자동차 사진과 색 분포가 닮은 덩어리만 차로 인정 (선택 기능)"""

    def setUp(self):
        import tempfile
        from pathlib import Path
        from unittest import mock

        import cv2
        from mission import vision
        self.tmp = tempfile.TemporaryDirectory()
        self.cars = Path(self.tmp.name)
        cv2.imwrite(str(self.cars / "front_1.png"), np.full((60, 80, 3), (40, 40, 210), np.uint8))      # 빨간 차를 등록
        patcher = mock.patch.object(vision, "CAR_VIEWS_DIR", self.cars)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self.tmp.cleanup)
        self.vision = vision

    def frame_with(self, color):
        frame = np.full((480, 640, 3), 120, np.uint8)
        frame[200:280, 250:350] = color
        return frame

    def test_score_is_high_for_the_registered_look_and_low_for_something_else(self):
        same = self.vision.car_look_score(self.frame_with((40, 40, 210)), (250, 200, 100, 80))
        other = self.vision.car_look_score(self.frame_with((210, 40, 40)), (250, 200, 100, 80))
        self.assertGreater(same, 0.9)
        self.assertLess(other, 0.3)

    def test_no_registered_photos_means_no_judgement(self):
        empty = self.cars / "empty"
        empty.mkdir()
        self.assertIsNone(self.vision.car_look_score(self.frame_with((40, 40, 210)), (250, 200, 100, 80), empty))

    def test_car_step_filters_by_look_only_when_enabled(self):
        cfg = MissionConfig()
        base = road(120)
        blue_thing = self.frame_with((210, 40, 40))                              # 우리 차(빨강)가 아닌 파란 물체
        for enabled, expect_found in ((False, True), (True, False)):
            cfg.track_use_car_look = enabled
            v = self.vision.Vision(self.vision.ListSource([blue_thing] * 3), cfg, self.vision.ColorRefs())
            v.baselines = {0: base}
            self.assertEqual(v.car_step(0.0).car is not None, expect_found, enabled)


def approach(stop_x):
    """차가 없다가 나타나 왼쪽에서 stop_x 까지 오고, 거기서 멈춘다"""
    return [None] * 3 + [(60 + 40 * i, 240) for i in range(int((stop_x - 60) / 40) + 1)] + [(stop_x, 240)] * 60


class TrackingFlowTest(unittest.TestCase):
    def tracked(self):
        m = make("ok")
        m.cfg.tracking_enabled = True
        m.vision.w.car_script = approach(440)
        return m

    def test_follows_the_car_then_goes_to_wait_pose_and_loads(self):
        m = self.tracked()
        m.run(max_jobs=1)
        hist = m.arm.history
        self.assertEqual(hist[0], "watch")                                   # 시작은 추적 시작 자세
        self.assertLess(hist.index("watch"), hist.index("wait"))             # 추적 뒤에 wait 로
        self.assertTrue(m.arm.nudges)                                        # 받침을 돌려 따라감
        self.assertLess(m.arm.nudges[0], 0)                                  # 차가 화면 왼쪽에서 나타나면 왼쪽(-)으로
        self.assertGreater(m.arm.nudges[-1], 0)                              # 차가 오른쪽에 멈추면 오른쪽(+)으로
        self.assertTrue(all(abs(d) <= m.cfg.track_max_step_deg + 1e-9 for d in m.arm.nudges))   # 한 번에 3도까지만
        self.assertLessEqual(abs(m.arm.pan), m.cfg.track_pan_limit_deg + 1e-9)                  # 좌우 한계 안
        self.assertTrue(any(h["done"] and h["result"] == "ok" for h in m.link.history))         # 이어서 상차 완료

    def test_pan_never_leaves_the_safe_range(self):
        m = self.tracked()
        m.cfg.track_pan_limit_deg = 10.0
        peak = []
        orig = m.arm.nudge_pan

        def spy(delta, center, limit):
            out = orig(delta, center, limit)
            peak.append(abs(out - center))
            return out
        m.arm.nudge_pan = spy
        m.run(max_jobs=1)
        self.assertTrue(peak)
        self.assertLessEqual(max(peak), 10.0 + 1e-9)

    def test_direction_follows_the_setting_and_never_flips_by_itself(self):
        """방향은 설정값(track_pan_sign)을 따른다. 차가 빠르게 지나가도 스스로 뒤집지 않는다"""
        m = make("ok")
        m.cfg.tracking_enabled = True
        m.cfg.track_pan_sign = -1
        m.vision.w.car_script = approach(440)
        m.run(max_jobs=1)
        self.assertGreater(m.arm.nudges[0], 0)                               # 차가 왼쪽인데 sign=-1 이면 +
        self.assertLess(m.arm.nudges[-1], 0)
        signs = [d > 0 for d in m.arm.nudges if d]
        flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        self.assertEqual(flips, 1)                                           # 차가 가운데를 지나갈 때 한 번 바뀔 뿐, 흔들리지 않는다

    def test_car_stopped_somewhere_else_goes_back_to_tracking(self):
        """횡단보도 등에서 멈춘 차: wait 자세에서 정차 구역 확인이 안 되면 다시 watch 로 돌아간다"""
        m = self.tracked()
        m.vision.arrival_step = lambda: False                                # 정차 구역 안에서 포스트잇이 안 보임
        m.max_wait = 150
        with self.assertRaises(TimeoutError):
            m.run(max_jobs=1)
        hist = m.arm.history
        first_wait = hist.index("wait")
        self.assertIn("watch", hist[first_wait + 1:])                        # wait 에 갔다가 다시 watch
        self.assertEqual(hist.count("wait"), 1)                              # 같은 자리 차를 계속 다시 쫓지 않는다 (움직임을 볼 때까지)

    def test_tracking_requires_its_setup_before_start(self):
        m = self.tracked()
        m.cfg.rois["track_road"] = [0, 0, 0, 0]
        with self.assertRaises(SafetyAbort):
            m.run(max_jobs=1)
        self.assertEqual(m.arm.pose, "rest")                                 # 준비가 안 됐으면 팔은 움직이지 않는다

    def test_tracking_off_keeps_the_old_fixed_wait_behavior(self):
        m = make("ok")
        m.run(max_jobs=1)
        self.assertNotIn("watch", m.arm.history)
        self.assertFalse(m.arm.nudges)


if __name__ == "__main__":
    unittest.main()
