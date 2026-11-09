"""mission.vision 시험 — 합성(그림) 화면으로: 티칭 → 색 강제 선택 → 포스트잇 찾기 → 도착 확인.

    python -m unittest tests.test_mission_vision
"""
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from mission.config import MissionConfig
from mission.m0_teach_colors import teach_from_frames
from mission.vision import (ColorRefs, ListSource, Note, Vision, classify_forced, find_note, is_locked, lock_colors,
                            vote_color)

RED = (30, 30, 220)        # BGR
BLUE = (220, 60, 30)
VIOLET = (200, 40, 160)    # 빨강과 파랑 사이 (애매한 색)
GRAY = (120, 120, 120)


def frame_with(color=None, rect=(250, 180, 80, 80), bg=GRAY, noise=0):
    f = np.full((480, 640, 3), bg, np.uint8)
    if color is not None:
        x, y, w, h = rect
        f[y:y + h, x:x + w] = color
    if noise:
        rng = np.random.default_rng(1)
        f = np.clip(f.astype(int) + rng.integers(-noise, noise + 1, f.shape), 0, 255).astype(np.uint8)
    return f


def taught_refs() -> ColorRefs:
    refs = ColorRefs()
    rect = (250, 180, 80, 80)
    for name, color in (("note_red", RED), ("note_blue", BLUE), ("ball_red", RED), ("ball_blue", BLUE)):
        frames = [frame_with(color, noise=6) for _ in range(15)]
        teach_from_frames(refs, name, frames, rect)
    return refs


class ColorTest(unittest.TestCase):
    def setUp(self):
        self.cfg = MissionConfig()
        self.cfg.rois["note"] = [200, 130, 200, 180]
        self.cfg.rois["arrival"] = [200, 130, 200, 180]
        self.refs = taught_refs()

    def test_teach_needs_enough_samples(self):
        with self.assertRaises(ValueError):
            ColorRefs().add_samples("note_red", [(0, 200, 200)])

    def test_forced_choice_red_and_blue(self):
        for color, want in ((RED, "red"), (BLUE, "blue")):
            nt = find_note(frame_with(color, noise=8), self.cfg.roi("note"), self.cfg, self.refs)
            self.assertIsNotNone(nt)
            self.assertEqual(classify_forced(nt, self.refs)[0], want)

    def test_forced_choice_never_returns_unreadable(self):
        """애매한 색도 항상 둘 중 하나를 고른다 ('판정 불가' 상태 없음)"""
        nt = Note((0, 0, 10, 10), (5, 5), 100, hsv=(150.0, 120.0, 150.0))      # 보라 계열 (빨강/파랑 어디도 아님)
        self.assertIn(classify_forced(nt, self.refs)[0], ("red", "blue"))
        nt2 = Note((0, 0, 10, 10), (5, 5), 100, hsv=(0.0, 0.0, 255.0))         # 흰색 (과노출)
        self.assertIn(classify_forced(nt2, self.refs)[0], ("red", "blue"))

    def test_hue_wraps_around(self):
        """빨강은 색상환 양끝(0 근처와 175 근처)이다"""
        nt = Note((0, 0, 10, 10), (5, 5), 100, hsv=(176.0, 230.0, 220.0))
        self.assertEqual(classify_forced(nt, self.refs)[0], "red")

    def test_only_note_region_decides(self):
        """파란 포스트잇 옆에 더 큰 빨간 물체가 있어도 포스트잇(크기·모양 맞는 덩어리 중 note ROI 안)만 본다"""
        f = frame_with(BLUE, rect=(260, 200, 40, 40))
        f[130:330, 340:395] = RED                    # note ROI 안이지만 모양이 길쭉(0.28)해서 포스트잇이 아님
        nt = find_note(f, self.cfg.roi("note"), self.cfg, self.refs)
        self.assertIsNotNone(nt)
        self.assertEqual(classify_forced(nt, self.refs)[0], "blue")

    def test_gray_floor_has_no_note(self):
        self.assertIsNone(find_note(frame_with(None, noise=5), self.cfg.roi("note"), self.cfg, self.refs))

    def test_tiny_blob_not_a_note(self):
        self.assertIsNone(find_note(frame_with(RED, rect=(250, 180, 8, 8)), self.cfg.roi("note"), self.cfg, self.refs))

    def test_vote_tie_uses_distance_then_default(self):
        red_note = Note((0, 0, 1, 1), (0, 0), 100, (0.0, 230.0, 220.0))
        blue_note = Note((0, 0, 1, 1), (0, 0), 100, (120.0, 230.0, 220.0))
        v = vote_color([red_note, blue_note], self.refs)
        self.assertIn(v.color, ("red", "blue"))
        self.assertFalse(v.used_default)            # 거리가 달라서 동률이 아님
        self.assertIsNone(vote_color([], self.refs))


class LockTest(unittest.TestCase):
    def test_lock_blocks_edit_and_detects_tamper(self):
        with tempfile.TemporaryDirectory() as d:
            path, lock = Path(d) / "colors.json", Path(d) / "colors.lock"
            refs = taught_refs()
            refs.save(path)
            self.assertFalse(is_locked(path, lock))
            with self.assertRaises(PermissionError):
                ColorRefs.load(path, require_lock=True)          # 잠그기 전에는 대회 모드로 못 읽는다
            lock_colors(path, lock)
            self.assertTrue(is_locked(path, lock))
            self.assertEqual(ColorRefs.load(path, require_lock=True).complete(), True)
            with self.assertRaises(PermissionError):
                refs.save(path)                                  # 잠긴 뒤 수정 금지
            path.write_text(path.read_text(encoding="utf-8") + " ", encoding="utf-8")   # 몰래 바꿈
            self.assertFalse(is_locked(path, lock))

    def test_incomplete_refs_cannot_lock(self):
        with tempfile.TemporaryDirectory() as d:
            path, lock = Path(d) / "colors.json", Path(d) / "colors.lock"
            refs = ColorRefs()
            teach_from_frames(refs, "note_red", [frame_with(RED, noise=5)] * 5, (250, 180, 80, 80))
            refs.save(path)
            with self.assertRaises(ValueError):
                lock_colors(path, lock)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class ArrivalTest(unittest.TestCase):
    def setUp(self):
        self.cfg = MissionConfig()
        self.cfg.rois["note"] = [200, 130, 200, 180]
        self.cfg.rois["arrival"] = [200, 130, 200, 180]
        self.refs = taught_refs()

    def _vision(self, frames, clock):
        return Vision(ListSource(frames), self.cfg, self.refs, clock=clock.now, sleep=clock.sleep)

    def test_arrives_only_after_standing_still(self):
        clk = FakeClock()
        v = self._vision([frame_with(BLUE)] * 100, clk)
        got = []
        for _ in range(40):
            got.append(v.arrival_step())
            clk.sleep(0.1)
        self.assertFalse(got[0])
        self.assertTrue(got[-1])                  # 1 초 이상 멈춰 있으면 도착
        self.assertEqual(got.index(True), 11)

    def test_moving_car_restarts_the_timer(self):
        clk = FakeClock()
        frames = [frame_with(BLUE, rect=(230 + 12 * i, 180, 80, 80)) for i in range(12)]     # 계속 움직임
        v = self._vision(frames, clk)
        for _ in range(12):
            self.assertFalse(v.arrival_step())
            clk.sleep(0.1)

    def test_no_note_no_arrival(self):
        clk = FakeClock()
        v = self._vision([frame_with(None)] * 30, clk)
        for _ in range(30):
            self.assertFalse(v.arrival_step())
            clk.sleep(0.1)

    def test_read_color_votes(self):
        clk = FakeClock()
        v = self._vision([frame_with(RED, noise=5)] * 60, clk)
        vote = v.read_color()
        self.assertEqual(vote.color, "red")
        self.assertEqual(vote.frames_used, self.cfg.vote_frames)


if __name__ == "__main__":
    unittest.main()
