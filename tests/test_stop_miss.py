"""완전 정차인데 상자를 못 찾으면 그 화면 저장 (2026-11-17) — 가짜 팔 · 가짜 카메라만 쓴다.

    python -m unittest tests.test_stop_miss
"""
import tempfile
import unittest
from pathlib import Path

from mission.config import MissionConfig
from mission.load_link import FakeLoadLink
from mission.motions import MotionBook
from mission.place_flow import PlaceRunner, Seen, YoloEye
from mission.yolo import Det
from tests.test_mission_place import Clock, FakeOps


class StopEye:
    """차 + 상자가 보이다가, 정지 신호 뒤에는 hide_box 면 상자 없이 차만 보인다. 저장 요청은 기록만"""

    def __init__(self, link, hide_box=True, fail=False):
        self.link, self.hide_box, self.fail = link, hide_box, fail
        self.saved = []

    def look(self):
        dets = [Det("car", 0.9, 200, 180, 180, 160)]
        if not (self.hide_box and self.link.state == "STOP_REQUEST"):
            dets.append(Det("box", 0.9, 250, 210, 80, 60))
        return Seen(dets, 640)

    def save_last(self, folder, stem):
        if self.fail:
            raise OSError("디스크가 꽉 찼어요")
        self.saved.append((Path(folder), stem))
        return [Path(folder) / f"{stem}.jpg", Path(folder) / f"{stem}_yolo.jpg"]


class StopMissTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.cfg = MissionConfig(track_period=0.1)
        self.link = FakeLoadLink()
        self.events = []

    def runner(self, eye):
        clock = Clock()
        r = PlaceRunner(self.cfg, FakeOps(), eye, self.link, MotionBook(self.root / "m.json", self.root / "b"),
                        clock=clock, sleep=clock.sleep, log=lambda ev, **kw: self.events.append((ev, kw)))
        r.max_wait = 30
        r.miss_dir = self.root / "stop_miss"
        return r

    def test_stopped_without_a_box_saves_the_screen(self):
        eye = StopEye(self.link)
        r = self.runner(eye)
        self.assertTrue(r.wait_car_and_stop())                                   # 차로 정차 확인은 된다
        self.assertIsNone(r.stop_box)
        self.assertEqual(len(eye.saved), 1)
        self.assertEqual(eye.saved[0][0], self.root / "stop_miss")
        self.assertEqual(len(r.miss_saved), 2)                                   # 원본 + YOLO 그림
        saved = [kw for ev, kw in self.events if ev == "stop_miss_saved"]
        self.assertEqual(saved[0]["seen"], ["car"])

    def test_box_seen_at_the_stop_saves_nothing(self):
        eye = StopEye(self.link, hide_box=False)
        r = self.runner(eye)
        self.assertTrue(r.wait_car_and_stop())
        self.assertIsNotNone(r.stop_box)
        self.assertEqual(eye.saved, [])
        self.assertEqual(r.miss_saved, [])

    def test_switched_off_saves_nothing(self):
        self.cfg.save_stop_miss = False
        eye = StopEye(self.link)
        self.assertTrue(self.runner(eye).wait_car_and_stop())
        self.assertEqual(eye.saved, [])

    def test_a_failed_save_does_not_stop_the_loading(self):
        eye = StopEye(self.link, fail=True)
        self.assertTrue(self.runner(eye).wait_car_and_stop())
        self.assertIn("stop_miss_save_failed", [ev for ev, _ in self.events])

    def test_eye_without_save_is_skipped(self):
        eye = StopEye(self.link)
        eye.save_last = None
        self.assertTrue(self.runner(eye).wait_car_and_stop())


class ShotEye(StopEye):
    """정차 화면을 떼어 두고(snapshot) 저장하는(save_shot) 것까지 흉내 낸다"""

    def __init__(self, link, **kw):
        super().__init__(link, **kw)
        self.shots = []

    def snapshot(self):
        return ("정차 화면", ["car", "box"])

    def save_shot(self, shot, folder, stem):
        self.shots.append((shot, Path(folder), stem))
        Path(folder).mkdir(parents=True, exist_ok=True)
        return [Path(folder) / f"{stem}.jpg", Path(folder) / f"{stem}_yolo.jpg"]


class PlaceFailTest(StopMissTest):
    """상자를 찾았고 공을 옮겼는데 넣기에 실패 → 그 차의 정차 화면 저장"""

    def stopped(self, hide_box=False):
        eye = ShotEye(self.link, hide_box=hide_box)
        r = self.runner(eye)
        r.fail_dir = self.root / "place_fail"
        self.assertTrue(r.wait_car_and_stop())
        return eye, r

    def test_failed_place_saves_the_stop_screen_with_its_info(self):
        import json
        eye, r = self.stopped()
        files = r.save_place_fail(1, "9 공이 든 상자가 안 보여요", "red")
        self.assertEqual(len(eye.shots), 1)
        self.assertEqual(eye.shots[0][0], ("정차 화면", ["car", "box"]))        # 실패 때 화면이 아니라 정차 순간 화면
        self.assertEqual(len(files), 3)                                          # 원본 · YOLO 그림 · 정보
        info = json.loads(Path(files[-1]).read_text(encoding="utf-8"))
        self.assertEqual(info["stop_box"], r.stop_box)
        self.assertEqual((info["attempt"], info["color"]), (1, "red"))
        self.assertIn("9", info["why"])
        self.assertIn("place_fail_saved", [ev for ev, _ in self.events])

    def test_only_once_per_car(self):
        eye, r = self.stopped()
        r.save_place_fail(1, "a")
        self.assertEqual(r.save_place_fail(2, "b"), [])                           # 같은 정차 화면이라 한 번만
        self.assertEqual(len(eye.shots), 1)
        self.assertTrue(r.wait_car_and_stop())                                    # 다음 차는 다시 저장
        r.save_place_fail(1, "c")
        self.assertEqual(len(eye.shots), 2)

    def test_box_not_found_at_the_stop_is_not_a_place_fail(self):
        eye, r = self.stopped(hide_box=True)                                      # 그 경우는 stop_miss 로 저장된다
        self.assertEqual(r.save_place_fail(1, "x"), [])
        self.assertEqual(eye.shots, [])

    def test_switched_off_saves_nothing(self):
        self.cfg.save_place_fail = False
        eye, r = self.stopped()
        self.assertIsNone(r.stop_shot)
        self.assertEqual(r.save_place_fail(1, "x"), [])

    def test_a_failed_save_does_not_stop_the_loading(self):
        eye, r = self.stopped()
        eye.save_shot = lambda *a: (_ for _ in ()).throw(OSError("디스크가 꽉 찼어요"))
        self.assertEqual(r.save_place_fail(1, "x"), [])
        self.assertIn("place_fail_save_failed", [ev for ev, _ in self.events])


class LoadingPlaceFailTest(unittest.TestCase):
    """상차 미션(Loader): 공을 집고 넣기에 실패한 회차에서 저장, 성공하면 저장 안 함"""

    from tests.test_mission_loading import NewFlowTest as _flow
    setUp = _flow.setUp
    build = _flow.build

    def run_one(self, miss):
        from unittest import mock
        loader = self.build(note="red")
        saved = []
        self.eye.snapshot = lambda: "정차 화면"
        self.eye.save_shot = lambda shot, folder, stem: saved.append((shot, Path(folder))) or []
        self.ops.miss = miss
        with mock.patch("mission.place_flow.PLACE_FAIL_DIR", Path(self.tmp.name) / "place_fail"):
            loader.run(max_jobs=1)
        return saved

    def test_place_miss_saves_the_stop_screen_once(self):
        saved = self.run_one([True, True])                                         # 2번 빗나가고 3번째 성공
        self.assertEqual(saved, [("정차 화면", Path(self.tmp.name) / "place_fail")])
        self.assertEqual(self.link.states()[-2], "SUCCESS")

    def test_success_saves_nothing(self):
        self.assertEqual(self.run_one([]), [])


def _cv2_ok():
    try:
        import cv2
        import numpy  # noqa: F401
        return hasattr(cv2, "imwrite")
    except Exception:
        return False


@unittest.skipUnless(_cv2_ok(), "OpenCV 가 없는 PC (Pi 컨테이너에서 확인)")
class YoloEyeSaveTest(unittest.TestCase):
    def test_writes_the_raw_and_the_drawn_photo(self):
        import numpy as np

        class Det0:
            def detect(self, frame):
                return [Det("car", 0.9, 10, 10, 50, 40)]

        eye = YoloEye(lambda: np.full((480, 640, 3), 120, np.uint8), Det0())
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(eye.save_last(d, "x"), [])                       # 아직 사진 없음
            eye.look()
            files = eye.save_last(Path(d) / "stop_miss", "x")
            self.assertEqual([f.name for f in files], ["x.jpg", "x_yolo.jpg"])
            self.assertTrue(all(f.stat().st_size > 0 for f in files))

    def test_snapshot_is_kept_after_new_photos(self):
        import numpy as np
        frames = iter([np.full((480, 640, 3), 10, np.uint8), np.full((480, 640, 3), 200, np.uint8)])

        class Det0:
            def detect(self, frame):
                return []

        eye = YoloEye(lambda: next(frames), Det0())
        eye.look()
        shot = eye.snapshot()                                                  # 정차 순간
        eye.look()                                                             # 그 뒤 사진이 바뀌어도
        self.assertEqual(int(shot[0][0, 0, 0]), 10)
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(len(eye.save_shot(shot, d, "s")), 2)


if __name__ == "__main__":
    unittest.main()
