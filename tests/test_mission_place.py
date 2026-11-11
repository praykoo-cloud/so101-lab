"""허브 4번 '바구니 넣기 학습' 저장소 · 단계 실행 시험 — 가짜 팔 · 가짜 카메라만 쓴다 (실물 · ROS2 없음).

    python -m unittest tests.test_mission_place
"""
import tempfile
import threading
import unittest
from pathlib import Path

from mission.config import MissionConfig
from mission.load_link import FakeLoadLink
from mission.motions import MotionBook
from mission.place_flow import FlowStopped, PlaceRunner, Seen
from mission.place_learn import PlaceBook, PlaceError, make_snap
from mission.yolo import Det

WIDTH = 640
PX_PER_DEG = 20.0


def snap(pan, basket_cx=None, grip=30.0, j2=0.0):
    basket = None if basket_cx is None else {"name": "box", "conf": 0.9, "cx": basket_cx, "cy": 240.0, "w": 80.0, "h": 60.0}
    return make_snap({"1": pan, "2": j2, "3": 0, "4": 0, "5": 0}, grip, basket, None)


def steps(pan=10.0, cx=320.0):
    return {"above": snap(pan, cx), "inside": snap(pan, None, j2=-20.0), "release": snap(pan, None, grip=70.0, j2=-20.0),
            "check": snap(pan, None, j2=5.0)}


class BookTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = PlaceBook(root / "place.json", root / "img", root / "backup")

    def test_sets_takes_names_images_and_reload(self):
        sid = self.book.create_set("기본_세트")
        self.assertEqual(self.book.chosen_set(), sid)                    # 첫 세트 = 실행용
        self.book.set_watch(sid, snap(0.0, 300.0), b"jpg-watch")
        tid = self.book.add_take(sid, steps(), {"above": b"a", "inside": b"b", "release": b"c", "check": b"d"},
                                 success=True, color="red")
        self.book.rename_take(sid, tid, "왼쪽_가까이")
        again = PlaceBook(self.book.path, self.book.image_dir, self.book.backup_dir)     # 다시 불러오기
        take = again.take(sid, tid)
        self.assertEqual(take["label"], "왼쪽_가까이")
        self.assertTrue(take["use"])
        self.assertEqual(again.image_path(sid, take["steps"]["above"]["image"]).read_bytes(), b"a")
        self.assertEqual(again.image_path(sid, "watch.jpg").read_bytes(), b"jpg-watch")
        self.assertIsNone(again.image_path(sid, "../place.json"))                   # 폴더 밖은 못 읽음

    def test_take_needs_all_four_steps(self):
        sid = self.book.create_set("a")
        partial = steps()
        del partial["check"]
        with self.assertRaises(PlaceError):
            self.book.add_take(sid, partial, {}, success=True)
        self.assertEqual(self.book.get_set(sid)["takes"], {})

    def test_failed_take_is_kept_but_not_used_and_learned_is_the_mean(self):
        sid = self.book.create_set("a")
        self.book.add_take(sid, steps(pan=10.0, cx=300.0), {}, success=True)
        self.book.add_take(sid, steps(pan=20.0, cx=340.0), {}, success=True)
        bad = self.book.add_take(sid, steps(pan=90.0, cx=0.0), {}, success=False)
        self.assertFalse(self.book.take(sid, bad)["use"])
        learned = self.book.learned(sid)
        self.assertEqual(learned["count"], 2)
        self.assertEqual(learned["poses"]["above"]["1"], 15.0)
        self.assertEqual(learned["target"]["cx"], 320.0)
        self.assertEqual(learned["release_grip"], 70.0)
        self.book.set_use(sid, bad, True)
        self.assertEqual(self.book.learned(sid)["count"], 3)

    def test_delete_keeps_backup_and_moves_chosen(self):
        a = self.book.create_set("a")
        b = self.book.create_set("b")
        tid = self.book.add_take(a, steps(), {"above": b"x"}, success=True)
        self.assertIsNotNone(self.book.delete_take(a, tid))
        self.assertTrue(any(self.book.backup_dir.iterdir()))
        self.book.delete_set(a)
        self.assertEqual(self.book.chosen_set(), b)
        with self.assertRaises(PlaceError):
            self.book.learned(b)                                                  # ✔ 회차가 없으면 이유를 말한다

    def test_names_are_checked(self):
        with self.assertRaises(PlaceError):
            self.book.create_set("a/b")
        self.book.create_set("같은이름")
        with self.assertRaises(PlaceError):
            self.book.create_set("같은이름")


class FakeOps:
    """가짜 팔: 관절 각도만 기억한다. 바구니는 받침 각도에 따라 화면에서 옆으로 움직인다"""

    def __init__(self):
        self.a = {"1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0}
        self.gripper = 30.0
        self.hot = False
        self.picked = []
        self.moves = 0

    def too_hot(self):
        return self.hot

    def go_wait(self):
        self.a = {"1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0}
        return True

    def angles(self):
        return dict(self.a)

    def grip_pct(self):
        return self.gripper

    def move_to(self, angles, timeout=12.0):
        self.a.update({j: float(v) for j, v in angles.items()})
        self.moves += 1
        return True

    def nudge(self, joint, deg):
        self.a[joint] = deg

    def grip(self, pct):
        self.gripper = pct

    def torque_arm(self, on):
        pass

    def hold(self):
        pass

    def play_pick(self, color, book, stop):
        self.picked.append(color)
        self.gripper = 30.0
        return {"reached": True, "stopped": False, "why": ""}


class FakeEye:
    """차가 basket_pan 방향에 서 있다. 받침이 거기를 보면 바구니가 화면 가운데(320)"""

    def __init__(self, ops, basket_pan=12.0, car=True):
        self.ops, self.basket_pan, self.car = ops, basket_pan, car
        self.loaded = False

    def look(self):
        dets = []
        if self.car:
            cx = WIDTH / 2 - (self.ops.a["1"] - self.basket_pan) * PX_PER_DEG
            dets.append(Det("car", 0.9, cx - 90, 180, 180, 160))
            name = "success_red" if self.loaded else "box"
            dets.append(Det(name, 0.9, cx - 40, 210, 80, 60))
            dets.append(Det("postit_red", 0.8, cx - 10, 220, 20, 20))
        return Seen(dets, WIDTH)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.01)


class RunnerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = PlaceBook(root / "place.json", root / "img", root / "backup")
        sid = self.book.create_set("a")
        self.book.add_take(sid, steps(pan=10.0, cx=320.0), {}, success=True)
        self.learned = self.book.learned(sid)
        self.cfg = MissionConfig(grip_ball_min_pct=20.0, grip_ball_max_pct=40.0, track_period=0.1,
                                 place_servo_gain_deg_per_px=0.04)
        self.ops = FakeOps()
        self.eye = FakeEye(self.ops)
        self.link = FakeLoadLink()
        self.clock = Clock()
        self.runner = PlaceRunner(self.cfg, self.ops, self.eye, self.link, MotionBook(root / "m.json", root / "b"),
                                  clock=self.clock, sleep=self.clock.sleep, log=lambda *a, **k: None)
        self.runner.max_wait = 30

    def test_car_and_box_seen_sends_stop_then_confirms_full_stop(self):
        self.assertTrue(self.runner.wait_car_and_stop())
        self.assertEqual(self.link.states(), ["STOP_REQUEST"])

    def test_stop_signal_is_quick_while_waiting(self):
        sleeps = []
        self.runner.sleep = lambda dt: (sleeps.append(dt), self.clock.sleep(dt))
        self.assertTrue(self.runner.wait_car_and_stop())
        stop_index = self.cfg.load_see_frames
        self.assertEqual(self.cfg.load_see_frames, 2)                            # 연속 2장이면 정지 신호
        self.assertEqual(sleeps[:stop_index - 1], [self.cfg.load_see_period])     # 기다리는 동안은 짧게 (0.05초)
        self.assertEqual(sleeps[stop_index - 1], self.cfg.track_period)           # 정지 신호 뒤(정차 확인)는 원래 주기

    def test_box_alone_is_not_enough(self):
        self.eye.car = False
        with self.assertRaises(TimeoutError):
            self.runner.wait_car_and_stop()
        self.assertEqual(self.link.states(), [])

    def test_color_and_pick(self):
        self.assertEqual(self.runner.read_color(), "red")
        self.assertEqual(self.runner.pick("red"), (True, "공을 집었어요"))
        self.ops.play_pick = lambda *a: (setattr(self.ops, "gripper", 0.0) or {"reached": True, "stopped": False})
        ok, why = self.runner.pick("red")
        self.assertFalse(ok)
        self.assertIn("공을 못 잡은", why)

    def test_above_servo_moves_pan_until_basket_is_where_it_was_learned(self):
        corr, why = self.runner.step_above(self.learned)
        self.assertIsNotNone(corr, why)
        self.assertAlmostEqual(self.ops.a["1"], 12.0, delta=1.0)              # 차가 2도 옆에 섰다 → 받침이 따라감
        self.assertAlmostEqual(corr["1"], 2.0, delta=1.0)
        ok, _ = self.runner.step_inside(self.learned, corr)
        self.assertAlmostEqual(self.ops.a["1"], 10.0 + corr["1"])                # 6 에도 같은 보정
        self.assertEqual(self.ops.a["2"], -20.0)
        self.runner.step_release(self.learned, corr)
        self.assertEqual(self.ops.gripper, 70.0)                                  # 학습한 % 만큼 열기

    def test_too_far_is_refused_for_safety(self):
        self.eye.basket_pan = 60.0
        corr, why = self.runner.step_above(self.learned)
        self.assertIsNone(corr)
        self.assertIn("넘게", why)

    def test_reversed_direction_is_caught_by_the_limit(self):
        self.cfg.track_pan_sign = -1
        corr, why = self.runner.step_above(self.learned)
        self.assertIsNone(corr)

    def test_auto_place_reports_each_step_and_success(self):
        seen = []
        original = self.runner.step_release

        def release(learned, corr):
            out = original(learned, corr)
            self.eye.loaded = True
            return out
        self.runner.step_release = release
        ok, why = self.runner.auto_place(self.learned, on_step=lambda n, k, w: seen.append((n, k)))
        self.assertTrue(ok, why)
        self.assertEqual(seen, [(5, True), (6, True), (7, True), (8, True)])

    def test_auto_place_fails_when_no_ball_is_seen(self):
        ok, why = self.runner.auto_place(self.learned)
        self.assertFalse(ok)
        self.assertIn("실패", why)

    def test_stop_button(self):
        stop = threading.Event()
        stop.set()
        self.runner.stop_event = stop
        with self.assertRaises(FlowStopped):
            self.runner.step_above(self.learned)


class SessionEye(FakeEye):
    def __init__(self, ops):
        super().__init__(ops)
        self.last = []

    def look(self):
        seen = super().look()
        self.last = seen.dets
        return seen

    def basket(self):
        box = next((d for d in self.last if d.name in ("box", "success_red")), None)
        return None if box is None else {"name": box.name, "conf": 0.9, "cx": box.cx, "cy": box.cy, "w": box.w, "h": box.h}

    def jpeg(self):
        return b"\xff\xd8fake"


class SessionTest(unittest.TestCase):
    """허브 4번 화면 뒤의 책상: 버튼 순서대로 한 회차를 끝까지 (가짜 팔 · 가짜 카메라)"""

    def setUp(self):
        import time as _t
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = PlaceBook(root / "place.json", root / "img", root / "backup")
        self.sid = self.book.create_set("a")
        self.cfg = MissionConfig(grip_ball_min_pct=20.0, grip_ball_max_pct=40.0, track_period=0.01,
                                 load_stop_confirm_seconds=0.1, load_verify_seconds=0.3, load_verify_hits=2,
                                 place_servo_gain_deg_per_px=0.04)
        self.ops = FakeOps()
        self.eye = SessionEye(self.ops)
        self.link = FakeLoadLink()
        from mission.place_page import PlaceSession
        self.s = PlaceSession(None, self.cfg, self.eye, MotionBook(root / "m.json", root / "b"), self.book,
                              ops=self.ops, log=lambda *a, **k: None, sleep=lambda dt: _t.sleep(0.001))
        self.addCleanup(self.s.stop_and_join, 5)

    def wait_for(self, what, timeout=5.0):
        import time as _t
        end = _t.monotonic() + timeout
        while _t.monotonic() < end:
            if self.s.status["waiting"] == what:
                return
            if not self.s.busy():
                self.fail(f"끝나 버림: {self.s.status['message']}")
            _t.sleep(0.01)
        self.fail(f"'{what}' 차례가 오지 않음: {self.s.status}")

    def teach_one(self, verdict="auto", put_ball=True):
        for num, pan, j2 in ((5, 12.0, 0.0), (6, 12.0, -20.0), (7, 12.0, -20.0), (8, 12.0, 5.0)):
            self.wait_for(f"teach{num}")
            self.ops.a.update({"1": pan, "2": j2})                     # 사람이 손으로 옮긴 자리
            if num == 7:
                self.s.handle("arm-torque", {"on": True})
                self.s.handle("grip", {"delta": 40})
                self.eye.loaded = put_ball
            self.assertIn("저장", self.s.handle("save", {"step": num}))
        self.wait_for("next")
        self.s.handle("next", {"verdict": verdict})

    def test_learn_one_take_then_success_sends_go(self):
        with self.assertRaises(PlaceError):
            self.s.handle("save", {"step": 5})                         # 차례가 아니면 거절
        self.s.start("learn", self.sid, self.link)
        self.teach_one()
        self.s._thread.join(5)
        self.assertEqual(self.link.states(), ["WAITING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS"])
        take = next(iter(self.book.get_set(self.sid)["takes"].values()))
        self.assertTrue(take["success"])
        self.assertEqual(take["steps"]["release"]["grip"], 70.0)          # 집게 % 도 저장
        self.assertAlmostEqual(take["steps"]["above"]["basket"]["cx"], WIDTH / 2 - 0 * PX_PER_DEG, delta=1)
        self.assertIsNotNone(self.book.get_set(self.sid)["watch"])        # 대기 자세 (한 번)
        self.assertTrue(self.book.image_path(self.sid, take["steps"]["check"]["image"]))

    def test_failed_take_waits_for_ball_and_picks_again(self):
        self.s.start("learn", self.sid, self.link)
        self.teach_one(put_ball=False)                                    # YOLO 판정 실패
        self.wait_for("ball")
        self.assertEqual(self.ops.picked, ["red"])
        self.s.handle("ball", {})
        self.eye.loaded = False
        self.teach_one(verdict="ok")                                      # 사람이 판정을 '성공' 으로 바꿈
        self.s._thread.join(5)
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2])
        self.assertEqual(self.ops.picked, ["red", "red"])                 # 4(공 집기)부터 다시
        takes = list(self.book.get_set(self.sid)["takes"].values())
        self.assertEqual([t["use"] for t in takes], [False, True])         # 실패 회차는 기록만
        self.assertEqual(self.link.states()[-1], "SUCCESS")

    def test_stop_while_teaching_keeps_the_car_until_person_releases(self):
        self.s.start("learn", self.sid, self.link)
        self.wait_for("teach5")
        self.assertEqual(self.link.pauses, [True])                        # 정지 신호 = /arm/pause True
        self.s.handle("stop", {})
        self.s._thread.join(5)
        self.assertEqual(self.link.states()[-1], "IN_PROGRESS")           # 집게가 바구니에 있을 수 있어 차는 그대로
        self.assertEqual(self.link.pauses, [True])
        self.assertIn("차 출발시키기", self.s.status["message"])
        self.assertIn("FAILED", self.s.handle("car-go", {}))              # 사람이 확인하고 출발
        self.assertEqual(self.link.states()[-1], "FAILED")
        self.assertEqual(self.link.pauses, [True, False])
        with self.assertRaises(PlaceError):
            self.s.handle("car-go", {})                                    # 두 번은 안 됨

    def test_success_pauses_then_releases_the_car(self):
        self.s.start("learn", self.sid, self.link)
        self.teach_one()
        self.s._thread.join(5)
        self.assertEqual(self.link.pauses, [True, False])

    def test_test_mode_from_step_5_uses_the_learned_mean(self):
        self.book.add_take(self.sid, steps(pan=10.0, cx=320.0), {}, success=True)
        with self.assertRaises(PlaceError):
            self.s.start("test", self.sid, self.link, 5)                  # 5 이후는 색을 골라야
        original = self.ops.grip

        def grip(pct):
            original(pct)
            self.eye.loaded = pct > 50
        self.ops.grip = grip
        self.s.start("test", self.sid, self.link, 5, "red", False)
        self.s._thread.join(5)
        results = [(r["step"], r["ok"]) for r in self.s.status["results"]]
        self.assertEqual(results, [(5, True), (6, True), (7, True), (8, True), (9, True)])
        self.assertEqual(self.ops.picked, [])                             # 5부터면 집기는 건너뜀
        self.assertEqual(self.link.states(), ["IN_PROGRESS", "SUCCESS"])


if __name__ == "__main__":
    unittest.main()
