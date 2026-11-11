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
            "lift": snap(pan, None, grip=70.0, j2=-2.0), "check": snap(pan, None, j2=5.0)}


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
        tid = self.book.add_take(sid, steps(), {"above": b"a", "inside": b"b", "release": b"c", "lift": b"l", "check": b"d"},
                                 success=True, color="red")
        self.book.rename_take(sid, tid, "왼쪽_가까이")
        again = PlaceBook(self.book.path, self.book.image_dir, self.book.backup_dir)     # 다시 불러오기
        take = again.take(sid, tid)
        self.assertEqual(take["label"], "왼쪽_가까이")
        self.assertTrue(take["use"])
        self.assertEqual(again.image_path(sid, take["steps"]["above"]["image"]).read_bytes(), b"a")
        self.assertEqual(again.image_path(sid, "watch.jpg").read_bytes(), b"jpg-watch")
        self.assertIsNone(again.image_path(sid, "../place.json"))                   # 폴더 밖은 못 읽음

    def test_old_take_without_lift_uses_the_above_pose(self):
        sid = self.book.create_set("a")
        old = steps(pan=6.0)
        del old["lift"]
        self.book.get_set(sid)["takes"]["t1"] = {"label": "예전", "created": "x", "use": True, "success": True,
                                                 "color": "red", "steps": old}          # 들어올리기가 생기기 전 회차
        self.book.get_set(sid)["next"] = 2
        learned = self.book.learned(sid)
        self.assertEqual(learned["poses"]["lift"], learned["poses"]["above"])      # 5 바구니 위 자세로 대신
        view = self.book.summary()["sets"][0]["takes"][0]["steps"]["lift"]
        self.assertTrue(view["filled"])
        self.assertEqual(view["grip"], 70.0)                                        # 7 의 집게 % (연 채)
        self.book.add_take(sid, steps(pan=8.0), {}, success=True)                  # 새 회차와 섞여도 평균이 된다
        self.assertEqual(self.book.learned(sid)["poses"]["lift"]["1"], 7.0)

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

    def move_to(self, angles, timeout=12.0, speed=None):
        self.a.update({j: float(v) for j, v in angles.items()})
        self.moves += 1
        self.speed = speed
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
        self.assertEqual(seen, [(5, True), (6, True), (7, True), (8, True), (9, True)])

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
                                 place_servo_gain_deg_per_px=0.04, place_seek=False)   # 학습한 넣기 방식 시험
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

    def teach_one(self, verdict="auto", put_ball=True, name=""):
        for num, pan, j2 in ((5, 12.0, 0.0), (6, 12.0, -20.0), (7, 12.0, -20.0), (8, 12.0, -2.0), (9, 12.0, 5.0)):
            self.wait_for(f"teach{num}")
            self.ops.a.update({"1": pan, "2": j2})                     # 사람이 손으로 옮긴 자리
            if num == 7:
                self.s.handle("arm-torque", {"on": True})
                self.s.handle("grip", {"delta": 40})
                self.eye.loaded = put_ball
            self.assertIn("저장", self.s.handle("save", {"step": num}))
        self.wait_for("next")
        self.s.handle("next", {"verdict": verdict, "label": name})

    def test_learn_one_take_then_success_sends_go(self):
        self.s.start("learn", self.sid, self.link)
        self.wait_for("teach5")
        with self.assertRaises(PlaceError):
            self.s.handle("save", {"step": 6})                         # 학습 중에는 차례가 아니면 거절
        self.teach_one(name="첫 회차")
        self.s._thread.join(5)
        self.assertEqual(self.link.states(), ["WAITING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS"])
        take = next(iter(self.book.get_set(self.sid)["takes"].values()))
        self.assertTrue(take["success"])
        self.assertEqual(take["label"], "첫 회차")                          # 화면의 회차 이름
        self.assertEqual(self.s.status["draft"], {})                        # 회차로 저장하면 지금 묶음은 비운다
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

    def test_hand_teaching_without_a_run_then_save_as_take(self):
        self.ops.a.update({"1": 12.0})
        self.assertIn("저장", self.s.handle("save", {"step": 5}))           # 학습을 안 돌려도 아무 단계나
        self.ops.a.update({"1": 13.0})
        self.assertIn("덮어씀", self.s.handle("save", {"step": 5}))         # 같은 단계 = 덮어쓰기
        self.assertEqual(self.s.status["draft"]["above"]["angles"]["1"], 13.0)
        self.assertTrue(self.s.status["draft"]["above"]["img"])
        self.assertEqual(self.s.draft_image("above"), b"\xff\xd8fake")
        with self.assertRaises(PlaceError):
            self.s.handle("take-save", {"sid": self.sid})                   # 아직 6 · 7 · 8 · 9 가 없음
        self.ops.a.update({"2": -20.0})
        self.s.handle("save", {"step": 9})
        self.s.handle("save", {"step": 8})
        self.s.handle("save", {"step": 6})                                  # 순서 상관없이
        self.s.handle("save", {"step": 7})
        self.assertIn("✘ 실패", self.s.handle("take-save", {"sid": self.sid, "label": "손 회차", "success": False}))
        take = next(iter(self.book.get_set(self.sid)["takes"].values()))
        self.assertEqual((take["label"], take["use"]), ("손 회차", False))
        self.assertEqual(take["steps"]["above"]["angles"]["1"], 13.0)
        self.assertTrue(self.book.image_path(self.sid, take["steps"]["above"]["image"]))
        self.assertEqual(self.s.status["draft"], {})
        self.s.handle("save", {"step": 5})
        self.assertIn("비웠", self.s.handle("draft-clear", {}))
        self.assertEqual(self.s.status["draft"], {})

    def test_pick_only_reads_the_postit_and_picks_that_ball(self):
        self.s.start("test", self.sid, self.link, 4, "auto")             # 학습 회차가 없어도 된다
        self.s._thread.join(5)
        self.assertEqual(self.ops.picked, ["red"])                         # 포스트잇 빨강 → 빨간공_집기
        self.assertEqual(self.s.status["color"], "red")
        self.assertEqual(self.link.history, [])                            # 차 신호는 안 보낸다
        self.assertTrue(self.s.status["results"][-1]["ok"])

    def test_chosen_color_skips_reading_the_postit(self):
        self.book.add_take(self.sid, steps(pan=10.0, cx=320.0), {}, success=True)
        self.s.start("test", self.sid, self.link, 2, "blue", False)
        self.s._thread.join(5)
        self.assertEqual(self.ops.picked[0], "blue")                       # 포스트잇은 빨강이지만 고른 파랑으로

    def test_camera_direction_needs_a_learned_take(self):
        with self.assertRaises(PlaceError):
            self.s.handle("cam-measure", {"sid": self.sid})                # ✔ 회차가 없으면 못 잰다
        self.assertFalse(self.s.state()["book"]["sets"][0]["cam"])

    def test_start_rules(self):
        with self.assertRaises(PlaceError):
            self.s.start("learn", self.sid, self.link, 7, "red")           # 빌려 올 성공 회차가 없음
        with self.assertRaises(PlaceError):
            self.s.start("learn", self.sid, self.link, 5, "auto")          # 5 이후는 색을 골라야
        with self.assertRaises(PlaceError):
            self.s.start("learn", self.sid, self.link, 3, "red")           # 3 은 2~4 묶음 안
        with self.assertRaises(PlaceError):
            self.s.start("test", self.sid, self.link, 2, "auto")           # 테스트 2 는 학습 회차가 필요

    def test_learn_from_step_7_borrows_5_and_6_from_the_latest_success(self):
        self.book.add_take(self.sid, steps(pan=3.0, cx=300.0), {}, success=True)
        self.book.add_take(self.sid, steps(pan=4.0, cx=310.0), {}, success=True)   # 가장 최근 성공
        self.ops.a.update({"1": 12.0, "2": -20.0})                         # 지금 상태: 바구니 안에서 공을 쥔 채
        self.s.start("learn", self.sid, self.link, 7, "red")
        self.wait_for("teach7")
        self.assertEqual(self.link.states(), ["IN_PROGRESS"])              # 차를 세워 둔 채 시작
        self.s.handle("grip", {"delta": 40})
        self.eye.loaded = True
        self.s.handle("save", {"step": 7})
        self.wait_for("teach8")
        self.ops.a.update({"2": -2.0})                                      # 8 들어올리기
        self.s.handle("save", {"step": 8})
        self.wait_for("teach9")
        self.s.handle("save", {"step": 9})
        self.wait_for("next")
        self.s.handle("next", {"verdict": "auto"})
        self.s._thread.join(5)
        takes = list(self.book.get_set(self.sid)["takes"].values())
        new = takes[-1]
        self.assertEqual(new["steps"]["above"]["angles"]["1"], 4.0)         # 5 · 6 은 최근 성공 회차에서
        self.assertEqual(new["steps"]["inside"]["angles"]["1"], 4.0)
        self.assertEqual(new["steps"]["release"]["angles"]["1"], 12.0)      # 7 · 8 · 9 는 지금 가르친 값
        self.assertEqual(new["steps"]["lift"]["angles"]["2"], -2.0)
        self.assertEqual(self.ops.picked, [])                               # 공 집기는 건너뜀
        self.assertEqual(self.link.states()[-1], "SUCCESS")

    def test_learn_from_10_only_goes_home_and_sends_go(self):
        self.s.start("learn", self.sid, self.link, 10, "blue")
        self.s._thread.join(5)
        self.assertEqual(self.link.states(), ["IN_PROGRESS", "SUCCESS"])

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
        self.assertEqual(results, [(5, True), (6, True), (7, True), (8, True), (9, True), (10, True)])
        self.assertEqual(self.ops.picked, [])                             # 5부터면 집기는 건너뜀
        self.assertEqual(self.link.states(), ["IN_PROGRESS", "SUCCESS"])



class KinOps(FakeOps):
    """가짜 팔 (모터 각도를 그대로 기억). 손끝 위치는 진짜 기구학(FK)으로 계산된다"""

    def __init__(self, a):
        super().__init__()
        self.a = dict(a)
        self.gripper = 30.0

    def nudge(self, joint, deg):
        self.a[joint] = deg


class XYEye:
    """손목 카메라 흉내: 집게가 아래를 보고 있으면 바구니가 손끝과 어긋난 만큼 화면에서 움직여 보인다 (1 cm = 20 픽셀)"""
    PX_PER_M = 2000.0

    def __init__(self, ops, basket_xyz):
        self.ops, self.basket = ops, basket_xyz
        self.loaded = False
        self.release_xy = None

    def tip(self):
        from mission.place_reach import tip_of
        return tip_of(self.ops.a)

    def look(self):
        t = self.tip()
        cx = 320 - self.PX_PER_M * (self.basket[1] - t[1])
        cy = 240 - self.PX_PER_M * (self.basket[0] - t[0])
        dets = []
        if 0 <= cx <= 640 and 0 <= cy <= 480:
            dets.append(Det("success_red" if self.loaded else "box", 0.9, cx - 40, cy - 30, 80, 60))
        return Seen(dets, WIDTH)


class YoloPlaceTest(unittest.TestCase):
    """YOLO 넣기: 차가 학습 때보다 3 cm · 2 cm 어긋나 서도 바구니 위로 맞추고, 수직으로 내려가 놓고, 올라온다"""

    def setUp(self):
        import numpy as np
        from mission.place_reach import angles_from_urdf, ik_down, tip_of, urdf_from_angles
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = PlaceBook(root / "place.json", root / "img", root / "backup")
        self.sid = self.book.create_set("a")
        above = {"1": 0.0, "2": -30.0, "3": 40.0, "4": 70.0, "5": -90.0}
        tip = tip_of(above)
        q_in = ik_down(tip - np.array([0, 0, 0.04]), urdf_from_angles(above))
        inside = angles_from_urdf(q_in)                                           # 학습 때 4 cm 아래로
        def snap5(a, basket=None, grip=30.0):
            return make_snap(a, grip, basket, None)
        target = {"name": "box", "conf": 0.9, "cx": 320.0, "cy": 240.0, "w": 80.0, "h": 60.0}
        self.book.add_take(self.sid, {"above": snap5(above, target), "inside": snap5(inside), "release": snap5(inside, grip=80.0),
                                      "lift": snap5(above, grip=80.0), "check": snap5(above)}, {}, success=True)
        self.learned = self.book.learned(self.sid)
        self.tip0 = tip
        self.ops = KinOps(above)
        self.basket = (float(tip[0]) + 0.03, float(tip[1]) - 0.02, float(tip[2]) - 0.05)   # 차가 어긋나 섬
        self.eye = XYEye(self.ops, self.basket)
        original = self.ops.grip

        def grip(pct):
            original(pct)
            if pct > 50:
                t = self.eye.tip()
                self.eye.release_xy = (float(t[0]), float(t[1]), float(t[2]))
                self.eye.loaded = True
        self.ops.grip = grip
        self.cfg = MissionConfig(track_period=0.01, place_servo_seconds=30.0, load_verify_seconds=0.3, load_verify_hits=2,
                                 place_servo_deadband_px=4.0)                   # 4 픽셀 = 2 mm
        self.clock = Clock()
        self.runner = PlaceRunner(self.cfg, self.ops, self.eye, FakeLoadLink(), MotionBook(root / "m.json", root / "b"),
                                  clock=self.clock, sleep=self.clock.sleep, log=lambda *a, **k: None)

    def test_without_camera_direction_it_uses_saved_poses(self):
        self.assertFalse(self.runner.yolo_ready(self.learned))                    # 카메라 방향을 재기 전에는 예전 방식

    def test_measure_then_place_follows_the_basket(self):
        cam = self.runner.measure_camera(self.learned)
        m = cam["m"]
        self.assertAlmostEqual(m[1][0], 2000, delta=150)                           # 손끝 x(앞) +1 m → 화면 y +2000 px
        self.assertAlmostEqual(m[0][1], 2000, delta=150)                           # 손끝 y(왼쪽) +1 m → 화면 x +2000 px
        self.book.set_cam(self.sid, cam)
        learned = self.book.learned(self.sid)
        self.assertTrue(self.runner.yolo_ready(learned))
        seen = []
        ok, why = self.runner.auto_place(learned, on_step=lambda n, k, w: seen.append((n, k)))
        self.assertTrue(ok, why)
        self.assertEqual(seen, [(5, True), (6, True), (7, True), (8, True), (9, True)])
        rx, ry, rz = self.eye.release_xy
        self.assertAlmostEqual(rx, self.basket[0], delta=0.004)                    # 바구니 바로 위에서 (4 mm 안)
        self.assertAlmostEqual(ry, self.basket[1], delta=0.004)
        self.assertAlmostEqual(rz, float(self.tip0[2]) - 0.04, delta=0.004)        # 학습한 깊이만큼 내려가서 놓음

    def ready(self):
        self.book.set_cam(self.sid, {"m": [[0.0, 2000.0], [2000.0, 0.0]], "z": 0.0})
        return self.book.learned(self.sid)

    def test_keeps_the_taught_gripper_tilt(self):
        from mission.place_reach import lean_of, urdf_from_angles
        learned = self.ready()
        taught = lean_of(urdf_from_angles(learned["poses"]["above"]))
        state, why = self.runner.yolo_above(learned)
        self.assertIsNotNone(state, why)
        self.assertAlmostEqual(lean_of(urdf_from_angles(self.ops.a)), taught, delta=0.02)   # 수직으로 세우지 않음

    def test_unreachable_goes_as_far_as_it_can_then_releases(self):
        from mission import place_flow
        from mission.place_reach import ReachError
        learned = self.ready()
        calls = {"n": 0}
        original = place_flow.Reach.go

        def go(reach, xyz, seed=None):
            calls["n"] += 1
            if calls["n"] > 2:                                                      # 두 번 옮긴 뒤로는 팔이 닿지 않음
                raise ReachError("닿지 않음")
            return original(reach, xyz, seed)
        place_flow.Reach.go = go
        self.addCleanup(setattr, place_flow.Reach, "go", original)
        seen = []
        ok, why = self.runner.auto_place(learned, on_step=lambda n, k, w: seen.append((n, k, w)))
        self.assertIn("여기서 넣어요", seen[0][2])                                 # 멈추지 않고 진행
        self.assertTrue(seen[1][1])
        self.assertIn("만 내려갔어요", seen[1][2])                                  # 6 도 가능한 만큼만
        self.assertEqual([n for n, *_ in seen], [5, 6, 7, 8, 9])
        self.assertTrue(self.eye.loaded)                                            # 집게는 열었다

    def test_ball_must_still_be_held_before_release(self):
        self.cfg.grip_ball_min_pct, self.cfg.grip_ball_max_pct = 20.0, 40.0
        learned = self.ready()
        state, why = self.runner.yolo_above(learned)
        self.assertIsNotNone(state, why)
        self.ops.gripper = 95.0                                                     # 공이 빠짐 (집게가 끝까지 닫혀 버림 흉내)
        ok, why = self.runner.release_here(learned)
        self.assertFalse(ok)
        self.assertIn("쥐고 있지 않아요", why)

    def test_check_fails_while_the_ball_is_still_in_the_gripper(self):
        self.cfg.grip_ball_min_pct, self.cfg.grip_ball_max_pct = 20.0, 40.0
        learned = self.ready()
        self.eye.loaded = True                                                      # YOLO 는 '공이 든 바구니' 라고 해도
        ok, why = self.runner.step_check(learned, {})
        self.assertFalse(ok)
        self.assertIn("아직 공을 쥐고", why)

    def test_measure_uses_the_other_side_when_one_side_is_out_of_reach(self):
        from mission import place_flow
        from mission.place_reach import ReachError
        original = place_flow.Reach.go
        tip0 = self.tip0

        def go(reach, xyz, seed=None):
            if xyz[0] > tip0[0] + 0.001:                                            # 앞으로는 팔이 닿지 않음
                raise ReachError("닿지 않음")
            return original(reach, xyz, seed)
        place_flow.Reach.go = go
        self.addCleanup(setattr, place_flow.Reach, "go", original)
        cam = self.runner.measure_camera(self.learned)
        self.assertAlmostEqual(cam["m"][1][0], 2000, delta=150)                   # 뒤로 움직여 재도 같은 값

    def test_too_far_is_refused(self):
        self.book.set_cam(self.sid, {"m": [[0.0, 2000.0], [2000.0, 0.0]], "z": 0.0})
        self.eye.basket = (self.basket[0] + 0.2, self.basket[1], self.basket[2])  # 20 cm 넘게 어긋남 (화면 밖)
        state, why = self.runner.yolo_above(self.book.learned(self.sid))
        self.assertIsNone(state)

    def test_wrong_direction_is_caught_by_the_limit(self):
        self.book.set_cam(self.sid, {"m": [[0.0, -2000.0], [-2000.0, 0.0]], "z": 0.0})   # 방향이 반대로 잘못 잼
        state, why = self.runner.yolo_above(self.book.learned(self.sid))
        self.assertIsNone(state)
        self.assertTrue("mm" in why or "보이지" in why, why)



class CamEye:
    """손목 카메라 흉내 (핀홀): 집게가 향하는 쪽을 보는 카메라. 바구니(12 cm 네모)가 가까울수록 크게 보인다"""
    F = 500.0

    def __init__(self, ops, basket_xyz, size=0.12):
        self.ops, self.basket, self.size = ops, basket_xyz, size
        self.loaded = False

    def view(self):
        import numpy as np
        from soarm.kinematics import frames
        from mission.place_reach import urdf_from_angles
        F = dict(frames(urdf_from_angles(self.ops.a)))
        tip = F["gripper_frame_joint"][:3, 3]
        zc = -F["wrist_roll"][:3, 2]
        xc = np.cross(zc, [0, 0, 1.0])
        xc /= np.linalg.norm(xc)
        yc = np.cross(zc, xc)
        p = np.asarray(self.basket) - tip
        depth = float(p @ zc)
        if depth <= 0.01:
            return None
        u = 320 + self.F * float(p @ xc) / depth
        v = 240 + self.F * float(p @ yc) / depth
        half = self.F * self.size / depth / 2
        x0, x1, y0, y1 = max(0, u - half), min(640, u + half), max(0, v - half), min(480, v + half)
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1 - x0, y1 - y0, depth

    def look(self):
        got = self.view()
        dets = []
        if got is not None:
            x, y, w, h, _ = got
            dets.append(Det("success_blue" if self.loaded else "box", 0.9, x, y, w, h))
        return Seen(dets, 640, 480)


class SeekTest(unittest.TestCase):
    """학습 없이 찾아가 넣기: 대기 자세에서 바구니를 가운데로 → 화면 50% 까지 다가감 → 열기 → 위로 → 성공"""

    def setUp(self):
        self.ops = KinOps({"1": -7.4, "2": -77.8, "3": 29.9, "4": 85.1, "5": -93.7})   # Pi 대기 자세 (앞으로 많이 기울어 있음)
        self.eye = CamEye(self.ops, (0.28, 0.03, -0.05))
        original = self.ops.grip
        self.released_at = None

        def grip(pct):
            original(pct)
            if pct > 50:
                from mission.place_reach import tip_of
                self.released_at = self.eye.view()
                from mission.place_reach import lean_of, urdf_from_angles
                self.drop_xy = tip_of(self.ops.a)[:2]                         # 공은 집게 아래로 떨어진다
                self.drop_lean = lean_of(urdf_from_angles(self.ops.a))
                self.eye.loaded = True
        self.ops.grip = grip
        self.cfg = MissionConfig(track_period=0.01, load_verify_seconds=0.3, load_verify_hits=2)
        self.clock = Clock()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.runner = PlaceRunner(self.cfg, self.ops, self.eye, FakeLoadLink(), MotionBook(Path(tmp.name) / "m.json", Path(tmp.name) / "b"),
                                  clock=self.clock, sleep=self.clock.sleep, log=lambda *a, **k: None)

    def run_seek(self):
        seen = []
        ok, why = self.runner.seek_place(on_step=lambda n, k, w: seen.append((n, k, w)))
        return ok, why, seen

    def test_finds_the_basket_fills_half_the_screen_releases_lifts_and_checks(self):
        from mission.place_reach import lean_of, urdf_from_angles
        lean0 = lean_of(urdf_from_angles(self.ops.a))
        ok, why, seen = self.run_seek()
        self.assertTrue(ok, seen)
        self.assertEqual([(n, k) for n, k, _ in seen], [(5, True), (6, True), (7, True), (8, True), (9, True)])
        x, y, w, h, depth = self.released_at
        self.assertGreaterEqual(w * h / (640 * 480), self.cfg.seek_area)            # 화면의 75% 이상
        self.assertLess(w * h / (640 * 480), 0.85)                                  # 너무 가까이 가지 않음
        self.assertLess(abs(x + w / 2 - 320), 40)                                   # 바구니가 화면 가운데
        self.assertLess(abs(y + h / 2 - 240), 40)
        import math
        self.assertLess(self.drop_lean, lean0)                                      # 다가가며 집게를 세웠다
        self.assertLessEqual(self.drop_lean, math.sin(math.radians(self.cfg.seek_final_tilt_deg)) + 0.05)   # 놓을 때 거의 수직 (IK 허용 오차 0.05)
        bx, by = self.eye.basket[:2]
        self.assertLess(math.hypot(self.drop_xy[0] - bx, self.drop_xy[1] - by), 0.03)   # 공이 바구니 가운데 3 cm 안에 떨어짐
        self.assertIn("50 mm", seen[3][2])                                          # 놓은 뒤 위로

    def test_yolo_losing_the_close_basket_still_releases_there(self):
        view = self.eye.look

        def look():                                                                 # 가까워지면 (화면 60% 넘게) YOLO 가 바구니를 놓친다
            seen = view()
            if seen.dets and seen.dets[0].w * seen.dets[0].h / (640 * 480) > 0.6:
                return Seen([], 640, 480)
            return seen
        self.eye.look = look
        ok, why, seen = self.run_seek()
        steps = {n: (k, w) for n, k, w in seen}
        self.assertTrue(steps[6][0], seen)
        self.assertIn("놓쳤지만", steps[6][1])
        self.assertTrue(steps[7][0])                                                 # 집게를 열었다

    def test_success_label_before_step_9_is_only_the_basket(self):
        self.eye.loaded = True                                                      # 처음부터 'success_blue' 로 보임 (쥔 공이 바구니 앞)
        ok, why, seen = self.run_seek()
        self.assertEqual([n for n, k, w in seen if k], [5, 6, 7, 8, 9])             # 5~8 은 그냥 바구니로 따라가고
        self.assertEqual(seen[-1][0], 9)                                             # 성공 판단은 9 에서만

    def test_basket_not_visible_fails_without_moving_far(self):
        self.eye.basket = (0.30, 0.6, -0.12)                                        # 화면 밖
        ok, why, seen = self.run_seek()
        self.assertFalse(ok)
        self.assertIn("보이지 않아요", why)

    def test_unreachable_goes_as_far_as_it_can_then_releases(self):
        from mission import place_flow
        from mission.place_reach import ReachError
        original = place_flow.Reach.go
        floor = {"z": None}

        def go(reach, xyz, seed=None):
            if floor["z"] is None:
                floor["z"] = float(reach.now()[0][2]) - 0.05                      # 시작보다 5 cm 아래로는 팔이 안 닿음
            if xyz[2] < floor["z"]:
                raise ReachError("닿지 않음")
            return original(reach, xyz, seed)
        place_flow.Reach.go = go
        self.addCleanup(setattr, place_flow.Reach, "go", original)
        ok, why, seen = self.run_seek()
        steps = {n: (k, w) for n, k, w in seen}
        self.assertTrue(steps[6][0])
        self.assertIn("여기서 집게를 열어요", steps[6][1])                           # 멈추지 않고 진행
        self.assertTrue(steps[7][0])

    def test_ball_still_in_gripper_is_not_success(self):
        self.cfg.grip_ball_min_pct, self.cfg.grip_ball_max_pct = 20.0, 40.0
        self.ops.grip = lambda pct: None                                            # 집게가 안 열림
        self.eye.loaded = False
        ok, why, seen = self.run_seek()
        self.assertFalse(ok)
        self.assertIn("아직 공을 쥐고", why)


if __name__ == "__main__":
    unittest.main()
