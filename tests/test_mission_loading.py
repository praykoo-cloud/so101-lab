"""상차 미션(팔이 차를 세움) · YOLO 풀이 · 16번 화면 책상 시험 — 가짜 팔 · 가짜 세계만 쓴다 (실물 · 카메라 · ROS2 없음).

    python -m unittest tests.test_mission_loading
"""
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from mission.config import MissionConfig
from mission.load_link import ARM_STATES, CAR_STATES, FakeLoadLink
from mission.loading import LoadAbort, Loader, Seen
from mission.motion_page import MotionDesk
from mission.motions import MotionBook, MotionError
from mission.yolo import CLASS_NAMES, MODELS_DIR, Det, Detector, decode, letterbox, pick_best, track_target
from soarm import Robot

PX_PER_DEG = 20.0
WIDTH = 640


class World:
    """가짜 세계. 차는 받침 각도로 치면 start 도에서 +방향으로 달려오고, 정지 신호를 받으면 조금 미끄러진 뒤 선다"""

    def __init__(self, link, start=-20.0, speed=4.0, slip=0.8, obeys=True, note="red"):
        self.link = link
        self.t = 0.0
        self.car = start                  # 차가 있는 방향 (받침 각도, 도)
        self.speed, self.slip, self.obeys, self.note = speed, slip, obeys, note
        self.present = True
        self.slid = 0.0
        self.loaded = None                # 바구니에 든 공 색
        self.leave_after_result = True
        self.stop_seen_at = None

    def clock(self):
        return self.t

    def sleep(self, dt):
        steps = max(1, int(round(dt / 0.05)))
        for _ in range(steps):
            self.step(dt / steps)

    def step(self, dt):
        self.t += dt
        state = self.link.state
        if not self.present:
            return
        if state in ("STOP_REQUEST", "IN_PROGRESS") and self.obeys:
            if self.stop_seen_at is None:
                self.stop_seen_at = self.t
            if self.slid < self.slip:                               # 미끄러지는 동안
                d = min(self.speed * dt, self.slip - self.slid)
                self.car += d
                self.slid += d
            else:
                self.link.on_car("AT_LOADING_ZONE")
        elif state in ("SUCCESS", "FAILED"):
            if self.leave_after_result:
                self.link.on_car("DRIVING")
                self.car += self.speed * 3 * dt
                if self.car > 60:
                    self.present = False
        else:
            self.link.on_car("DRIVING")
            self.car += self.speed * dt
            if self.car > 60:
                self.present = False

    def stopped(self):
        return self.obeys and self.slid >= self.slip and self.link.state in ("STOP_REQUEST", "IN_PROGRESS")


class FakeArm:
    def __init__(self, world, watch_pan=-20.0):
        self.world = world
        self.poses = {"watch": watch_pan}
        self.pan = 0.0
        self.torque = False
        self.hot = False
        self.grip = 30.0
        self.went = []

    def preflight(self, needed):
        return [f"자세 '{n}' 이 티칭되지 않았습니다" for n in needed if n not in self.poses]

    def torque_on(self):
        self.torque = True

    def too_hot(self):
        return self.hot

    def go(self, pose, **kw):
        self.went.append(pose)
        self.pan = self.poses[pose]
        return True

    def pan_angle(self):
        return self.pan

    def nudge_pan(self, delta, center, limit):
        self.pan = max(center - limit, min(center + limit, self.pan + delta))
        return self.pan

    def grip_pct(self):
        return self.grip


class FakeEye:
    """세계를 받침 각도 기준으로 '찍는다': 차가 받침이 보는 방향에서 벗어난 만큼 화면에서도 옆으로 벗어난다"""

    def __init__(self, world, arm):
        self.world, self.arm = world, arm
        self.hide_note = False

    def look(self):
        w = self.world
        dets = []
        cx = WIDTH / 2 + (w.car - self.arm.pan) * PX_PER_DEG
        if w.present and 0 <= cx <= WIDTH:
            name = f"success_{w.loaded}" if w.loaded else "box"
            dets.append(Det("car", 0.95, cx - 90, 180, 180, 160))
            dets.append(Det(name, 0.9, cx - 40, 200, 80, 60))
            if w.stopped() and not w.loaded and not self.hide_note:
                dets.append(Det(f"postit_{w.note}", 0.8, cx - 10, 220, 20, 20))
        return Seen(dets, WIDTH)


class SimLoader(Loader):
    """팔 동작만 가짜로 바꾼 Loader (판단 · 순서는 진짜 코드)"""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.played = []
        self.pick_ok = True
        self.place_results = None          # 예: [False, True] = 첫 넣기는 빗나감
        self.place_tolerance_deg = 4.0     # 받침 보정이 실제 차 방향과 이만큼 안이어야 공이 들어간다

    def play_motion(self, name, pan_offset=0.0):
        self.played.append((name, round(pan_offset, 1)))
        self.sleep(1.0)
        world = self.arm.world
        if "집기" in name:
            self.arm.grip = 30.0 if self.pick_ok else 0.0
        elif name == "바구니_넣기":
            hit = self.place_results.pop(0) if self.place_results else True
            aim = self.book.car_ref_pan() + pan_offset
            if hit and abs(aim - world.car) <= self.place_tolerance_deg and world.stopped():
                world.loaded = "red" if any("빨간" in n for n, _ in self.played[-2:]) else "blue"
            self.arm.pan = aim
        return {"reached": True, "stopped": False, "max_err": 0.5, "why": "", "done": 3, "errors": []}

    def turn_to(self, pan):
        self.arm.pan = pan


def one_point():
    return [{"angles": {j: 0.0 for j in (1, 2, 3, 4, 5)}, "grip": 50.0}]


class LoadingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.book = MotionBook(Path(self.tmp.name) / "motions.json", Path(self.tmp.name) / "backup")
        for name in ("빨간공_집기", "파란공_집기"):
            self.book.add_take(name, one_point())
        self.book.set_car_ref(10.0)
        self.book.add_take("바구니_넣기", one_point(), ref_pan=10.0)
        self.cfg = MissionConfig(grip_ball_min_pct=20.0, grip_ball_max_pct=40.0, track_gain_deg_per_px=0.04,
                                 track_period=0.1, load_retry_wait_seconds=1.0, load_leave_timeout=10.0,
                                 load_use_tracking=True)          # 이 묶음은 예전 추적 방식 시험
        self.link = FakeLoadLink()
        self.events = []

    def build(self, **world_kw):
        self.world = World(self.link, **world_kw)
        self.arm = FakeArm(self.world)
        self.eye = FakeEye(self.world, self.arm)
        loader = SimLoader(self.cfg, self.arm, self.eye, self.link, self.book, clock=self.world.clock,
                           sleep=self.world.sleep, log=lambda event, **kw: self.events.append((event, kw)))
        loader.max_wait = 60
        return loader

    def event(self, name):
        return [kw for e, kw in self.events if e == name]

    def test_full_cycle_red(self):
        loader = self.build(note="red")
        loader.run(max_jobs=1)
        self.assertEqual(self.link.states(), ["WAITING", "TRACKING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS", "WAITING"])
        self.assertEqual([n for n, _ in loader.played], ["빨간공_집기", "바구니_넣기"])
        self.assertEqual(self.world.loaded, "red")
        in_progress = [h for h in self.link.history if h["state"] == "IN_PROGRESS"][0]
        self.assertEqual((in_progress["color"], in_progress["attempt"]), ("red", 1))
        self.assertEqual(self.arm.went[0], "watch")                    # 전원 켜면 대기 자세
        self.assertEqual(self.arm.went[-1], "watch")                   # 끝나면 대기 자세로 복귀

    def test_blue_uses_blue_pick_motion(self):
        loader = self.build(note="blue")
        loader.run(max_jobs=1)
        self.assertEqual([n for n, _ in loader.played], ["파란공_집기", "바구니_넣기"])
        self.assertEqual(self.link.states()[-2], "SUCCESS")

    def test_stop_request_is_sent_near_reference_angle_then_keeps_tracking(self):
        loader = self.build(slip=3.0)
        loader.run(max_jobs=1)
        sent = self.event("stop_request")[0]
        self.assertGreaterEqual(sent["pan"], 10.0 - self.cfg.load_stop_lead_deg - 0.01)
        self.assertLess(sent["pan"], 10.0 + 3.0)
        stopped = self.event("car_stopped")[0]
        self.assertGreater(stopped["pan"], sent["pan"])                # 정지 신호 뒤에도 미끄러진 차를 계속 따라감
        self.assertLessEqual(abs(stopped["off_center_px"]), self.cfg.track_deadband_px + 1)

    def test_waits_confirm_seconds_after_car_really_stops(self):
        loader = self.build(slip=2.0)
        loader.boot()
        stop = loader.track_and_stop()
        self.assertIsNotNone(stop)
        slide_time = 2.0 / 4.0                                         # 미끄러지는 데 걸린 시간
        waited = self.world.t - (self.world.stop_seen_at + slide_time)
        self.assertGreaterEqual(waited, self.cfg.load_stop_confirm_seconds - 0.25)
        self.assertAlmostEqual(stop.pan, self.world.car, delta=self.cfg.track_deadband_px / PX_PER_DEG + 0.5)

    def test_place_is_shifted_to_where_the_car_actually_stopped(self):
        loader = self.build(slip=5.0)                                  # 기준(10도)보다 한참 지나서 섬
        loader.place_tolerance_deg = 2.0
        loader.run(max_jobs=1)
        name, offset = loader.played[-1]
        self.assertEqual(name, "바구니_넣기")
        self.assertGreater(offset, 1.5)                                # 지나간 만큼 받침을 더 돌려 넣음
        self.assertEqual(self.link.states()[-2], "SUCCESS")

    def test_car_that_ignores_stop_is_released_with_failed(self):
        self.cfg.load_stop_timeout = 4.0
        loader = self.build(obeys=False)
        loader.run(max_jobs=1)
        states = self.link.states()
        self.assertIn("STOP_REQUEST", states)
        self.assertNotIn("IN_PROGRESS", states)
        self.assertEqual(states[-2], "FAILED")
        self.assertEqual(loader.played, [])                            # 움직이는 차에는 손대지 않음

    def test_retry_then_success_counts_attempts(self):
        loader = self.build()
        loader.place_results = [False, True]
        loader.run(max_jobs=1)
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2])
        self.assertEqual(self.link.states()[-2], "SUCCESS")

    def test_three_failures_send_failed(self):
        loader = self.build()
        loader.place_results = [False, False, False]
        loader.run(max_jobs=1)
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2, 3])
        self.assertEqual(self.link.states()[-2], "FAILED")
        self.assertIsNone(self.world.loaded)

    def test_empty_gripper_skips_place(self):
        loader = self.build()
        loader.pick_ok = False
        loader.run(max_jobs=1)
        self.assertNotIn("바구니_넣기", [n for n, _ in loader.played])
        self.assertEqual(self.link.states()[-2], "FAILED")

    def test_unreadable_color_sends_failed_without_guessing(self):
        loader = self.build()
        self.eye.hide_note = True
        loader.run(max_jobs=1)
        self.assertEqual(loader.played, [])
        self.assertEqual(self.link.states()[-2], "FAILED")

    def test_overheat_aborts_but_still_releases_car(self):
        loader = self.build()
        original = loader.read_color

        def hot_after_color():
            self.arm.hot = True
            return original()
        loader.read_color = hot_after_color
        with self.assertRaises(LoadAbort):
            loader.run(max_jobs=1)
        self.assertEqual(self.link.state, "FAILED")

    def test_missing_teaching_blocks_start(self):
        self.book.delete_take("파란공_집기", "t1")
        loader = self.build()
        with self.assertRaises(LoadAbort) as ctx:
            loader.run(max_jobs=1)
        self.assertIn("파란공_집기", str(ctx.exception))
        self.assertFalse(self.arm.torque)                              # 준비가 안 됐으면 팔을 움직이지 않음

    def test_no_car_keeps_waiting(self):
        loader = self.build(start=200.0)                               # 차가 안 보임
        loader.max_wait = 5
        with self.assertRaises(TimeoutError):
            loader.run(max_jobs=1)
        self.assertNotIn("STOP_REQUEST", self.link.states())

    def test_message_numbers_match_team_interface(self):
        self.assertEqual(ARM_STATES.index("STOP_REQUEST"), 2)


class WorldOps:
    """기본 방식 시험용 팔: 받침 각도는 FakeArm.pan 과 같다. 7(집게 해제) 때 바구니 위에 맞춰져 있으면 공이 들어간다"""

    def __init__(self, world, arm, events):
        self.world, self.arm, self.events = world, arm, events
        self.a = {"1": arm.pan, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0}
        self.miss = []                    # 예: [True] = 첫 넣기는 빗나감

    def too_hot(self):
        return self.arm.hot

    def go_wait(self):
        self.events.append(("go_wait", {}))
        self.arm.go("watch")
        self.a["1"] = self.arm.pan
        return True

    def angles(self):
        self.a["1"] = self.arm.pan
        return dict(self.a)

    def grip_pct(self):
        return self.arm.grip

    def move_to(self, angles, timeout=12.0, speed=None):
        self.a.update({j: float(v) for j, v in angles.items()})
        self.arm.pan = self.a["1"]
        self.world.sleep(0.5)
        return True

    def nudge(self, joint, deg):
        self.a[joint] = deg
        if joint == "1":
            self.arm.pan = deg

    def grip(self, pct):
        if pct > 50 and self.a["2"] < -10:                      # 바구니 안에서 열었다
            miss = self.miss.pop(0) if self.miss else False
            if not miss and abs(self.arm.pan - self.world.car) <= 1.5 and self.world.stopped():
                self.world.loaded = self.color
        self.arm.grip = pct

    def torque_arm(self, on):
        pass

    def hold(self):
        pass

    def play_pick(self, color, book, stop):
        self.color = color
        self.events.append(("ops_pick", {"color": color}))
        self.arm.grip = 30.0
        self.world.sleep(1.0)
        return {"reached": True, "stopped": False, "why": ""}


class NewFlowTest(unittest.TestCase):
    """기본 방식 (2026-11-11): 차 + 상자가 보이면 바로 정지 → 집기 → 4번 학습으로 넣기 → 대기 자세 → 출발"""

    def setUp(self):
        from mission.place_learn import PlaceBook, make_snap
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.book = MotionBook(root / "motions.json", root / "backup")
        for name in ("빨간공_집기", "파란공_집기"):
            self.book.add_take(name, one_point())
        self.places = PlaceBook(root / "place.json", root / "place", root / "backup")
        sid = self.places.create_set("a")

        def snap(j2, grip=30.0, cx=None):
            basket = None if cx is None else {"name": "box", "conf": 0.9, "cx": cx, "cy": 230.0, "w": 80.0, "h": 60.0}
            return make_snap({"1": -18.0, "2": j2, "3": 0, "4": 0, "5": 0}, grip, basket, None)
        self.places.add_take(sid, {"above": snap(0.0, cx=WIDTH / 2), "inside": snap(-20.0), "release": snap(-20.0, 80.0),
                                   "lift": snap(0.0, 80.0), "check": snap(5.0)}, {}, success=True)
        self.cfg = MissionConfig(grip_ball_min_pct=20.0, grip_ball_max_pct=40.0, track_period=0.1,
                                 load_retry_wait_seconds=1.0, load_leave_timeout=10.0, place_servo_gain_deg_per_px=0.04,
                                 place_seek=False)                                 # 학습한 넣기 방식 시험
        self.link = FakeLoadLink()
        self.events = []

    def build(self, **world_kw):
        self.world = World(self.link, **world_kw)
        self.arm = FakeArm(self.world)
        self.eye = FakeEye(self.world, self.arm)
        self.ops = WorldOps(self.world, self.arm, self.events)
        loader = Loader(self.cfg, self.arm, self.eye, self.link, self.book, clock=self.world.clock, sleep=self.world.sleep,
                        log=lambda event, **kw: self.events.append((event, kw)), places=self.places, ops=self.ops)
        loader.max_wait = 60
        return loader

    def names(self):
        return [e for e, _ in self.events]

    def test_full_cycle_stops_as_soon_as_car_and_box_are_seen(self):
        loader = self.build(note="red")
        loader.run(max_jobs=1)
        self.assertEqual(self.link.states(), ["WAITING", "STOP_REQUEST", "IN_PROGRESS", "SUCCESS", "WAITING"])
        self.assertNotIn("TRACKING", self.link.states())                      # 따라가지 않고 보이면 바로 정지
        self.assertEqual(self.world.loaded, "red")
        self.assertEqual(self.arm.went[-1], "watch")                          # 대기 자세로 돌아간 뒤 출발 신호
        self.assertEqual(self.link.pauses, [True, False])                     # 차 코드는 /arm/pause 로 선다 · 간다

    def test_error_away_from_wait_pose_keeps_the_car_stopped(self):
        loader = self.build(note="red")
        from mission.place_flow import FlowAbort

        def broken(*a, **k):
            self.arm.go = lambda name, **kw: False                             # 이제부터 대기 자세로 못 돌아감
            raise FlowAbort("모터 통신 끊김")
        self.ops.play_pick = broken
        with self.assertRaises(LoadAbort):
            loader.run(max_jobs=1)
        self.assertEqual(self.link.states()[-1], "IN_PROGRESS")
        self.assertEqual(self.link.pauses, [True])                            # 차는 서 있는 채로
        self.assertIn("hold_car", self.names())

    def test_failure_waits_for_the_ball_then_picks_again(self):
        loader = self.build(note="blue")
        self.ops.miss = [True]
        loader.run(max_jobs=1)
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2])
        self.assertEqual([kw["color"] for e, kw in self.events if e == "ops_pick"], ["blue", "blue"])   # 4(공 집기)부터 다시
        self.assertEqual(self.link.states()[-2], "SUCCESS")

    def test_three_failures_send_failed(self):
        loader = self.build()
        self.ops.miss = [True, True, True]
        loader.run(max_jobs=1)
        attempts = [h["attempt"] for h in self.link.history if h["state"] == "IN_PROGRESS"]
        self.assertEqual(attempts, [1, 2, 3])
        self.assertEqual(self.link.states()[-2], "FAILED")
        self.assertIsNone(self.world.loaded)

    def test_waits_for_car_power_before_going_to_the_wait_pose(self):
        loader = self.build(start=200.0)
        self.world.present = False                                             # 차 전원이 아직 꺼져 있음
        loader.wait_car_power = True
        loader.max_wait = 3
        with self.assertRaises(TimeoutError):                                  # 차가 안 켜졌으면 계속 기다린다
            loader.boot()
        self.assertEqual(self.arm.went, [])
        self.link.on_car("IDLE")                                               # 차 전원 켜짐 → 첫 상태 메시지
        loader.boot()
        self.assertEqual(self.arm.went, ["watch"])

    def test_missing_place_learning_blocks_start(self):
        self.places.delete_set(self.places.chosen_set())
        loader = self.build()
        with self.assertRaises(LoadAbort) as ctx:
            loader.run(max_jobs=1)
        self.assertIn("4번", str(ctx.exception))
        self.assertFalse(self.arm.torque)


class PauseTest(unittest.TestCase):
    """/arm/pause: 세우는 동안 True 를 1초마다, 풀면 False 를 3번만 (사람이 세운 차를 멋대로 출발시키지 않게)"""

    def test_pause_follows_hold_states(self):
        sent = []
        link = FakeLoadLink()
        link._publish_pause = sent.append
        link.set("WAITING")
        self.assertEqual(sent, [])                                             # 처음에는 아무것도 안 보냄
        link.set("STOP_REQUEST")
        link.set("IN_PROGRESS", "red", 1)
        link._pause_tick()
        self.assertEqual(sent, [True, True, True])                             # 세우는 동안은 계속 True
        link.set("SUCCESS")
        for _ in range(6):
            link._pause_tick()
        self.assertEqual(sent[3:], [False, False, False])                      # 출발은 3번만
        link.set("WAITING")
        link._pause_tick()
        self.assertEqual(len(sent), 6)


class InterfaceTest(unittest.TestCase):
    def test_message_numbers_match_team_interface(self):
        self.assertEqual(ARM_STATES.index("STOP_REQUEST"), 2)
        self.assertEqual(ARM_STATES.index("SUCCESS"), 4)
        self.assertEqual(ARM_STATES.index("FAILED"), 5)
        self.assertEqual(CAR_STATES.index("AT_LOADING_ZONE"), 3)


class YoloTest(unittest.TestCase):
    def test_decode_maps_boxes_back_to_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        _img, scale, left, top = letterbox(frame, 320)
        self.assertEqual((scale, left, top), (0.5, 0, 40))
        out = np.zeros((1, 12, 3), dtype=np.float32)
        out[0, :4, 0] = [160, 160, 40, 30]                 # 상자: 320 입력의 한가운데
        out[0, 4 + CLASS_NAMES.index("box"), 0] = 0.9
        out[0, :4, 1] = [162, 161, 40, 30]                 # 같은 상자에 겹친 네모 (정리돼야 함)
        out[0, 4 + CLASS_NAMES.index("box"), 1] = 0.6
        out[0, :4, 2] = [50, 100, 20, 20]                  # 확신이 낮은 것 (버려져야 함)
        out[0, 4 + CLASS_NAMES.index("car"), 2] = 0.2
        dets = decode(out, scale, left, top, 640, 480, conf=0.45, iou=0.5)
        self.assertEqual([d.name for d in dets], ["box"])
        self.assertAlmostEqual(dets[0].cx, 320, delta=1)
        self.assertAlmostEqual(dets[0].cy, 240, delta=1)
        self.assertAlmostEqual(dets[0].w, 80, delta=1)

    def test_decode_drops_box_covering_whole_frame(self):
        out = np.zeros((1, 12, 1), dtype=np.float32)
        out[0, :4, 0] = [160, 160, 320, 240]               # 화면 전체를 덮는 '차' (차 없는 장면에서 실제로 나온 오인식)
        out[0, 4 + CLASS_NAMES.index("car"), 0] = 0.6
        self.assertEqual(decode(out, 0.5, 0, 40, 640, 480, conf=0.45, iou=0.5), [])

    def test_track_target_prefers_box_over_car(self):
        car, box = Det("car", 0.99, 0, 0, 100, 100), Det("box", 0.6, 10, 10, 30, 30)
        self.assertIs(track_target([car, box]), box)
        self.assertIs(track_target([car]), car)
        self.assertIsNone(track_target([Det("ball_red", 0.9, 0, 0, 5, 5)]))
        self.assertIsNone(pick_best([], ("box",)))

    @unittest.skipUnless((MODELS_DIR / "arm_v1_yolo11n_320.onnx").exists(), "모델 파일이 없음")
    def test_real_model_loads_and_runs_on_blank_frame(self):
        det = Detector()
        found = det.detect(np.full((480, 640, 3), 114, dtype=np.uint8))
        self.assertIsInstance(found, list)                 # 빈 사진에서 터지지 않는다 (찾는 것은 없어야 정상)
        self.assertEqual([d for d in found if d.conf > 0.8], [])
        self.assertEqual(det.detect(None), [])


class DeskTest(unittest.TestCase):
    """16번 화면의 버튼 → 일 (가짜 팔)"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.bot = Robot("fake", speed=2000)
        self.addCleanup(self.bot.close)
        book = MotionBook(Path(self.tmp.name) / "motions.json", Path(self.tmp.name) / "backup")
        self.desk = MotionDesk(self.bot, MissionConfig(speed_slow=2000), book)

    def wait(self):
        self.desk.join(10)
        self.assertFalse(self.desk.busy())

    def test_teach_save_play_mark_choose(self):
        d = self.desk
        d.handle("torque", {"on": True})
        d.handle("point-add", {})
        self.bot.move(1, self.bot.angles()[1] + 10)
        self.bot.wait_until_settled()
        d.handle("point-add", {})
        self.assertEqual(len(d.state()["draft"]), 2)
        msg = d.handle("save", {"name": "빨간공_집기", "label": ""})
        self.assertIn("1차", msg)
        self.assertEqual(d.state()["draft"], [])
        d.handle("point-add", {})
        d.handle("save", {"name": "빨간공_집기", "label": "두번째"})
        d.handle("play", {"name": "빨간공_집기", "take": "t1"})
        self.wait()
        self.assertEqual(d.status["state"], "done", d.status)
        d.handle("mark", {"name": "빨간공_집기", "take": "t1", "ok": True})
        red = [m for m in d.state()["motions"] if m["name"] == "빨간공_집기"][0]
        self.assertEqual([t["label"] for t in red["takes"]], ["1차", "두번째"])
        self.assertEqual(red["takes"][0]["stats"]["ok"], 1)
        self.assertTrue(red["takes"][0]["recommended"])
        d.handle("choose", {"name": "빨간공_집기", "take": "t2"})
        self.assertEqual(d.book.chosen("빨간공_집기")[0], "t2")
        blue = [m for m in d.state()["motions"] if m["name"] == "파란공_집기"][0]
        self.assertEqual(blue["takes"], [])                            # 파랑은 따로

    def test_relative_take_remembers_car_reference(self):
        d = self.desk
        d.handle("point-add", {})
        self.assertIn("주의", d.handle("save", {"name": "바구니_넣기"}))  # 기준 없이 저장하면 경고
        d.handle("car-ref", {})
        d.handle("point-add", {})
        d.handle("save", {"name": "바구니_넣기"})
        self.assertIsNone(d.book.take("바구니_넣기", "t1")["ref_pan"])
        self.assertAlmostEqual(d.book.take("바구니_넣기", "t2")["ref_pan"], self.bot.angles()[1], delta=0.1)

    def test_errors_are_plain_messages(self):
        d = self.desk
        for action, data in (("save", {"name": "빨간공_집기"}), ("point-undo", {}), ("stop", {}),
                             ("play", {"name": "없는동작", "take": "t1"}), ("mark", {"name": "빨간공_집기", "take": "t9", "ok": True}),
                             ("goto", {"pose": "없는자세"}), ("모르는버튼", {})):
            with self.assertRaises(MotionError, msg=action):
                d.handle(action, data)

    def test_create_rename_delete_from_buttons(self):
        d = self.desk
        d.handle("create", {"name": "차 보기", "relative": True})
        d.handle("rename", {"name": "차 보기", "new": "바구니_보기"})
        d.handle("point-add", {})
        d.handle("save", {"name": "바구니_보기"})
        d.handle("take-rename", {"name": "바구니_보기", "take": "t1", "label": "가까이"})
        d.handle("take-delete", {"name": "바구니_보기", "take": "t1"})
        d.handle("delete", {"name": "바구니_보기"})
        self.assertNotIn("바구니_보기", [m["name"] for m in d.state()["motions"]])


if __name__ == "__main__":
    unittest.main()
