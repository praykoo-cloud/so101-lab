"""mission.hub 시험 — 허브 서버 (화면 · 보안 · 설정 저장 · 오류 안내). 실제 카메라 · 팔은 쓰지 않는다.

    python -m unittest tests.test_mission_hub
"""
import json
import re
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest import mock

import numpy as np

from mission.config import MissionConfig
import socket
import time

from mission.hub import CATEGORIES, Hub, make_server
from mission.teach_preview import LatestCamera


class StubCapture:
    """코드로 만든 영상 (진짜 카메라 아님)"""

    def read(self):
        frame = np.full((480, 640, 3), (60, 60, 60), np.uint8)
        frame[100:200, 100:200] = (30, 30, 220)
        return True, frame

    def release(self):
        pass


class StubDetector:
    """YOLO 대신: 늘 차와 상자를 하나씩 찾았다고 한다"""

    def detect(self, frame):
        from mission.yolo import Det
        return [Det("car", 0.9, 200, 150, 200, 150), Det("box", 0.9, 260, 180, 80, 60)]


class HubTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg_path = Path(self.tmp.name) / "config.json"
        # 티칭 화면은 설정을 기본 경로에 저장하므로, 시험 중에는 임시 경로로 바꿔 실제 설정 파일을 건드리지 않는다
        cfg_path = self.cfg_path

        class TmpConfig(MissionConfig):
            @classmethod
            def load(cls, path=None):
                return MissionConfig.load.__func__(cls, cfg_path)

            def save(self, path=None):
                return MissionConfig.save(self, cfg_path)
        patcher = mock.patch("mission.teach_preview.MissionConfig", TmpConfig)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.hub = Hub(config_path=self.cfg_path)
        self.server = make_server(self.hub, 0)
        self.server.timeout = 0.2
        self.base = f"http://127.0.0.1:{self.server.server_port}"
        self.stop = False

        def loop():
            while not self.stop:
                self.server.handle_request()
        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.stop = True
        self.thread.join(timeout=2)
        self.server.server_close()
        self.hub.close()
        self.tmp.cleanup()

    def get(self, path, headers=None):
        req = urllib.request.Request(self.base + path, headers=headers or {})
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.read()

    def post(self, path, data, token=True):
        headers = {"Content-Type": "application/json"}
        if token:
            headers["X-Hub-Token"] = self.hub.token
        req = urllib.request.Request(self.base + path, data=json.dumps(data).encode(), headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            body = e.read()
            return e.code, (json.loads(body) if body.startswith(b"{") else {})

    def stub_camera(self):
        self.hub.camera = LatestCamera(StubCapture(), sample_colors=True)
        self.assertTrue(self.hub.camera.ready.wait(2))

    # ── 화면 ────────────────────────────────────────────────────────────
    def test_home_shows_only_new_menu_in_colored_groups_and_no_fake_wording(self):
        status, body = self.get("/hub")
        text = body.decode()
        self.assertEqual(status, 200)
        self.assertEqual([c[0] for c in CATEGORIES], [1, 2, 3, 4, 5, 6])      # 예전 1~15번은 첫 화면에서 숨김
        for num, title, *_ in CATEGORIES:
            self.assertIn(title, text)
        for group in ("준비", "테스트", "실행", "설정"):
            self.assertIn(group, text)
        for cls in ("num g-prep", "num g-test", "num g-run", "num g-set"):    # 묶음마다 번호 색이 다르다
            self.assertIn(cls, text)
        self.assertIn(".card .num{color:#fff", text)                          # 카드 설명 글자색이 번호의 흰색을 덮지 않게 (번호가 안 보이던 결함)
        for old_title in ("구역 설정", "색상 등록", "기준색 잠금", "집기 연습 · 학습", "자동차 모양 등록", "작업 기록"):
            self.assertNotIn(old_title, text)
        for word in ("가상", "가짜", "PRACTICE", "연습용", "fake"):
            self.assertNotIn(word, text)

    def test_track_test_is_gone_and_missing_arm_is_explained(self):
        for path in ("/p/track", "/api/track/state", "/img/track.jpg"):          # 4번 차 추적 테스트는 삭제
            with self.assertRaises(urllib.error.HTTPError) as cm:
                self.get(path)
            self.assertEqual(cm.exception.code, 404, path)
        self.assertEqual(self.post("/api/track/start", {})[0], 404)
        self.hub.ensure_arm = lambda: "팔 포트가 보이지 않아요. 팔 12V 전원과 USB 케이블을 확인하세요."
        for path in ("/p/watchpose", "/p/place"):                               # 팔이 없으면 이유를 홈에 보여 준다
            status, body = self.get(path)
            self.assertIn("팔 포트가 보이지 않아요", body.decode(), path)
        self.assertEqual(self.post("/api/watch/save", {})[0], 400)                # 화면을 열기 전이면 안내
        self.assertEqual(self.post("/api/place/learn", {})[0], 400)

    def test_every_page_has_the_yolo_camera_window(self):
        for path in ("/hub", "/p/settings", "/p/yolo", "/p/run18", "/p/doctor"):
            status, body = self.get(path)
            self.assertIn("id=livecam", body.decode(), path)
        self.hub.ensure_camera = lambda: "카메라를 열지 못했어요: 시험용 문제"
        with self.assertRaises(urllib.error.HTTPError) as cm:                     # 카메라가 없으면 이유를 글로
            self.get("/img/live.jpg")
        self.assertEqual(cm.exception.code, 503)
        self.assertIn("시험용 문제", cm.exception.read().decode())
        self.hub.camera_failed_at = 0.0
        self.hub.ensure_camera = Hub.ensure_camera.__get__(self.hub)
        self.stub_camera()
        self.hub.get_detector = lambda: StubDetector()
        status, body = self.get("/img/live.jpg")
        self.assertEqual((status, body[:2]), (200, b"\xff\xd8"))                   # JPEG

    def test_live_camera_reuses_place_yolo_while_place_is_running(self):
        import time as _t

        class Eye:
            looked_at = _t.monotonic()

            def jpeg(self):
                return b"\xff\xd8place"

        class Place:
            eye = Eye()

            def busy(self):
                return True

            def stop_and_join(self, timeout=None):
                pass
        self.hub.place = Place()
        self.hub.ensure_camera = lambda: self.fail("no second YOLO while place runs")
        status, body = self.get("/img/live.jpg")
        self.assertEqual(body, b"\xff\xd8place")
        Place.eye.looked_at = _t.monotonic() - 5                                   # 오래된 사진이면 쓰지 않는다
        self.hub.live = (0.0, None)
        self.assertIsNone(self.hub.place_jpeg())

    def open_place(self):
        from mission import place_learn
        root = Path(self.tmp.name)
        for name, value in (("PLACE_FILE", root / "place.json"), ("PLACE_DIR", root / "place")):
            patcher = mock.patch.object(place_learn, name, value)                 # 실제 data/ 를 건드리지 않게
            patcher.start()
            self.addCleanup(patcher.stop)
        self.hub.allow_fake = True
        self.hub.arm_port = "fake"
        self.stub_camera()
        self.hub.get_detector = lambda: StubDetector()
        with mock.patch("mission.motions.MOTIONS_FILE", root / "motions.json"):
            status, body = self.get("/p/place")
        self.assertEqual(status, 200)
        return body.decode()

    def test_place_page_sets_and_rules(self):
        text = self.open_place()
        self.assertIn("바구니 넣기 학습", text)
        self.assertNotIn("id=livecam", text)                                      # 4번은 큰 YOLO 화면이 따로 있다
        status, state = 200, json.loads(self.get("/api/place/state")[1])
        self.assertEqual((state["busy"], state["book"]["sets"]), (False, []))
        self.assertEqual(self.post("/api/place/set-create", {"label": "기본_세트"}, token=False)[0], 403)
        status, out = self.post("/api/place/set-create", {"label": "기본_세트"})
        self.assertEqual(status, 200, out)
        sid = json.loads(self.get("/api/place/state")[1])["book"]["chosen"]
        status, out = self.post("/api/place/test", {"sid": sid, "start": 5, "color": "red", "no_ros": True})
        self.assertEqual(status, 400)
        self.assertIn("✔ 사용 회차가 없어요", out["message"])                       # 학습 전에는 테스트 못 함
        status, out = self.post("/api/place/save", {"step": 5})
        self.assertEqual(status, 400)
        self.assertIn("차례가 아니", out["message"])                               # 순서대로만 저장
        self.assertEqual(self.post("/api/place/stop", {})[0], 400)
        self.assertTrue((Path(self.tmp.name) / "place.json").exists())

    def test_place_learn_starts_and_stops_with_screen_only_link(self):
        self.open_place()
        self.post("/api/place/set-create", {"label": "a"})
        sid = json.loads(self.get("/api/place/state")[1])["book"]["chosen"]
        self.hub.place.ops.go_wait = lambda: True                                 # 가짜 팔: 대기 자세는 바로 도착
        self.hub.place.ops.angles = lambda: {j: 0.0 for j in "12345"}
        status, out = self.post("/api/place/learn", {"sid": sid, "no_ros": True})
        self.assertEqual(status, 200, out)
        self.assertIn("arm_in_use", dir(self.hub))
        self.assertIn("바구니 넣기 학습(4번)", self.hub.arm_in_use() or "")         # 다른 화면은 팔을 못 쓴다
        status, out = self.post("/api/place/stop", {})
        self.assertEqual(status, 200)
        self.hub.place.stop_and_join(5)
        state = json.loads(self.get("/api/place/state")[1])
        self.assertFalse(state["busy"])
        self.assertTrue(state["link"]["fake"])
        self.assertIsNotNone(state["book"]["sets"][0]["watch"])                   # 처음 학습하면 대기 자세 저장

    def test_tilt_setting_accepts_only_zero_or_sign(self):
        self.assertEqual(self.post("/api/settings", {"settings": {"place_servo_tilt_sign": "-1"}})[0], 200)
        self.assertEqual(MissionConfig.load(self.cfg_path).place_servo_tilt_sign, -1)
        status, out = self.post("/api/settings", {"settings": {"place_servo_tilt_sign": "2"}})
        self.assertEqual(status, 400)
        self.assertIn("0, 1, -1", out["message"])

    def test_pages_open(self):
        for path in ("/p/doctor", "/p/settings", "/p/lock", "/p/store", "/p/run9", "/p/run10"):
            status, body = self.get(path)
            self.assertEqual(status, 200, path)
            for word in ("가상", "가짜", "PRACTICE"):
                self.assertNotIn(word, body.decode(), path)

    def test_unknown_page_404(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.get("/p/nothing")
        self.assertEqual(cm.exception.code, 404)

    def test_camera_error_is_explained_not_crash(self):
        self.hub.ensure_camera = lambda: "손목 카메라 영상이 들어오지 않아요. USB 연결과 카메라 번호(설정 값 6번)를 확인하세요."
        status, body = self.get("/go/view")                       # 303 → 홈에 안내 문장
        self.assertEqual(status, 200)
        self.assertIn("손목 카메라 영상이 들어오지 않아요", body.decode())

    def test_pose_without_arm_is_explained(self):
        self.stub_camera()
        self.hub.ensure_arm = lambda: "팔 포트가 보이지 않아요. 팔 12V 전원과 USB 케이블을 확인하세요."
        status, body = self.get("/go/pose")
        self.assertIn("팔 포트가 보이지 않아요", body.decode())

    def test_view_mode_shows_camera_page_and_frame(self):
        self.stub_camera()
        status, body = self.get("/go/view")
        self.assertEqual(status, 200)
        page = body.decode()
        self.assertIn("허브로", page)                              # 허브로 돌아가는 링크
        self.assertNotIn("가상 연습", page)
        token = re.search(r'"token": "([^"]+)"', page).group(1)
        with self.assertRaises(urllib.error.HTTPError) as cm:      # 토큰 없으면 거부
            self.get("/frame")
        self.assertEqual(cm.exception.code, 403)
        status, jpg = self.get("/frame", {"X-Teaching-Token": token})
        self.assertEqual(status, 200)
        self.assertTrue(jpg.startswith(b"\xff\xd8"))               # 실제 JPEG

    def test_finish_returns_to_hub(self):
        self.stub_camera()
        page = self.get("/go/view")[1].decode()
        token = re.search(r'"token": "([^"]+)"', page).group(1)
        req = urllib.request.Request(self.base + "/action", data=json.dumps({"action": "finish"}).encode(),
                                     headers={"X-Teaching-Token": token, "Content-Type": "application/json"}, method="POST")
        urllib.request.urlopen(req, timeout=5).read()
        self.assertIsNone(self.hub.teach_cls)                      # 끝내면 허브 상태로
        self.assertIn("배달 한방", self.get("/")[1].decode())

    def test_roi_saved_through_real_teaching_flow(self):
        """구역 설정: 영상 멈추기 → 네모 → 저장 (설정 파일에 기록됨)"""
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        token = re.search(r'"token": "([^"]+)"', page).group(1)

        def act(data):
            req = urllib.request.Request(self.base + "/action", data=json.dumps(data).encode(),
                                         headers={"X-Teaching-Token": token, "Content-Type": "application/json"}, method="POST")
            return json.loads(urllib.request.urlopen(req, timeout=5).read())
        self.get("/frame", {"X-Teaching-Token": token})            # 화면을 한 번 읽어야 저장 가능 (끊김 방지 규칙)
        frozen = act({"action": "freeze"})["frozen"]
        self.assertTrue(frozen)
        out = act({"action": "save", "name": "arrival", "frozen": frozen, "rect": [100, 100, 120, 80]})
        self.assertIn("저장을 완료", out["message"])
        self.assertEqual(MissionConfig.load(self.cfg_path).roi("arrival"), [100, 100, 120, 80])   # 임시 설정 파일에 기록됨

    def test_silent_connections_and_stale_tab_do_not_block_the_page(self):
        """브라우저가 열어 둔 빈 연결(요청을 안 보냄)과 예전 탭의 요청이 있어도 화면은 바로 뜬다 ('로드 중' 멈춤 방지)"""
        silent = []
        for _ in range(4):
            s = socket.create_connection(("127.0.0.1", self.server.server_port))
            silent.append(s)                                     # 아무것도 보내지 않고 연결만 유지
        try:
            started = time.monotonic()
            status, body = self.get("/hub")
            self.assertEqual(status, 200)
            self.assertLess(time.monotonic() - started, 1.5)     # 예전 구조에서는 연결마다 3초씩 막혔다
            with self.assertRaises(urllib.error.HTTPError) as cm:   # 예전 탭의 /frame 요청도 바로 거절
                self.get("/frame", {"X-Teaching-Token": "old-token"})
            self.assertIn(cm.exception.code, (403, 404))
        finally:
            for s in silent:
                s.close()

    # ── 보안 ────────────────────────────────────────────────────────────
    def test_post_without_token_forbidden(self):
        status, _ = self.post("/api/settings", {"settings": {}}, token=False)
        self.assertEqual(status, 403)

    def test_lan_mode_accepts_ip_host_only(self):
        ip = urllib.request.Request(self.base + "/hub", headers={"Host": "192.168.0.23:8081"})
        with self.assertRaises(urllib.error.HTTPError) as ctx:          # 기본: Pi 의 IP 로는 못 연다
            urllib.request.urlopen(ip, timeout=5)
        self.assertEqual(ctx.exception.code, 403)
        self.hub.allow_lan = True
        try:
            self.assertEqual(urllib.request.urlopen(ip, timeout=5).status, 200)
            name = urllib.request.Request(self.base + "/hub", headers={"Host": "evil.example.com"})
            with self.assertRaises(urllib.error.HTTPError) as ctx:      # --lan 이어도 도메인 이름은 거절
                urllib.request.urlopen(name, timeout=5)
            self.assertEqual(ctx.exception.code, 403)
        finally:
            self.hub.allow_lan = False

    def test_non_local_host_forbidden(self):
        req = urllib.request.Request(self.base + "/hub", headers={"Host": "evil.example.com"})
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(cm.exception.code, 403)

    # ── 설정 저장 ───────────────────────────────────────────────────────
    def test_settings_saved_and_visible(self):
        status, out = self.post("/api/settings", {"settings": {"pose_tolerance_deg": "5.5", "stop_move_px": "9", "camera": "2"},
                                                  "rois": {"arrival": [10, 20, 100, 80]}})
        self.assertEqual(status, 200, out)
        cfg = MissionConfig.load(self.cfg_path)
        self.assertEqual(cfg.pose_tolerance_deg, 5.5)
        self.assertEqual(cfg.stop_move_px, 9.0)
        self.assertEqual(cfg.camera, 2)
        self.assertEqual(cfg.roi("arrival"), [10, 20, 100, 80])
        page = self.get("/p/settings")[1].decode()
        self.assertIn("5.5", page)
        self.assertIn(str(self.cfg_path), page)                    # 저장 파일 위치가 적혀 있다

    def test_bad_settings_rejected_with_reason_and_nothing_saved(self):
        cases = [
            ({"pose_tolerance_deg": "abc"}, "숫자"),
            ({"pose_tolerance_deg": "999"}, "사이"),
            ({"grip_ball_min_pct": "70", "grip_ball_max_pct": "30"}, "최소"),
            ({"temp_warn": "80", "temp_stop": "60"}, "경고 온도"),
            ({"attempts_max": "9"}, "고칠 수 없는"),                # 규정: 시도 3회는 수정 불가
            ({"nope": "1"}, "고칠 수 없는"),
        ]
        for settings, word in cases:
            status, out = self.post("/api/settings", {"settings": settings})
            self.assertEqual(status, 400, settings)
            self.assertIn(word, out["message"], settings)
        self.assertFalse(self.cfg_path.exists())                   # 아무것도 저장되지 않았다

    def test_bad_roi_rejected(self):
        for rois in ({"arrival": [600, 400, 100, 100]}, {"arrival": [-1, 0, 10, 10]}, {"arrival": [0, 0, 10, 0]}, {"zzz": [1, 2, 3, 4]}):
            status, out = self.post("/api/settings", {"settings": {}, "rois": rois})
            self.assertEqual(status, 400, rois)

    def _paths(self):
        import soarm.robot as rb
        from mission import hub as hubmod
        tmp = Path(self.tmp.name)
        self.poses, self.colors, self.lock = tmp / "poses.json", tmp / "colors.json", tmp / "colors.lock"
        self.poses.write_text(json.dumps({"wait": {"timestamp": "2026-11-09 10:00:00", "positions": {}},
                                          "above_red": {"timestamp": "2026-11-09 10:05:00", "positions": {}}}), encoding="utf-8")
        self.hub.backup_dir = tmp / "backup"
        return (mock.patch.object(rb, "POSES_FILE", self.poses), mock.patch.object(hubmod, "COLORS_FILE", self.colors),
                mock.patch.object(hubmod, "LOCK_FILE", self.lock))

    def test_delete_pose_roi_grip_with_backup(self):
        """저장된 값을 보고 → 지우고(백업 자동) → 다시 저장할 수 있다. 실제 데이터 파일은 건드리지 않는다"""
        patches = self._paths()
        with patches[0], patches[1], patches[2]:
            page = self.get("/p/settings")[1].decode()
            self.assertIn("저장된 항목 확인", page)
            self.assertIn("2026-11-09 10:00:00", page)                     # 저장 시각이 보인다
            status, out = self.post("/api/delete", {"kind": "pose", "name": "wait"})
            self.assertEqual(status, 200, out)
            left = json.loads(self.poses.read_text(encoding="utf-8"))
            self.assertNotIn("wait", left)
            self.assertIn("above_red", left)                              # 다른 자세는 그대로
            backups = list(self.hub.backup_dir.glob("poses.json.*.bak"))
            self.assertEqual(len(backups), 1)
            self.assertIn("wait", json.loads(backups[0].read_text(encoding="utf-8")))     # 지운 값이 백업에 남음
            self.assertEqual(self.post("/api/delete", {"kind": "pose", "name": "wait"})[0], 400)   # 이미 없음
            self.assertEqual(self.post("/api/delete", {"kind": "pose", "name": "evil"})[0], 400)   # 모르는 이름
            self.post("/api/settings", {"settings": {"grip_ball_min_pct": "10", "grip_ball_max_pct": "30"},
                                        "rois": {"arrival": [10, 20, 100, 80]}})
            self.assertEqual(self.post("/api/delete", {"kind": "roi", "name": "arrival"})[0], 200)
            self.assertEqual(self.post("/api/delete", {"kind": "ballgrip", "name": "ballgrip"})[0], 200)
            cfg = MissionConfig.load(self.cfg_path)
            self.assertEqual(cfg.roi("arrival"), [0, 0, 0, 0])
            self.assertEqual(cfg.grip_ball_max_pct, 0.0)
            self.assertEqual(self.post("/api/delete", {"kind": "pose", "name": "above_red"}, token=False)[0], 403)

    def test_delete_color_only_when_unlocked(self):
        from mission.vision import ColorRefs, lock_colors
        patches = self._paths()
        refs = ColorRefs()
        for name in ("note_red", "note_blue", "ball_red", "ball_blue"):
            refs.add_samples(name, [(0, 220, 220)] * 3)
        refs.save(self.colors)
        with patches[0], patches[1], patches[2]:
            self.assertEqual(self.post("/api/delete", {"kind": "color", "name": "ball_blue"})[0], 200)
            self.assertNotIn("ball_blue", ColorRefs.read(self.colors).refs)
            refs2 = ColorRefs.read(self.colors)
            refs2.add_samples("ball_blue", [(120, 220, 220)] * 3)
            refs2.save(self.colors)                                       # 다시 등록
            lock_colors(self.colors, self.lock)
            status, out = self.post("/api/delete", {"kind": "color", "name": "note_red"})
            self.assertEqual(status, 400)
            self.assertIn("잠겨", out["message"])                         # 잠긴 기준색은 못 지운다
            self.assertIn("note_red", ColorRefs.read(self.colors).refs)

    def test_delete_from_the_roi_teaching_screen_and_see_limits(self):
        """구역 설정 화면(4번) 안에서 저장값 · 오차 범위가 보이고 바로 지울 수 있다"""
        self.hub.backup_dir = Path(self.tmp.name) / "backup"
        self.post("/api/settings", {"settings": {"stop_move_px": "7"}, "rois": {"arrival": [10, 20, 100, 80]}})
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        self.assertIn("저장값 지우기", page)
        self.assertIn("오차 범위", page)
        self.assertIn("7.0픽셀", page)                                    # 설정에서 바꾼 오차 범위가 화면에 보인다
        token = re.search(r'"token": "([^"]+)"', page).group(1)

        def call(path, data=None):
            req = urllib.request.Request(self.base + path, data=None if data is None else json.dumps(data).encode(),
                                         headers={"X-Teaching-Token": token, "Content-Type": "application/json"},
                                         method="GET" if data is None else "POST")
            return json.loads(urllib.request.urlopen(req, timeout=5).read())
        state = call("/state")
        self.assertIn("arrival", state["saved"])
        self.assertIn("x=10, y=20", state["details"]["arrival"])         # 저장된 값이 읽을 수 있게 나온다
        out = call("/action", {"action": "delete", "name": "arrival"})   # 카메라 영상이 없어도 지울 수 있다
        self.assertIn("지웠어요", out["message"])
        self.assertNotIn("arrival", out["saved"])
        self.assertEqual(MissionConfig.load(self.cfg_path).roi("arrival"), [0, 0, 0, 0])
        self.assertTrue(list(self.hub.backup_dir.glob("config.json.*.bak")))      # 백업이 남았다

    def test_compare_current_pose_with_saved_wait_pose_is_read_only(self):
        """대기 자세 확인: 팔을 움직이지 않고 저장한 자세와 관절별 차이를 보여 주며, 허용 오차 안이면 '맞아요'"""
        class StubDriver:
            @staticmethod
            def get_torque(sid):
                return False

        class StubArm:
            driver = StubDriver()

            def __init__(self):
                self.now = {1: 1.0, 2: -19.0, 3: 41.0, 4: 49.0, 5: 0.5, 6: None}
                self.moves = 0

            def poses(self):
                return {"wait": {"angles": {1: 0.0, 2: -20.0, 3: 40.0, 4: 50.0, 5: 0.0}}}

            def angles(self):
                return self.now

            def gripper_percent(self):
                return 20.0

            def close(self):
                pass

            def __getattr__(self, name):                       # 움직이는 명령이 불리면 시험이 실패하게 한다
                if name in ("move", "move_all", "go_pose", "torque", "gripper", "save_pose"):
                    raise AssertionError(f"비교가 팔을 움직이려 했어요: {name}")
                raise AttributeError(name)

        arm = StubArm()
        self.hub.arm = arm
        self.stub_camera()
        page = self.get("/go/pose")[1].decode()
        self.assertIn("저장한 자세와 실시간 비교", page)
        token = re.search(r'"token": "([^"]+)"', page).group(1)

        def act(data):
            req = urllib.request.Request(self.base + "/action", data=json.dumps(data).encode(),
                                         headers={"X-Teaching-Token": token, "Content-Type": "application/json"}, method="POST")
            try:
                return 200, json.loads(urllib.request.urlopen(req, timeout=5).read())
            except urllib.error.HTTPError as error:
                return error.code, json.loads(error.read())
        status, out = act({"action": "compare", "name": "wait"})
        self.assertEqual(status, 200, out)
        self.assertIn("✔ 맞아요", out["message"])
        self.assertIn("가장 큰 차이 1.0°", out["message"])
        self.assertIn("허용 ±4°", out["message"])                          # 설정의 오차 범위가 보인다
        arm.now[2] = -10.0                                               # 어깨가 10도 어긋남
        status, out = act({"action": "compare", "name": "wait"})
        self.assertIn("✘ 조금 더 맞춰 주세요", out["message"])
        self.assertIn("2번 +10.0°", out["message"])                        # 어느 관절이 얼마나 다른지 나온다
        self.assertEqual(act({"action": "compare", "name": "above_red"})[0], 400)    # 저장 안 한 자세
        self.assertEqual(act({"action": "compare", "name": "ballgrip"})[0], 400)     # 집게도 저장 전
        self.assertEqual(act({"action": "compare", "name": "evil"})[0], 400)

    def test_compare_is_not_available_on_other_screens(self):
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        token = re.search(r'"token": "([^"]+)"', page).group(1)
        req = urllib.request.Request(self.base + "/action", data=json.dumps({"action": "compare", "name": "wait"}).encode(),
                                     headers={"X-Teaching-Token": token, "Content-Type": "application/json"}, method="POST")
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(cm.exception.code, 400)

    # ── 차 추적 설정 · 빈 도로 사진 · 방향 확인 ─────────────────────────
    def _teach_call(self, token):
        def act(data):
            req = urllib.request.Request(self.base + "/action", data=json.dumps(data).encode(),
                                         headers={"X-Teaching-Token": token, "Content-Type": "application/json"}, method="POST")
            try:
                return 200, json.loads(urllib.request.urlopen(req, timeout=5).read())
            except urllib.error.HTTPError as error:
                return error.code, json.loads(error.read())
        return act

    def test_tracking_settings_shown_saved_and_validated(self):
        page = self.get("/p/settings")[1].decode()
        self.assertIn("차 추적 사용", page)
        self.assertIn("가로 맞추기 방향", page)
        status, out = self.post("/api/settings", {"settings": {"tracking_enabled": "true", "track_pan_sign": "-1",
                                                               "track_stop_seconds": "3", "track_pan_limit_deg": "20"}})
        self.assertEqual(status, 200, out)
        cfg = MissionConfig.load(self.cfg_path)
        self.assertTrue(cfg.tracking_enabled)
        self.assertEqual(cfg.track_pan_sign, -1)
        self.assertEqual(cfg.track_stop_seconds, 3.0)
        self.assertEqual(cfg.track_pan_limit_deg, 20.0)
        for bad, word in (({"track_pan_sign": "2"}, "1 또는 -1"), ({"tracking_enabled": "maybe"}, "켜짐"),
                          ({"track_pan_limit_deg": "500"}, "사이"), ({"track_min_area": "9999", "track_max_area": "100"}, "최소 크기")):
            status, out = self.post("/api/settings", {"settings": bad})
            self.assertEqual(status, 400, bad)
            self.assertIn(word, out["message"], bad)

    def test_save_empty_road_baseline_photo(self):
        from mission import vision
        folder = Path(self.tmp.name) / "watch_baselines"
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        self.assertIn("빈 도로 기준 사진 1장 저장", page)
        act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
        with mock.patch.object(vision, "BASELINES_DIR", folder):
            self.get("/frame", {"X-Teaching-Token": re.search(r'"token": "([^"]+)"', page).group(1)})
            status, out = act({"action": "baseline"})
            self.assertEqual(status, 200, out)
            target = folder / "pan_+000.png"
            self.assertTrue(target.exists())                                        # 받침 가운데(0도) 사진
            self.assertEqual(act({"action": "baseline"})[0], 200)                  # 다시 찍으면 이전 사진은 .bak 으로
            self.assertTrue(target.with_suffix(".bak.png").exists())

    def test_baseline_only_on_the_zone_screen(self):
        self.stub_camera()
        page = self.get("/go/view")[1].decode()
        act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
        self.assertEqual(act({"action": "baseline"})[0], 400)

    def _probe_setup(self, direction):
        """받침을 +1° 돌리면 물체가 화면에서 direction*20 픽셀 움직이는 가짜 카메라 + 가짜 팔"""
        import cv2
        from mission import vision

        class Driver:
            @staticmethod
            def get_torque(sid):
                return False

        class Arm:
            driver = Driver()

            def __init__(self):
                self.now = {1: 0.0}
                self.locked = False
                self.moves = []

            def poses(self):
                return {}

            def angles(self):
                return dict(self.now)

            def torque(self, on):
                self.locked = on

            def move(self, joint, deg, speed=None):
                self.moves.append(deg)
                self.now[joint] = deg

            def wait_until_settled(self, timeout=5.0):
                return True

            def close(self):
                pass

        arm = Arm()

        class Capture:
            def __init__(self, with_object=True):
                self.with_object = with_object

            def read(self):
                frame = np.full((480, 640, 3), 120, np.uint8)
                if self.with_object:
                    x = int(300 + direction * 20 * arm.now[1])
                    frame[200:280, x:x + 100] = 30
                return True, frame

            def release(self):
                pass
        folder = Path(self.tmp.name) / "watch_baselines"
        folder.mkdir(exist_ok=True)
        cv2.imwrite(str(folder / "pan_+000.png"), np.full((480, 640, 3), 120, np.uint8))
        self.hub.arm = arm
        self.hub.camera = LatestCamera(Capture(), sample_colors=True)
        self.assertTrue(self.hub.camera.ready.wait(2))
        return arm, mock.patch.object(vision, "BASELINES_DIR", folder), Capture

    def test_track_probe_measures_direction_with_a_still_object(self):
        for direction, expected in ((-1, 1), (1, -1)):                                # + 로 돌리면 물체가 왼쪽 → 방향 +1
            arm, patch, _ = self._probe_setup(direction)
            with patch:
                page = self.get("/go/pose")[1].decode()
                self.hub.teaching.settle_seconds = 0.1                            # 새 영상 한 장이 들어올 만큼만
                act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
                self.assertIn("추적 방향 확인", page)
                status, out = act({"action": "probe_pan"})
                self.assertEqual(status, 200, out)
                self.assertEqual(MissionConfig.load(self.cfg_path).track_pan_sign, expected)
                self.assertEqual(arm.now[1], 0.0)                                   # 원래 각도로 돌아와 있다
                self.assertLessEqual(max(abs(m) for m in arm.moves), 2.0 + 1e-9)     # 2도를 넘게 돌리지 않는다
            self.hub.leave()

    def test_track_probe_refuses_without_baseline_or_object_and_does_not_move(self):
        from mission import vision
        arm, patch, Capture = self._probe_setup(-1)
        with mock.patch.object(vision, "BASELINES_DIR", Path(self.tmp.name) / "none"), \
                mock.patch.object(vision, "BASELINE_FILE", Path(self.tmp.name) / "none.png"):      # 기준 사진 없음
            page = self.get("/go/pose")[1].decode()
            act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
            status, out = act({"action": "probe_pan"})
            self.assertEqual(status, 400)
            self.assertIn("빈 도로 기준 사진", out["message"])
        self.hub.leave()
        self.hub.camera.close()
        self.hub.camera = LatestCamera(Capture(with_object=False), sample_colors=True)   # 물체가 안 보임
        self.assertTrue(self.hub.camera.ready.wait(2))
        with patch:
            page = self.get("/go/pose")[1].decode()
            act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
            status, out = act({"action": "probe_pan"})
            self.assertEqual(status, 400)
            self.assertIn("정지한 차", out["message"])
        self.assertEqual(arm.moves, [])                                              # 팔은 한 번도 움직이지 않았다

    def test_baseline_sweep_takes_photos_for_each_pan_angle_and_returns_the_arm(self):
        """받침을 좌우로 돌려 각도별 빈 도로 사진을 모두 찍고, 팔을 원래 각도로 돌려놓는다"""
        from mission import vision
        arm, patch, _ = self._probe_setup(-1)
        arm.poses = lambda: {"watch": {"angles": {1: 0.0, 2: -20.0, 3: 40.0, 4: 50.0, 5: 0.0}}}
        arm.now.update({2: -20.0, 3: 40.0, 4: 50.0, 5: 0.0})                  # 추적 시작 자세에 맞춰 둔 상태
        folder = Path(self.tmp.name) / "sweep_out"
        with mock.patch.object(vision, "BASELINES_DIR", folder):
            page = self.get("/go/pose")[1].decode()
            self.assertIn("빈 도로 기준 사진 전체 촬영", page)
            self.hub.teaching.settle_seconds = 0.0                           # 시험에서는 기다리지 않는다
            act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
            status, out = act({"action": "baseline_sweep"})
            self.assertEqual(status, 200, out)
            names = sorted(p.name for p in folder.glob("pan_*.png"))
            limit = int(self.hub.cfg().track_pan_limit_deg)
            self.assertIn("pan_+000.png", names)
            self.assertIn("pan_+024.png", names)                              # 좌우 한계(25도) 안에서 2도 간격
            self.assertIn("pan_-024.png", names)
            self.assertEqual(len(names), 2 * (limit // 2) + 1)
            self.assertEqual(arm.now[1], 0.0)                                 # 원래 각도로 돌아왔다
            self.assertLessEqual(max(abs(m) for m in arm.moves), limit + 1e-9)   # 한계를 넘게 돌리지 않았다

    def test_baseline_sweep_refuses_when_the_arm_is_not_at_the_watch_pose(self):
        from mission import vision
        arm, patch, _ = self._probe_setup(-1)
        arm.poses = lambda: {"watch": {"angles": {1: 0.0, 2: -20.0, 3: 40.0, 4: 50.0, 5: 0.0}}}
        arm.now.update({2: -5.0, 3: 40.0, 4: 50.0, 5: 0.0})                    # 어깨가 15도 어긋남
        with mock.patch.object(vision, "BASELINES_DIR", Path(self.tmp.name) / "sweep_out"):
            page = self.get("/go/pose")[1].decode()
            act = self._teach_call(re.search(r'"token": "([^"]+)"', page).group(1))
            status, out = act({"action": "baseline_sweep"})
            self.assertEqual(status, 400)
            self.assertIn("어긋나요", out["message"])
            self.assertEqual(arm.moves, [])                                    # 어긋난 채로는 팔을 움직이지 않는다

    # ── 결과 메시지 위치 · 저장된 사진 보기 · 자동차 모양 등록 ───────────
    def test_result_message_is_near_the_top_not_hidden_at_the_bottom(self):
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        self.assertLess(page.index('id="status"'), page.index("오차 범위"))       # 저장 결과가 맨 아래가 아니라 위쪽에 보인다
        self.assertLess(page.index('id="status"'), page.index('id="pose"'))      # 항목 선택 칸보다도 위

    def _photo_dirs(self):
        import cv2
        from mission import vision
        tmp = Path(self.tmp.name)
        shots, cars = tmp / "shots", tmp / "cars"
        shots.mkdir()
        cars.mkdir()
        base = np.full((480, 640, 3), 120, np.uint8)
        cv2.imwrite(str(shots / "pan_+000.png"), base)
        cv2.imwrite(str(shots / "pan_+002.png"), base)
        cv2.imwrite(str(cars / "front_1.png"), np.full((60, 80, 3), (30, 30, 200), np.uint8))
        cv2.imwrite(str(cars / "left_1.png"), np.full((60, 80, 3), (30, 200, 30), np.uint8))
        return (mock.patch.object(vision, "BASELINES_DIR", shots), mock.patch.object(vision, "CAR_VIEWS_DIR", cars),
                mock.patch.object(vision, "BASELINE_FILE", tmp / "none.png"))

    def test_saved_photos_page_shows_baselines_zones_cars_and_settings(self):
        import cv2
        patches = self._photo_dirs()
        self.post("/api/settings", {"settings": {}, "rois": {"arrival": [100, 100, 200, 100]}})
        with patches[0], patches[1], patches[2]:
            page = self.get("/p/photos")[1].decode()
            for word in ("저장된 사진", "pan_+000.png", "pan_+002.png", "등록한 우리 자동차 사진 (2장)", "앞 (1장)", "왼쪽 옆 (1장)",
                         "설정 값 한눈에", "자세 확인 오차"):
                self.assertIn(word, page)
            status, png = self.get("/img/baseline/pan_+000.png")
            self.assertEqual((status, png[:4]), (200, b"\x89PNG"))
            self.assertEqual(self.get("/img/car/front_1.png")[0], 200)
            status, overlay = self.get("/img/zones.png")                         # 구역 네모가 그려진 사진
            self.assertEqual(status, 200)
            image = cv2.imdecode(np.frombuffer(overlay, np.uint8), cv2.IMREAD_COLOR)
            self.assertFalse(np.array_equal(image[100, 150], [120, 120, 120]))   # 네모 위쪽 변이 색칠됨
            self.assertTrue(np.array_equal(image[300, 500], [120, 120, 120]))    # 바깥은 그대로

    def test_photo_routes_refuse_paths_outside_the_photo_folders(self):
        patches = self._photo_dirs()
        with patches[0], patches[1], patches[2]:
            for bad in ("/img/baseline/..%2f..%2fcar.png", "/img/baseline/../x.png", "/img/baseline/config.json", "/img/car/evil.txt",
                        "/img/car/..%5c..%5cset.png", "/img/other/front_1.png", "/img/baseline/legacy.png"):
                with self.assertRaises(urllib.error.HTTPError, msg=bad) as cm:
                    self.get(bad)
                self.assertEqual(cm.exception.code, 404, bad)

    def test_viewing_photos_in_a_new_tab_does_not_close_the_teaching_screen(self):
        patches = self._photo_dirs()
        self.stub_camera()
        page = self.get("/go/roi")[1].decode()
        token = re.search(r'"token": "([^"]+)"', page).group(1)
        self.assertIn("저장된 사진 보기", page)                                  # 티칭 화면 위쪽 링크
        with patches[0], patches[1], patches[2]:
            self.get("/p/photos")
            self.get("/p/settings")
        self.assertIsNotNone(self.hub.teach_cls)                               # 구역 설정 화면은 그대로 열려 있다
        self.assertEqual(self.get("/frame", {"X-Teaching-Token": token})[0], 200)

    def test_register_the_car_from_several_directions_view_and_delete(self):
        from mission import vision
        cars = Path(self.tmp.name) / "car_views"
        self.hub.backup_dir = Path(self.tmp.name) / "backup"
        self.stub_camera()
        with mock.patch.object(vision, "CAR_VIEWS_DIR", cars):
            page = self.get("/go/car")[1].decode()
            token = re.search(r'"token": "([^"]+)"', page).group(1)
            self.assertIn("자동차 모양 등록", page)
            for label in ("앞", "뒤", "왼쪽 옆", "오른쪽 옆", "앞 왼쪽 비스듬히", "위에서"):
                self.assertIn(label, page)                                      # 여러 방향
            act = self._teach_call(token)
            for _ in range(2):                                                   # 같은 방향(앞)을 두 장
                self.get("/frame", {"X-Teaching-Token": token})
                frozen = act({"action": "freeze"})[1]["frozen"]
                status, out = act({"action": "save", "name": "front", "frozen": frozen, "rect": [100, 100, 120, 80]})
                self.assertEqual(status, 200, out)                               # 덮어쓰기 확인 없이 쌓인다
            self.assertEqual(sorted(p.name for p in cars.glob("*.png")), ["front_1.png", "front_2.png"])
            self.get("/frame", {"X-Teaching-Token": token})
            frozen = act({"action": "freeze"})[1]["frozen"]
            self.assertEqual(act({"action": "save", "name": "side_view", "frozen": frozen, "rect": [100, 100, 120, 80]})[0], 400)   # 모르는 방향
            state = json.loads(urllib.request.urlopen(urllib.request.Request(self.base + "/state", headers={"X-Teaching-Token": token})).read())
            self.assertIn("front", state["saved"])
            self.assertIn("2장", state["details"]["front"])
            status, out = act({"action": "delete", "name": "front"})
            self.assertEqual(status, 200, out)
            self.assertEqual(list(cars.glob("*.png")), [])
            self.assertEqual(len(list(self.hub.backup_dir.glob("car_views.front.*/*.png"))), 2)     # 지운 사진은 백업 폴더에 있다

    def test_car_look_settings_exist(self):
        status, out = self.post("/api/settings", {"settings": {"track_use_car_look": "true", "track_look_min_score": "0.6"}})
        self.assertEqual(status, 200, out)
        cfg = MissionConfig.load(self.cfg_path)
        self.assertTrue(cfg.track_use_car_look)
        self.assertEqual(cfg.track_look_min_score, 0.6)
        self.assertEqual(self.post("/api/settings", {"settings": {"track_look_min_score": "2"}})[0], 400)

    def test_lock_needs_all_colors_and_unlock_needs_confirmation(self):
        status, out = self.post("/api/lock", {"action": "lock"})
        self.assertEqual(status, 400)                              # 기준색이 덜 등록됨 (또는 파일 없음)
        status, out = self.post("/api/lock", {"action": "unlock"})
        self.assertEqual(status, 400)
        self.assertIn("확인", out["message"])

    # ── 15번 집기 연습 ──────────────────────────────────────────────────
    def test_practice_page_status_and_controls_need_token(self):
        log = Path(self.tmp.name) / "practice.jsonl"
        with mock.patch("mission.practice.LOG_FILE", log):
            status, body = self.get("/p/practice")
        text = body.decode()
        self.assertEqual(status, 200)
        for word in ("집기 연습", "바구니 포스트잇 색을 카메라로 읽어서", "단계마다 확인", "정지", "아직 기록이 없어요"):
            self.assertIn(word, text)
        status, body = self.get("/api/practice/status")
        self.assertEqual(json.loads(body)["state"], "idle")
        for path in ("/api/practice/start", "/api/practice/resume", "/api/practice/stop", "/api/practice/apply-grip"):
            self.assertEqual(self.post(path, {}, token=False)[0], 403)         # 토큰 없이는 팔을 움직일 수 없다
        self.assertEqual(self.post("/api/practice/stop", {})[0], 400)          # 연습 중이 아니면 거절

    def test_practice_start_is_refused_without_camera_or_arm_and_never_moves_anything(self):
        self.hub.ensure_camera = lambda: "카메라를 열지 못했어요: 시험"
        status, out = self.post("/api/practice/start", {"color_mode": "red"})
        self.assertEqual(status, 400)
        self.assertIn("카메라", out["message"])
        self.hub.ensure_camera = lambda: None
        self.hub.ensure_arm = lambda: "팔에 연결하지 못했어요: 시험"
        status, out = self.post("/api/practice/start", {"color_mode": "red"})
        self.assertEqual(status, 400)
        self.assertIn("팔", out["message"])
        self.assertEqual(self.post("/api/practice/start", {"color_mode": "green"})[0], 400)    # 잘못된 선택

    def test_apply_grip_range_validates_and_backs_up(self):
        self.hub.backup_dir = Path(self.tmp.name) / "backup"
        self.post("/api/settings", {"settings": {"stop_move_px": "7"}})                  # 설정 파일을 먼저 만든다
        self.assertEqual(self.post("/api/practice/apply-grip", {"min": 40, "max": 20})[0], 400)
        self.assertEqual(self.post("/api/practice/apply-grip", {"min": "abc", "max": 20})[0], 400)
        status, out = self.post("/api/practice/apply-grip", {"min": 24.5, "max": 36})
        self.assertEqual(status, 200)
        cfg = MissionConfig.load(self.cfg_path)
        self.assertEqual((cfg.grip_ball_min_pct, cfg.grip_ball_max_pct), (24.5, 36.0))
        self.assertTrue(list(self.hub.backup_dir.glob("config.json.*.bak")))               # 바꾸기 전 설정이 백업됨

    def test_teaching_screens_are_blocked_while_practice_runs(self):
        class Running:
            def running(self):
                return True
        self.hub.practice = Running()
        self.assertIsNotNone(self.hub.enter("roi"))
        self.assertIn("집기 연습", self.hub.enter("pose"))                  # 숨긴 15번 대신 이름으로 안내
        self.assertIn("집기 연습", self.hub.open_motion())
        self.assertIn("집기 연습", self.hub.open_watch())
        self.hub.practice = None


if __name__ == "__main__":
    unittest.main()
