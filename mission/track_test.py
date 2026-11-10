"""차 추적 테스트: 팔이 차(상자)를 쳐다보며 받침(1번 관절)만 돌려 따라가는 것만 시험한다.

허브 4번 화면이 이 부품을 쓴다. **정지 신호 · 출발 신호 · 집기 · 넣기는 하지 않는다** (차에게 아무 신호도 보내지 않는다).
움직이는 관절은 받침(1번) 하나뿐이고, 처음 자리에서 좌우 한계(설정 track_pan_limit_deg) 밖으로는 절대 안 간다.

    test = TrackTest(cfg, arm, eye, sign=1)         # eye.look() 이 지금 사진에서 찾은 것을 돌려준다
    test.start()                                     # 추적 시작 (스레드)
    test.status()                                    # 화면에 보여 줄 숫자들
    test.stop()                                      # 멈춘다: 팔은 그 자리에 힘을 준 채 선다

보는 것
- 찾은 대상(상자가 보이면 상자, 아니면 차), 화면 가운데에서 벗어난 정도(픽셀), 받침이 처음에서 몇 도 돌았나
- 놓친 횟수, 평균 벗어남 → 방향(track_pan_sign)과 세기(track_gain_deg_per_px)를 맞추는 데 쓴다

고칠 숫자: 허브 6번 설정의 '차 추적' 묶음 (돌리는 세기, 가운데 허용 폭, 한 번에 최대 각도, 좌우 한계 …)

단어장
- 받침(1번 관절) : 팔 맨 아래에서 좌우로 도는 모터
- 가운데 허용 폭 : 대상이 화면 가운데에서 이 안(픽셀)이면 안 돌린다 (떨림 방지)
- 놓침           : 대상이 설정 시간(track_lost_seconds) 동안 안 보여서 처음 자리로 돌아가는 것
"""
from __future__ import annotations

import threading
import time

from .loading import Seen
from .yolo import draw, track_target

HINT_WORSE_TICKS = 4          # 받침을 돌렸는데도 이 횟수만큼 연달아 더 벗어나면 "방향이 반대일 수 있다" 안내


def idle_status(cfg) -> dict:
    """아직 시작하지 않았을 때 화면에 보여 줄 값"""
    return {"state": "idle", "message": "아직 시작하지 않았어요.", "found": False, "target": None, "label": None, "conf": None,
            "error_px": None, "centered": False, "pan": None, "pan_from_center": None, "at_limit": False, "frames": 0,
            "found_frames": 0, "lost_events": 0, "mean_abs_error_px": None,
            "sign": -1 if cfg.track_pan_sign < 0 else 1, "limit": float(cfg.track_pan_limit_deg),
            "deadband": float(cfg.track_deadband_px), "hint": "", "camera_ok": True}


class TrackAbort(RuntimeError):
    """과열 · 받침 각도를 못 읽음 등. 추적을 멈춘다"""


class SnapEye:
    """허브 카메라(LatestCamera) + YOLO. look() = 지금 사진에서 찾은 것. render() = 찾은 것을 그린 마지막 사진(JPEG)"""

    def __init__(self, camera, detector):
        self.camera, self.detector = camera, detector
        self.last_frame = None
        self.last_dets: list = []
        self.width = 640

    def look(self) -> Seen | None:
        sample = self.camera.snapshot()
        if sample is None:
            return None
        frame = sample[1]
        self.last_frame, self.last_dets, self.width = frame, self.detector.detect(frame), frame.shape[1]
        return Seen(self.last_dets, self.width)

    def render(self, deadband_px: float = 0.0) -> bytes | None:
        import cv2
        if self.last_frame is None:
            return None
        img = draw(self.last_frame, self.last_dets)
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)                      # 화면 가운데
        if deadband_px > 0:
            for x in (int(w / 2 - deadband_px), int(w / 2 + deadband_px)):           # 가운데 허용 폭
                cv2.line(img, (x, 0), (x, h), (0, 200, 200), 1)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else None


class TrackTest:
    def __init__(self, cfg, arm, eye, sign: int | None = None, go_start: bool = False,
                 clock=time.monotonic, sleep=time.sleep, log=None):
        self.cfg, self.arm, self.eye = cfg, arm, eye
        self.sign = -1 if (cfg.track_pan_sign if sign is None else sign) < 0 else 1
        self.go_start = go_start
        self.clock, self.sleep = clock, sleep
        self._log = log or (lambda event, **kw: print(f"[{time.strftime('%H:%M:%S')}] {event} {kw if kw else ''}", flush=True))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.center: float | None = None
        self.limit = float(cfg.track_pan_limit_deg)
        self._lost_since: float | None = None
        self._returning = False
        self._abs_sum = 0.0
        self._prev_error: float | None = None
        self._prev_moved = False
        self._worse = 0
        self._lock = threading.Lock()
        self._status = {**idle_status(cfg), "sign": self.sign}

    # ── 상태 ────────────────────────────────────────────────────────────
    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _set(self, **changes) -> None:
        with self._lock:
            self._status.update(changes)

    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ── 한 번 보고 한 번 돌리기 ─────────────────────────────────────────
    def prepare(self) -> None:
        """시작 준비: 지금 자리에서 힘을 주고(튀지 않음) 받침의 처음 각도를 기억한다. go_start 면 대기 자세로 먼저 간다"""
        self.arm.torque_on()
        if self.go_start and not self.arm.go(self.cfg.load_wait_pose):
            raise TrackAbort(f"대기 자세('{self.cfg.load_wait_pose}')로 가지 못했어요")
        self.center = self.arm.pan_angle()
        if self.center is None:
            raise TrackAbort("받침(1번 관절) 각도를 읽지 못했어요")

    def tick(self) -> None:
        cfg = self.cfg
        if self.arm.too_hot():
            raise TrackAbort("모터 과열")
        seen = self.eye.look()
        now = self.clock()
        pan = self.arm.pan_angle()
        if pan is None:
            raise TrackAbort("받침 각도를 읽지 못했어요")
        st = self.status()
        frames = st["frames"] + 1
        self._set(frames=frames, pan=round(pan, 1), pan_from_center=round(pan - self.center, 1),
                  at_limit=abs(pan - self.center) >= self.limit - 0.2, camera_ok=seen is not None)
        target = track_target(seen.dets) if seen else None
        if target is None:                                                   # 대상이 안 보임
            self._prev_error = None
            self._worse = 0
            self._set(found=False, target=None, label=None, conf=None, error_px=None, centered=False,
                      message="영상이 끊겼어요." if seen is None else "차(상자)가 안 보여요. 찾는 중…")
            self._lost_since = self._lost_since if self._lost_since is not None else now
            if now - self._lost_since >= cfg.track_lost_seconds:
                if not self._returning:                                      # 놓쳤다 → 처음 자리로 천천히 돌아간다
                    self._returning = True
                    self._set(lost_events=self.status()["lost_events"] + 1)
                    self._log("lost", msg="대상을 놓쳤어요 → 처음 자리로")
                delta = max(-cfg.track_max_step_deg, min(cfg.track_max_step_deg, self.center - pan))
                if abs(delta) > 0.2:
                    self.arm.nudge_pan(delta, self.center, self.limit)
            return
        self._lost_since = None
        self._returning = False
        from .yolo import LABELS
        error = target.cx - seen.width / 2                                  # +: 대상이 화면 오른쪽
        self._abs_sum += abs(error)
        found_frames = st["found_frames"] + 1
        centered = abs(error) <= cfg.track_deadband_px
        self._set(found=True, target=target.name, label=LABELS.get(target.name, target.name), conf=round(target.conf, 2),
                  error_px=round(error), centered=centered, found_frames=found_frames,
                  mean_abs_error_px=round(self._abs_sum / found_frames, 1),
                  message="가운데에 맞췄어요." if centered else ("오른쪽에 있어서 따라가는 중…" if error > 0 else "왼쪽에 있어서 따라가는 중…"))
        moved = False
        if not centered:
            step = self.sign * cfg.track_gain_deg_per_px * error
            step = max(-cfg.track_max_step_deg, min(cfg.track_max_step_deg, step))
            before = pan
            after = self.arm.nudge_pan(step, self.center, self.limit)
            moved = after is not None and abs(after - before) > 0.05
        # 돌렸는데도 연달아 더 벗어나면 방향이 반대일 수 있다는 안내 (차가 빠르게 움직이는 중일 수도 있다)
        if self._prev_moved and self._prev_error is not None and abs(error) > abs(self._prev_error) + 5:
            self._worse += 1
        elif moved or centered:
            self._worse = 0
        self._prev_error, self._prev_moved = error, moved
        hint = ""
        if self._worse >= HINT_WORSE_TICKS:
            hint = "받침을 돌릴수록 더 벗어나요. 차가 빨리 움직이는 중이거나 방향이 반대일 수 있어요 → '방향 뒤집기'를 눌러 보세요."
        elif self.status()["at_limit"] and not centered:
            hint = "받침이 좌우 한계에 닿았어요 (설정의 '좌우 한계'). 더 돌지 않습니다."
        self._set(hint=hint)

    # ── 스레드 ──────────────────────────────────────────────────────────
    def start(self) -> None:
        if self.running():
            raise RuntimeError("이미 추적 중이에요.")
        self._stop.clear()
        self._set(state="starting", message="준비 중…")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            self.prepare()
            self._set(state="running", message="차(상자)를 찾는 중…")
            while not self._stop.is_set():
                self.tick()
                self.sleep(self.cfg.track_period)
            self._set(state="done", message="멈췄어요. 팔은 그 자리에서 힘을 주고 있어요.")
        except Exception as error:                                           # 과열 · 통신 끊김 · 토크 꺼짐 등 → 그 자리에서 멈춘다
            self._set(state="error", message=f"문제가 생겨 멈췄어요: {error}")
        finally:
            try:
                self.arm.hold()                                              # 목표 = 지금 위치 (공중에서 힘을 빼지 않는다)
            except Exception:
                pass

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float = 10.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout)
