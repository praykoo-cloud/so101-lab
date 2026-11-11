"""상차 한 사이클의 단계들 (허브 4번 학습 · 테스트, 상차 미션 실행이 같이 쓴다).

단계 번호 (허브 4번 화면과 같다)
  2 대기 자세로 가서 차를 기다림
  3 차와 상자가 함께 보이면 정지 신호 → 완전 정차 확인
  4 포스트잇 색 읽기 → 그 색 공 집기 (3번 동작 티칭의 빨간공_집기 / 파란공_집기)
  5 바구니 위 (학습한 자세로 간 뒤, 바구니가 학습 때와 같은 자리에 보이도록 조금씩 맞춤 = 실시간 맞추기)
  6 바구니 안 (5 에서 맞춘 만큼 더해서)
  7 집게 해제 (학습한 % 만큼 열기)
  8 성공 확인 자세 → '공이 든 상자' 가 보이면 성공
  9 성공: 대기 자세 → 출발 신호(SUCCESS)      10 실패: 4 부터 다시 (3번째 실패면 FAILED)

고칠 숫자: 허브 6번 설정의 '바구니 넣기 (4번)' 묶음 (맞추기 세기 · 허용 폭 · 최대 보정 각도 …)

단어장
- 보정(correction) : 학습한 자세에서 실제로 더 움직인 각도. 6 · 7 · 8 에도 그만큼 더한다
- 연속 N장         : 사진 N장이 끊기지 않고 계속 같은 결과일 때만 인정 (한 장 잘못 본 것에 속지 않게)
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from .m3_pick_place import band_ok
from .motions import MotionError, play
from .yolo import BASKET_NAMES, pick_best

PICK_MOTION = {"red": "빨간공_집기", "blue": "파란공_집기"}
COLOR_MIN_HITS = 3
JOINTS = ("1", "2", "3", "4", "5")


class FlowStopped(RuntimeError):
    """사람이 '정지' 를 눌렀다"""


class FlowAbort(RuntimeError):
    """하드웨어 문제 (과열 · 각도 못 읽음). 더 진행하면 안 된다"""


@dataclass
class Seen:
    dets: list
    width: int


class YoloEye:
    """카메라 한 장 + YOLO. look() = 지금 사진에서 찾은 것, jpeg() = 찾은 것을 그린 마지막 사진.

    read_frame : 사진 한 장을 주는 함수 (없으면 None)
    det_lock   : YOLO 를 여러 스레드가 동시에 쓰지 않게 (허브 카메라 창과 함께 쓸 때)
    """

    def __init__(self, read_frame, detector, det_lock=None):
        self.read_frame, self.detector = read_frame, detector
        self.det_lock = det_lock or threading.Lock()
        self.last_frame = None
        self.last_dets: list = []
        self.looked_at: float | None = None      # 마지막으로 YOLO 를 돌린 시각 (허브 카메라 창이 이 사진을 같이 쓴다)
        self._jpeg = None

    def look(self) -> Seen | None:
        frame = self.read_frame()
        if frame is None:
            return None
        with self.det_lock:
            dets = self.detector.detect(frame)
        self.last_frame, self.last_dets, self._jpeg = frame, dets, None
        self.looked_at = time.monotonic()
        return Seen(dets, frame.shape[1])

    def jpeg(self) -> bytes | None:
        if self._jpeg is None and self.last_frame is not None:
            import cv2
            from .yolo import draw
            ok, buf = cv2.imencode(".jpg", draw(self.last_frame, self.last_dets))
            self._jpeg = buf.tobytes() if ok else None
        return self._jpeg

    def basket(self) -> dict | None:
        """마지막 사진에서 가장 확실한 바구니(상자) 하나"""
        box = pick_best(self.last_dets, BASKET_NAMES)
        return None if box is None else {"name": box.name, "conf": round(box.conf, 3), "cx": round(box.cx, 1),
                                         "cy": round(box.cy, 1), "w": round(box.w, 1), "h": round(box.h, 1)}


class ArmOps:
    """팔에게 시키는 일을 한곳에 (시험에서는 이것만 가짜로 바꿔 끼운다). arm = ArmIO"""

    def __init__(self, arm, cfg):
        self.arm, self.cfg = arm, cfg
        self.robot = arm.robot

    def too_hot(self) -> bool:
        return self.arm.too_hot()

    def go_wait(self) -> bool:
        self.robot.torque(True)                      # 지금 자리에서 잠근 뒤 간다 (튀지 않음)
        return self.arm.go(self.cfg.load_wait_pose)

    def angles(self) -> dict:
        now = self.robot.angles()
        if any(now.get(int(j)) is None for j in JOINTS):
            raise FlowAbort("관절 각도를 읽지 못했어요. 전원과 케이블을 확인하세요.")
        return {j: float(now[int(j)]) for j in JOINTS}

    def grip_pct(self):
        return self.robot.gripper_percent()

    def move_to(self, angles: dict, timeout: float = 12.0) -> bool:
        self.robot.move_all({int(j): float(v) for j, v in angles.items()}, speed=self.cfg.speed_slow)
        return self.robot.wait_until_settled(timeout=timeout)

    def nudge(self, joint: str, deg: float) -> None:
        self.robot.move(int(joint), deg, speed=self.cfg.speed_slow)
        self.robot.wait_until_settled(timeout=3.0)

    def grip(self, pct: float) -> None:
        self.arm.grip(pct)

    def torque_arm(self, on: bool) -> None:
        """팔(1~5번)만 힘 주기 / 빼기. 집게(6번)는 그대로 → 쥔 공이 떨어지지 않는다"""
        for j in JOINTS:
            self.robot.torque(bool(on), int(j))

    def hold(self) -> None:
        self.robot.stop()

    def play_pick(self, color: str, book, stop) -> dict:
        _tid, take = book.chosen(PICK_MOTION[color])
        self.robot.torque(True)
        return play(self.robot, take["points"], speed=self.cfg.speed_slow, stop=stop, too_hot=self.arm.too_hot)


class PlaceRunner:
    """단계 하나하나를 실행한다. 판단 · 순서는 여기, 팔 · 카메라 · 통신은 바꿔 끼울 수 있다"""

    def __init__(self, cfg, ops, eye, link, motions, clock=time.monotonic, sleep=time.sleep, log=None, stop=None):
        self.cfg, self.ops, self.eye, self.link, self.motions = cfg, ops, eye, link, motions
        self.clock, self.sleep = clock, sleep
        self._log = log or (lambda event, **kw: print(f"[{time.strftime('%H:%M:%S')}] {event} {kw if kw else ''}", flush=True))
        self.stop_event = stop or threading.Event()
        self.max_wait: float | None = None           # 시험용: 차를 기다리는 최대 시간 (None = 계속)

    def log(self, event, **kw):
        self._log(event, **kw)

    def check(self) -> None:
        if self.stop_event.is_set():
            raise FlowStopped("정지 버튼")
        if self.ops.too_hot():
            raise FlowAbort("모터 과열")

    def pause(self, seconds: float) -> None:
        """기다리는 동안에도 정지 버튼을 본다"""
        end = self.clock() + seconds
        while self.clock() < end:
            self.check()
            self.sleep(min(0.2, max(0.0, end - self.clock())))

    # ── 2 · 3) 차가 오면 바로 정지 신호 ─────────────────────────────────
    def wait_car_and_stop(self) -> bool:
        """차 + 상자가 연속 N장 보이면 STOP_REQUEST, 상자가 정해진 시간 동안 안 움직이면 True (완전 정차).
        정지 신호를 냈는데 시간 안에 안 서면 False"""
        cfg = self.cfg
        need = max(1, int(cfg.load_see_frames))
        t0 = self.clock()
        streak = 0
        stop_at = anchor = still_since = None
        while True:
            self.check()
            now = self.clock()
            if stop_at is None and self.max_wait is not None and now - t0 > self.max_wait:
                raise TimeoutError("자동차가 오지 않았어요 (시험용 시간 제한)")
            if stop_at is not None and now - stop_at > cfg.load_stop_timeout:
                self.log("stop_timeout", msg="정지 신호를 냈지만 완전 정차를 확인하지 못했어요")
                return False
            seen = self.eye.look()
            car = pick_best(seen.dets, ("car",)) if seen else None
            box = pick_best(seen.dets, BASKET_NAMES) if seen else None
            if stop_at is None:
                streak = streak + 1 if (car is not None and box is not None) else 0
                if streak >= need:
                    stop_at = now
                    self.link.set("STOP_REQUEST", "", 0)
                    self.log("stop_request", frames=streak)
            else:
                target = box or car
                if target is None:
                    anchor = still_since = None
                elif anchor is None or math.hypot(target.cx - anchor[0], target.cy - anchor[1]) > cfg.track_stop_px:
                    anchor, still_since = (target.cx, target.cy), now
                elif now - still_since >= cfg.load_stop_confirm_seconds:
                    self.log("car_stopped", x=round(target.cx), y=round(target.cy))
                    return True
            # 차를 기다리는 동안은 짧게 쉬어 빨리 알아챈다. 정지 신호 뒤(정차 확인)는 원래 주기
            self.sleep(cfg.load_see_period if stop_at is None else cfg.track_period)

    # ── 4) 색 · 공 집기 ─────────────────────────────────────────────────
    def read_color(self) -> str | None:
        counts = {"red": 0, "blue": 0}
        weight = {"red": 0.0, "blue": 0.0}
        for _ in range(max(1, self.cfg.load_color_frames)):
            self.check()
            seen = self.eye.look()
            note = pick_best(seen.dets, ("postit_red", "postit_blue")) if seen else None
            if note is not None:
                color = note.name.split("_")[1]
                counts[color] += 1
                weight[color] += note.conf
            self.sleep(0.05)
        self.log("color_vote", counts=counts)
        if counts["red"] + counts["blue"] < COLOR_MIN_HITS:
            return None
        if counts["red"] != counts["blue"]:
            return "red" if counts["red"] > counts["blue"] else "blue"
        return "red" if weight["red"] >= weight["blue"] else "blue"

    def pick(self, color: str) -> tuple[bool, str]:
        self.check()
        try:
            rep = self.ops.play_pick(color, self.motions, self.stop_event)
        except MotionError as e:
            return False, str(e)
        self.log("pick", color=color, reached=rep.get("reached"), why=rep.get("why"))
        if rep.get("stopped"):
            self.check()
            return False, f"공 집기가 도중에 멈췄어요 ({rep.get('why')})"
        if self.cfg.grip_ball_max_pct > 0:
            pct = self.ops.grip_pct()
            if not band_ok(self.cfg, pct):
                return False, f"집게가 공 크기에서 멈추지 않았어요 ({pct}%). 공을 못 잡은 것 같아요"
        return True, "공을 집었어요"

    # ── 5) 바구니 위: 학습 자세로 간 뒤 실시간 맞추기 ──────────────────
    def step_above(self, learned: dict) -> tuple[dict | None, str]:
        """성공하면 (보정 각도, 설명). 실패하면 (None, 이유)"""
        cfg = self.cfg
        base = learned["poses"]["above"]
        self.check()
        self.ops.move_to(base)
        target = learned.get("target")
        if target is None:
            return self._correction(base), "학습한 바구니 위치가 없어 맞추기 없이 학습 자세 그대로 갔어요"
        t0 = self.clock()
        pan_sign = 1 if cfg.track_pan_sign >= 0 else -1
        tilt_sign = int(cfg.place_servo_tilt_sign)
        missed = 0
        while True:
            self.check()
            if self.clock() - t0 > cfg.place_servo_seconds:
                return None, f"{cfg.place_servo_seconds:g}초 안에 바구니 위로 맞추지 못했어요"
            seen = self.eye.look()
            box = pick_best(seen.dets, BASKET_NAMES) if seen else None
            if box is None:
                missed += 1
                if missed >= 10:
                    return None, "바구니(상자)가 카메라에 보이지 않아요"
                self.sleep(cfg.track_period)
                continue
            missed = 0
            ex, ey = box.cx - target["cx"], box.cy - target["cy"]
            x_ok = abs(ex) <= cfg.place_servo_deadband_px
            y_ok = tilt_sign == 0 or abs(ey) <= cfg.place_servo_deadband_px
            if x_ok and y_ok:
                corr = self._correction(base)
                self.log("above_ok", ex=round(ex), ey=round(ey), corr=corr)
                return corr, f"바구니 위로 맞췄어요 (가로 {ex:+.0f}, 세로 {ey:+.0f} 픽셀 차이)"
            now = self.ops.angles()
            moves = []
            if not x_ok:
                moves.append(("1", pan_sign * cfg.place_servo_gain_deg_per_px * ex))
            if not y_ok:
                moves.append(("2", tilt_sign * cfg.place_servo_gain_deg_per_px * ey))
            for joint, step in moves:
                step = max(-cfg.track_max_step_deg, min(cfg.track_max_step_deg, step))
                goal = now[joint] + step
                if abs(goal - base[joint]) > cfg.place_servo_max_deg:
                    return None, (f"바구니에 맞추려면 {joint}번 관절을 학습 자세보다 {cfg.place_servo_max_deg:g}도 넘게 "
                                  "움직여야 해서 멈췄어요 (차가 너무 벗어나 섰거나 방향 설정이 반대일 수 있어요)")
                self.ops.nudge(joint, goal)

    def _correction(self, base: dict) -> dict:
        now = self.ops.angles()
        return {j: round(now[j] - base[j], 2) for j in JOINTS}

    # ── 6 · 7 · 8 · 9) 보정한 만큼 더해서 이동 ─────────────────────────
    def step_inside(self, learned: dict, corr: dict) -> tuple[bool, str]:
        self.check()
        goal = {j: learned["poses"]["inside"][j] + corr.get(j, 0.0) for j in JOINTS}
        ok = self.ops.move_to(goal)
        return True, "바구니 안으로 넣었어요" + ("" if ok else " (목표에 다 못 갔어요: 걸렸을 수 있어요)")

    def step_release(self, learned: dict, corr: dict) -> tuple[bool, str]:
        self.check()
        goal = {j: learned["poses"]["release"][j] + corr.get(j, 0.0) for j in JOINTS}
        self.ops.move_to(goal)
        pct = learned.get("release_grip")
        if pct is None:
            pct = self.cfg.grip_open_pct
        self.ops.grip(pct)
        return True, f"집게를 {pct:g}% 로 열었어요"

    def step_lift(self, learned: dict, corr: dict) -> tuple[bool, str]:
        """8 들어올리기: 집게를 연 채로 학습한 자세로 들어 올린다 (바구니에서 집게를 빼기)"""
        self.check()
        pose = learned["poses"].get("lift") or learned["poses"]["above"]
        goal = {j: pose[j] + corr.get(j, 0.0) for j in JOINTS}
        ok = self.ops.move_to(goal)
        return True, "집게를 연 채로 들어 올렸어요" + ("" if ok else " (목표에 다 못 갔어요: 걸렸을 수 있어요)")

    def step_check(self, learned: dict, corr: dict) -> tuple[bool, str]:
        self.check()
        goal = dict(learned["poses"]["check"])
        goal["1"] = goal["1"] + corr.get("1", 0.0)               # 확인 자세는 받침 보정만
        self.ops.move_to(goal)
        ok = self.basket_has_ball()
        return ok, "공이 든 바구니가 보여요 (성공)" if ok else "공이 든 바구니가 안 보여요 (실패)"

    def basket_has_ball(self) -> bool:
        hits = 0
        t0 = self.clock()
        while self.clock() - t0 < self.cfg.load_verify_seconds:
            self.check()
            seen = self.eye.look()
            if seen and pick_best(seen.dets, ("success_red", "success_blue")) is not None:
                hits += 1
                if hits >= self.cfg.load_verify_hits:
                    return True
            self.sleep(0.1)
        return False

    def auto_place(self, learned: dict, on_step=None) -> tuple[bool, str]:
        """5 → 6 → 7 → 8 → 9 자동. on_step(번호, 성공?, 설명) 을 단계마다 부른다"""
        corr, why = self.step_above(learned)
        if on_step:
            on_step(5, corr is not None, why)
        if corr is None:
            return False, why
        for num, fn in ((6, self.step_inside), (7, self.step_release), (8, self.step_lift), (9, self.step_check)):
            ok, why = fn(learned, corr)
            if on_step:
                on_step(num, ok, why)
            if not ok:
                return False, why
        return True, why
