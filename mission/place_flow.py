"""상차 한 사이클의 단계들 (허브 4번 학습 · 테스트, 상차 미션 실행이 같이 쓴다).

단계 번호 (허브 4번 화면과 같다)
  2 대기 자세로 가서 차를 기다림
  3 차와 상자가 함께 보이면 정지 신호 → 완전 정차 확인
  4 포스트잇 색 읽기 → 그 색 공 집기 (3번 동작 티칭의 빨간공_집기 / 파란공_집기)
  5 바구니 위 (학습한 자세로 간 뒤, 바구니가 학습 때와 같은 자리에 보이도록 조금씩 맞춤 = 실시간 맞추기)
  6 바구니 안 (5 에서 맞춘 만큼 더해서)
  7 집게 해제 (학습한 % 만큼 열기)
  8 들어올리기 (집게를 연 채로)
  9 성공 확인 자세 → '공이 든 상자' 가 보이면 성공
  10 성공: 대기 자세 → 출발 신호(SUCCESS)      11 실패: 4 부터 다시 (3번째 실패면 FAILED)

넣기 방식 (설정)
  바구니 찾아가기 (place_seek, 기본, 학습 필요 없음): seek_place()
    시작 = 대기 자세(차 · 바구니가 보이는 자세). 시작할 때 손끝을 앞뒤 · 좌우로 조금 움직여 카메라 방향을 스스로 잰다.
    5 바구니를 화면 가운데로 → 6 가운데를 지키며 아래로 다가가 바구니가 화면의 50% 이상 → 7 공을 쥐고 있는지 보고 집게 열기
    → 8 5 cm 위로 → 9 집게가 열려 있고 '공이 든 바구니' 가 보이면 성공.
    집게 기울기: 대기 자세는 앞을 보느라 많이 기울어 있다 (Pi 실측 약 57°). 그대로 놓으면 공이 바구니 앞에 떨어지므로
    아래로 한 칸 갈 때마다 seek_tilt_step_deg 씩 세워 seek_final_tilt_deg(5°) 까지 세운 뒤 놓는다 (못 세우면 세울 수 있는 만큼).
    팔이 더 못 가는 곳이면 멈추지 않고 거기서 집게를 연다.
  아래 두 방식은 place_seek 를 끄면 쓴다 (place_use_yolo)
  YOLO 넣기 (기본): 5 = 집게를 아래로 향하고 손끝을 앞뒤 · 좌우로 옮겨 바구니를 학습한 픽셀 자리에 맞춤 (높이는 학습한 5 높이),
                    6 = 그 자리에서 학습한 깊이(5 와 6 의 높이 차)만큼 수직으로 내려감, 8 = 수직으로 다시 올라옴.
                    세트마다 한 번 '카메라 방향 재기' 가 필요하다 (measure_camera)
  저장 자세       : 학습한 모터 값 평균 + 받침 보정 (예전 방식, 카메라 방향 값이 없으면 자동으로 이 방식)

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
from .place_reach import Reach, ReachError, lean_of, tip_of, urdf_from_angles
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
    height: int = 480


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
        return Seen(dets, frame.shape[1], frame.shape[0])

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

    def move_to(self, angles: dict, timeout: float = 12.0, speed: int | None = None) -> bool:
        self.robot.move_all({int(j): float(v) for j, v in angles.items()}, speed=speed or self.cfg.speed_slow)
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
        ok, held = self.holding()
        if not ok:
            return False, held
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
        if self.cfg.grip_ball_max_pct > 0 and band_ok(self.cfg, self.ops.grip_pct()):
            return False, "집게가 아직 공을 쥐고 있어요 (놓지 못함 → 실패)"
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

    # ── YOLO 넣기 ───────────────────────────────────────────────────────
    def yolo_ready(self, learned: dict) -> bool:
        """YOLO 넣기를 쓸 수 있나 (설정이 켜져 있고, 카메라 방향 · 바구니 목표 자리가 있다)"""
        return bool(self.cfg.place_use_yolo and learned.get("cam") and learned.get("target"))

    def _box_px(self, frames: int = 3) -> tuple[float, float] | None:
        """바구니 가운데 픽셀 (여러 장 평균). 못 찾으면 None"""
        xs, ys = [], []
        for _ in range(frames * 3):
            self.check()
            seen = self.eye.look()
            box = pick_best(seen.dets, BASKET_NAMES) if seen else None
            if box is not None:
                xs.append(box.cx)
                ys.append(box.cy)
                if len(xs) >= frames:
                    break
            self.sleep(0.05)
        if not xs:
            return None
        return sum(xs) / len(xs), sum(ys) / len(ys)

    def _reach(self, learned: dict) -> Reach:
        """학습한 5 자세의 집게 기울기를 유지하는 손끝 이동기"""
        return Reach(self.ops, lean_of(urdf_from_angles(learned["poses"]["above"])))

    def holding(self) -> tuple[bool, str]:
        """집게가 아직 공을 쥐고 있나 (7 집게 해제 전 확인). 공 크기 범위를 안 쓰는 설정이면 늘 True"""
        if self.cfg.grip_ball_max_pct <= 0:
            return True, ""
        pct = self.ops.grip_pct()
        if band_ok(self.cfg, pct):
            return True, ""
        return False, f"집게가 공을 쥐고 있지 않아요 (집게 {pct}%). 바구니에 넣기 전에 공이 빠진 것 같아요"

    def measure_camera(self, learned: dict) -> dict:
        """카메라 방향 재기: 바구니 위에서 손끝을 앞뒤 · 좌우로 조금 움직여 보고 바구니가 화면에서 얼마나 움직이는지 잰다.
        반환 {"m": [[px/m, px/m], [px/m, px/m]], "z": 높이 m}. 실패하면 ReachError"""
        reach = self._reach(learned)
        self.check()
        self.ops.move_to(learned["poses"]["above"])
        tip, q = reach.now()
        d = self.cfg.place_calib_mm / 1000.0
        p0 = self._box_px()
        if p0 is None:
            raise ReachError("바구니가 카메라에 보이지 않아요. 바구니가 보이는 곳에서 다시 재세요")
        cols = []
        for axis in (0, 1):
            for step in (d, -d):                              # 한쪽으로 팔이 닿지 않으면 반대쪽으로 움직여 잰다
                moved = tip.copy()
                moved[axis] += step
                try:
                    reach.go(moved, q)
                except ReachError:
                    continue
                break
            else:
                raise ReachError("바구니 위에서 앞뒤 · 좌우로 조금도 움직일 수 없어요. 5 바구니 위 자세를 팔이 덜 뻗은 곳으로 다시 가르치세요")
            p1 = self._box_px()
            reach.go(tip, q)
            if p1 is None:
                raise ReachError("움직여 보는 동안 바구니가 화면에서 사라졌어요. 설정의 '카메라 방향 재기 거리' 를 줄이세요")
            cols.append(((p1[0] - p0[0]) / step, (p1[1] - p0[1]) / step))
        m = [[cols[0][0], cols[1][0]], [cols[0][1], cols[1][1]]]
        det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
        if abs(det) < 1e3:                                    # 거의 안 움직였다 (1 cm 에 몇 픽셀도 안 됨)
            raise ReachError("손끝을 움직여도 바구니가 화면에서 거의 안 움직였어요. 카메라 · 바구니 위치를 확인하세요")
        self.log("camera_measured", m=[[round(v) for v in row] for row in m])
        return {"m": [[round(v, 1) for v in row] for row in m], "z": round(float(tip[2]), 4)}

    def yolo_above(self, learned: dict) -> tuple[dict | None, str]:
        """5) 학습한 5 자세로 간 뒤, 집게를 아래로 향하고 손끝을 옮겨 바구니를 목표 픽셀 자리에 맞춘다.
        성공하면 (상태, 설명). 상태 = {"xyz": 맞춘 손끝 위치, "q": URDF 각도, "corr": 받침 보정}"""
        cfg = self.cfg
        reach = self._reach(learned)
        base = learned["poses"]["above"]
        target, cam = learned["target"], learned["cam"]
        m = cam["m"]
        det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
        inv = [[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]]
        self.check()
        self.ops.move_to(base)
        start, q = reach.now()
        xyz = start.copy()
        t0 = self.clock()
        missed = 0
        last = None

        def done(why):
            ok, held = self.holding()
            if not ok:
                return None, held
            corr = {j: round(v - base[j], 2) for j, v in self.ops.angles().items()}
            self.log("yolo_above_ok", why=why, xyz=[round(float(v), 3) for v in xyz])
            return {"xyz": xyz, "q": q, "corr": corr}, why

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
            last = (ex, ey)
            if abs(ex) <= cfg.place_servo_deadband_px and abs(ey) <= cfg.place_servo_deadband_px:
                return done(f"바구니 위로 맞췄어요 (가로 {ex:+.0f}, 세로 {ey:+.0f} 픽셀 차이)")
            dx = -(inv[0][0] * ex + inv[0][1] * ey) * cfg.place_yolo_gain      # 손끝을 옮길 거리 (m)
            dy = -(inv[1][0] * ex + inv[1][1] * ey) * cfg.place_yolo_gain
            size = math.hypot(dx, dy)
            limit = cfg.place_step_mm / 1000.0
            if size > limit:
                dx, dy = dx * limit / size, dy * limit / size
            goal = xyz.copy()
            goal[0] += dx
            goal[1] += dy
            if math.hypot(goal[0] - start[0], goal[1] - start[1]) * 1000 > cfg.place_max_shift_mm:
                return None, (f"바구니에 맞추려면 학습한 5 자리에서 {cfg.place_max_shift_mm:g} mm 넘게 옮겨야 해서 멈췄어요 "
                              "(차가 너무 벗어나 섰거나 카메라 방향을 다시 재야 할 수 있어요)")
            try:
                q = reach.go(goal, q)
            except ReachError:
                # 더 가면 팔이 닿지 않는다 → 여기까지 맞춘 자리에서 넣는다 (멈추지 않고 진행)
                return done(f"팔이 더 갈 수 없어 여기서 넣어요 (가로 {last[0]:+.0f}, 세로 {last[1]:+.0f} 픽셀 차이)")
            xyz = goal

    def yolo_here(self, learned: dict, start: int) -> dict:
        """6 이후부터 시작할 때: 지금 손끝을 기준으로 상태를 만든다 (6 = 지금이 바구니 위, 7 이후 = 지금이 바구니 안)"""
        tip, q = Reach(self.ops).now()
        xyz = tip.copy()
        if start >= 7:
            xyz[2] += self.depth(learned)
        corr = {j: round(v - learned["poses"]["above"][j], 2) for j, v in self.ops.angles().items()}
        return {"xyz": xyz, "q": q, "corr": corr}

    def depth(self, learned: dict) -> float:
        """학습한 5(바구니 위)와 6(바구니 안)의 손끝 높이 차 (m)"""
        return max(0.0, float(tip_of(learned["poses"]["above"])[2] - tip_of(learned["poses"]["inside"])[2]))

    def _vertical(self, learned: dict, state: dict, z_from: float, z_to: float) -> float:
        """z_from → z_to 로 곧게. 팔이 닿지 않으면 그 앞에서 멈춘다. 반환: 실제로 간 높이"""
        reach = self._reach(learned)
        n = max(1, int(self.cfg.place_down_steps))
        q = state["q"]
        reached = z_from
        for i in range(1, n + 1):
            self.check()
            goal = state["xyz"].copy()
            goal[2] = z_from + (z_to - z_from) * i / n
            try:
                q = reach.go(goal, q)
            except ReachError:
                break
            reached = float(goal[2])
        state["q"] = q
        return reached

    def yolo_down(self, learned: dict, state: dict) -> tuple[bool, str]:
        """6) 맞춘 자리에서 학습한 깊이만큼 수직으로 내려간다"""
        depth = self.depth(learned)
        floor = float(tip_of(learned["poses"]["inside"])[2]) - self.cfg.place_extra_down_mm / 1000.0
        z_top = float(state["xyz"][2])
        z_to = max(z_top - depth, floor)
        reached = self._vertical(learned, state, z_top, z_to)
        down = (z_top - reached) * 1000
        ok, held = self.holding()
        if not ok:
            return False, held
        if down < (z_top - z_to) * 1000 - 1:
            return True, f"팔이 더 내려갈 수 없어 {down:.0f} mm 만 내려갔어요 (여기서 집게를 열어요)"
        return True, f"바구니 안으로 {down:.0f} mm 곧게 내려갔어요"

    def release_here(self, learned: dict, corr: dict | None = None) -> tuple[bool, str]:
        """7) 그 자리에서 집게만 연다 (열기 전에 공을 쥐고 있는지 확인)"""
        self.check()
        ok, held = self.holding()
        if not ok:
            return False, held
        pct = learned.get("release_grip")
        if pct is None:
            pct = self.cfg.grip_open_pct
        self.ops.grip(pct)
        return True, f"집게를 {pct:g}% 로 열었어요"

    def yolo_lift(self, learned: dict, state: dict) -> tuple[bool, str]:
        """8) 집게를 연 채로 5 의 높이까지 수직으로 올라온다"""
        z_now = float(Reach(self.ops).now()[0][2])
        reached = self._vertical(learned, state, z_now, float(state["xyz"][2]))
        if reached < float(state["xyz"][2]) - 0.001:                  # 곧게 다 못 올라왔다 → 학습한 8 자세로
            pose = learned["poses"].get("lift") or learned["poses"]["above"]
            self.ops.move_to({j: pose[j] + state["corr"].get(j, 0.0) for j in JOINTS})
            return True, "곧게 다 못 올라와서 학습한 8 자세로 들어 올렸어요"
        return True, "집게를 연 채로 곧게 들어 올렸어요"

    def yolo_steps(self, learned: dict, state: dict):
        """6 · 7 · 8 · 9 (번호, 실행 함수) — YOLO 넣기"""
        return ((6, lambda: self.yolo_down(learned, state)), (7, lambda: self.release_here(learned)),
                (8, lambda: self.yolo_lift(learned, state)), (9, lambda: self.step_check(learned, state["corr"])))

    def pose_steps(self, learned: dict, corr: dict):
        """6 · 7 · 8 · 9 (번호, 실행 함수) — 저장 자세 방식"""
        return ((6, lambda: self.step_inside(learned, corr)), (7, lambda: self.step_release(learned, corr)),
                (8, lambda: self.step_lift(learned, corr)), (9, lambda: self.step_check(learned, corr)))

    # ── 바구니 찾아가 넣기 (학습 없이) ─────────────────────────────────
    def _feat(self):
        """(가로 차이 px, 세로 차이 px, 넓이 비율 0~1) — 바구니 네모를 화면 가운데와 비교. 못 찾으면 None"""
        seen = self.eye.look()
        box = pick_best(seen.dets, BASKET_NAMES) if seen else None
        if box is None:
            return None
        w, h = float(seen.width), float(getattr(seen, "height", 0) or 480)
        return box.cx - w / 2, box.cy - h / 2, (box.w * box.h) / (w * h)

    def _feat_avg(self, n: int = 3):
        got = []
        for _ in range(n * 3):
            self.check()
            f = self._feat()
            if f is not None:
                got.append(f)
                if len(got) >= n:
                    break
            self.sleep(0.05)
        if not got:
            return None
        return tuple(sum(v[i] for v in got) / len(got) for i in range(3))

    def _probe(self, reach: Reach, tip, q) -> tuple[list, float]:
        """손끝을 앞뒤 · 좌우로 조금 움직여 '손끝 1 m 당 바구니가 화면에서 움직이는 픽셀' 2×2 를 잰다. (행렬, 그때 넓이 비율)"""
        d = self.cfg.seek_probe_mm / 1000.0
        f0 = self._feat_avg()
        if f0 is None:
            raise ReachError("바구니가 카메라에 보이지 않아요 (대기 자세에서 차 위 바구니가 보여야 해요)")
        cols = []
        for axis in (0, 1):
            for step in (d, -d):                              # 한쪽으로 팔이 닿지 않으면 반대쪽으로
                moved = tip.copy()
                moved[axis] += step
                try:
                    reach.go(moved, q)
                except ReachError:
                    continue
                break
            else:
                raise ReachError("팔이 바구니 쪽으로 조금도 움직일 수 없는 자세예요. 대기 자세를 팔이 덜 뻗은 자세로 다시 저장하세요")
            f1 = self._feat_avg()
            reach.go(tip, q)
            if f1 is None:
                raise ReachError("카메라 방향을 재는 동안 바구니가 화면에서 사라졌어요. 설정의 '카메라 방향 재는 거리' 를 줄이세요")
            cols.append(((f1[0] - f0[0]) / step, (f1[1] - f0[1]) / step))
        m = [[cols[0][0], cols[1][0]], [cols[0][1], cols[1][1]]]
        if abs(m[0][0] * m[1][1] - m[0][1] * m[1][0]) < 1e3:
            raise ReachError("손끝을 움직여도 바구니가 화면에서 거의 안 움직였어요. 카메라 · 바구니가 잘 보이는지 확인하세요")
        self.log("seek_probe", m=[[round(v) for v in row] for row in m])
        return m, f0[2]

    def _seek(self, reach: Reach, q, report) -> tuple[bool, str, list]:
        """5 · 6) 바구니를 화면 가운데로 맞추고, 가운데를 지키며 아래로 다가가 바구니가 화면의 seek_area 이상이 되게"""
        cfg = self.cfg
        start, q = reach.now()
        try:
            m, area0 = self._probe(reach, start, q)
        except ReachError as e:
            report(5, False, str(e))
            return False, str(e), q
        xyz = start.copy()
        t0 = self.clock()
        missed, centered_once, stuck_xy, stuck_down, stuck_tilt = 0, False, False, False, False
        last = None                                      # 마지막으로 바구니를 본 때 (가로, 세로, 넓이, 손끝, 각도, 기울기)
        backups = 0
        final_lean = math.sin(math.radians(cfg.seek_final_tilt_deg))
        lean_step = math.sin(math.radians(cfg.seek_tilt_step_deg))

        def upright() -> bool:
            return reach.lean <= final_lean + 1e-3 or stuck_tilt

        while True:
            self.check()
            if self.clock() - t0 > cfg.seek_seconds:
                why = f"{cfg.seek_seconds:g}초 안에 바구니 위로 가지 못했어요"
                report(5 if not centered_once else 6, False, why)
                return False, why, q
            f = self._feat()
            if f is None:
                missed += 1
                if missed >= cfg.seek_lost_frames:
                    if last is not None and last[2] >= cfg.seek_lost_ok_area:
                        # 너무 가까워져 YOLO 가 바구니를 놓침 (공에 가리거나 네모가 화면을 넘음) → 이미 바구니 바로 위
                        if not centered_once:
                            report(5, True, f"바구니를 거의 가운데로 맞췄어요 (가로 {last[0]:+.0f}, 세로 {last[1]:+.0f} 픽셀)")
                        report(6, True, f"바구니가 너무 가까워 YOLO 가 놓쳤지만 바로 전에 화면의 {last[2] * 100:.0f}% 였어요 → 여기서 집게를 열어요")
                        return True, "", q
                    if last is not None and backups < 2:
                        # 바구니를 마지막으로 본 자리로 돌아가 조금 위로 → 다시 찾기
                        backups += 1
                        missed = 0
                        reach.lean = last[5]
                        back = last[3].copy()
                        back[2] += cfg.seek_down_mm / 1000.0
                        try:
                            q = reach.go(back, last[4])
                            xyz = back
                        except ReachError:
                            try:
                                q = reach.go(last[3], last[4])
                                xyz = last[3].copy()
                            except ReachError:
                                pass
                        continue
                    why = ("바구니가 카메라에 안 보여요 (쥔 공에 가렸거나 화면 밖으로 나감). "
                           "대기 자세에서 바구니가 화면에 잘 보이는지 확인하세요")
                    report(5 if not centered_once else 6, False, why)
                    return False, why, q
                self.sleep(self.cfg.track_period)
                continue
            missed = 0
            ex, ey, area = f
            last = (ex, ey, area, xyz.copy(), q, reach.lean)
            centered = abs(ex) <= cfg.seek_center_px and abs(ey) <= cfg.seek_center_px
            if centered and not centered_once:
                centered_once = True
                report(5, True, f"바구니를 화면 가운데로 맞췄어요 (가로 {ex:+.0f}, 세로 {ey:+.0f} 픽셀)")
            done = area >= cfg.seek_area and (centered or stuck_xy) and upright()
            if done or (stuck_down and (centered or stuck_xy) and upright()):
                if not centered_once:
                    report(5, True, f"가운데를 다 못 맞췄지만 진행해요 (팔이 더 못 감 · 가로 {ex:+.0f}, 세로 {ey:+.0f} 픽셀)")
                note = "" if done else " — 팔이 더 못 내려가서 여기서 집게를 열어요"
                self.log("seek_ok", area=round(area, 2), ex=round(ex), ey=round(ey), xyz=[round(float(v), 3) for v in xyz])
                report(6, True, f"바구니가 화면의 {area * 100:.0f}% 를 차지해요 (바구니 바로 위){note}")
                return True, "", q
            scale = math.sqrt(max(area, 1e-4) / max(area0, 1e-4))   # 가까워질수록 같은 거리에 화면에서 더 많이 움직인다
            if not centered and not stuck_xy:
                a, b, c, d = (v * scale for v in (m[0][0], m[0][1], m[1][0], m[1][1]))
                det = a * d - b * c
                dx = -(d * ex - b * ey) / det * cfg.seek_gain
                dy = -(-c * ex + a * ey) / det * cfg.seek_gain
                size = math.hypot(dx, dy)
                limit = cfg.seek_step_mm / 1000.0
                if size > limit:
                    dx, dy = dx * limit / size, dy * limit / size
                goal = xyz.copy()
                goal[0] += dx
                goal[1] += dy
                if math.hypot(goal[0] - start[0], goal[1] - start[1]) * 1000 > cfg.seek_max_travel_mm:
                    why = (f"바구니를 따라 옆으로 {cfg.seek_max_travel_mm:g} mm 넘게 가야 해서 멈췄어요 "
                           "(바구니가 너무 멀거나, 카메라 방향을 잘못 잰 것 같아요)")
                    report(5 if not centered_once else 6, False, why)
                    return False, why, q
                try:
                    q = reach.go(goal, q)
                    xyz = goal
                except ReachError:
                    stuck_xy = True                              # 옆으로는 더 못 감 → 아래로만
            else:
                # 가운데 → 아래로 한 칸 (집게도 조금씩 세운다). 바구니가 이미 충분히 크면 그 자리에서 집게만 세운다
                old = reach.lean
                new = max(final_lean, old - lean_step)
                goal = xyz.copy()
                if area < cfg.seek_area and not stuck_down:
                    # 목표 비율에 가까울수록 조금씩 (넓이는 거리의 제곱에 반비례 → 남은 거리 ≈ 1 - √(지금/목표))
                    left = 1.0 - math.sqrt(max(area, 1e-4) / cfg.seek_area)
                    goal[2] -= cfg.seek_down_mm / 1000.0 * max(0.15, min(1.0, 2.0 * left))
                    if (start[2] - goal[2]) * 1000 > cfg.seek_max_drop_mm:
                        stuck_down = True
                        goal = xyz.copy()
                tries = [(goal, new), (goal, old)] if goal[2] != xyz[2] else []
                if new < old - 1e-6:
                    tries.append((xyz.copy(), new))              # 내려가지 못하면 그 자리에서 세우기만
                moved = False
                for g, lean in tries:
                    reach.lean = lean
                    try:
                        q = reach.go(g, q)
                    except ReachError:
                        continue
                    if g[2] < xyz[2]:
                        stuck_xy = False                         # 높이가 바뀌면 옆으로 다시 시도
                    xyz, moved = g, True
                    break
                else:
                    reach.lean = old
                if not moved:
                    if goal[2] < xyz[2]:
                        stuck_down = True
                    if new < old - 1e-6:
                        stuck_tilt = True
                    if not tries or (stuck_down and (stuck_tilt or new >= old - 1e-6)):
                        stuck_down, stuck_tilt = True, True

    def seek_place(self, on_step=None, start: int = 5, before=None) -> tuple[bool, str]:
        """학습 없이 넣기 (5~9). 지금 자세(보통 대기 자세)에서 시작. on_step(번호, 성공?, 설명), before(번호) = 단계 전에 부름"""
        cfg = self.cfg
        report = on_step or (lambda *a: None)
        wait = before or (lambda n: None)
        reach = Reach(self.ops, lean_of(urdf_from_angles(self.ops.angles())), speed=cfg.seek_speed)
        lean0 = reach.lean                                    # 시작(대기) 자세의 기울기
        q = reach.now()[1]
        if start <= 6:
            wait(5)
            ok, why, q = self._seek(reach, q, report)
            if not ok:
                return False, why
        if start <= 7:
            wait(7)
            self.check()
            ok, why = self.holding()
            if not ok:
                report(7, False, why)
                return False, why
            self.ops.grip(cfg.grip_open_pct)
            self.pause(cfg.seek_drop_seconds)
            report(7, True, f"집게를 {cfg.grip_open_pct:g}% 로 열었어요")
        if start <= 8:
            wait(8)
            tip, q = reach.now()
            n = 2
            up = 0.0
            step_lean = math.sin(math.radians(cfg.seek_tilt_step_deg))
            for i in range(1, n + 1):
                self.check()
                goal = tip.copy()
                goal[2] += cfg.seek_lift_mm / 1000.0 * i / n
                lean = reach.lean
                while True:                                   # 곧게 못 올라가면 집게를 조금씩 다시 기울이며 (대기 자세 기울기까지)
                    reach.lean = lean
                    try:
                        q = reach.go(goal, q)
                        up = cfg.seek_lift_mm * i / n
                        break
                    except ReachError:
                        if lean >= lean0 - 1e-6:
                            lean = None
                            break
                        lean = min(lean0, lean + step_lean)
                if lean is None:
                    break
            report(8, True, f"집게를 연 채로 {up:.0f} mm 위로 올라왔어요" + ("" if up >= cfg.seek_lift_mm - 0.5 else " (팔이 더 못 올라감)"))
        wait(9)
        self.check()
        if cfg.grip_ball_max_pct > 0 and band_ok(cfg, self.ops.grip_pct()):
            why = "집게가 아직 공을 쥐고 있어요 (놓지 못함 → 실패)"
            report(9, False, why)
            return False, why
        ok = self.basket_has_ball()
        why = "공이 든 바구니가 보여요 (성공)" if ok else "공이 든 바구니가 안 보여요 (실패)"
        report(9, ok, why)
        return ok, why

    def auto_place(self, learned: dict, on_step=None) -> tuple[bool, str]:
        """5 → 6 → 7 → 8 → 9 자동. on_step(번호, 성공?, 설명) 을 단계마다 부른다"""
        if self.yolo_ready(learned):
            state, why = self.yolo_above(learned)
            steps = None if state is None else self.yolo_steps(learned, state)
        else:
            state, why = self.step_above(learned)
            steps = None if state is None else self.pose_steps(learned, state)
        if on_step:
            on_step(5, state is not None, why)
        if state is None:
            return False, why
        for num, fn in steps:
            ok, why = fn()
            if on_step:
                on_step(num, ok, why)
            if not ok:
                return False, why
        return True, why
