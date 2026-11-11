"""상차 미션 (팔이 차를 세우고 출발시키는 방식). 팀 레포 docs/interfaces.md (2026-11-10) 의 흐름을 따른다.

실행 (Pi, 허브를 끈 뒤):
    .venv/bin/python -m mission.loading --check       # 준비가 됐는지만 확인 (팔 안 움직임)
    .venv/bin/python -m mission.loading --no-ros      # 차 통신 없이 연습 (신호는 화면에만 출력)
    ./상차_도커.sh                                     # 진짜: Docker 안에서 ROS2 로 차와 통신
    .venv/bin/python -m mission.loading --no-ros --jobs 1   # 차 한 대만 처리하고 끝

순서 (한 대, 기본 = 2026-11-11 방식. 단계 번호는 허브 4번과 같다)
  0) 켜 두면 차의 첫 상태 메시지(차 전원 켜짐)를 기다렸다가 대기 자세로 간다          (--no-ros 면 바로)
  2) 대기 자세에서 YOLO 로 지켜본다                                                   → WAITING
  3) 차 + 상자가 연속 N장 함께 보이면 바로 정지 신호, 상자가 1.5초 안 움직이면 완전 정차  → STOP_REQUEST
  4) 포스트잇 색 → 그 색 공 집기 (3번 동작 티칭)                                       → IN_PROGRESS
  5~9) 바구니 위에 맞추기 → 안으로 → 집게 해제 → 들어올리기 → 성공 확인 (허브 4번 ★ 세트의 학습 값)
  9) 성공: 대기 자세로 돌아간 뒤 SUCCESS (출발)
  10) 실패: 대기 자세에서 공을 받침대에 놓는 시간만큼 기다렸다가 4) 부터. 3번째도 실패면 FAILED (공 없이 출발)

예전 방식 (설정 '예전 추적 방식' 을 켰을 때)
  1) 대기 자세(watch)에서 YOLO 로 차 위 상자(없으면 차)를 찾는다                      → WAITING
  2) 보이면 받침(1번 관절)만 돌려 화면 가운데에 두고 따라간다                         → TRACKING
  3) 받침이 '기준 차 위치' 각도에 오면 차에게 정지 신호                               → STOP_REQUEST
  4) 그 뒤에도 계속 따라가며, 차가 1.5초(설정) 동안 안 움직이면 '완전 정차'. 그 순간의 받침 각도를 임시로 기억
  5) 포스트잇 색을 읽는다 (여러 장을 보고 다수결)                                     → IN_PROGRESS
  6) 그 색의 집기 동작(빨간공_집기 / 파란공_집기) → 바구니_넣기 (기억한 각도만큼 받침을 더 돌려서)
  7) '공이 든 상자' 가 보이면 성공. 아니면 3번까지 다시                                → SUCCESS / FAILED (= 차 출발)
  8) 대기 자세로 돌아가 차가 떠나면 다시 1)

먼저 필요한 것 (--check 가 알려 준다)
  허브 1번: 대기 자세 저장   ·   허브 3번: 빨간공_집기 / 파란공_집기 / 바구니_넣기 (★ 선택) + ⑤ 기준 차 위치

고칠 숫자: 허브 6번 설정의 'YOLO · 상차' 묶음 (정지 신호를 미리 내는 각도, 완전 정차 확인 시간 …)

안전
- 문제가 생기면(과열 · 통신 · 동작 없음) 차에게 FAILED 를 보내 출발시키고, 팔은 그 자리에서 멈춘다.
- 끝낼 때(Ctrl+C): 대기 자세 → (rest 자세가 있으면 거기로) → 토크 끄기 → 포트 닫기.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from .config import RUNTIME_DIR, MissionConfig
from .m3_pick_place import band_ok
from .motions import MotionBook, MotionError, play
from .place_flow import ArmOps, FlowAbort, PlaceRunner
from .place_learn import PlaceBook, PlaceError
from .yolo import BASKET_NAMES, pick_best, track_target

PICK_MOTION = {"red": "빨간공_집기", "blue": "파란공_집기"}
PLACE_MOTION = "바구니_넣기"
LOOK_MOTION = "바구니_보기"          # 선택: 포스트잇이 안 보일 때 바구니를 더 가까이 보는 동작 (있으면 쓴다)
COLOR_MIN_HITS = 3                   # 포스트잇이 이만큼은 보여야 색을 정한다


class LoadAbort(RuntimeError):
    """하드웨어 문제 등으로 더 진행하면 안 될 때. 차는 FAILED 로 출발시키고 팔은 멈춘다"""


@dataclass
class Seen:
    dets: list
    width: int


@dataclass
class Stop:
    pan: float                       # 차가 완전히 멈췄을 때 차를 바라본 받침 각도
    box: object                      # 그때 찾은 상자(또는 차)


class Eye:
    """카메라 + YOLO. look() = 지금 사진에서 찾은 것"""

    def __init__(self, source, detector):
        self.source, self.detector = source, detector

    def look(self) -> Seen | None:
        cap = getattr(self.source, "cap", None)
        if cap is not None and hasattr(cap, "grab"):       # 카메라에 쌓인 옛 사진을 버리고 가장 최근 것을 본다 (추적이 늦지 않게)
            for _ in range(2):
                cap.grab()
        frame = self.source.read()
        if frame is None:
            return None
        return Seen(self.detector.detect(frame), frame.shape[1])


class _Leaving:
    """재생 중에도 '차가 먼저 떠났는가' 를 물어볼 수 있게 (play 의 stop 자리에 넣는다)"""

    def __init__(self, loader):
        self.loader = loader

    def is_set(self) -> bool:
        return self.loader.car_left_early()


class Loader:
    def __init__(self, cfg: MissionConfig, arm, eye, link, book: MotionBook,
                 clock=time.monotonic, sleep=time.sleep, log=None, places: PlaceBook | None = None, ops=None):
        self.cfg, self.arm, self.eye, self.link, self.book = cfg, arm, eye, link, book
        self.places = places                          # 허브 4번 학습 (기본 방식에서 쓴다)
        self.ops = ops                                # 팔에게 시키는 일 (None 이면 arm 으로 만든다)
        self.wait_car_power = False                   # 켜면 차의 첫 상태 메시지를 받은 뒤 대기 자세로 간다
        self.clock, self.sleep = clock, sleep
        self._log = log or (lambda event, **kw: print(f"[{time.strftime('%H:%M:%S')}] {event} {kw if kw else ''}", flush=True))
        self.jobs_done = 0
        self.max_wait: float | None = None            # 시험용: 차를 기다리는 최대 시간 (None = 계속)
        self.car_was_stopped = False                  # 이번 차가 '상차 지점에 섬' 을 알려 온 적이 있는가

    def log(self, event, **kw):
        self._log(event, **kw)

    # ── 준비 확인 ───────────────────────────────────────────────────────
    def problems(self) -> list[str]:
        probs = [p.replace("(m0_teach_poses.py)", "(허브 1번 대기 자세 저장)") for p in self.arm.preflight([self.cfg.load_wait_pose])]
        names = (*PICK_MOTION.values(), PLACE_MOTION) if self.cfg.load_use_tracking else tuple(PICK_MOTION.values())
        for name in names:
            try:
                self.book.chosen(name)
            except MotionError as e:
                probs.append(str(e))
        if self.cfg.load_use_tracking:
            if self.book.car_ref_pan() is None:
                probs.append("'기준 차 위치' 가 저장되지 않았어요 (허브 3번 ⑤).")
        else:
            try:
                if self.places is None:
                    raise PlaceError("바구니 넣기 학습 파일을 읽지 못했어요.")
                self.places.learned(self.places.chosen_set())
            except PlaceError as e:
                probs.append(str(e))
        return probs

    # ── 기본 방식 (2026-11-11): 보이면 바로 정지 → 집기 → 학습한 넣기 ─────
    def runner(self) -> PlaceRunner:
        ops = self.ops if self.ops is not None else ArmOps(self.arm, self.cfg)
        r = PlaceRunner(self.cfg, ops, self.eye, self.link, self.book, clock=self.clock, sleep=self.sleep, log=self._log)
        r.max_wait = self.max_wait
        return r

    def serve_one_new(self) -> str:
        """차 한 대: "success" | "failed". 결과를 알리기 전에 대기 자세로 돌아간다"""
        self.car_was_stopped = False
        result, why = "failed", ""
        runner = self.runner()
        aborted = False
        try:
            if not runner.wait_car_and_stop():
                return result
            color = runner.read_color()
            if color is None:
                self.log("no_color", msg="포스트잇 색을 읽지 못했어요 → 공 없이 출발시킵니다")
                return result
            learned = self.places.learned(self.places.chosen_set())
            for n in range(1, self.cfg.attempts_max + 1):
                if self.car_left_early():
                    self.log("car_left_early", msg="차가 먼저 떠났어요 → 중단")
                    return result
                self.link.set("IN_PROGRESS", color, n)
                self.log("attempt", n=n, color=color)
                ok, why = runner.pick(color)
                if ok:
                    ok, why = runner.auto_place(learned, on_step=lambda num, k, w: self.log("step", n=num, ok=k, why=w))
                self.log("attempt_result", n=n, ok=ok, why=why)
                if ok:
                    result = "success"
                    return result
                if n < self.cfg.attempts_max:
                    runner.ops.go_wait()                             # 바구니에서 빠져나와 기다린다
                    runner.pause(self.cfg.load_retry_wait_seconds)   # 떨어진 공을 사람이 받침대에 놓는 시간 (규정이 허용)
            return result
        except FlowAbort as e:
            aborted = True
            self.log("ERROR", reason=str(e))
            raise LoadAbort(str(e)) from e
        except Exception as e:
            aborted = True
            self.log("ERROR", reason=str(e))
            raise
        finally:
            home = False
            try:
                home = bool(self.go_wait())                          # 9) 대기 자세로 돌아간 뒤 (오류여도 한 번 시도)
            except Exception as e:
                self.log("warn", msg=f"대기 자세로 가지 못했어요: {e}")
            if home or not self.link.hold:
                self.link.set("SUCCESS" if result == "success" else "FAILED")   # 결과를 알려 차를 출발시킨다
            else:
                # 집게가 바구니 안에 있을 수 있다 → 차를 세워 둔 채로 둔다 (사람이 팔을 치우고 차 패널로 출발)
                self.log("hold_car", msg="팔이 대기 자세로 못 돌아가서 차를 세워 둡니다. 팔을 치운 뒤 차를 직접 출발시키세요.")
            self.log("result", result=result, why=why, aborted=aborted)

    # ── 팔 (시험에서는 이 세 개만 바꿔 끼운다) ──────────────────────────
    def play_motion(self, name: str, pan_offset: float = 0.0) -> dict:
        _tid, take = self.book.chosen(name)
        return play(self.arm.robot, take["points"], speed=self.cfg.speed_slow, pan_offset=pan_offset,
                    stop=_Leaving(self), too_hot=self.arm.too_hot, sleep=self.sleep)

    def turn_to(self, pan: float) -> None:
        self.arm.robot.move(1, pan, speed=self.cfg.speed_slow)
        self.arm.robot.wait_until_settled(timeout=8.0)

    def go_wait(self) -> bool:
        return self.arm.go(self.cfg.load_wait_pose)

    # ── 자동차 상태 ─────────────────────────────────────────────────────
    def car_left_early(self) -> bool:
        """차가 '상차 지점에 섬' 을 알린 뒤 다시 '주행 중' 이 됐다 = 신호가 끊겨 먼저 떠났다"""
        state = self.link.car_state
        if state == "AT_LOADING_ZONE":
            self.car_was_stopped = True
        return self.car_was_stopped and state == "DRIVING"

    # ── 1~4) 추적 → 정지 신호 → 완전 정차 ───────────────────────────────
    def track_and_stop(self) -> Stop | None:
        """차가 완전히 멈추면 Stop. 정지 신호를 냈는데 안 멈추거나 사라지면 None (그 차는 포기)"""
        cfg = self.cfg
        center = self.arm.pan_angle()
        ref = self.book.car_ref_pan()
        if center is None or ref is None:
            raise LoadAbort("받침 각도 또는 기준 차 위치를 읽지 못했어요")
        lo = min(center, ref) - cfg.load_pan_margin_deg
        hi = max(center, ref) + cfg.load_pan_margin_deg
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        direction = 0 if abs(ref - center) < 0.5 else (1 if ref > center else -1)
        sign = 1 if cfg.track_pan_sign >= 0 else -1
        t0 = self.clock()
        tracking = False
        stop_at = lost_since = still_since = None
        anchor = None                                           # 정차 확인 기준점 (cx, cy, pan)
        while True:
            if self.arm.too_hot():
                raise LoadAbort("모터 과열")
            now = self.clock()
            if stop_at is None and self.max_wait is not None and now - t0 > self.max_wait:
                raise TimeoutError("자동차가 오지 않았습니다 (시험용 시간 제한)")
            if stop_at is not None and now - stop_at > cfg.load_stop_timeout:
                self.log("stop_timeout", msg="정지 신호를 냈지만 완전 정차를 확인하지 못했어요")
                return None
            seen = self.eye.look()
            target = track_target(seen.dets) if seen else None
            if target is None:
                anchor = still_since = None
                lost_since = lost_since if lost_since is not None else now
                if stop_at is None and tracking and now - lost_since >= cfg.track_lost_seconds:
                    self.log("lost", msg="차를 놓쳤어요 → 대기 자세로")
                    tracking = False
                    self.link.set("WAITING", "", 0)
                    self.go_wait()
                    lost_since = None
                self.sleep(cfg.track_period)
                continue
            lost_since = None
            if not tracking and stop_at is None:
                tracking = True
                self.link.set("TRACKING", "", 0)
                self.log("tracking", what=target.name)
            pan = self.arm.pan_angle()
            if pan is None:
                raise LoadAbort("받침 각도를 읽지 못했어요")
            error = target.cx - seen.width / 2                  # +: 화면 오른쪽
            # 3) 정지 신호: 받침이 기준 각도(조금 앞)에 왔을 때
            if stop_at is None:
                if direction == 0:
                    reached = abs(error) <= cfg.track_deadband_px
                else:
                    reached = (pan - (ref - cfg.load_stop_lead_deg * direction)) * direction >= 0
                if reached:
                    stop_at = now
                    self.link.set("STOP_REQUEST", "", 0)
                    self.log("stop_request", pan=round(pan, 1), ref=ref)
            # 4) 정지 신호 뒤에도 계속 추적하면서, 기준점에서 안 벗어난 채 정해진 시간이 지나면 완전 정차
            if stop_at is not None:
                moved = anchor is None or math.hypot(target.cx - anchor[0], target.cy - anchor[1]) > cfg.track_stop_px \
                    or abs(pan - anchor[2]) > 0.3
                if moved:
                    anchor, still_since = (target.cx, target.cy, pan), now
                elif now - still_since >= cfg.load_stop_confirm_seconds:
                    self.log("car_stopped", pan=round(pan, 1), off_center_px=round(error), car=self.link.car_state)
                    return Stop(pan, target)
            if abs(error) > cfg.track_deadband_px:
                step = sign * cfg.track_gain_deg_per_px * error
                step = max(-cfg.track_max_step_deg, min(cfg.track_max_step_deg, step))
                self.arm.nudge_pan(step, mid, half)
            self.sleep(cfg.track_period)

    # ── 5) 포스트잇 색 ──────────────────────────────────────────────────
    def read_color(self) -> str | None:
        counts = {"red": 0, "blue": 0}
        weight = {"red": 0.0, "blue": 0.0}
        for _ in range(max(1, self.cfg.load_color_frames)):
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

    # ── 7) 성공 확인 ────────────────────────────────────────────────────
    def basket_has_ball(self) -> bool:
        hits = 0
        t0 = self.clock()
        while self.clock() - t0 < self.cfg.load_verify_seconds:
            seen = self.eye.look()
            if seen and pick_best(seen.dets, ("success_red", "success_blue")) is not None:
                hits += 1
                if hits >= self.cfg.load_verify_hits:
                    return True
            self.sleep(0.1)
        return False

    def look_at(self, stop: Stop) -> None:
        """대기 자세로 간 뒤 받침만 차가 선 방향으로 돌린다 (차를 다시 바라본다)"""
        self.go_wait()
        self.turn_to(stop.pan)

    # ── 6) 한 번의 시도 ─────────────────────────────────────────────────
    def attempt(self, color: str, stop: Stop) -> bool:
        rep = self.play_motion(PICK_MOTION[color])
        self.log("pick", reached=rep["reached"], max_err=rep["max_err"], why=rep["why"])
        if rep["stopped"]:
            return False
        if self.cfg.grip_ball_max_pct > 0:                      # 공을 쥔 그리퍼 % 를 티칭해 뒀을 때만 확인
            pct = self.arm.grip_pct()
            if not band_ok(self.cfg, pct):
                self.log("pick_empty", grip=pct, msg="그리퍼가 공 크기에서 멈추지 않았어요 (공을 못 잡음)")
                return False
        _tid, take = self.book.chosen(PLACE_MOTION)
        ref = take.get("ref_pan")
        ref = self.book.car_ref_pan() if ref is None else ref
        offset = stop.pan - ref
        if abs(offset) > self.cfg.load_offset_max_deg:
            self.log("offset_too_big", offset=round(offset, 1))
            return False
        rep = self.play_motion(PLACE_MOTION, offset)
        self.log("place", offset=round(offset, 1), reached=rep["reached"], max_err=rep["max_err"], why=rep["why"])
        if rep["stopped"]:
            return False
        if self.basket_has_ball():                              # 넣기 동작이 끝난 자리에서 먼저 본다
            return True
        self.look_at(stop)                                      # 안 보이면 차를 다시 바라보고 한 번 더
        return self.basket_has_ball()

    # ── 한 대 처리 ──────────────────────────────────────────────────────
    def serve_one(self) -> str:
        """차 한 대: "success" | "failed". 차가 오기 전에는 계속 기다린다"""
        self.car_was_stopped = False
        result = "failed"
        try:
            stop = self.track_and_stop()
            if stop is None:
                return result
            color = self.read_color()
            if color is None and self._has_motion(LOOK_MOTION):
                self.play_motion(LOOK_MOTION, stop.pan - (self.book.car_ref_pan() or stop.pan))
                color = self.read_color()
            if color is None:
                self.log("no_color", msg="포스트잇 색을 읽지 못했어요 → 공 없이 출발시킵니다")
                return result
            for n in range(1, self.cfg.attempts_max + 1):
                if self.car_left_early():
                    self.log("car_left_early", msg="차가 먼저 떠났어요 → 중단")
                    return result
                if self.arm.too_hot():
                    raise LoadAbort("모터 과열")
                self.link.set("IN_PROGRESS", color, n)
                self.log("attempt", n=n, color=color)
                if self.attempt(color, stop):
                    result = "success"
                    return result
                if n < self.cfg.attempts_max:
                    self.look_at(stop)
                    self.sleep(self.cfg.load_retry_wait_seconds)   # 떨어진 공을 사람이 제자리에 놓는 시간 (규정이 허용)
            return result
        except Exception as e:
            self.log("ERROR", reason=str(e))
            raise
        finally:
            # 어떤 경우에도 결과를 알려 차를 출발시킨다 (오류도 FAILED)
            self.link.set("SUCCESS" if result == "success" else "FAILED")
            self.log("result", result=result)

    def _has_motion(self, name: str) -> bool:
        try:
            self.book.chosen(name)
            return True
        except MotionError:
            return False

    def wait_car_gone(self) -> None:
        """결과를 알린 뒤: 차가 '주행 중' 을 알리거나 상자가 t_left 초 동안 안 보이면 떠난 것"""
        t0 = self.clock()
        gone_since = None
        while self.clock() - t0 < self.cfg.load_leave_timeout:
            if self.link.car_state == "DRIVING" and self.car_was_stopped:
                break
            seen = self.eye.look()
            if seen and pick_best(seen.dets, BASKET_NAMES + ("car",)) is not None:
                gone_since = None
            else:
                gone_since = gone_since if gone_since is not None else self.clock()
                if self.clock() - gone_since >= self.cfg.t_left:
                    break
            self.sleep(0.2)
        else:
            self.log("warn", msg="차가 떠나는 것을 확인하지 못했지만 대기로 돌아갑니다")

    def wait_for_car_power(self) -> None:
        """차 전원이 켜져 첫 상태 메시지(/car/status)가 올 때까지 기다린다"""
        self.log("wait_car_power", msg="차 전원이 켜지기를 기다리는 중 (/car/status)")
        t0 = self.clock()
        while self.link.car_state is None:
            if self.max_wait is not None and self.clock() - t0 > self.max_wait:
                raise TimeoutError("차의 상태 메시지가 오지 않았습니다 (시험용 시간 제한)")
            self.sleep(0.2)
        self.log("car_power", car=self.link.car_state)

    def boot(self) -> None:
        probs = self.problems()
        if probs:
            raise LoadAbort("시작 전 점검 실패: " + " / ".join(probs))
        if self.wait_car_power:
            self.wait_for_car_power()
        self.arm.torque_on()                                    # 지금 자리에서 잠금 (튀지 않음)
        if not self.go_wait():
            raise LoadAbort("대기 자세로 가지 못했어요")
        self.link.set("WAITING", "", 0)
        self.log("ready", pose=self.cfg.load_wait_pose)

    def run(self, max_jobs: int | None = None) -> None:
        self.boot()
        while max_jobs is None or self.jobs_done < max_jobs:
            if self.cfg.load_use_tracking:
                self.serve_one()
            else:
                self.serve_one_new()
            self.jobs_done += 1
            self.go_wait()                                      # 대기 자세로 복귀
            self.wait_car_gone()
            self.link.set("WAITING", "", 0)


# ── 실행 진입점 ─────────────────────────────────────────────────────────
def _file_logger(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)

    def log(event, **kw):
        line = json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "event": event, **kw}, ensure_ascii=False, default=str)
        print(line, flush=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    return log


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="상차 미션 (팔이 차를 세우고 출발시킴)")
    ap.add_argument("--check", action="store_true", help="준비가 됐는지만 확인하고 끝 (팔을 움직이지 않음)")
    ap.add_argument("--no-ros", action="store_true", help="ROS2 없이 (차에게 보낼 신호를 화면에만 출력)")
    ap.add_argument("--port", default=None, help="팔 포트 (기본: 설정 값). fake = 가짜 팔")
    ap.add_argument("--jobs", type=int, default=None, help="차를 이만큼 처리하고 끝 (기본: 계속)")
    ap.add_argument("--no-car-wait", action="store_true", help="차 전원(첫 상태 메시지)을 기다리지 않고 바로 대기 자세로")
    args = ap.parse_args(argv)

    cfg = MissionConfig.load()
    if args.port:
        cfg.port = args.port
    from .arm_io import ArmIO
    from .load_link import FakeLoadLink, RosLoadLink
    from .yolo import Detector
    try:
        book = MotionBook()
        places = PlaceBook()
        detector = Detector(cfg.yolo_model, cfg.yolo_conf)
    except (MotionError, PlaceError, FileNotFoundError) as e:
        print(f"준비 문제: {e}")
        return 1
    try:
        arm = ArmIO.open(cfg)
    except Exception as e:                                       # 포트 없음 · 다른 프로그램(허브)이 사용 중
        print(f"팔에 연결하지 못했습니다: {e}")
        return 1

    if args.check:
        try:
            probs = Loader(cfg, arm, None, FakeLoadLink(), book, places=places).problems()
        finally:
            arm.close()
        if probs:
            print("아직 준비가 안 된 것:")
            for p in probs:
                print("  ✘", p)
            return 1
        print("✔ 대기 자세 · 공 집기 동작 · " + ("기준 차 위치" if cfg.load_use_tracking else "4번 넣기 학습 ★ 세트")
              + " · YOLO 모델이 모두 준비됐어요.")
        return 0

    from .vision import CameraSource
    try:
        src = CameraSource(cfg)
    except Exception as e:
        print(f"카메라를 열지 못했습니다: {e}")
        arm.close()
        return 1
    try:
        link = FakeLoadLink(echo=True) if args.no_ros else RosLoadLink(cfg.state_hz)
    except Exception as e:                                       # rclpy · common_interfaces 가 없음 (Docker 밖)
        print(f"ROS2 를 시작하지 못했습니다: {e}  (연습은 --no-ros)")
        src.close()
        arm.close()
        return 1
    loader = Loader(cfg, arm, Eye(src, detector), link, book, log=_file_logger(RUNTIME_DIR / "loading.log"), places=places)
    loader.wait_car_power = cfg.load_wait_car_power and not args.no_ros and not args.no_car_wait
    link.start()
    aborted = False
    try:
        loader.run(max_jobs=args.jobs)
    except LoadAbort as e:
        aborted = True
        print(f"안전 중단: {e}")
        try:
            arm.hold()                                           # 그 자리에서 멈춘다 (힘은 준 채)
        except Exception:
            pass
        return 2
    except KeyboardInterrupt:
        print("중단 (사람이 Ctrl+C)")
    finally:
        try:
            if not aborted:                                      # 정리: 대기 자세 → (rest) → 토크 끄기
                loader.go_wait()
                if "rest" in arm.robot.poses():
                    arm.go("rest")
                arm.torque_off()
        finally:
            link.stop()
            src.close()
            arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
