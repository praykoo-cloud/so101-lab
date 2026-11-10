"""집기 연습 · 학습 (허브 15번). 대회 코드(mission_main)와 분리된 **연습 전용** 도구다.

하는 일
  - 집기 → 바구니에 넣기를 5단계로 나눠 반복한다 (단계마다 확인 / 어느 단계까지만 / 무한 반복).
  - 색은 빨강만 / 파랑만 / 번갈아 / **바구니 포스트잇을 카메라로 읽어서** 중에서 고른다.
  - 시도마다 결과를 기록(runtime/practice/log.jsonl)하고, 기록으로 '공을 쥔 그리퍼 % 범위'를 제안한다.
    (제안은 사람이 허브에서 눈으로 보고 '적용'을 눌러야만 설정에 들어간다.)
  - GPU · AI 학습이 아니다: 연습한 결과를 모아서 기준값을 다듬는 방식이다.

안전
  - 정지 버튼은 지금 하는 동작(몇 초)이 끝난 뒤에 멈춘다. 급하면 팔 전원 스위치를 끈다.
  - 끝나면(정지 · 오류 포함) 대기 자세로 간 뒤 토크를 끈다 (try / finally).
  - 모터 온도가 중단 기준을 넘으면 스스로 멈춘다.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from .config import RUNTIME_DIR, MissionConfig
from .m3_pick_place import band_ok
from .vision import REF_NAMES

PRACTICE_DIR = RUNTIME_DIR / "practice"
LOG_FILE = PRACTICE_DIR / "log.jsonl"

# (번호, 이름, 설명)
STEPS = [
    (1, "위로 이동", "그리퍼를 열고 공 바로 위로 이동"),
    (2, "내려가 잡기", "공 높이로 내려가 그리퍼를 닫고, 닫힌 정도를 읽음"),
    (3, "들어올리기", "공 위로 들어올려 잠깐 유지한 뒤, 닫힌 정도를 다시 읽음"),
    (4, "바구니로 이동", "바구니 위 → 바구니 안쪽으로 이동"),
    (5, "놓고 빠지기", "그리퍼를 열어 놓고, 위 → 중간 후퇴 자세로 빠짐 (이어서 바구니 안 공을 카메라로 확인)"),
]
COLOR_MODES = {
    "red": "빨강만",
    "blue": "파랑만",
    "alternate": "빨강·파랑 번갈아",
    "basket": "바구니 포스트잇 색을 카메라로 읽어서",
}
COLORS = ("red", "blue")


class _Stop(Exception):
    """사용자가 정지를 눌렀다"""


class _Halt(Exception):
    """안전 문제로 연습을 멈춘다 (메시지를 사용자에게 보여 준다)"""


@dataclass
class Options:
    color_mode: str = "alternate"
    stop_after: int = 5                      # 1~5: 이 단계까지만 하고, 5 미만이면 공을 받침대로 되돌려 놓는다
    confirm_each_step: bool = False          # 단계마다 '다음 단계' 를 눌러야 진행
    max_cycles: int | None = None            # None = 정지를 누를 때까지 무한 반복
    expected_color: str | None = None        # 바구니 색 읽기 모드에서 실제 포스트잇 색 (정답을 알려 주면 정확도를 센다)

    def validate(self) -> None:
        if self.color_mode not in COLOR_MODES:
            raise ValueError("색 선택이 올바르지 않아요.")
        if not isinstance(self.stop_after, int) or not 1 <= self.stop_after <= 5:
            raise ValueError("'어느 단계까지' 는 1~5 중 하나여야 해요.")
        if self.max_cycles is not None and (not isinstance(self.max_cycles, int) or not 1 <= self.max_cycles <= 10000):
            raise ValueError("반복 횟수는 1~10000 이거나 비워 두세요(무한).")
        if self.expected_color not in (None, "red", "blue"):
            raise ValueError("실제 포스트잇 색은 빨강 · 파랑 · 모름 중 하나여야 해요.")

    @classmethod
    def from_dict(cls, data: dict) -> "Options":
        def to_int(value, default):
            if value in (None, ""):
                return default
            try:
                return int(value)
            except (TypeError, ValueError):
                raise ValueError("숫자를 넣어 주세요.") from None
        opts = cls(
            color_mode=str(data.get("color_mode", "alternate")),
            stop_after=to_int(data.get("stop_after"), 5),
            confirm_each_step=bool(data.get("confirm_each_step", False)),
            max_cycles=to_int(data.get("max_cycles"), None),
            expected_color=(data.get("expected_color") or None),
        )
        opts.validate()
        return opts


# ── 기록 · 통계 · 제안 ──────────────────────────────────────────────────
def append_log(record: dict, path: Path = LOG_FILE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def read_log(path: Path = LOG_FILE) -> list[dict]:
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return []
    out = []
    for line in lines:
        try:
            item = json.loads(line)
        except ValueError:
            continue                              # 깨진 줄은 건너뛴다
        if isinstance(item, dict):
            out.append(item)
    return out


def summarize(records: list[dict]) -> dict:
    per = {c: {"cycles": 0, "full": 0, "success": 0, "fail": 0, "unknown": 0} for c in COLORS}
    reading = {"total": 0, "correct": 0, "wrong": 0}
    for r in records:
        c = r.get("color")
        if c not in per:
            continue
        p = per[c]
        p["cycles"] += 1
        if r.get("steps_done") == 5:
            p["full"] += 1
            if r.get("outcome") in ("success", "fail", "unknown"):
                p[r["outcome"]] += 1
        if r.get("decided_by") == "post-it" and r.get("expected") in COLORS:
            reading["total"] += 1
            reading["correct" if r["expected"] == c else "wrong"] += 1
    return {"per_color": per, "reading": reading, "total": len(records)}


def suggest_grip(records: list[dict], margin: float = 2.0, min_samples: int = 5) -> dict:
    """공을 확실히 쥔 시도들의 그리퍼 % 로 '공 쥠 범위' 를 제안한다. 제안만 하고 적용은 사람이 한다"""
    good, bad = [], []
    for r in records:
        for key in ("closed_pct", "lifted_pct"):
            value = r.get(key)
            if not isinstance(value, (int, float)):
                continue
            if r.get("good_hold"):
                good.append(float(value))
            elif r.get("bad_hold"):
                bad.append(float(value))
    if len(good) < min_samples:
        return {"ok": False, "n": len(good),
                "message": f"공을 확실히 쥔 값이 {len(good)}개예요. {min_samples}개 이상 모이면 제안할 수 있어요 "
                           f"(바구니에 넣기까지 성공했거나, 들어올렸을 때 카메라로 공이 그리퍼에 보인 시도만 셉니다)."}
    lo, hi = max(0.0, min(good) - margin), min(100.0, max(good) + margin)
    overlap = [b for b in bad if lo <= b <= hi]
    mean = sum(good) / len(good)
    msg = (f"공을 쥔 값 {len(good)}개: {min(good):.1f} ~ {max(good):.1f}% (평균 {mean:.1f}%). "
           f"여유 ±{margin:g}% 를 둔 제안: {lo:.1f} ~ {hi:.1f}%.")
    if overlap:
        msg += f" ⚠ 공을 못 쥔 시도의 값 {len(overlap)}개가 이 범위 안에 있어요 → 그리퍼 % 만으로는 성공/실패를 가르기 어려우니 적용 전에 확인하세요."
    return {"ok": True, "n": len(good), "min": round(lo, 1), "max": round(hi, 1), "overlap": len(overlap), "message": msg}


# ── 연습 실행기 ─────────────────────────────────────────────────────────
class Practice:
    def __init__(self, arm, vision, cfg: MissionConfig, refs=None, log_file: Path = LOG_FILE,
                 clock=time.monotonic, sleep=time.sleep):
        self.arm, self.vision, self.cfg, self.refs = arm, vision, cfg, refs
        self.log_file = Path(log_file)
        self.clock, self.sleep = clock, sleep
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._resume = threading.Event()
        self.thread: threading.Thread | None = None
        self.opts = Options()
        self.state = "idle"                       # idle | running | waiting | stopping
        self.message = "연습을 시작하지 않았어요."
        self.cycle = 0
        self.step = 0
        self.color: str | None = None
        self.error: str | None = None
        self.last: dict | None = None
        self.run_id = ""

    # 시작 전 점검: 문제 목록 (비어 있으면 시작 가능)
    def problems(self, opts: Options) -> list[str]:
        cfg, out = self.cfg, []
        needed = ["wait"]
        colors = COLORS if opts.color_mode in ("alternate", "basket") else (opts.color_mode,)
        for c in colors:
            needed += [f"above_{c}", c]
        if opts.stop_after >= 4:
            needed += ["above_basket", "basket", "retreat_mid"]
        out += self.arm.preflight(needed)
        if opts.color_mode == "basket":
            if not cfg.roi_taught("note"):
                out.append("포스트잇 구역(4번 '포스트잇')을 그리지 않았어요")
            if self.refs is None or not all(n in self.refs.refs for n in REF_NAMES):
                out.append("기준색 4개(5번)가 모두 등록되지 않았어요")
        if opts.stop_after == 5 and (self.refs is None or not all(n in self.refs.refs for n in REF_NAMES)):
            out.append("바구니 안 공 확인에 기준색(5번)과 바구니·바깥·그리퍼 구역(4번)이 필요해요")
        return out

    def start(self, opts: Options) -> None:
        opts.validate()
        with self._lock:
            if self.running():
                raise ValueError("이미 연습 중이에요.")
            found = self.problems(opts)
            if found:
                raise ValueError("시작할 수 없어요: " + " / ".join(found))
            self.opts = opts
            self._stop.clear()
            self._resume.clear()
            self.cycle = self.step = 0
            self.color = self.last = self.error = None
            self.run_id = datetime.now().strftime("%Y%m%d-%H%M%S")
            self.state, self.message = "running", "시작하는 중"
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def resume(self) -> None:
        self._resume.set()

    def stop(self) -> None:
        self._stop.set()
        self._resume.set()
        if self.running():
            self.state, self.message = "stopping", "멈추는 중 (지금 동작이 끝나면 멈춰요)"

    def join(self, timeout: float | None = None) -> None:
        if self.thread is not None:
            self.thread.join(timeout)

    def status(self) -> dict:
        return {"state": self.state, "message": self.message, "cycle": self.cycle, "step": self.step,
                "color": self.color, "error": self.error, "last": self.last, "running": self.running(),
                "steps": [{"n": n, "name": name, "desc": desc} for n, name, desc in STEPS]}

    # 내부 ----------------------------------------------------------------
    def _set(self, state: str, message: str) -> None:
        if self.state != "stopping":
            self.state = state
        self.message = message

    def _check(self) -> None:
        if self._stop.is_set():
            raise _Stop()
        if self.arm.too_hot():
            raise _Halt("모터 온도가 중단 기준을 넘었어요. 식힌 뒤 다시 하세요.")

    def _go(self, pose: str) -> None:
        self._check()
        if not self.arm.go(pose):
            raise _Halt(f"'{pose}' 자세로 가지 못했어요. 팔 주변과 자세 티칭을 확인하세요.")

    def _grip(self, pct: float) -> None:
        self._check()
        self.arm.grip(pct)

    def _wait_user(self, message: str) -> None:
        """사용자가 '계속' 을 누를 때까지 기다린다 (정지를 누르면 끝)"""
        self._resume.clear()
        self._set("waiting", message)
        while not self._resume.wait(0.2):
            if self._stop.is_set():
                break
        self._check()
        self._set("running", "계속합니다")

    def _run(self) -> None:
        try:
            self.arm.torque_on()
            self._set("running", "대기 자세로 이동")
            self._go("wait")
            while True:
                if self.opts.max_cycles is not None and self.cycle >= self.opts.max_cycles:
                    self._set("running", f"정해진 {self.opts.max_cycles}번을 모두 했어요.")
                    break
                self._check()
                self.cycle += 1
                need_help = self._cycle()
                if self.opts.max_cycles is not None and self.cycle >= self.opts.max_cycles:
                    continue
                if need_help:
                    self._wait_user(need_help)
        except _Stop:
            self.message = "정지했어요."
        except _Halt as error:
            self.error = str(error)
            self.message = f"안전 때문에 멈췄어요: {error}"
        except Exception as error:                    # noqa: BLE001  (연습 도구는 어떤 오류든 팔을 안전하게 접고 알려야 한다)
            self.error = f"예상하지 못한 오류: {error}"
            self.message = self.error
        finally:
            self._finish()

    def _finish(self) -> None:
        try:
            self.arm.go("wait")                       # 대기 자세 → 토크 끄기 (공중에서 토크를 끄면 떨어지므로 자세가 먼저)
        except Exception as error:                    # noqa: BLE001
            self.error = self.error or f"대기 자세로 돌아가지 못했어요: {error}"
        try:
            self.arm.torque_off()
        except Exception:                             # noqa: BLE001
            pass
        self.state = "idle"
        self.step = 0

    def _choose_color(self) -> tuple[str, str, dict]:
        mode = self.opts.color_mode
        if mode in COLORS:
            return mode, "fixed", {}
        if mode == "alternate":
            return COLORS[(self.cycle - 1) % 2], "alternate", {}
        while True:                                   # basket: 대기 자세에서 포스트잇을 읽는다 (기준색으로 빨강/파랑 중 가까운 쪽)
            self._go("wait")
            self._set("running", "바구니 포스트잇 색을 읽는 중")
            vote = self.vision.read_color()
            if vote is not None:
                return vote.color, "post-it", {"counts": vote.counts, "mean_dist": vote.mean_dist,
                                               "used_default": vote.used_default}
            self._wait_user("포스트잇이 안 보여요. 바구니(포스트잇)를 카메라 앞에 놓고 '계속' 을 누르세요.")

    def _gate(self, idx: int, color: str) -> None:
        self._check()
        self.step = idx
        name = STEPS[idx - 1][1]
        if self.opts.confirm_each_step:
            self._wait_user(f"{self.cycle}번째 · {name} 를 할까요? (색: {color})  '다음 단계' 를 누르세요.")
        self._set("running", f"{self.cycle}번째 · {idx}/{self.opts.stop_after}단계 {name} (색: {color})")

    def _cycle(self) -> str | None:
        """한 번 시도. 사용자가 해 줄 일이 있으면 그 안내 문장을 돌려준다"""
        t0 = self.clock()
        o, cfg = self.opts, self.cfg
        color, decided, info = self._choose_color()
        self.color = color
        above = f"above_{color}"
        closed = lifted = None
        held = None
        done = 0
        for idx in range(1, o.stop_after + 1):
            self._gate(idx, color)
            if idx == 1:
                self._grip(cfg.grip_open_pct)
                self._go(above)
            elif idx == 2:
                self._go(color)
                self._grip(0.0)
                closed = self.arm.grip_pct()
            elif idx == 3:
                self._go(above)
                self.sleep(0.6)
                lifted = self.arm.grip_pct()
                if cfg.roi_taught("gripper"):
                    held = bool(self.vision.ball_in_gripper(color))
            elif idx == 4:
                self._go("above_basket")
                self._go("basket")
            elif idx == 5:
                self._grip(cfg.grip_open_pct)
                self.sleep(0.4)
                self._go("above_basket")
                self._go("retreat_mid")
            done = idx
        outcome, help_text = "partial", None
        if done == 5:
            outcome = self.vision.check_success(color)
            self._go("wait")
            help_text = "공을 받침대에 다시 올려놓고 '공 올려놨어요' 를 누르세요."
        else:
            help_text = self._put_back(color, done)
            self._go("wait")
        closed_ok = band_ok(cfg, closed) if closed is not None else None
        lifted_ok = band_ok(cfg, lifted) if lifted is not None else None
        good = (done == 5 and outcome == "success") or (done >= 3 and held is True)
        bad = done in (3, 4) and held is False
        record = {"ts": datetime.now().isoformat(timespec="seconds"), "run": self.run_id, "n": self.cycle,
                  "color": color, "decided_by": decided, "expected": o.expected_color if decided == "post-it" else None,
                  "vote": info, "steps_done": done, "closed_pct": closed, "lifted_pct": lifted,
                  "in_band_close": closed_ok, "in_band_lift": lifted_ok, "held": held,
                  "outcome": outcome, "good_hold": bool(good), "bad_hold": bool(bad),
                  "seconds": round(self.clock() - t0, 1)}
        append_log(record, self.log_file)
        self.last = record
        return help_text

    def _put_back(self, color: str, done: int) -> str | None:
        """5단계 전에 끝낼 때: 공을 받침대 원래 자리에 되돌려 놓는다. 공이 안 보이면 사람이 놓도록 안내"""
        above = f"above_{color}"
        if done == 1:
            pass                                       # 아직 안 집음
        elif done == 2:
            self._grip(self.cfg.grip_open_pct)         # 내려간 자리에서 놓음
            self._go(above)
        elif done == 3:
            self._go(color)
            self._grip(self.cfg.grip_open_pct)
            self._go(above)
        else:                                          # 4: 바구니 안쪽에서 공을 쥔 채 → 왔던 길로 돌아가 받침대에 놓는다
            self._go("above_basket")
            self._go("retreat_mid")
            self._go("wait")
            self._go(above)
            self._go(color)
            self._grip(self.cfg.grip_open_pct)
            self._go(above)
        stand = f"stand_{color}"
        if self.cfg.roi_taught(stand) and not self.vision.ball_at_origin(color):
            return "공이 받침대에서 안 보여요. 받침대에 올려 놓고 '공 올려놨어요' 를 누르세요."
        return None                                    # 확인됨(또는 받침대 구역 미설정 → 확인 생략)하고 자동 계속
