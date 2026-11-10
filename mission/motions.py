"""동작 티칭 저장소 + 재생기. "동작" = 자세(지점) 여러 개를 순서대로 이은 것.

화면으로 쓰려면 허브 3번 (python -m mission.hub --start motion). 이 파일은 그 뒤에서 일하는 부품이다.

    book = MotionBook()                               # data/motions.json 을 읽고 쓴다
    book.create("빨간공_집기")                        # 동작 칸 만들기 (기본 3개는 처음부터 있다)
    tid = book.add_take("빨간공_집기", points)        # 같은 동작을 다시 가르치면 회차가 하나씩 늘어난다
    book.choose("빨간공_집기", tid)                   # 여러 회차 중 "이걸 쓴다"
    report = play(robot, points, speed=120)           # 지점 순서대로 움직여 본다

단어장
- 동작(motion) : 이름이 붙은 움직임 하나 (예: 빨간공_집기)
- 회차(take)   : 같은 동작을 가르친 한 번 한 번. 여러 개를 두고 가장 안정적인 것을 고른다
- 지점(point)  : 동작 안의 자세 하나 = 관절 1~5 의 각도 + 그리퍼 열림 %
- 기준 받침 각도(ref_pan) : '차 위치에 맞추는 동작'을 가르칠 때 차가 서 있던 방향 (받침 = 1번 관절).
                            실제 차가 다른 곳에 서면 그 차이만큼 받침을 더 돌려서 재생한다

고칠 숫자: 아래 상수 (REACH_TOL_DEG, GRIP_CHANGE_PCT, DEFAULT_PAUSE)
"""
from __future__ import annotations

import json
import os
import shutil
import time
from datetime import datetime
from pathlib import Path

from .config import ROOT, RUNTIME_DIR

MOTIONS_FILE = ROOT / "data" / "motions.json"
ARM_JOINTS = (1, 2, 3, 4, 5)        # 그리퍼(6번)는 % 로 따로 다룬다
REACH_TOL_DEG = 3.0                 # 지점에서 이만큼 안에 들어오면 "도달"
GRIP_CHANGE_PCT = 5.0               # 그리퍼 값이 이만큼 넘게 달라질 때만 그리퍼를 움직인다 (쥔 공을 놓치지 않게)
DEFAULT_PAUSE = 0.3                 # 지점마다 잠깐 쉬는 시간(초)
MAX_POINTS = 40
MAX_RUNS_KEPT = 30                  # 회차마다 재생 기록은 최근 이만큼만 둔다

# 처음부터 있는 동작 칸: 이름 → (차 위치에 맞추는 동작인가, 설명)
DEFAULT_MOTIONS = {
    "빨간공_집기": (False, "받침대의 빨간 공을 집어 들어 올리기 (공 자리는 고정)"),
    "파란공_집기": (False, "받침대의 파란 공을 집어 들어 올리기 (공 자리는 고정)"),
    "바구니_넣기": (True, "공을 든 채 바구니 위로 가서 놓고 빠져나오기 (차가 선 자리에 맞춰 받침을 돌림)"),
}


class MotionError(ValueError):
    """이름이 없거나 잘못된 요청. 메시지는 화면에 그대로 보여 준다"""


def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def clean_name(name) -> str:
    name = str(name or "").strip()
    if not 1 <= len(name) <= 30:
        raise MotionError("이름은 1~30자로 적어 주세요.")
    if any(ch in name for ch in '/\\"\'<>\n\r\t'):
        raise MotionError("이름에 / \\ 따옴표 < > 는 쓸 수 없어요.")
    return name


class MotionBook:
    """동작 · 회차를 data/motions.json 에 보관한다. 지우기 전에는 backup 폴더에 원본을 복사한다"""

    def __init__(self, path: Path | str = MOTIONS_FILE, backup_dir: Path | str | None = None):
        self.path = Path(path)
        self.backup_dir = Path(backup_dir) if backup_dir else RUNTIME_DIR / "backup"
        self.data = self._read()

    # ── 파일 ────────────────────────────────────────────────────────────
    def _read(self) -> dict:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            data = {}
        except (ValueError, OSError) as e:
            raise MotionError(f"동작 파일을 읽지 못했어요 ({self.path}): {e}") from e
        data.setdefault("version", 1)
        data.setdefault("car_ref", None)
        motions = data.setdefault("motions", {})
        for name, (relative, _help) in DEFAULT_MOTIONS.items():
            motions.setdefault(name, {"relative": relative, "chosen": None, "next": 1, "takes": {}})
        return data

    def save(self) -> None:
        """임시 파일에 다 쓴 뒤 이름을 바꾼다 (쓰는 도중 전원이 꺼져도 원본이 깨지지 않게)"""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.data, indent=2, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp, self.path)

    def backup(self) -> Path | None:
        if not self.path.exists():
            return None
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        dest = self.backup_dir / f"motions-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.json"
        shutil.copy2(self.path, dest)
        return dest

    # ── 찾기 ────────────────────────────────────────────────────────────
    @property
    def motions(self) -> dict:
        return self.data["motions"]

    def motion(self, name: str) -> dict:
        if name not in self.motions:
            raise MotionError(f"동작 '{name}' 이 없어요.")
        return self.motions[name]

    def take(self, name: str, tid: str) -> dict:
        takes = self.motion(name)["takes"]
        if tid not in takes:
            raise MotionError(f"'{name}' 에 회차 '{tid}' 가 없어요.")
        return takes[tid]

    # ── 동작 ────────────────────────────────────────────────────────────
    def create(self, name: str, relative: bool = False) -> str:
        name = clean_name(name)
        if name in self.motions:
            raise MotionError(f"동작 '{name}' 은 이미 있어요.")
        self.motions[name] = {"relative": bool(relative), "chosen": None, "next": 1, "takes": {}}
        self.save()
        return name

    def rename(self, old: str, new: str) -> str:
        new = clean_name(new)
        entry = self.motion(old)
        if new == old:
            return new
        if new in self.motions:
            raise MotionError(f"동작 '{new}' 은 이미 있어요.")
        # 순서를 지키며 이름만 바꾼다
        self.data["motions"] = {(new if k == old else k): v for k, v in self.motions.items()}
        assert self.motions[new] is entry
        self.save()
        return new

    def set_relative(self, name: str, relative: bool) -> None:
        self.motion(name)["relative"] = bool(relative)
        self.save()

    def delete_motion(self, name: str) -> Path | None:
        self.motion(name)
        saved = self.backup()
        del self.motions[name]
        self.save()
        return saved

    # ── 회차 ────────────────────────────────────────────────────────────
    def add_take(self, name: str, points: list[dict], label: str = "", ref_pan: float | None = None) -> str:
        entry = self.motion(name)
        points = [check_point(p) for p in points]
        if not points:
            raise MotionError("지점이 하나도 없어요. '지점 추가'를 먼저 누르세요.")
        if len(points) > MAX_POINTS:
            raise MotionError(f"지점은 {MAX_POINTS}개까지예요.")
        number = int(entry.get("next", 1))
        tid = f"t{number}"
        entry["next"] = number + 1
        entry["takes"][tid] = {"label": (str(label).strip() or f"{number}차")[:30], "created": now_text(),
                               "ref_pan": None if ref_pan is None else round(float(ref_pan), 2),
                               "points": points, "runs": []}
        if entry.get("chosen") is None:              # 첫 회차는 일단 그것을 쓴다 (나중에 바꿀 수 있다)
            entry["chosen"] = tid
        self.save()
        return tid

    def rename_take(self, name: str, tid: str, label: str) -> None:
        label = str(label or "").strip()
        if not 1 <= len(label) <= 30:
            raise MotionError("회차 이름은 1~30자로 적어 주세요.")
        self.take(name, tid)["label"] = label
        self.save()

    def delete_take(self, name: str, tid: str) -> Path | None:
        entry = self.motion(name)
        self.take(name, tid)
        saved = self.backup()
        del entry["takes"][tid]
        if entry.get("chosen") == tid:               # 쓰던 회차를 지웠으면 추천 회차(없으면 남은 첫 회차)로
            entry["chosen"] = self.recommend(name) or next(iter(entry["takes"]), None)
        self.save()
        return saved

    def choose(self, name: str, tid: str) -> None:
        self.take(name, tid)
        self.motion(name)["chosen"] = tid
        self.save()

    def chosen(self, name: str) -> tuple[str, dict]:
        """미션이 쓸 회차. 없으면 MotionError"""
        entry = self.motion(name)
        tid = entry.get("chosen")
        if tid is None or tid not in entry["takes"]:
            raise MotionError(f"동작 '{name}' 에 선택된 회차가 없어요 (허브 3번에서 가르치고 ★ 로 고르세요).")
        return tid, entry["takes"][tid]

    # ── 재생 기록 · 성적 ────────────────────────────────────────────────
    def add_run(self, name: str, tid: str, report: dict) -> None:
        runs = self.take(name, tid)["runs"]
        runs.append({"t": now_text(), "reached": bool(report.get("reached")), "stopped": bool(report.get("stopped")),
                     "max_err": report.get("max_err"), "ok": None})
        del runs[:-MAX_RUNS_KEPT]
        self.save()

    def mark_run(self, name: str, tid: str, ok: bool) -> None:
        """방금(가장 최근) 재생을 사람이 '잘 됨 / 안 됨' 으로 표시"""
        runs = self.take(name, tid)["runs"]
        if not runs:
            raise MotionError("아직 재생한 적이 없어요. 먼저 재생해 보세요.")
        runs[-1]["ok"] = bool(ok)
        self.save()

    def stats(self, name: str, tid: str) -> dict:
        runs = [r for r in self.take(name, tid)["runs"] if not r.get("stopped")]
        errs = [r["max_err"] for r in runs if r.get("max_err") is not None]
        ok = sum(1 for r in runs if r.get("ok") is True)
        bad = sum(1 for r in runs if r.get("ok") is False)
        return {"plays": len(runs), "ok": ok, "bad": bad,
                "reached": sum(1 for r in runs if r.get("reached")),
                "mean_err": round(sum(errs) / len(errs), 2) if errs else None,
                "worst_err": round(max(errs), 2) if errs else None}

    def score(self, name: str, tid: str) -> tuple | None:
        """클수록 안정적. 재생한 적이 없으면 None (추천하지 않는다)"""
        s = self.stats(name, tid)
        if s["plays"] == 0:
            return None
        judged = s["ok"] + s["bad"]
        ok_rate = s["ok"] / judged if judged else 0.5        # 사람이 표시하지 않았으면 중간값
        reach_rate = s["reached"] / s["plays"]
        return (round(ok_rate, 3), round(reach_rate, 3), -(s["mean_err"] if s["mean_err"] is not None else 99.0), s["plays"])

    def recommend(self, name: str) -> str | None:
        best = None
        for tid in self.motion(name)["takes"]:
            sc = self.score(name, tid)
            if sc is not None and (best is None or sc > best[0]):
                best = (sc, tid)
        return best[1] if best else None

    # ── 기준 차 위치 ────────────────────────────────────────────────────
    def set_car_ref(self, pan: float, box: list | None = None) -> None:
        """차가 '기준 정차 위치'에 서 있을 때 차를 바라보는 받침 각도. 미션이 이 각도에서 정지 신호를 낸다"""
        self.data["car_ref"] = {"pan": round(float(pan), 2), "box": box, "t": now_text()}
        self.save()

    def car_ref_pan(self) -> float | None:
        ref = self.data.get("car_ref")
        return None if not ref else ref.get("pan")

    def summary(self) -> list[dict]:
        """화면용 요약"""
        out = []
        for name, entry in self.motions.items():
            best = self.recommend(name)
            takes = []
            for tid, t in entry["takes"].items():
                last = t["runs"][-1] if t["runs"] else None
                takes.append({"id": tid, "label": t["label"], "created": t["created"], "points": len(t["points"]),
                              "ref_pan": t.get("ref_pan"), "stats": self.stats(name, tid),
                              "chosen": entry.get("chosen") == tid, "recommended": best == tid,
                              "last_ok": None if last is None else last.get("ok"),
                              "last_marked": bool(last is not None and last.get("ok") is not None)})
            out.append({"name": name, "relative": bool(entry.get("relative")), "takes": takes,
                        "help": DEFAULT_MOTIONS.get(name, (None, ""))[1]})
        return out


# ── 지점 ────────────────────────────────────────────────────────────────
def check_point(p: dict) -> dict:
    try:
        angles = {str(j): round(float(p["angles"][str(j)] if str(j) in p["angles"] else p["angles"][j]), 2) for j in ARM_JOINTS}
    except (KeyError, TypeError, ValueError) as e:
        raise MotionError("지점에 관절 1~5 의 각도가 다 들어 있지 않아요.") from e
    grip = p.get("grip")
    return {"angles": angles, "grip": None if grip is None else round(max(0.0, min(100.0, float(grip))), 1),
            "pause": round(max(0.0, min(10.0, float(p.get("pause", DEFAULT_PAUSE)))), 2)}


def capture_point(robot) -> dict:
    """지금 팔 모양을 지점 하나로 읽는다 (토크가 꺼져 있어도 읽을 수 있다)"""
    angles = robot.angles()
    if any(angles.get(j) is None for j in ARM_JOINTS):
        raise MotionError("관절 각도를 읽지 못했어요. 전원과 케이블을 확인하세요.")
    return check_point({"angles": {j: angles[j] for j in ARM_JOINTS}, "grip": robot.gripper_percent()})


def play(robot, points: list[dict], *, speed: int = 120, pan_offset: float = 0.0, stop=None, on_step=None,
         too_hot=None, settle_timeout: float = 12.0, reach_tol: float = REACH_TOL_DEG, sleep=time.sleep) -> dict:
    """지점을 순서대로 간다. 토크가 켜져 있어야 한다 (꺼져 있으면 RobotError).

    pan_offset : 받침(1번 관절)에 더할 각도 (차가 기준 위치와 다른 곳에 섰을 때)
    stop       : threading.Event. set 되면 지금 지점까지만 가고 멈춘다
    too_hot    : 호출하면 True/False. True 면 멈춘다 (모터 과열)
    돌려주는 값: {"reached": 모든 지점 도달?, "stopped": 도중에 멈췄나, "errors": [지점별 오차(도)], "max_err", "done": 간 지점 수, "why"}
    """
    report = {"reached": True, "stopped": False, "errors": [], "max_err": None, "done": 0, "why": ""}
    last_grip = robot.gripper_percent()
    for i, p in enumerate(points):
        if stop is not None and stop.is_set():
            report.update(stopped=True, why="정지 버튼")
            break
        if too_hot is not None and too_hot():
            report.update(stopped=True, why="모터 과열")
            break
        if on_step:
            on_step(i, len(points))
        target = {int(j): float(v) for j, v in p["angles"].items()}
        target[1] += pan_offset
        robot.move_all(target, speed=speed)
        robot.wait_until_settled(timeout=settle_timeout)
        now = robot.angles()
        errs = [abs(now[j] - target[j]) for j in ARM_JOINTS if now.get(j) is not None]
        err = round(max(errs), 2) if len(errs) == len(ARM_JOINTS) else None
        report["errors"].append(err)
        if err is None or err > reach_tol:
            report["reached"] = False
        grip = p.get("grip")
        if grip is not None and (last_grip is None or abs(grip - last_grip) > GRIP_CHANGE_PCT):
            robot.gripper(grip, speed=speed)
            robot.wait_until_settled(timeout=4.0)
            sleep(0.3)
            last_grip = grip
        sleep(p.get("pause", DEFAULT_PAUSE))
        report["done"] = i + 1
    known = [e for e in report["errors"] if e is not None]
    report["max_err"] = max(known) if known else None
    if report["stopped"] or report["done"] < len(points):
        report["reached"] = False
    return report
