"""허브 4번 '바구니 넣기 학습' 의 저장소: 세트 · 회차 · 캡처 사진 · 모터 값을 보관하고 언제든 다시 불러온다.

저장 위치
    data/place_learn.json                     이름 · 모터 값 · 찾은 바구니 위치 · 성공 여부
    data/place_learn/<세트번호>/watch.jpg      대기 자세 YOLO 캡처 (세트마다 한 번)
    data/place_learn/<세트번호>/<회차번호>_<단계>.jpg   회차마다 3 · 5 · 9 단계 YOLO 캡처
    mission/runtime/backup/                   지우기 전에 원본을 복사해 두는 곳

구조 (책장 비유)
    세트(책) = 대기 자세 1개 + 여러 회차.   ★ 하나가 실행용 세트
    회차(쪽) = 3 정차 사진 (완전 정차 순간의 YOLO 사진 + 상자 중심 x·y · 가로·세로) → 5 바구니 위 모터 값 → 9 성공 확인 모터 값.
              ✔ 사용 / ✘ 제외.  3 은 '정차 사진 → 5 모터 값' 학습용 데이터 (없어도 회차는 저장된다)
    (2026-11-16 바뀜: 6 집게 해제는 늘 RELEASE_PCT(60%) 로 열고, 7 은 5 자세로 다시 간다 → 가르치지 않는다.
     그 전 회차의 6 바구니 안 · 7 집게 해제 · 8 들어올리기 값은 파일에 그대로 남아 있다)

단어장
- 세트 · 회차 : 위 비유 참고. 폴더 이름은 s1, t1 처럼 영어 번호, 화면에 보이는 이름은 한글로 따로 둔다
- 평균(mean)  : ✔ 사용 회차들의 값을 더해 개수로 나눈 것. 실행할 때 이 값을 기준으로 쓴다
"""
from __future__ import annotations

import json
import os
import re
import shutil
from datetime import datetime
from pathlib import Path

from .config import RUNTIME_DIR
from .place_model import fit as fit_model, samples_from_takes

ROOT = Path(__file__).resolve().parent.parent
PLACE_FILE = ROOT / "data" / "place_learn.json"
PLACE_DIR = ROOT / "data" / "place_learn"

# 단계 이름 (영어 = 파일 · 코드용, 한글 = 화면용)
STEPS = ("stop", "above", "check")                 # 지금 가르치는 단계 (3 정차 사진 · 5 바구니 위 · 9 성공 확인)
REQUIRED_STEPS = ("above", "check")                # 회차로 저장하려면 꼭 있어야 하는 단계
LEGACY_STEPS = ("inside", "release", "lift")       # 2026-11-16 전 회차에만 있는 단계 (예전 넣기 방식 · 시험용)
STEP_NUM = {"stop": 3, "above": 5, "inside": 6, "release": 7, "lift": 8, "check": 9}
STEP_LABELS = {"stop": "3 정차 사진", "above": "5 바구니 위", "check": "9 성공 확인"}
LEGACY_LABELS = {"inside": "6 바구니 안", "release": "7 집게 해제", "lift": "8 들어올리기"}
RELEASE_PCT = 60.0                                 # 6 집게 해제: 늘 이 % 로 연다 (가르치지 않음)


def take_step(take: dict, step: str) -> dict | None:
    """회차의 한 단계. 3 정차 사진이 없는 회차(예전 회차 · 5 부터 학습)는 None.
    들어올리기(lift)가 생기기 전 회차는 5(바구니 위) 자세 + 7 의 집게 % 로 채운다 (filled=True)"""
    steps = take["steps"]
    if step in steps:
        return steps[step]
    if step == "stop":
        return None
    if step == "lift" and "above" in steps:
        filled = dict(steps["above"])
        filled["grip"] = steps.get("release", {}).get("grip", filled.get("grip"))
        filled["basket"] = None
        filled["image"] = None
        filled["filled"] = True
        return filled
    raise PlaceError(f"회차에 {STEP_LABELS.get(step) or LEGACY_LABELS.get(step, step)} 값이 없어요.")
JOINTS = ("1", "2", "3", "4", "5")
ID_RE = re.compile(r"^[st][0-9]{1,6}$")


class PlaceError(ValueError):
    """잘못된 요청. 메시지는 화면에 그대로 보여 준다"""


def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def clean_label(text, what: str = "이름") -> str:
    text = str(text or "").strip()
    if not 1 <= len(text) <= 30:
        raise PlaceError(f"{what}은 1~30자로 적어 주세요.")
    if any(ch in text for ch in '/\\"\'<>\n\r\t'):
        raise PlaceError(f"{what}에 / \\ 따옴표 < > 는 쓸 수 없어요.")
    return text


def make_snap(angles: dict, grip, basket: dict | None, image: str | None) -> dict:
    """한 순간의 기록: 관절 1~5 각도 · 그리퍼 % · 찾은 바구니 · 사진 파일 이름"""
    try:
        clean = {j: round(float(angles[j] if j in angles else angles[int(j)]), 2) for j in JOINTS}
    except (KeyError, TypeError, ValueError) as e:
        raise PlaceError("관절 1~5 각도를 다 읽지 못했어요. 전원과 케이블을 확인하세요.") from e
    return {"angles": clean, "grip": None if grip is None else round(float(grip), 1),
            "basket": basket, "image": image, "t": now_text()}


class PlaceBook:
    def __init__(self, path: Path | str | None = None, image_dir: Path | str | None = None,
                 backup_dir: Path | str | None = None):
        self.path = Path(path) if path else PLACE_FILE
        self.image_dir = Path(image_dir) if image_dir else PLACE_DIR
        self.backup_dir = Path(backup_dir) if backup_dir else RUNTIME_DIR / "backup"
        self.data = self._read()

    # ── 파일 ────────────────────────────────────────────────────────────
    def _read(self) -> dict:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            data = {}
        except (ValueError, OSError) as e:
            raise PlaceError(f"넣기 학습 파일을 읽지 못했어요 ({self.path}): {e}") from e
        data.setdefault("version", 1)
        data.setdefault("chosen", None)
        data.setdefault("next", 1)
        data.setdefault("sets", {})
        return data

    def reload(self) -> None:
        self.data = self._read()

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
        dest = self.backup_dir / f"place_learn-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.json"
        shutil.copy2(self.path, dest)
        return dest

    def write_image(self, sid: str, name: str, jpeg: bytes | None) -> str | None:
        if not jpeg:
            return None
        folder = self.image_dir / sid
        folder.mkdir(parents=True, exist_ok=True)
        (folder / name).write_bytes(jpeg)
        return name

    def image_path(self, sid: str, name: str) -> Path | None:
        """사진 파일 경로 (세트 폴더 밖은 절대 안 준다)"""
        if not ID_RE.match(sid or "") or not re.match(r"^[A-Za-z0-9_]+\.jpg$", name or ""):
            return None
        base = (self.image_dir / sid).resolve()
        target = (base / name).resolve()
        if target.parent != base or not target.is_file():
            return None
        return target

    # ── 세트 ────────────────────────────────────────────────────────────
    @property
    def sets(self) -> dict:
        return self.data["sets"]

    def get_set(self, sid: str) -> dict:
        if sid not in self.sets:
            raise PlaceError("그 세트가 없어요. 화면을 새로고침하세요.")
        return self.sets[sid]

    def create_set(self, label: str = "") -> str:
        number = int(self.data["next"])
        sid = f"s{number}"
        self.data["next"] = number + 1
        label = clean_label(label, "세트 이름") if str(label or "").strip() else f"세트{number}"
        if any(s["label"] == label for s in self.sets.values()):
            raise PlaceError(f"'{label}' 세트는 이미 있어요.")
        self.sets[sid] = {"label": label, "created": now_text(), "watch": None, "next": 1, "takes": {}}
        if self.data["chosen"] is None:              # 첫 세트는 일단 실행용으로
            self.data["chosen"] = sid
        self.save()
        return sid

    def rename_set(self, sid: str, label: str) -> None:
        label = clean_label(label, "세트 이름")
        entry = self.get_set(sid)
        if any(s["label"] == label for k, s in self.sets.items() if k != sid):
            raise PlaceError(f"'{label}' 세트는 이미 있어요.")
        entry["label"] = label
        self.save()

    def delete_set(self, sid: str) -> Path | None:
        self.get_set(sid)
        saved = self.backup()
        folder = self.image_dir / sid
        if folder.exists():                          # 사진도 지우지 않고 백업 폴더로 옮긴다
            dest = self.backup_dir / f"place_learn-{sid}-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}"
            self.backup_dir.mkdir(parents=True, exist_ok=True)
            shutil.move(str(folder), str(dest))
        del self.sets[sid]
        if self.data["chosen"] == sid:
            self.data["chosen"] = next(iter(self.sets), None)
        self.save()
        return saved

    def choose_set(self, sid: str) -> None:
        self.get_set(sid)
        self.data["chosen"] = sid
        self.save()

    def chosen_set(self) -> str:
        sid = self.data.get("chosen")
        if sid is None or sid not in self.sets:
            raise PlaceError("실행용(★) 세트가 없어요. 허브 4번에서 세트를 만들고 학습하세요.")
        return sid

    def set_watch(self, sid: str, snap: dict, jpeg: bytes | None) -> None:
        entry = self.get_set(sid)
        snap = dict(snap)
        snap["image"] = self.write_image(sid, "watch.jpg", jpeg)
        entry["watch"] = snap
        self.save()

    # ── 회차 ────────────────────────────────────────────────────────────
    def take(self, sid: str, tid: str) -> dict:
        takes = self.get_set(sid)["takes"]
        if tid not in takes:
            raise PlaceError("그 회차가 없어요. 화면을 새로고침하세요.")
        return takes[tid]

    def add_take(self, sid: str, steps: dict, images: dict, success: bool, color: str | None = None,
                 label: str = "") -> str:
        """5 · 9 가 있어야 저장한다 (3 정차 사진은 있으면 같이). images = {단계: JPEG bytes}"""
        entry = self.get_set(sid)
        missing = [STEP_LABELS[s] for s in REQUIRED_STEPS if s not in steps]
        if missing:
            raise PlaceError("아직 저장 안 한 단계가 있어요: " + ", ".join(missing))
        number = int(entry["next"])
        tid = f"t{number}"
        entry["next"] = number + 1
        saved_steps = {}
        for step in STEPS + LEGACY_STEPS:
            if step not in steps:
                continue
            snap = dict(steps[step])
            snap["image"] = self.write_image(sid, f"{tid}_{STEP_NUM[step]}_{step}.jpg", images.get(step))
            saved_steps[step] = snap
        label = clean_label(label, "회차 이름") if str(label or "").strip() else f"{number}회차"
        entry["takes"][tid] = {"label": label, "created": now_text(), "use": bool(success),
                               "success": bool(success), "color": color if color in ("red", "blue") else None,
                               "steps": saved_steps}
        self.save()
        return tid

    def rename_take(self, sid: str, tid: str, label: str) -> None:
        self.take(sid, tid)["label"] = clean_label(label, "회차 이름")
        self.save()

    def set_use(self, sid: str, tid: str, use: bool) -> None:
        self.take(sid, tid)["use"] = bool(use)
        self.save()

    def delete_take(self, sid: str, tid: str) -> Path | None:
        take = self.take(sid, tid)
        saved = self.backup()
        for snap in take["steps"].values():                 # 사진도 지우지 않고 백업 폴더로 옮긴다
            path = self.image_path(sid, snap.get("image") or "")
            if path is not None:
                self.backup_dir.mkdir(parents=True, exist_ok=True)
                shutil.move(str(path), str(self.backup_dir / f"place_learn-{sid}-{path.name}"))
        del self.get_set(sid)["takes"][tid]
        self.save()
        return saved

    def set_cam(self, sid: str, cam: dict) -> None:
        """카메라 방향 재기 결과 (손끝 1 m 움직임당 바구니가 화면에서 움직이는 픽셀 2×2)"""
        entry = self.get_set(sid)
        entry["cam"] = dict(cam, t=now_text())
        self.save()

    def latest_success(self, sid: str) -> tuple[str, dict] | None:
        """가장 최근의 성공 회차 (tid, 회차). 없으면 None. 6~8 부터 학습할 때 앞 단계 값을 여기서 빌린다"""
        takes = [(tid, t) for tid, t in self.get_set(sid)["takes"].items() if t.get("success")]
        if not takes:
            return None
        return max(takes, key=lambda item: int(item[0][1:]))

    def step_image(self, sid: str, snap: dict) -> bytes | None:
        """저장된 단계 사진의 JPEG bytes (없으면 None)"""
        path = self.image_path(sid, snap.get("image") or "")
        return None if path is None else path.read_bytes()

    # ── 학습 값 (실행 · 테스트가 쓰는 기준) ────────────────────────────
    def learned(self, sid: str) -> dict:
        """✔ 사용 회차들의 평균. 없으면 PlaceError"""
        entry = self.get_set(sid)
        used = [t for t in entry["takes"].values() if t.get("use")]
        if not used:
            raise PlaceError(f"세트 '{entry['label']}' 에 ✔ 사용 회차가 없어요. 4번에서 학습하거나 회차를 ✔ 로 바꾸세요.")

        def mean_angles(step):
            return {j: round(sum(take_step(t, step)["angles"][j] for t in used) / len(used), 2) for j in JOINTS}

        def has(t, step):
            try:
                return take_step(t, step) is not None
            except PlaceError:
                return False

        targets = [t["steps"]["above"].get("basket") for t in used]
        targets = [b for b in targets if b]
        poses = {step: mean_angles(step) for step in REQUIRED_STEPS}
        for step in LEGACY_STEPS:                     # 예전 회차만 있으면 예전 단계도 (예전 넣기 방식 · 시험용)
            if all(has(t, step) for t in used):
                poses[step] = mean_angles(step)
        return {
            "set": sid, "label": entry["label"], "count": len(used),
            "poses": poses,
            "target": None if not targets else {
                "cx": round(sum(b["cx"] for b in targets) / len(targets), 1),
                "cy": round(sum(b["cy"] for b in targets) / len(targets), 1),
                "w": round(sum(b["w"] for b in targets) / len(targets), 1)},
            "release_grip": RELEASE_PCT,
            "cam": entry.get("cam"),
            # 정차 사진의 상자 위치 → 5 자세 (2차식). 정차 사진이 있는 ✔ 회차가 모자라면 None → 평균 자세로
            "model": fit_model(samples_from_takes(used)),
        }

    @staticmethod
    def _step_view(take: dict, step: str) -> dict | None:
        snap = take_step(take, step)
        if snap is None:
            return None
        return {"angles": snap["angles"], "grip": snap.get("grip"), "basket": snap.get("basket"),
                "image": snap.get("image"), "t": snap.get("t"), "filled": bool(snap.get("filled"))}

    def summary(self) -> dict:
        """화면용 요약"""
        out = []
        for sid, entry in self.sets.items():
            takes = [{"id": tid, "label": t["label"], "created": t["created"], "use": t.get("use"),
                      "success": t.get("success"), "color": t.get("color"),
                      "steps": {s: self._step_view(t, s) for s in STEPS}}
                     for tid, t in entry["takes"].items()]
            out.append({"id": sid, "label": entry["label"], "created": entry["created"],
                        "chosen": self.data.get("chosen") == sid, "watch": entry.get("watch"), "takes": takes,
                        "cam": entry.get("cam"),
                        "used": sum(1 for t in takes if t["use"])})
        return {"sets": out, "chosen": self.data.get("chosen"), "steps": STEP_LABELS}
