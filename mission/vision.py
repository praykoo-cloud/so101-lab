"""카메라로 보는 일 전부: 기준색(티칭), 포스트잇 찾기, 색 강제 선택, 공 찾기, 도착·떠남 확인.

핵심 규칙 (기획안)
- 색은 **티칭한 기준색**(note_red / note_blue)과 비교해서 **가까운 쪽을 항상 고른다** ("판정 불가" 상태 없음).
- 색은 **포스트잇 영역 안에서만** 본다. 먼저 위치 · 모양으로 포스트잇을 찾고, 그 픽셀만 비교한다.
  (주변 공 · 바구니 색의 면적은 영향을 주지 않는다)
- 기준색은 주문이 정해지기 전에 빨강 · 파랑 둘 다 등록하고 **잠근다** (colors.lock). 잠그면 고칠 수 없다.
- 성공 판정은 하나의 함수(check_success)만 쓴다: 바구니 안에서 공이 오래(안정) 보이고 + 그리퍼가 비어 있을 때만 성공.
  "공이 안 보임" 은 성공 증거가 아니다.

cv2 · numpy 가 필요하다 (.venv 에 설치됨).
"""
from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .config import RUNTIME_DIR, MissionConfig

COLORS_FILE = RUNTIME_DIR / "colors.json"
LOCK_FILE = RUNTIME_DIR / "colors.lock"
REF_NAMES = ("note_red", "note_blue", "ball_red", "ball_blue")
DEFAULT_COLOR = "red"        # 거리 · 투표가 정확히 같을 때만 쓰는 고정값 (로그에 남긴다)


# ══ 기준색 (티칭) ═══════════════════════════════════════════════════════
def hue_dist(a: float, b: float) -> float:
    """색상(0~180, 원형) 차이를 0~1 로"""
    d = abs(float(a) - float(b))
    return min(d, 180.0 - d) / 90.0


def hsv_distance(hsv, ref: dict) -> float:
    """색 하나(h,s,v)와 기준색 사이 거리. 색상을 가장 크게 본다"""
    return (0.6 * hue_dist(hsv[0], ref["h"])
            + 0.3 * abs(float(hsv[1]) - ref["s"]) / 255.0
            + 0.1 * abs(float(hsv[2]) - ref["v"]) / 255.0)


def circular_mean_hue(hues) -> float:
    ang = np.asarray(hues, dtype=float) * (math.pi / 90.0)          # OpenCV 색상 1 = 2도
    m = math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))
    return (m * 90.0 / math.pi) % 180.0


class ColorRefs:
    """티칭한 기준색 모음. refs[name] = {"h","s","v","h_tol","s_tol","v_tol","n"}"""

    def __init__(self, refs: dict | None = None):
        self.refs = refs or {}

    def add_samples(self, name: str, samples: list[tuple[float, float, float]]) -> dict:
        """samples: [(h, s, v), ...] 여러 장에서 뽑은 값. 평균과 허용 폭(퍼짐 기준)을 저장"""
        if name not in REF_NAMES:
            raise ValueError(f"기준색 이름은 {REF_NAMES} 중 하나")
        if len(samples) < 3:
            raise ValueError("샘플이 너무 적습니다 (3장 이상)")
        arr = np.asarray(samples, dtype=float)
        h = circular_mean_hue(arr[:, 0])
        dh = np.array([min(abs(x - h), 180 - abs(x - h)) for x in arr[:, 0]])
        ref = {
            "h": round(h, 2), "s": round(float(arr[:, 1].mean()), 2), "v": round(float(arr[:, 2].mean()), 2),
            "h_tol": round(max(8.0, 3.0 * float(dh.std()) + float(dh.max())), 2),
            "s_tol": round(max(60.0, 3.0 * float(arr[:, 1].std())), 2),
            "v_tol": round(max(80.0, 3.0 * float(arr[:, 2].std())), 2),
            "n": len(samples),
        }
        self.refs[name] = ref
        return ref

    def complete(self) -> bool:
        return all(n in self.refs for n in REF_NAMES)

    def save(self, path: Path = COLORS_FILE) -> Path:
        if is_locked(path):
            raise PermissionError("기준색이 잠겨 있습니다(주문 배정 전에 등록·검증하고 고정한 설정). 경기 중 수정 금지")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.refs, indent=2, ensure_ascii=False, sort_keys=True), encoding="utf-8")
        return path

    @classmethod
    def read(cls, path: Path = COLORS_FILE) -> "ColorRefs":
        """티칭 도중용: 덜 등록됐어도 있는 것만 읽는다"""
        try:
            refs = json.loads(Path(path).read_text(encoding="utf-8"))
        except FileNotFoundError:
            raise FileNotFoundError(f"기준색 파일이 없습니다: {path} (m0_teach_colors.py 로 티칭)")
        return cls(refs)

    @classmethod
    def load(cls, path: Path = COLORS_FILE, require_lock: bool = True) -> "ColorRefs":
        """대회용: 4개가 모두 있고(+ require_lock 이면 잠겨 있고 내용이 안 바뀌어야) 읽힌다"""
        refs = cls.read(path).refs
        obj = cls(refs)
        if require_lock and not is_locked(path):
            raise PermissionError("기준색이 잠기지 않았습니다. 빨강·파랑을 모두 등록·검증한 뒤 m0_teach_colors.py 에서 lock 하세요")
        if not obj.complete():
            raise ValueError(f"기준색이 덜 등록됐습니다. 필요한 것: {REF_NAMES}, 있는 것: {sorted(refs)}")
        return obj


def _file_sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def lock_colors(path: Path = COLORS_FILE, lock: Path = LOCK_FILE) -> Path:
    """기준색을 잠근다 (주문 배정 전에). 파일 내용의 해시를 적어 두어 몰래 바뀐 것도 알아챈다"""
    refs = ColorRefs.read(path)
    if not refs.complete():                               # 빨강·파랑·공 모두 있어야 잠글 수 있다
        raise ValueError(f"기준색이 덜 등록됐습니다. 필요한 것: {REF_NAMES}, 있는 것: {sorted(refs.refs)}")
    lock.write_text(json.dumps({"sha256": _file_sha(path), "locked_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                                "names": sorted(refs.refs)}, indent=2), encoding="utf-8")
    return lock


def is_locked(path: Path = COLORS_FILE, lock: Path | None = None) -> bool:
    lock = lock or (Path(path).with_name("colors.lock"))
    try:
        info = json.loads(lock.read_text(encoding="utf-8"))
        return info.get("sha256") == _file_sha(path)
    except (FileNotFoundError, ValueError, OSError):
        return False


# ══ 화면에서 찾기 ═══════════════════════════════════════════════════════
def crop(frame, roi):
    x, y, w, h = [int(v) for v in roi]
    return frame[y:y + h, x:x + w], (x, y)


def vivid_mask(hsv, s_min: int = 80, v_min: int = 50):
    """선명한(채도 · 밝기가 어느 정도 되는) 픽셀. 회색 바닥 · 흰 벽은 빠진다"""
    return ((hsv[:, :, 1] >= s_min) & (hsv[:, :, 2] >= v_min)).astype(np.uint8) * 255


def mask_near(hsv, ref: dict, scale: float = 1.0):
    """기준색에 가까운 픽셀"""
    h = hsv[:, :, 0].astype(float)
    dh = np.minimum(np.abs(h - ref["h"]), 180.0 - np.abs(h - ref["h"]))
    ok = ((dh <= ref["h_tol"] * scale)
          & (np.abs(hsv[:, :, 1].astype(float) - ref["s"]) <= ref["s_tol"] * scale)
          & (np.abs(hsv[:, :, 2].astype(float) - ref["v"]) <= ref["v_tol"] * scale))
    return ok.astype(np.uint8) * 255


@dataclass
class Note:
    bbox: tuple          # 화면 전체 기준 (x, y, w, h)
    centroid: tuple      # 화면 전체 기준 (cx, cy)
    area: int
    hsv: tuple           # 포스트잇 픽셀의 중앙값 (h, s, v)


def find_note(frame, roi, cfg: MissionConfig, refs: ColorRefs | None = None) -> Note | None:
    """ROI 안에서 포스트잇을 찾는다: 크기 · 모양(가로세로 비)이 맞는 가장 큰 덩어리.
    refs 가 있으면 빨강/파랑 기준색(허용 폭 1.5 배)에 드는 픽셀만, 없으면 선명한 모든 픽셀 (티칭용)"""
    sub, (ox, oy) = crop(frame, roi)
    if sub.size == 0:
        return None
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    if refs is not None and "note_red" in refs.refs and "note_blue" in refs.refs:
        mask = mask_near(hsv, refs.refs["note_red"], 1.5) | mask_near(hsv, refs.refs["note_blue"], 1.5)
    else:
        mask = vivid_mask(hsv)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = [int(v) for v in stats[i]]
        if not (cfg.note_area_min <= area <= cfg.note_area_max):
            continue
        aspect = w / max(1, h)
        if not (cfg.note_aspect_min <= aspect <= cfg.note_aspect_max):
            continue
        if best is None or area > best[0]:
            best = (area, i, (x, y, w, h), cents[i])
    if best is None:
        return None
    area, i, (x, y, w, h), (cx, cy) = best
    px = hsv[labels == i]
    med = tuple(float(v) for v in np.median(px, axis=0))
    return Note((x + ox, y + oy, w, h), (float(cx + ox), float(cy + oy)), area, med)


BASELINE_FILE = RUNTIME_DIR / "watch_baseline.png"     # 옛 방식: 받침 가운데(0도)에서 찍은 '빈 도로' 한 장
BASELINES_DIR = RUNTIME_DIR / "watch_baselines"        # 받침 각도별 '빈 도로' 사진 (pan_+000.png, pan_-002.png ...)


CAR_VIEWS_DIR = RUNTIME_DIR / "car_views"              # 우리 자동차 등록 사진 (방향별: front_1.png, left_2.png ...)


def car_view_name(direction: str, number: int) -> str:
    return f"{direction}_{number}.png"


def list_car_views(folder=None) -> dict:
    """{방향: [파일 경로, ...]} (사진 번호 순)"""
    from .config import CAR_DIRECTIONS
    folder = Path(CAR_VIEWS_DIR if folder is None else folder)
    out = {d: [] for d in CAR_DIRECTIONS}
    if folder.exists():
        for path in sorted(folder.glob("*.png")):
            direction, _, number = path.stem.rpartition("_")
            if direction in out and number.isdigit():
                out[direction].append(path)
        for paths in out.values():
            paths.sort(key=lambda p: int(p.stem.rpartition("_")[2]))
    return out


def save_car_view(direction: str, crop, folder=None) -> Path:
    """자동차를 네모로 잘라 낸 사진을 저장한다 (방향마다 번호가 1, 2, 3 … 으로 늘어난다)"""
    from .config import CAR_DIRECTIONS
    if direction not in CAR_DIRECTIONS:
        raise ValueError(f"모르는 방향: {direction}")
    folder = Path(CAR_VIEWS_DIR if folder is None else folder)
    folder.mkdir(parents=True, exist_ok=True)
    number = len(list_car_views(folder)[direction]) + 1
    path = folder / car_view_name(direction, number)
    if not cv2.imwrite(str(path), crop):
        raise ValueError("사진을 저장하지 못했어요. 폴더 권한을 확인하세요.")
    return path


def _hist(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [8, 4, 4], [0, 180, 0, 256, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def car_look_score(frame, bbox, folder=None) -> float | None:
    """화면의 한 덩어리가 '등록한 우리 자동차 사진들'과 색 분포로 얼마나 닮았는지 (0~1, 클수록 닮음).
    등록한 사진이 없으면 None (= 판단하지 않음). 완벽한 방법은 아니고 '엉뚱한 물체'를 걸러내는 보조 수단이다"""
    x, y, w, h = [int(v) for v in bbox]
    candidate = frame[max(0, y):y + h, max(0, x):x + w]
    if candidate.size == 0:
        return 0.0
    cand = _hist(candidate)
    best = None
    for paths in list_car_views(folder).values():
        for path in paths:
            image = cv2.imread(str(path))
            if image is None or image.size == 0:
                continue
            score = max(0.0, float(cv2.compareHist(cand.astype(np.float32), _hist(image).astype(np.float32), cv2.HISTCMP_CORREL)))
            best = score if best is None else max(best, score)
    return best


def baseline_name(offset: int) -> str:
    return f"pan_{offset:+04d}.png"


def read_baselines(folder=None, legacy=None) -> dict:
    """{각도(정수 도): 사진}. 폴더에 사진이 없으면 옛 한 장 사진을 0도로 쓴다 (경로는 부를 때 정한다 → 시험에서 바꿔 끼울 수 있음)"""
    out = {}
    folder = Path(BASELINES_DIR if folder is None else folder)
    legacy = BASELINE_FILE if legacy is None else legacy
    if folder.exists():
        for path in sorted(folder.glob("pan_*.png")):
            try:
                offset = int(path.stem.split("_", 1)[1])
            except ValueError:
                continue
            image = cv2.imread(str(path))
            if image is not None:
                out[offset] = image
    if not out and Path(legacy).exists():
        image = cv2.imread(str(legacy))
        if image is not None:
            out[0] = image
    return out


@dataclass
class Car:
    bbox: tuple          # 화면 전체 기준 (x, y, w, h)
    centroid: tuple      # (cx, cy)
    area: int


@dataclass
class CarSeen:
    """한 프레임에서 본 결과. car 가 None 이면 차를 못 찾음"""
    car: Car | None
    width: int
    height: int


def find_car(frame, baseline, roi, cfg: MissionConfig) -> Car | None:
    """빈 도로 기준 사진과 비교해 **바뀐 가장 큰 덩어리** = 차 후보 (색은 보지 않는다).
    roi = 차가 다니는 도로 구역 (없으면 화면 전체). 조명이 화면 전체에서 바뀐 경우는 너무 커서 제외된다"""
    if baseline is None or frame.shape != baseline.shape:
        return None
    if roi is None or roi[2] <= 0 or roi[3] <= 0:
        roi = [0, 0, frame.shape[1], frame.shape[0]]
    sub, (ox, oy) = crop(frame, roi)
    base, _ = crop(baseline, roi)
    if sub.size == 0:
        return None
    gray = cv2.GaussianBlur(cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY), (7, 7), 0)
    base_gray = cv2.GaussianBlur(cv2.cvtColor(base, cv2.COLOR_BGR2GRAY), (7, 7), 0)
    mask = (cv2.absdiff(gray, base_gray) >= cfg.track_diff_threshold).astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = [int(v) for v in stats[i]]
        if cfg.track_min_area <= area <= cfg.track_max_area and (best is None or area > best[0]):
            best = (area, (x + ox, y + oy, w, h), (float(cents[i][0] + ox), float(cents[i][1] + oy)))
    return Car(best[1], best[2], best[0]) if best else None


@dataclass
class ColorVote:
    color: str
    counts: dict
    mean_dist: dict
    used_default: bool = False        # 동률이라 고정 기본값을 썼는가 (로그에 남긴다)
    frames_used: int = 0


def classify_forced(note: Note, refs: ColorRefs) -> tuple[str, float, float]:
    """포스트잇 색을 빨강/파랑 중 **가까운 쪽으로 항상** 고른다. → (색, 빨강 거리, 파랑 거리)"""
    dr = hsv_distance(note.hsv, refs.refs["note_red"])
    db = hsv_distance(note.hsv, refs.refs["note_blue"])
    return ("red" if dr <= db else "blue"), dr, db


def vote_color(notes: list[Note], refs: ColorRefs) -> ColorVote | None:
    """여러 프레임의 포스트잇을 투표. 동률이면 거리 합이 작은 쪽, 그래도 같으면 고정 기본값"""
    if not notes:
        return None
    counts = {"red": 0, "blue": 0}
    dsum = {"red": 0.0, "blue": 0.0}
    for nt in notes:
        c, dr, db = classify_forced(nt, refs)
        counts[c] += 1
        dsum["red"] += dr
        dsum["blue"] += db
    mean = {k: dsum[k] / len(notes) for k in dsum}
    if counts["red"] != counts["blue"]:
        color, used_default = max(counts, key=counts.get), False
    elif abs(mean["red"] - mean["blue"]) > 1e-9:
        color, used_default = min(mean, key=mean.get), False
    else:
        color, used_default = DEFAULT_COLOR, True
    return ColorVote(color, counts, {k: round(v, 4) for k, v in mean.items()}, used_default, len(notes))


def ball_area(frame, roi, ref: dict | None) -> int:
    """ROI 안에서 공 기준색에 드는 픽셀 수"""
    if ref is None:
        return 0
    sub, _ = crop(frame, roi)
    if sub.size == 0:
        return 0
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(mask_near(hsv, ref, 1.2), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return int(cv2.countNonZero(mask))


def motion_amount(a, b, roi) -> float:
    """두 프레임 사이 ROI 의 평균 밝기 차이 (0~255). 사람 손 · 물체가 움직이면 커진다"""
    sa, _ = crop(a, roi)
    sb, _ = crop(b, roi)
    if sa.size == 0 or sa.shape != sb.shape:
        return 0.0
    return float(np.abs(cv2.cvtColor(sa, cv2.COLOR_BGR2GRAY).astype(int)
                        - cv2.cvtColor(sb, cv2.COLOR_BGR2GRAY).astype(int)).mean())


# ══ 프레임 공급 ═════════════════════════════════════════════════════════
class CameraSource:
    """실제 카메라 (soarm.color.open_camera 와 같은 방식)"""

    def __init__(self, cfg: MissionConfig, studio: bool = False):
        from .studio import open_capture            # studio=True 면 ArmStudio 가상 카메라
        self.cap = open_capture(cfg, studio)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class ListSource:
    """시험용: 미리 만든 프레임 목록을 차례로 준다 (끝나면 마지막 것을 계속)"""

    def __init__(self, frames):
        self.frames = list(frames)
        self.i = 0

    def read(self):
        if not self.frames:
            return None
        f = self.frames[min(self.i, len(self.frames) - 1)]
        self.i += 1
        return f

    def close(self):
        pass


# ══ 실제 카메라로 보는 Vision ═══════════════════════════════════════════
class Vision:
    """기획안의 관측 함수 모음. 시간(clock)과 쉬기(sleep)를 바꿔 끼울 수 있어서 시험이 빠르다"""

    def __init__(self, source, cfg: MissionConfig, refs: ColorRefs, clock=time.monotonic, sleep=time.sleep):
        self.src, self.cfg, self.refs = source, cfg, refs
        self.clock, self.sleep = clock, sleep
        self._arr_start = None
        self._arr_pos = None
        self._left_since = None
        self.last_vote: ColorVote | None = None

    def _frame(self):
        f = self.src.read()
        if f is None:
            self.sleep(0.05)
        return f

    # 도착 ------------------------------------------------------------
    def arrival_step(self) -> bool:
        """wait 자세에서 한 프레임 보고, 포스트잇이 arrival 구역 안에서 stop_seconds 동안 멈춰 있으면 True"""
        f = self._frame()
        note = find_note(f, self.cfg.roi("note"), self.cfg, self.refs) if f is not None else None
        if note is not None and self.cfg.roi_taught("arrival"):
            x, y, w, h = self.cfg.roi("arrival")
            cx, cy = note.centroid
            if not (x <= cx <= x + w and y <= cy <= y + h):
                note = None
        if note is None:
            self._arr_start = self._arr_pos = None
            return False
        now = self.clock()
        if self._arr_pos is None or math.dist(note.centroid, self._arr_pos) > self.cfg.stop_move_px:
            self._arr_start, self._arr_pos = now, note.centroid       # 움직였으니 다시 시작
            return False
        return (now - self._arr_start) >= self.cfg.stop_seconds

    def reset_arrival(self):
        self._arr_start = self._arr_pos = None

    # 추적 (watch 자세에서 차 찾기) ------------------------------------
    def load_baselines(self, folder=None, legacy=None):
        """받침 각도별 빈 도로 사진을 읽는다 {watch 자세 기준 각도(도): 사진}. 옛 한 장 사진은 0도로 쓴다"""
        self.baselines = read_baselines(folder, legacy)
        return bool(self.baselines)

    def has_baseline(self) -> bool:
        if getattr(self, "baselines", None) is None:
            self.load_baselines()
        return 0 in (getattr(self, "baselines", None) or {})        # 최소한 받침이 가운데(0도)일 때의 사진은 있어야 한다

    def car_step(self, pan_offset: float = 0.0) -> CarSeen | None:
        """한 프레임을 보고 차를 찾는다. **지금 받침 각도(watch 자세 기준 pan_offset)와 가장 가까운 기준 사진**과 비교한다.
        가까운 사진이 없으면(허용 각도 밖) 틀린 비교를 하느니 '못 찾음'으로 둔다. 프레임이 안 들어오면 None"""
        f = self._frame()
        if f is None or not self.has_baseline():
            return None
        key = min(self.baselines, key=lambda k: abs(k - pan_offset))
        if abs(key - pan_offset) > self.cfg.track_baseline_tol_deg:
            return CarSeen(None, f.shape[1], f.shape[0])
        roi = self.cfg.roi("track_road") if self.cfg.roi_taught("track_road") else None
        car = find_car(f, self.baselines[key], roi, self.cfg)
        if car is not None and self.cfg.track_use_car_look:        # 등록한 우리 자동차 모양과 닮지 않으면 다른 물체로 본다
            score = car_look_score(f, car.bbox)
            if score is not None and score < self.cfg.track_look_min_score:
                car = None
        return CarSeen(car, f.shape[1], f.shape[0])

    # 색 --------------------------------------------------------------
    def read_color(self) -> ColorVote | None:
        notes = []
        for _ in range(self.cfg.vote_frames * 3):
            f = self._frame()
            if f is None:
                continue
            nt = find_note(f, self.cfg.roi("note"), self.cfg, self.refs)
            if nt is not None:
                notes.append(nt)
            if len(notes) >= self.cfg.vote_frames:
                break
        self.last_vote = vote_color(notes, self.refs)
        return self.last_vote

    # 공 --------------------------------------------------------------
    def _ball_present(self, roi_name: str, color: str) -> bool:
        f = self._frame()
        if f is None or not self.cfg.roi_taught(roi_name):
            return False
        return ball_area(f, self.cfg.roi(roi_name), self.refs.refs.get(f"ball_{color}")) >= self.cfg.ball_area_min

    def ball_at_origin(self, color: str) -> bool:
        return self._ball_present(f"stand_{color}", color)

    def ball_in_gripper(self, color: str) -> bool:
        return self._ball_present("gripper", color)

    def ball_outside(self, color: str) -> bool:
        return self._ball_present("outside", color)

    def ball_inside_stable(self, color: str, seconds: float) -> bool:
        """바구니 안에서 공이 seconds 동안 **계속** 보이면 True (한 번이라도 안 보이면 False)"""
        t0 = self.clock()
        seen = 0
        while True:
            if not self._ball_present("basket_inside", color):
                return False
            seen += 1
            if self.clock() - t0 >= seconds and seen >= 2:
                return True
            self.sleep(0.05)

    def check_success(self, color: str) -> str:
        """**성공 판정은 이 함수 하나만 쓴다.** → "success" | "fail" | "unknown"
        success: 바구니 안에서 공이 안정적으로 보이고 + 그리퍼가 비어 있음
        fail   : 공이 그리퍼에 남았거나 바구니 밖에 보임 (양성 증거가 있을 때만)
        unknown: 그 밖 (예: 아무 데서도 안 보임)"""
        if self.ball_in_gripper(color) or self.ball_outside(color):
            return "fail"
        if self.ball_inside_stable(color, self.cfg.success_stable_seconds):
            return "success"
        return "unknown"

    # 사람 · 떠남 -----------------------------------------------------
    def area_quiet(self, seconds: float, threshold: float = 4.0) -> bool:
        """받침대 주변(stand_area)에서 seconds 동안 움직임이 없으면 True"""
        if not self.cfg.roi_taught("stand_area"):
            return False
        prev = self._frame()
        t0 = self.clock()
        while self.clock() - t0 < seconds:
            self.sleep(0.1)
            cur = self._frame()
            if prev is not None and cur is not None and motion_amount(prev, cur, self.cfg.roi("stand_area")) > threshold:
                return False
            prev = cur
        return True

    def car_left_step(self) -> bool:
        """포스트잇이 arrival 구역에서 t_left 초 동안 안 보이면 True"""
        f = self._frame()
        note = find_note(f, self.cfg.roi("note"), self.cfg, self.refs) if f is not None else None
        now = self.clock()
        if note is not None:
            self._left_since = None
            return False
        self._left_since = self._left_since if self._left_since is not None else now
        return (now - self._left_since) >= self.cfg.t_left
