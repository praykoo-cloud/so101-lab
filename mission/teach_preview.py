"""손목 영상으로 자세·구역·색을 설정하는 경기 전 티칭 화면.

Pi: python -m mission.teach_preview --port /dev/ttyACM0   (1~12번을 한 화면에서 고르려면 python -m mission.hub)
VS Code의 Ports에서 8081을 전달하고 브라우저로 연다. 항상 실물 카메라 · 실물 팔로만 동작한다.
"""
from __future__ import annotations

import argparse
import json
import secrets
import time
from collections import deque
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from threading import Event, Thread

from soarm import Robot
from soarm.driver_sdk import JOINT_IDS, JOINT_LIMITS, unwrap_position

from .config import CAR_DIRECTIONS, MissionConfig, POSE_HELP, ROI_HELP
from .m0_teach_colors import teach_from_frames
from .m0_teach_poses import BALL_TOLERANCE_PCT
from .vision import (COLORS_FILE, LOCK_FILE, ColorRefs, ListSource, Vision, classify_forced, find_note,
                     list_car_views, save_car_view)


POSE_LABELS = {
    "watch": "추적 시작 자세 (멀리서 도로 전체)",
    "wait": "포스트잇 대기 자세 (정차 위치)",
    "above_red": "빨간 공 위",
    "red": "빨간 공 집기",
    "above_blue": "파란 공 위",
    "blue": "파란 공 집기",
    "above_basket": "바구니 위",
    "basket": "바구니에 놓기",
    "retreat_mid": "바구니에서 빠져나온 자세",
    "recovery_wait": "떨어진 공을 기다리는 자세",
    "rest": "전원을 끌 때 두는 휴식 자세",
}
ROI_LABELS = {
    "track_road": "추적할 도로 구역 (추적 시작 자세에서)",
    "arrival": "차가 멈추는 구역", "note": "포스트잇을 찾는 구역",
    "stand_red": "받침대의 빨간 공 자리", "stand_blue": "받침대의 파란 공 자리",
    "stand_area": "받침대 주변", "basket_inside": "바구니 안쪽",
    "outside": "바구니 밖 바닥", "gripper": "집게 안쪽",
}
COLOR_LABELS = {
    "note_red": "빨간 포스트잇", "note_blue": "파란 포스트잇",
    "ball_red": "빨간 공", "ball_blue": "파란 공",
}
TITLES = {"view": "카메라 보기", "pose": "자세 티칭", "roi": "구역 설정", "color": "색상 등록",
          "verify": "색상 확인", "arrival": "차 도착 확인", "car": "자동차 모양 등록"}
CAMERA_MAX_AGE = 0.5
PREVIEW_MAX_AGE = 2.0


class LatestCamera:
    """카메라를 계속 읽고, 읽은 시각과 최신 영상 한 장만 남긴다."""

    def __init__(self, capture, sample_colors=False):
        self.capture = capture
        self.latest = None
        self.samples = deque(maxlen=15) if sample_colors else None
        self.ready = Event()
        self.stop = Event()
        self.worker = Thread(target=self.read_frames, daemon=True)
        self.worker.start()

    def read_frames(self):
        import cv2

        try:
            while not self.stop.is_set():
                try:
                    ok, image = self.capture.read()
                except cv2.error:
                    ok, image = False, None
                if ok and image is not None and image.size:
                    self.latest = (time.monotonic(), image)
                    if self.samples is not None:
                        self.samples.append(self.latest)
                    self.ready.set()
                else:
                    self.latest = None
                    if self.samples is not None:
                        self.samples.clear()
                    self.stop.wait(0.05)
        finally:
            self.latest = None
            self.capture.release()

    def snapshot(self):
        sample = self.latest
        if sample is not None and time.monotonic() - sample[0] <= CAMERA_MAX_AGE:
            return sample
        return None

    def close(self):
        self.stop.set()
        self.worker.join(timeout=1)


class Teaching:
    def __init__(self, arm, camera, config, fake, mode="pose"):
        self.arm, self.camera, self.config, self.fake = arm, camera, config, fake
        self.mode = mode
        self.practice_saved = set()
        self.last_frame = None
        self.released = False
        self.finished = False
        self.deleter = None                     # 허브가 넣어 준다: 저장값 지우기 (백업 포함)
        self.on_finish = None                   # 허브가 넣어 준다: '티칭 끝내기'를 누르면 응답을 보내기 전에 허브로 돌아가게
        self.settle_seconds = 0.7               # 받침을 돌린 뒤 흔들림이 가라앉고 새 영상이 들어올 때까지 기다리는 시간 (시험에서만 0)
        self.frozen = None
        self.refs = ColorRefs()
        if not fake and mode in ("color", "verify", "arrival") and COLORS_FILE.exists():
            self.refs = ColorRefs.read(COLORS_FILE)
        if fake and mode in ("verify", "arrival"):
            self.refs.add_samples("note_red", [(0, 230, 230)] * 3)
            self.refs.add_samples("note_blue", [(120, 230, 230)] * 3)
        if fake and mode == "arrival":
            config.rois["arrival"] = [50, 50, 200, 160]
            config.rois["note"] = [50, 50, 200, 160]
        self.arrival_source = ListSource([])
        self.arrival_vision = Vision(self.arrival_source, config, self.refs)
        self.arrival_at = None
        self.arrived = False

    def items(self):
        if self.mode == "pose":
            items = [
                {"name": name, "label": POSE_LABELS[name], "help": desc}
                for name, desc in POSE_HELP.items()
            ]
            items[0]["help"] = "차를 상차 위치에 세우고 포스트잇 전체가 보이도록 팔을 맞추세요."
            items.append({"name": "ballgrip", "label": "공을 쥔 집게",
                          "help": "공을 집게 사이에 넣고 손으로 집게를 닫은 상태에서 저장하세요."})
            return items
        if self.mode == "roi":
            return [{"name": name, "label": label, "help": ROI_HELP[name]}
                    for name, label in ROI_LABELS.items()]
        if self.mode == "color":
            return [{"name": name, "label": label,
                     "help": f"{label}의 색이 고른 부분만 네모로 그리세요. 반사광과 주변 배경은 피하세요."}
                    for name, label in COLOR_LABELS.items()]
        if self.mode == "car":
            return [{"name": name, "label": label,
                     "help": f"우리 자동차의 ‘{label}’ 모습이 보이게 차를 두고 화면을 멈춘 뒤, 차 전체를 네모로 감싸세요. "
                             "같은 방향 사진을 여러 장(조명·거리 다르게) 찍어도 돼요."}
                    for name, label in CAR_DIRECTIONS.items()]
        return []

    def locked(self):
        return not self.fake and self.mode in ("roi", "color") and LOCK_FILE.exists()

    def saved(self):
        if self.mode == "car":
            return sorted(name for name, paths in list_car_views().items() if paths)
        if self.mode == "roi":
            return sorted(name for name in ROI_LABELS if self.config.roi_taught(name))
        if self.mode in ("color", "verify", "arrival", "view"):
            return sorted(self.refs.refs)
        names = set(self.practice_saved) if self.fake else set(self.arm.poses())
        if self.config.grip_ball_max_pct > 0:
            names.add("ballgrip")
        return sorted(names)

    def snapshot(self):
        import cv2

        if self.fake:
            import numpy as np

            frame = np.full((480, 640, 3), (68, 50, 37), dtype=np.uint8)
            cv2.putText(frame, "PRACTICE", (182, 242), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (255, 255, 255), 3)
            cv2.rectangle(frame, (75, 85), (200, 190), (0, 0, 230), -1)
            cv2.rectangle(frame, (420, 90), (516, 166), (230, 0, 0), -1)
            return time.monotonic(), frame
        return self.camera.snapshot()

    @staticmethod
    def jpeg(frame):
        import cv2

        try:
            ok, image = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        except cv2.error:
            return None
        return image.tobytes() if ok else None

    def frame(self):
        sample = self.snapshot()
        if sample is None:
            self.last_frame = None
            self.arrival_vision.reset_arrival()
            self.arrival_at = None
            return None, ""
        captured_at, frame = sample
        result = ""
        if self.mode == "verify":
            frame, result = self.verify(frame)
        elif self.mode == "arrival":
            frame, result = self.arrival(frame, captured_at)
        image = self.jpeg(frame)
        self.last_frame = captured_at if image else None
        return image, result

    def verify(self, frame):
        import cv2

        if not {"note_red", "note_blue"} <= self.refs.refs.keys():
            return frame, "untaught"
        height, width = frame.shape[:2]
        roi = self.config.roi("note") if self.config.roi_taught("note") else [0, 0, width, height]
        note = find_note(frame, roi, self.config, self.refs)
        if note is None:
            return frame, "missing"
        color, _, _ = classify_forced(note, self.refs)
        x, y, w, h = note.bbox
        marked = frame.copy()
        cv2.rectangle(marked, (x, y), (x + w, y + h), (0, 0, 255) if color == "red" else (255, 0, 0), 2)
        return marked, color

    def arrival(self, frame, captured_at):
        import cv2

        if not all(self.config.roi_taught(name) for name in ("arrival", "note")):
            return frame, "needs_zones"
        if not {"note_red", "note_blue"} <= self.refs.refs.keys():
            return frame, "untaught"
        if self.arrival_at is None or captured_at - self.arrival_at > CAMERA_MAX_AGE:
            self.arrival_vision.reset_arrival()
        if captured_at != self.arrival_at:
            self.arrival_source.frames[:] = [frame]
            self.arrived = self.arrival_vision.arrival_step()
            self.arrival_at = captured_at
        marked, _ = self.verify(frame)
        if marked is frame:
            marked = frame.copy()
        x, y, w, h = self.config.roi("arrival")
        cv2.rectangle(marked, (x, y), (x + w, y + h), (30, 200, 40), 2)
        return marked, "arrived" if self.arrived else "waiting"

    def camera_action(self, data):
        if self.mode not in ("roi", "color", "car"):
            raise ValueError("확인 화면에서는 설정을 저장하지 않아요.")
        if self.locked():
            raise ValueError("기준색이 잠겼어요. 주문 배정 전이라면 메뉴 7번에서 잠금을 확인하세요.")
        action = data.get("action")
        if action == "freeze":
            sample = self.snapshot()
            if sample is None:
                raise ValueError("영상이 끊겼어요. 다시 연결된 뒤 화면을 멈춰 주세요.")
            _, image = sample
            frames = [image]
            if self.mode == "color":
                samples = [(stamp, frame) for stamp, frame in self.camera.samples.copy()
                           if time.monotonic() - stamp <= PREVIEW_MAX_AGE] if not self.fake else []
                frames = [frame for _, frame in samples] if not self.fake else [image] * 3
                if len(frames) < 3:
                    raise ValueError("색을 비교할 영상이 부족해요. 잠시 기다렸다가 다시 멈춰 주세요.")
                image = frames[-1]
            self.frozen = (secrets.token_urlsafe(10), image, frames)
            return "멈춘 영상에서 저장할 부분을 마우스로 네모 그리세요."
        if action != "save":
            raise ValueError("저장할 항목을 다시 선택하세요.")
        if not self.frozen or data.get("frozen") != self.frozen[0]:
            raise ValueError("먼저 현재 화면을 멈춘 뒤 네모를 그리세요.")
        labels = {"roi": ROI_LABELS, "car": CAR_DIRECTIONS}.get(self.mode, COLOR_LABELS)
        name = data.get("name")
        if not isinstance(name, str) or name not in labels:
            raise ValueError("저장할 항목을 다시 선택하세요.")
        if self.mode != "car" and name in self.saved() and data.get("replace") is not True:     # 자동차 사진은 방향마다 여러 장 OK
            raise ValueError("이미 저장됐어요. 바꿀지 확인한 뒤 다시 저장하세요.")
        rect = data.get("rect")
        _, frame, frames = self.frozen
        height, width = frame.shape[:2]
        if not isinstance(rect, list) or len(rect) != 4 or any(type(v) is not int for v in rect):
            raise ValueError("영상 위에 마우스로 네모를 그리세요.")
        x, y, w, h = rect
        if not (w >= 4 and h >= 4 and 0 <= x <= width - w and 0 <= y <= height - h):
            raise ValueError("네모가 너무 작거나 화면 밖이에요. 영상 안에 다시 그리세요.")
        if self.mode == "roi":
            config = self.config if self.fake else MissionConfig.load()
            config.rois[name] = rect
            if not self.fake:
                config.save()
            self.config = config
        elif self.mode == "car":
            path = save_car_view(name, frame[y:y + h, x:x + w])
            count = len(list_car_views()[name])
            return f"‘{labels[name]}’ 자동차 사진을 저장했어요 (이 방향 {count}장, 크기 {w}×{h}픽셀). 저장 위치: {path}"
        else:
            refs = ColorRefs.read(COLORS_FILE) if not self.fake and COLORS_FILE.exists() else ColorRefs(dict(self.refs.refs))
            teach_from_frames(refs, name, frames, rect)
            if not self.fake:
                if self.locked():
                    raise ValueError("기준색이 잠겨 있어요.")
                refs.save(COLORS_FILE)
            self.refs = refs
        return f"‘{labels[name]}’ 저장을 완료했어요."

    def details(self):
        """저장된 값을 사람이 읽을 수 있게 (이름 → 설명). 화면의 '저장된 값' 줄에 보인다"""
        cfg, out = self.config, {}
        if self.mode == "pose":
            import soarm.robot as rb

            poses = rb._load_json(rb.POSES_FILE, {})
            for name in POSE_HELP:
                if name in poses:
                    out[name] = f"저장 시각 {poses[name].get('timestamp', '')}"
            if cfg.grip_ball_max_pct > 0:
                out["ballgrip"] = (f"{cfg.grip_ball_min_pct:.0f}% ~ {cfg.grip_ball_max_pct:.0f}% "
                                   f"(쥔 값 ±{BALL_TOLERANCE_PCT:.0f}% 안이면 공을 쥔 것으로 봐요)")
        elif self.mode == "roi":
            for name in ROI_LABELS:
                if cfg.roi_taught(name):
                    x, y, w, h = cfg.roi(name)
                    out[name] = f"x={x}, y={y}, 가로={w}, 세로={h} (픽셀)"
        elif self.mode == "color":
            for name, ref in self.refs.refs.items():
                out[name] = (f"색상 {ref['h']}±{ref['h_tol']} · 채도 {ref['s']}±{ref['s_tol']} · "
                             f"밝기 {ref['v']}±{ref['v_tol']} (샘플 {ref['n']}장)")
        elif self.mode == "car":
            for name, paths in list_car_views().items():
                if paths:
                    out[name] = f"{len(paths)}장 · " + ", ".join(p.name for p in paths) + " (‘저장된 사진 보기’에서 볼 수 있어요)"
        return out

    def limits(self):
        """이 화면에서 알아 두면 좋은 오차 범위 · 기준값 (모두 설정 값 8번에서 고칠 수 있다)"""
        cfg = self.config
        lines = []
        if self.mode == "pose":
            lines += [f"자세 확인 오차 ±{cfg.pose_tolerance_deg}° · 대기 자세에서 관절이 이 안에 있어야 ‘팔이 비켰다’고 알려요.",
                      f"공을 쥔 집게 허용 폭 ±{BALL_TOLERANCE_PCT:.0f}% · 모터 온도 경고 {cfg.temp_warn}° / 중단 {cfg.temp_stop}°"]
        elif self.mode == "roi":
            lines += [f"차가 멈췄다고 보는 기준 · {cfg.stop_seconds}초 동안 움직임 {cfg.stop_move_px}픽셀 이내",
                      f"포스트잇 크기 {cfg.note_area_min}~{cfg.note_area_max} 픽셀 · 가로/세로 비율 {cfg.note_aspect_min}~{cfg.note_aspect_max}"]
        elif self.mode == "color":
            lines += ["색은 등록한 기준색 중 가까운 쪽을 골라요. 허용 폭(±)이 넓을수록 조명 변화에 강하지만 다른 색과 섞일 수 있어요.",
                      f"색 투표 {cfg.vote_frames}장 · 같은 색을 조명이 다른 때마다 한 번씩 더 등록하면 더 튼튼해져요."]
        elif self.mode == "car":
            views = list_car_views()
            total = sum(len(v) for v in views.values())
            lines += [f"등록한 사진 {total}장 · 방향 {sum(1 for v in views.values() if v)}/{len(CAR_DIRECTIONS)}",
                      "차 전체가 네모 안에 들어오게 감싸고, 배경이 많이 들어가지 않게 하세요.",
                      f"추적에서 차로 인정하는 크기 {cfg.track_min_area}~{cfg.track_max_area}픽셀 · "
                      f"모양 확인(등록한 차와 닮은 것만 인정) {'켜짐' if cfg.track_use_car_look else '꺼짐'} · 최소 닮음 {cfg.track_look_min_score:g}"]
        elif self.mode in ("verify", "arrival"):
            lines += [f"정차 기준 · {cfg.stop_seconds}초 동안 움직임 {cfg.stop_move_px}픽셀 이내",
                      f"포스트잇 크기 {cfg.note_area_min}~{cfg.note_area_max} 픽셀 · 투표 {cfg.vote_frames}장"]
        else:
            lines.append(f"카메라 {cfg.camera} · 영상 {cfg.width}×{cfg.height} 픽셀")
        if self.mode in ("pose", "roi"):
            lines.append(f"차 추적 {'켜짐' if cfg.tracking_enabled else '꺼짐'} · 한 번에 최대 {cfg.track_max_step_deg:g}° · 좌우 한계 ±{cfg.track_pan_limit_deg:g}° · "
                         f"정차 {cfg.track_stop_seconds:g}초(움직임 {cfg.track_stop_px:g}픽셀 이내) · 차 크기 {cfg.track_min_area}~{cfg.track_max_area}픽셀 · 방향 {cfg.track_pan_sign:+d}")
        lines.append("이 값들은 허브의 8번(설정 값)에서 언제든 고칠 수 있어요.")
        return lines

    def save_baseline(self):
        """추적용 '빈 도로 기준 사진' 저장. 추적 시작 자세에서, 차 · 사람이 없는 도로를 찍는다 (4번 화면)"""
        import cv2
        from .vision import BASELINES_DIR, baseline_name

        if self.mode != "roi":
            raise ValueError("구역 설정(4번) 화면에서만 찍을 수 있어요.")
        sample = self.snapshot()
        if sample is None:
            raise ValueError("영상이 끊겼어요. 다시 연결된 뒤 찍어 주세요.")
        BASELINES_DIR.mkdir(parents=True, exist_ok=True)
        target = BASELINES_DIR / baseline_name(0)       # 받침이 가운데(0도)일 때의 사진 한 장
        if target.exists():                             # 이전 사진은 .bak 으로 남긴다
            target.replace(target.with_suffix(".bak.png"))
        if not cv2.imwrite(str(target), sample[1]):
            raise ValueError("사진을 저장하지 못했어요. 폴더 권한을 확인하세요.")
        return ("받침이 가운데일 때의 빈 도로 사진 1장을 저장했어요. 지금 화면에 차 · 사람이 없었는지 확인하세요. "
                "추적하려면 3번에서 ‘빈 도로 기준 사진 전체 촬영’(받침 좌우 각도별)도 해야 해요. "
                f"저장 위치: {target}")

    def baseline_sweep(self):
        """받침을 좌우로 step 도씩 돌려 가며 각도별 '빈 도로' 사진을 모두 찍는다 (추적 시작 자세 기준 ±좌우 한계).
        추적 중 받침이 돌면 배경이 밀리므로, 지금 각도와 맞는 사진과 비교하기 위해 필요하다"""
        import shutil

        import cv2
        from .vision import BASELINES_DIR, baseline_name

        if self.mode != "pose" or self.arm is None:
            raise ValueError("자세 티칭(3번) 화면에서만 할 수 있어요.")
        cfg = self.config
        saved = self.arm.poses().get("watch")
        if saved is None:
            raise ValueError("먼저 ‘추적 시작 자세’를 저장하세요.")
        now = self.arm.angles()
        worst = 0.0
        for sid in range(1, 6):
            target, current = saved["angles"].get(sid), now.get(sid)
            if target is None or current is None:
                raise ValueError(f"{sid}번 모터 값을 읽지 못했어요. USB와 모터 연결을 확인하세요.")
            worst = max(worst, abs(current - target))
        if worst > cfg.pose_tolerance_deg:
            raise ValueError(f"팔이 저장한 ‘추적 시작 자세’와 {worst:.1f}° 어긋나요 (허용 ±{cfg.pose_tolerance_deg:g}°). "
                             "‘저장한 자세와 실시간 비교’로 먼저 맞춘 뒤 다시 하세요.")
        if self.snapshot() is None:
            raise ValueError("영상이 끊겼어요. 다시 연결된 뒤 해 주세요.")
        step = max(1.0, cfg.track_baseline_step_deg)
        reach = int(cfg.track_pan_limit_deg // step)
        offsets = [0] + [k * int(round(step)) for k in range(1, reach + 1)] + [-k * int(round(step)) for k in range(1, reach + 1)]
        start = now[1]
        photos, skipped = {}, []
        self.arm.torque(True)                           # 지금 자리에서 잠금
        self.released = False
        try:
            for offset in offsets:
                self.arm.move(1, start + offset, speed=cfg.speed_slow)
                self.arm.wait_until_settled(timeout=5.0)
                time.sleep(self.settle_seconds)         # 흔들림이 가라앉고 새 영상이 들어올 때까지
                reached = (self.arm.angles().get(1) or start) - start
                sample = self.snapshot()
                if sample is None:
                    raise ValueError("촬영 중 영상이 끊겼어요. 연결을 확인하고 다시 하세요. (팔은 원래 자리로 돌려놔요)")
                if abs(reached - offset) > 1.0:         # 관절 범위에 막혀 못 간 각도는 건너뛴다 (엉뚱한 각도 이름으로 저장하지 않음)
                    skipped.append(offset)
                    continue
                photos[offset] = sample[1]
        finally:
            self.arm.move(1, start, speed=cfg.speed_slow)    # 어떤 경우에도 원래 각도로
            self.arm.wait_until_settled(timeout=5.0)
        if 0 not in photos:
            raise ValueError("가운데(0도) 사진을 찍지 못했어요.")
        if BASELINES_DIR.exists():                      # 이전 사진 묶음은 통째로 .bak 폴더에 남긴다
            backup = BASELINES_DIR.with_name(BASELINES_DIR.name + ".bak")
            shutil.rmtree(backup, ignore_errors=True)
            BASELINES_DIR.rename(backup)
        BASELINES_DIR.mkdir(parents=True, exist_ok=True)
        for offset, image in photos.items():
            cv2.imwrite(str(BASELINES_DIR / baseline_name(offset)), image)
        low, high = min(photos), max(photos)
        extra = f" (관절 범위 때문에 못 찍은 각도: {sorted(skipped)})" if skipped else ""
        return (f"빈 도로 기준 사진 {len(photos)}장을 찍었어요 · 받침 {low:+d}° ~ {high:+d}°{extra}. "
                "팔은 원래 각도로 돌려놨고 힘은 켜져 있어요. 이 범위 밖에서는 차를 찾지 않아요(안전).")

    def track_probe(self):
        """추적 방향 확인: 정지한 물체(차)를 보면서 받침을 +2° 돌려 보고, 물체가 어느 쪽으로 움직이는지로 방향을 정한다"""
        import cv2
        from .vision import find_car, read_baselines

        if self.mode != "pose" or self.arm is None:
            raise ValueError("자세 티칭(3번) 화면에서만 확인할 수 있어요.")
        baselines = read_baselines()
        baseline = baselines.get(0)
        if baseline is None:
            raise ValueError("먼저 4번에서 ‘빈 도로 기준 사진’을 저장하세요 (추적 시작 자세에서, 빈 도로).")
        cfg = self.config
        roi = cfg.roi("track_road") if cfg.roi_taught("track_road") else None

        def locate():
            sample = self.snapshot()
            if sample is None:
                raise ValueError("영상이 끊겼어요. 다시 연결된 뒤 해 주세요.")
            car = find_car(sample[1], baseline, roi, cfg)
            if car is None:
                raise ValueError("정지한 차(물체)가 화면에서 보이지 않아요. 추적 시작 자세에서 도로 위에 차를 세워 두고 다시 하세요.")
            return car.centroid[0]

        before = locate()
        self.arm.torque(True)                           # 지금 자리에서 잠금 (튀지 않음)
        self.released = False
        start = self.arm.angles().get(1)
        if start is None:
            raise ValueError("받침(1번 모터) 각도를 읽지 못했어요.")
        try:
            self.arm.move(1, start + 2.0, speed=cfg.speed_slow)
            self.arm.wait_until_settled(timeout=4.0)
            time.sleep(self.settle_seconds)             # 새 영상이 들어올 때까지
            after = locate()
        finally:
            self.arm.move(1, start, speed=cfg.speed_slow)    # 어떤 경우에도 원래 각도로
            self.arm.wait_until_settled(timeout=4.0)
        shift = after - before
        if abs(shift) < 3:
            raise ValueError(f"받침을 돌려도 물체가 거의 안 움직였어요 ({shift:+.1f}픽셀). 물체가 정지해 있는지 확인하세요.")
        sign = 1 if shift < 0 else -1                   # + 로 돌렸더니 화면 왼쪽으로 가면, 오른쪽에 있던 물체가 가운데로 온다
        config = MissionConfig.load()
        config.track_pan_sign = sign
        config.save()
        self.config = config
        return (f"받침을 +2° 돌렸더니 물체가 {shift:+.1f}픽셀 움직였어요 → 추적 방향 {sign:+d} 로 저장했어요. "
                "팔은 원래 각도로 돌려놨고 힘은 켜져 있어요 (자세를 다시 잡으려면 ‘팔 힘 풀기’).")

    def compare(self, data):
        """지금 팔 자세를 저장한 자세와 비교한다 (읽기만, 팔을 움직이지 않는다). 손으로 맞추면서 '맞아요'가 뜰 때까지 본다"""
        if self.mode != "pose" or self.arm is None:
            raise ValueError("자세 티칭 화면에서만 비교할 수 있어요.")
        name = data.get("name")
        if name == "ballgrip":
            cfg = self.config
            if cfg.grip_ball_max_pct <= 0:
                raise ValueError("아직 저장하지 않았어요. 공을 집게에 물린 채 먼저 저장하세요.")
            pct = self.arm.gripper_percent()
            if pct is None:
                raise ValueError("집게가 열린 정도를 읽지 못했어요.")
            inside = cfg.grip_ball_min_pct <= pct <= cfg.grip_ball_max_pct
            return (f"지금 집게 {pct:.1f}% · 저장한 범위 {cfg.grip_ball_min_pct:.0f}% ~ {cfg.grip_ball_max_pct:.0f}% → "
                    + ("범위 안이에요 (공을 쥔 것으로 봐요)" if inside else "범위 밖이에요"))
        if name not in POSE_HELP:
            raise ValueError("비교할 자세를 다시 선택하세요.")
        saved = self.arm.poses().get(name)
        if saved is None:
            raise ValueError("아직 저장하지 않은 자세예요. 먼저 저장하세요.")
        now = self.arm.angles()
        parts, worst = [], 0.0
        for sid in range(1, 6):                       # 팔 관절 5개 (집게 6번 제외)
            target, current = saved["angles"].get(sid), now.get(sid)
            if target is None or current is None:
                raise ValueError(f"{sid}번 모터 값을 읽지 못했어요. USB와 모터 연결을 확인하세요.")
            diff = current - target
            worst = max(worst, abs(diff))
            parts.append(f"{sid}번 {diff:+.1f}°")
        tolerance = self.config.pose_tolerance_deg
        verdict = "✔ 맞아요" if worst <= tolerance else "✘ 조금 더 맞춰 주세요"
        return (f"‘{POSE_LABELS.get(name, name)}’와 비교 · {' · '.join(parts)} · "
                f"가장 큰 차이 {worst:.1f}° (허용 ±{tolerance:g}°) → {verdict}")

    def delete(self, data):
        """저장값 지우기 (허브가 백업을 남기고 지운다). 지운 뒤 3·4·5번에서 다시 저장하면 된다"""
        if self.deleter is None:
            raise ValueError("이 화면에서는 저장값을 지울 수 없어요. 허브(1~12번)에서 열어 주세요.")
        if self.mode not in ("pose", "roi", "color", "car"):
            raise ValueError("이 화면에는 지울 저장값이 없어요.")
        name = data.get("name")
        kind = "ballgrip" if name == "ballgrip" else self.mode
        message = self.deleter({"kind": kind, "name": name})
        self.config = MissionConfig.load()
        if self.mode == "color":
            self.refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
        return message

    def state(self):
        return {"saved": self.saved(), "released": self.released, "locked": self.locked(),
                "frozen": self.frozen[0] if self.frozen else None, "details": self.details()}

    def require_torque_off(self):
        if any(self.arm.driver.get_torque(sid) is not False for sid in JOINT_IDS):
            raise ValueError("모터 6개의 힘이 모두 풀렸는지 확인하지 못했어요. 팔을 받치고 다시 시도하세요.")

    def action(self, data):
        action = data.get("action")
        if action == "finish":
            self.finished = True
            if self.on_finish is not None:
                self.on_finish()
            return "티칭을 끝냈어요. 이 창을 닫아도 돼요."
        if action == "delete":                   # 지우기 · 비교는 카메라 없이도 된다
            return self.delete(data)
        if action == "compare":
            return self.compare(data)
        if action == "baseline":                 # 추적 준비: 빈 도로 사진 · 방향 확인
            return self.save_baseline()
        if action == "probe_pan":
            return self.track_probe()
        if action == "baseline_sweep":
            return self.baseline_sweep()
        if (self.last_frame is None or time.monotonic() - self.last_frame > PREVIEW_MAX_AGE
                or (not self.fake and self.camera.snapshot() is None)):
            raise ValueError("카메라 영상이 들어오지 않아요. USB 연결과 카메라를 쓰는 다른 프로그램을 확인하세요.")
        if self.mode != "pose":
            return self.camera_action(data)
        if action == "release":
            self.released = False
            self.arm.torque(False)
            self.require_torque_off()
            self.released = True
            return "팔의 힘을 풀었어요. 팔을 받친 채 손으로 자세를 잡아주세요."
        if action != "save" or data.get("name") not in (*POSE_HELP, "ballgrip"):
            raise ValueError("저장할 자세를 다시 선택하세요.")
        if not self.released:
            raise ValueError("팔을 받친 상태에서 ‘팔 힘 풀기’를 먼저 눌러주세요.")
        name = data["name"]
        if name in self.saved() and data.get("replace") is not True:
            raise ValueError("이미 저장된 값이에요. 바꿀지 확인한 뒤 다시 눌러주세요.")
        self.require_torque_off()
        if self.fake:
            self.practice_saved.add(name)
            return "연습용으로 저장했어요. 프로그램을 닫으면 사라져요."
        positions = self.arm.positions()
        for sid in JOINT_IDS:
            raw, limit = positions.get(sid), JOINT_LIMITS[sid]
            if raw is None:
                raise ValueError(f"{sid}번 모터의 위치를 읽지 못했어요. USB와 모터 연결을 확인하세요.")
            if not limit["min"] <= unwrap_position(raw, limit) <= limit["max"]:
                raise ValueError(f"{sid}번 모터가 저장 범위를 벗어났어요. 팔을 받친 채 범위 안으로 자세를 옮기세요.")
        if name == "ballgrip":
            pct = self.arm.gripper_percent()
            if pct is None:
                raise ValueError("집게가 열린 정도를 읽지 못했어요.")
            self.config.grip_ball_min_pct = max(0.0, pct - BALL_TOLERANCE_PCT)
            self.config.grip_ball_max_pct = min(100.0, pct + BALL_TOLERANCE_PCT)
            self.config.save()
            return f"공을 쥔 집게의 열림 정도를 저장했어요. ({pct:.1f}%)"
        self.arm.save_pose(name, positions=positions)
        return f"‘{POSE_LABELS[name]}’ 자세를 저장했어요."


def make_handler(teaching):
    token = secrets.token_urlsafe()
    payload = json.dumps({"token": token, "items": teaching.items(), "fake": teaching.fake,
                          "mode": teaching.mode, "title": TITLES[teaching.mode],
                          "limits": teaching.limits()}, ensure_ascii=False)
    page = Path(__file__).with_suffix(".html").read_text(encoding="utf-8")
    page = page.replace("/* TEACHING_DATA */ null", payload).encode()

    class Handler(BaseHTTPRequestHandler):
        timeout = 3

        def log_message(self, *args):
            pass

        def send(self, status, content_type, body=b"", result=""):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            if result:
                self.send_header("X-Camera-Result", result)
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, status, data):
            self.send(status, "application/json; charset=utf-8", json.dumps(data, ensure_ascii=False).encode())

        def local_host(self):
            return self.headers.get("Host", "").partition(":")[0] in ("127.0.0.1", "localhost")

        def do_GET(self):
            if not self.local_host():
                self.send(403, "text/plain")
            elif self.path == "/":
                self.send(200, "text/html; charset=utf-8", page)
            elif self.path in ("/frame", "/state") and self.headers.get("X-Teaching-Token") != token:
                self.send(403, "text/plain")
            elif self.path == "/frame":
                jpg, result = teaching.frame()
                self.send(200, "image/jpeg", jpg, result) if jpg else self.send(503, "text/plain")
            elif self.path.startswith("/frozen/"):
                frozen = teaching.frozen
                jpg = teaching.jpeg(frozen[1]) if frozen and self.path == f"/frozen/{frozen[0]}" else None
                self.send(200, "image/jpeg", jpg) if jpg else self.send(404, "text/plain")
            elif self.path == "/state":
                self.send_json(200, teaching.state())
            else:
                self.send(404, "text/plain")

        def do_POST(self):
            if (self.path != "/action" or not self.local_host()
                    or self.headers.get("X-Teaching-Token") != token):
                self.send(403, "text/plain")
                return
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 1 <= size <= 1024:
                    raise ValueError("요청을 다시 보내주세요.")
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError("요청을 다시 보내주세요.")
                message = teaching.action(data)
                response = {"message": message, **teaching.state()}
            except (ValueError, OSError, RuntimeError) as error:
                self.send_json(400, {"message": str(error), "released": teaching.released, "locked": teaching.locked()})
                return
            self.send_json(200, response)

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description="카메라로 보는 한국어 티칭")
    parser.add_argument("--port", help="실물 팔 연결 포트 (Pi 예: /dev/ttyACM0)")
    parser.add_argument("--web-port", type=int, default=8081)
    parser.add_argument("--mode", choices=TITLES, default="pose")
    args = parser.parse_args(argv)
    args.fake = False                       # 가상 · 연습용 모드는 없앴다 (항상 실물 카메라 · 실물 팔)
    cfg = MissionConfig() if args.fake else MissionConfig.load()
    try:
        with ExitStack() as resources:
            camera = None
            if not args.fake:
                from soarm.color import open_camera

                camera = LatestCamera(open_camera(cfg.camera, cfg.width, cfg.height), sample_colors=args.mode == "color")
                resources.callback(camera.close)
                if not camera.ready.wait(timeout=3) or camera.snapshot() is None:
                    raise ValueError("손목 카메라 영상이 들어오지 않아요. USB와 카메라 번호를 확인하세요.")
            arm = None
            if args.mode == "pose":
                arm = resources.enter_context(Robot("fake" if args.fake else args.port or cfg.port))
                if not args.fake and not arm.real:
                    raise ValueError("가상 팔 포트가 선택됐어요. 실물 팔 포트를 지정하세요.")
            teaching = Teaching(arm, camera, cfg, args.fake, args.mode)
            server = resources.enter_context(HTTPServer(("127.0.0.1", args.web_port), make_handler(teaching)))
            server.timeout = 0.5
            port = server.server_port
            print(f"{TITLES[args.mode]} 화면: http://127.0.0.1:{port}/", flush=True)
            print(f"Pi에 접속 중이면 VS Code의 Ports에서 {port} 포트를 전달한 뒤 브라우저로 여세요.", flush=True)
            print("이미 열려 있는 배달 한방 탭은 새로고침하세요.", flush=True)
            print("끝내려면 화면의 ‘티칭 끝내기’를 누르거나 이 터미널에서 Ctrl+C를 누르세요.", flush=True)
            while not teaching.finished:
                server.handle_request()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as error:
        print(f"티칭 화면을 열지 못했어요. {error}")
        return 1
    print("티칭 화면을 닫았어요.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
