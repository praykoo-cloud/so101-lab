"""기준색 티칭: 빨강 · 파랑 포스트잇과 빨강 · 파랑 공을 카메라로 보여 줘서 색을 외우게 한다.

왜: 판정은 "티칭한 두 기준색 중 가까운 쪽" 을 항상 고른다. 현장 조명에서 직접 등록해야 정확하다.

실행:
    python -m mission.m0_teach_colors                      # 안내 메뉴
    python -m mission.m0_teach_colors status               # 무엇이 등록됐나
    python -m mission.m0_teach_colors teach note_red       # 빨간 포스트잇 등록 (카메라)
    python -m mission.m0_teach_colors teach note_blue
    python -m mission.m0_teach_colors teach ball_red       # 빨간 공 (받침대에 놓인 상태로)
    python -m mission.m0_teach_colors teach ball_blue
    python -m mission.m0_teach_colors verify               # 지금 화면의 포스트잇을 판정해 본다 (실시간)
    python -m mission.m0_teach_colors lock                 # 4개 모두 등록한 뒤 고정 (주문 배정 전에!)
    python -m mission.m0_teach_colors unlock               # 고정 풀기 (주문 배정 전에만)
    python -m mission.m0_teach_colors teach note_red --image 사진.png --rect 100,80,60,60   # 사진으로 (연습·시험)

teach 가 하는 일: 화면이 뜨면 스페이스 → 그 순간의 사진에서 마우스로 색 영역(네모)을 드래그 → Enter.
같은 자리를 15장 찍어서 색의 평균과 흔들림을 저장한다 (조명이 다를 때마다 한 번 더 teach 하면 평균이 넓어진다).

규칙 (경기 규정과 팀 약속)
- 빨강과 파랑을 **둘 다** 등록하고 검증한 뒤 lock 한다. 주문(포스트잇)이 정해진 뒤에는 고치지 않는다.
- 이 도구에는 "현재 주문" 을 지정하는 기능이 없다. 있어서도 안 된다.
"""
from __future__ import annotations

import argparse

import numpy as np

from .config import MissionConfig
from .vision import (COLORS_FILE, LOCK_FILE, REF_NAMES, ColorRefs, circular_mean_hue, classify_forced,
                     find_note, is_locked, lock_colors, vivid_mask)


def sample_hsv_in_rect(frame, rect) -> tuple[float, float, float] | None:
    """사각형 안의 선명한 픽셀 중앙값 (h, s, v). 선명한 픽셀이 너무 적으면 None"""
    import cv2
    x, y, w, h = [int(v) for v in rect]
    sub = frame[y:y + h, x:x + w]
    if sub.size == 0:
        return None
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    m = vivid_mask(hsv) > 0
    if int(m.sum()) < 30:
        return None
    px = hsv[m]
    return (circular_mean_hue(px[:, 0]), float(np.median(px[:, 1])), float(np.median(px[:, 2])))


def teach_from_frames(refs: ColorRefs, name: str, frames, rect) -> dict:
    samples = [s for s in (sample_hsv_in_rect(f, rect) for f in frames) if s is not None]
    return refs.add_samples(name, samples)


def _status(refs: ColorRefs):
    for n in REF_NAMES:
        r = refs.refs.get(n)
        print(f"  [{'등록됨' if r else '  --  '}] {n:10s}", (f"h={r['h']} s={r['s']} v={r['v']} (샘플 {r['n']}장)" if r else ""))
    print("  잠금:", "잠김 (고정됨)" if is_locked(COLORS_FILE, LOCK_FILE) else "안 잠김")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="기준색 티칭")
    ap.add_argument("action", nargs="?", default="status", choices=["status", "teach", "verify", "lock", "unlock"])
    ap.add_argument("name", nargs="?", choices=REF_NAMES)
    ap.add_argument("--image", help="카메라 대신 사진 (여러 번 쓰면 여러 장)", action="append")
    ap.add_argument("--rect", help="사진에서 색 영역 x,y,w,h (창을 안 띄움)")
    ap.add_argument("--studio", action="store_true", help="ArmStudio 가상(손목) 카메라 사용")
    args = ap.parse_args(argv)
    cfg = MissionConfig.load()

    refs = ColorRefs.read(COLORS_FILE) if COLORS_FILE.exists() else ColorRefs()
    if args.action == "status":
        _status(refs)
        return 0
    if args.action == "lock":
        if not refs.complete():
            print("빨강·파랑 포스트잇과 공 4개를 모두 등록해야 잠글 수 있습니다:")
            _status(refs)
            return 1
        lock_colors(COLORS_FILE, LOCK_FILE)
        print("잠갔습니다. 이제 기준색을 고칠 수 없습니다 (주문 배정 후 수정 금지).")
        return 0
    if args.action == "unlock":
        if input("주문이 아직 배정되기 전입니까? (yes 입력) ").strip().lower() != "yes":
            print("취소")
            return 1
        LOCK_FILE.unlink(missing_ok=True)
        print("잠금을 풀었습니다. 수정 후 다시 lock 하세요.")
        return 0

    import cv2
    if args.action == "teach":
        if not args.name:
            print("무엇을 등록할지 적어 주세요:", REF_NAMES)
            return 1
        if is_locked(COLORS_FILE, LOCK_FILE):
            print("기준색이 잠겨 있습니다. 고치려면 먼저 unlock (주문 배정 전에만)")
            return 1
        if args.image:
            frames = [cv2.imread(p) for p in args.image]
            if any(f is None for f in frames):
                print("사진을 열 수 없습니다")
                return 1
            if len(frames) == 1:
                frames = frames * 15
            if args.rect:
                rect = [int(v) for v in args.rect.split(",")]
            else:
                rect = cv2.selectROI("drag the colored object then Enter", frames[0])
                cv2.destroyAllWindows()
        else:
            from .studio import open_capture
            cap = open_capture(cfg, args.studio)
            try:
                print("화면이 뜹니다. 스페이스 → 그 사진에서 색 영역을 드래그 → Enter")
                still = None
                while still is None:
                    ok, f = cap.read()
                    if not ok:
                        continue
                    cv2.imshow("live (SPACE to freeze, q to quit)", f)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord(" "):
                        still = f
                    elif k == ord("q"):
                        cv2.destroyAllWindows()
                        return 1
                cv2.destroyAllWindows()
                rect = cv2.selectROI("drag the colored object then Enter", still)
                cv2.destroyAllWindows()
                frames = []
                while len(frames) < 15:
                    ok, f = cap.read()
                    if ok:
                        frames.append(f)
            finally:
                cap.release()
        try:
            ref = teach_from_frames(refs, args.name, frames, rect)
        except ValueError as e:
            print(f"등록하지 못했습니다: {e} (색 영역을 더 크게/선명한 곳으로)")
            return 1
        refs.save(COLORS_FILE)
        print(f"등록: {args.name} → {ref}")
        _status(refs)
        return 0

    if args.action == "verify":
        if not (refs.refs.get("note_red") and refs.refs.get("note_blue")):
            print("먼저 note_red, note_blue 를 등록하세요")
            return 1
        from .studio import open_capture
        cap = open_capture(cfg, args.studio)
        print("창에 판정이 표시됩니다 (q 로 끝). 빨강/파랑 포스트잇을 번갈아 보여 주세요.")
        try:
            while True:
                ok, f = cap.read()
                if not ok:
                    continue
                note = find_note(f, cfg.roi("note") if cfg.roi_taught("note") else [0, 0, f.shape[1], f.shape[0]], cfg, refs)
                out = f.copy()
                if note:
                    color, dr, db = classify_forced(note, refs)
                    x, y, w, h = note.bbox
                    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255) if color == "red" else (255, 0, 0), 2)
                    cv2.putText(out, f"{color}  d_red={dr:.2f} d_blue={db:.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    cv2.putText(out, "포스트잇 없음 / no note", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                cv2.imshow("verify (q to quit)", out)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
