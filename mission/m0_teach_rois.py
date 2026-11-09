"""구역(ROI) 티칭: 카메라 화면에서 "여기만 보겠다" 는 네모를 마우스로 그려서 저장한다 (mission/config.json).

실행:
    python -m mission.m0_teach_rois                       # 모든 구역을 차례로
    python -m mission.m0_teach_rois --only note           # 하나만
    python -m mission.m0_teach_rois --list                # 티칭 상태만 보기
    python -m mission.m0_teach_rois --image 사진.png       # 카메라 대신 사진으로 (연습)
    python -m mission.m0_teach_rois --set note=250,180,160,120    # 숫자로 직접 (x,y,w,h)

구역마다 팔이 있어야 하는 자세가 다르다 (화면이 달라지기 때문). 구역 이름 옆 설명에 자세가 적혀 있다.
  1) m0_teach_poses 로 저장한 자세로 팔을 보내 둔다 (예: go wait)
  2) 이 도구를 실행해 해당 구역만 그린다:  --only arrival
창이 뜨면: 마우스로 네모 드래그 → Enter(확정) / c(건너뜀)

주의: 카메라를 움직이면 구역이 어긋난다. 그리퍼 카메라는 고정하고, 자세를 바꾼 뒤에는 구역을 다시 확인한다.
"""
from __future__ import annotations

import argparse

from .config import ROI_HELP, MissionConfig


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="구역 티칭")
    ap.add_argument("--only", help="이 구역만")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--image", help="카메라 대신 쓸 사진 파일")
    ap.add_argument("--studio", action="store_true", help="ArmStudio 가상(손목) 카메라 사용")
    ap.add_argument("--set", action="append", default=[], help="이름=x,y,w,h")
    args = ap.parse_args(argv)
    cfg = MissionConfig.load()

    if args.set:
        for item in args.set:
            name, _, vals = item.partition("=")
            if name not in ROI_HELP:
                print(f"모르는 구역: {name}. 있는 것: {list(ROI_HELP)}")
                return 1
            x, y, w, h = [int(v) for v in vals.split(",")]
            cfg.rois[name] = [x, y, w, h]
        cfg.save()
        print("저장했습니다:", {k: cfg.rois[k] for k in (i.partition('=')[0] for i in args.set)})
        return 0

    if args.list:
        for name, help_ in ROI_HELP.items():
            print(f"  [{'티칭됨' if cfg.roi_taught(name) else '  --  '}] {name:14s} {cfg.roi(name)}  {help_}")
        return 0

    import cv2
    names = [args.only] if args.only else list(ROI_HELP)
    for n in names:
        if n not in ROI_HELP:
            print(f"모르는 구역: {n}")
            return 1

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"사진을 열 수 없습니다: {args.image}")
            return 1
        grab, release = (lambda: frame), (lambda: None)
    else:
        from .studio import open_capture
        cap = open_capture(cfg, args.studio)

        def grab():
            ok, f = cap.read()
            return f if ok else None
        release = cap.release

    try:
        for name in names:
            print(f"\n[{name}] {ROI_HELP[name]}\n  현재: {cfg.roi(name)}   (창에서 드래그 → Enter, 건너뛰기 c)")
            f = grab()
            if f is None:
                print("카메라 화면을 읽지 못했습니다")
                return 1
            x, y, w, h = cv2.selectROI(f"{name}: drag then Enter (c = skip)", f, showCrosshair=False)
            cv2.destroyAllWindows()
            if w > 0 and h > 0:
                cfg.rois[name] = [int(x), int(y), int(w), int(h)]
                cfg.save()
                print(f"  저장: {cfg.rois[name]}")
            else:
                print("  건너뜀")
    finally:
        release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
