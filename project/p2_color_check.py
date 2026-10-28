"""프로젝트 2 — 검사 자리의 색 판별 (빨강 · 파랑 · 없음).

    python project/p2_color_check.py              창에 결과를 그린다 (q 로 끝)
    python project/p2_color_check.py --no-window  글자만 (라즈베리파이 SSH)

할 일
  1. 빨강 · 파랑 종이를 번갈아 놓고, 아무것도 없을 때도 본다. 결과가 바뀔 때만 한 줄씩 찍힌다
  2. 잘못 나오면 project/color.json 을 고친다
     - 종이를 놓았는데 none → min_ratio 를 낮춘다 (0.30 → 0.2)
     - 빈 자리인데 red → 바닥 색이 걸린 것. 빨강의 S(채도) 하한을 올린다 (100 → 140)
  3. STABLE 장 연속으로 같은 색이어야 '확정' 으로 본다 (손이 지나갈 때 잘못 판별하지 않게)
"""
import sys
import time

from soarm.color import ColorConfig, classify, draw, open_camera

STABLE = 5
window = "--no-window" not in sys.argv

cfg = ColorConfig.load()
cap = open_camera(cfg.camera, cfg.width, cfg.height)
if window:
    import cv2

last, count, shown = None, 0, None
t_end = time.time() + float(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else None
try:
    while t_end is None or time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            print("영상 없음")
            break
        color, ratios = classify(frame, cfg)
        count = count + 1 if color == last else 1
        last = color
        if count == STABLE and color != shown:
            shown = color
            print(f"{time.strftime('%H:%M:%S')}  확정: {color:5s}  {ratios}")
        if window:
            cv2.imshow("color (q: 끝)", draw(frame, cfg, color, ratios))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
finally:
    cap.release()
    if window:
        cv2.destroyAllWindows()
