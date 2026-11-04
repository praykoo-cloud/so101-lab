"""카메라 4 — 책상 위 어디에 있든 색 물체 찾기 (화면 전체에서 가장 큰 색 덩어리).

    python camera/c4_find_color.py                 빨강. 창에 상자 · 중심 픽셀을 그린다 (q 끝, m 마스크 보기)
    python camera/c4_find_color.py --color blue
    python camera/c4_find_color.py --no-window --seconds 5     글자만 (라즈베리파이)

'검사 자리' 프로젝트(p1 · p2)는 정해 둔 작은 상자(ROI) 안의 색 비율만 본다.
여기서는 화면 전체에서 덩어리를 찾아 그 중심 (u, v) 픽셀을 낸다 → c5 · c6 에서 로봇 좌표로 바꾼다.

할 일
  1. 빨강 물체를 책상 여기저기로 옮기며 (u, v) 가 따라 바뀌는지 본다. u 는 오른쪽, v 는 아래로 커진다
  2. m 으로 마스크(흰 = 빨강으로 본 픽셀)를 본다. 팔 · 옷 · 바닥이 희게 나오면 그게 잘못 찾는 원인
  3. 팔에 빨간 부품이 있으면 --zone x y w h 로 책상 부분만 보게 한다
"""
import sys
import time

import cv2

from camlib import color_mask, draw_blob, find_blob, load_settings, open_cam

color = sys.argv[sys.argv.index("--color") + 1] if "--color" in sys.argv else "red"
zone = [int(v) for v in sys.argv[sys.argv.index("--zone") + 1:][:4]] if "--zone" in sys.argv else None
window = "--no-window" not in sys.argv
t_end = time.time() + float(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else None

cfg = load_settings()
cap = open_cam(cfg)
show_mask, last_print = False, 0.0
try:
    while t_end is None or time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            sys.exit("영상이 오지 않습니다")
        blob = find_blob(frame, color, zone=zone)
        if time.time() - last_print >= 0.5:                 # 0.5초마다 한 줄
            last_print = time.time()
            print(f"{time.strftime('%H:%M:%S')}  {color}: " +
                  (f"u={blob['u']:6.1f}  v={blob['v']:6.1f}  넓이 {blob['area']:5d} px" if blob else "없음"))
        if window:
            show = cv2.cvtColor(color_mask(frame, color), cv2.COLOR_GRAY2BGR) if show_mask else frame.copy()
            if zone:
                x, y, w, h = zone
                cv2.rectangle(show, (x, y), (x + w, y + h), (0, 255, 255), 1)
            cv2.imshow("find color (q: quit, m: mask)", draw_blob(show, blob))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("m"):
                show_mask = not show_mask
finally:
    cap.release()
    cv2.destroyAllWindows()
