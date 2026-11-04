"""카메라 2 — 화면 보기 · 사진 찍기 · 격자 · fps.

    python camera/c2_view.py                  창으로 본다. s 사진, g 격자, q 끝
    python camera/c2_view.py --no-window      창 없이 (라즈베리파이 SSH): 3초 동안 fps 를 재고 사진 한 장

할 일
  1. 격자(g)를 켜고 팔 받침이 화면 아래쪽 가운데쯤 오도록 카메라를 맞춘다
  2. 책상 위 물체를 놓을 자리가 화면 안에 다 들어오는지 본다 (가장자리는 왜곡이 크다)
  3. 형광등 · 창문 빛이 책상에 번쩍이면(흰 얼룩) 카메라 각도를 바꾼다. 색 찾기가 틀리는 가장 큰 원인
"""
import sys
import time

import cv2

from camlib import load_settings, open_cam, save_jpg, stamp


def grid(img, step=80):
    h, w = img.shape[:2]
    for x in range(step, w, step):
        cv2.line(img, (x, 0), (x, h), (200, 200, 200), 1)
    for y in range(step, h, step):
        cv2.line(img, (0, y), (w, y), (200, 200, 200), 1)
    cv2.line(img, (w // 2, 0), (w // 2, h), (0, 200, 255), 1)     # 가운데 세로선
    return img


cfg = load_settings()
cap = open_cam(cfg)
window = "--no-window" not in sys.argv
t_end = None if window else time.time() + 3
show_grid, n, t0, fps = False, 0, time.time(), 0.0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            sys.exit("영상이 오지 않습니다")
        n += 1
        if time.time() - t0 >= 1.0:
            fps, n, t0 = n / (time.time() - t0), 0, time.time()
            if not window:
                print(f"{frame.shape[1]}x{frame.shape[0]}  {fps:.1f} fps")
        if not window:
            if time.time() > t_end:
                print("사진:", save_jpg(frame, f"view_{stamp()}"))
                break
            continue
        show = grid(frame.copy()) if show_grid else frame.copy()
        cv2.putText(show, f"cam {cfg['camera']}  {fps:.1f} fps   s: photo  g: grid  q: quit", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imshow("camera", show)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("g"):
            show_grid = not show_grid
        if key == ord("s"):
            print("사진:", save_jpg(frame, f"view_{stamp()}"))
finally:
    cap.release()
    cv2.destroyAllWindows()
