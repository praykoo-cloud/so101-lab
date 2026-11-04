"""카메라 8 — 화면 없는 라즈베리파이의 카메라를 노트북 브라우저로 보기 (ROS2 · Docker 없이).

    python camera/c8_web_stream.py                 → 노트북 브라우저에서 http://so101-01.local:8080
    python camera/c8_web_stream.py --color red     빨강 덩어리 상자를 그려서
    python camera/c8_web_stream.py --port 8081

Pi 직접 세팅(venv)한 so101-lab 에서 돈다. Docker · ROS2 를 쓰는 팀은 scripts/web_view.py (토픽 → 웹).
같은 Wi-Fi 에서만 보인다. 끝은 Ctrl+C.

할 일
  1. Pi 에 로지텍 카메라를 꽂고 실행 → 노트북 브라우저로 본다
  2. 브라우저 위 숫자(fps)를 보며 --width 320 --height 240 으로 바꾸면 얼마나 빨라지는지 본다
"""
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from camlib import draw_blob, find_blob, load_settings, open_cam


def arg(name, default):
    return type(default)(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


PORT = arg("--port", 8080)
COLOR = arg("--color", "")
cfg = load_settings()
cfg["width"], cfg["height"] = arg("--width", cfg["width"]), arg("--height", cfg["height"])
cap = open_cam(cfg)
latest, count, cond = None, 0, threading.Condition()

PAGE = """<!doctype html><meta charset="utf-8"><title>so101 camera</title>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif">
<p style="margin:8px">camera {cam} · {w}x{h}</p><img src="/stream" style="max-width:100%"></body>"""


def grab():
    global latest, count
    n, t0, fps = 0, time.time(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        if COLOR:
            draw_blob(frame, find_blob(frame, COLOR))
        n += 1
        if time.time() - t0 >= 1:
            fps, n, t0 = n / (time.time() - t0), 0, time.time()
        cv2.putText(frame, f"{fps:.1f} fps", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with cond:
            latest, count = buf.tobytes(), count + 1
            cond.notify_all()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path != "/stream":
            body = PAGE.format(cam=cfg["camera"], w=cfg["width"], h=cfg["height"]).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        seen = 0
        try:
            while True:
                with cond:
                    cond.wait_for(lambda: count != seen, timeout=5)
                    jpg, seen = latest, count
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        except ConnectionError:                  # 브라우저 탭을 닫음
            pass


threading.Thread(target=grab, daemon=True).start()
print(f"카메라 {cfg['camera']} {cfg['width']}x{cfg['height']} → 브라우저에서 http://{socket.gethostname()}.local:{PORT}  (Ctrl+C 끝)")
try:
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
except KeyboardInterrupt:
    print("\n끝")
finally:
    cap.release()
