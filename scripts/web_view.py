"""ROS2 영상 토픽을 웹 페이지로 (Pi 에 화면이 없을 때 노트북 브라우저로 보기). 컨테이너 안에서 실행.

    python3 scripts/web_view.py                                   # /color/debug_image/compressed
    python3 scripts/web_view.py /camera/image_raw                 # 원본 (bgr8 · rgb8 · mono8)
    python3 scripts/web_view.py /ascamera/camera_publisher/rgb0/image --port 8081

노트북 브라우저: http://<Pi 주소>:8080  (같은 Wi-Fi. compose 가 network_mode: host 라 Pi 의 포트 그대로)
WSL · rqt 없이 보기만 한다. 토픽 이름이 /compressed 로 끝나면 JPEG 를 그대로, 아니면 JPEG 로 바꿔 보낸다.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

PAGE = """<!doctype html><meta charset="utf-8"><title>{topic}</title>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif">
<p style="margin:8px">{topic} <span id="t"></span></p><img src="/stream" style="max-width:100%">
</body>"""


class Viewer(Node):
    def __init__(self, topic):
        super().__init__("web_view")
        self.jpg, self.count, self.cond = None, 0, threading.Condition()
        if topic.endswith("/compressed"):
            self.create_subscription(CompressedImage, topic, self.on_jpg, qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self.on_raw, qos_profile_sensor_data)

    def put(self, data):
        with self.cond:
            self.jpg, self.count = data, self.count + 1
            self.cond.notify_all()

    def on_jpg(self, msg):
        self.put(bytes(msg.data))

    def on_raw(self, msg):
        ch = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(msg.encoding)
        if ch is None:
            self.get_logger().warn(f"모르는 형식: {msg.encoding}", once=True)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.put(buf.tobytes())


def serve(node, topic, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path != "/stream":
                body = PAGE.format(topic=topic).encode()
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
                    with node.cond:
                        node.cond.wait_for(lambda: node.count != seen, timeout=5)
                        jpg, seen = node.jpg, node.count
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                                      # 브라우저 탭을 닫음

    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic", nargs="?", default="/color/debug_image/compressed")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    rclpy.init()
    node = Viewer(args.topic)
    threading.Thread(target=serve, args=(node, args.topic, args.port), daemon=True).start()
    node.get_logger().info(f"{args.topic} → http://<Pi 주소>:{args.port}")
    last, t0 = 0, time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.time() - t0 > 5:                      # 5초마다 받은 장 수
                node.get_logger().info(f"{(node.count - last) / (time.time() - t0):.1f} fps")
                last, t0 = node.count, time.time()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
