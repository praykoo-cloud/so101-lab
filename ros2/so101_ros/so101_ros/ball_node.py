"""ball_node — 카메라 영상에서 색 공 찾기 (비전). 로봇은 모른다. 찾은 자리만 토픽으로 낸다.

    ros2 run so101_ros ball_node --ros-args -p color:=red
    ros2 param set /ball_node hsv_low "[18, 60, 80]"      범위를 직접 (돌아가는 중에 바꿔도 다음 장부터 적용)
    ros2 param set /ball_node hsv_high "[35, 255, 255]"

구독 /camera/image_raw            sensor_msgs/Image        camera_node 가 내는 영상 (bgr8)
발행 /ball/pixel                  geometry_msgs/PointStamped  공 중심 x = u, y = v (픽셀), z = 넓이 (픽셀 수). 못 찾으면 안 낸다
     /ball/debug_image/compressed sensor_msgs/CompressedImage  원 · 십자를 그린 JPEG (브라우저: scripts/web_view.py)

색: red · blue · yellow · green (HSV 범위는 아래 RANGES). 빨강은 색상환 양 끝이라 두 구간
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from soarm.color import DEFAULT_RANGES

# HSV 범위 (H 0 ~ 180, S · V 0 ~ 255). 빨강 · 파랑은 soarm 과 같고, 노랑 · 초록을 더했다
RANGES = dict(DEFAULT_RANGES,
              yellow=[[[20, 100, 100], [35, 255, 255]]],
              green=[[[40, 70, 50], [85, 255, 255]]])


class BallNode(Node):
    def __init__(self):
        super().__init__("ball_node")
        self.color = self.declare_parameter("color", "red").value
        self.declare_parameter("min_area", 300)
        self.declare_parameter("hsv_low", [0, 0, 0])          # 둘 다 [0, 0, 0] 이면 color 의 범위를 쓴다
        self.declare_parameter("hsv_high", [0, 0, 0])
        if self.color not in RANGES:
            raise SystemExit(f"color 는 {list(RANGES)} 중 하나")
        self.pub = self.create_publisher(PointStamped, "/ball/pixel", 10)
        self.debug = self.create_publisher(CompressedImage, "/ball/debug_image/compressed", 2)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 5)
        self.seen = None
        self.get_logger().info(f"{self.color} 공 찾기 (범위를 바꾸려면 hsv_low · hsv_high 파라미터)")

    def on_image(self, msg: Image):
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = np.ascontiguousarray(frame[:, :, ::-1])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo_p = list(self.get_parameter("hsv_low").value)
        hi_p = list(self.get_parameter("hsv_high").value)
        ranges = [[lo_p, hi_p]] if any(hi_p) else RANGES[self.color]
        min_area = int(self.get_parameter("min_area").value)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 작은 점(잡음) 지우기
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
        found = False
        if n > 1:
            i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))                     # 0 번은 배경
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= min_area:
                u, v = float(cent[i][0]), float(cent[i][1])
                out = PointStamped()
                out.header = msg.header                                              # 영상과 같은 시각
                out.point.x, out.point.y, out.point.z = u, v, float(area)
                self.pub.publish(out)
                cv2.circle(frame, (int(u), int(v)), int((area / 3.14) ** 0.5), (0, 255, 0), 2)
                cv2.drawMarker(frame, (int(u), int(v)), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
                found = True
        if found != self.seen:
            self.get_logger().info("공 찾음" if found else "공 없음")
            self.seen = found
        cv2.drawMarker(frame, (msg.width // 2, msg.height // 2), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)   # 화면 가운데
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            c = CompressedImage()
            c.header, c.format, c.data = msg.header, "jpeg", jpg.tobytes()
            self.debug.publish(c)


def main():
    rclpy.init()
    node = BallNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
