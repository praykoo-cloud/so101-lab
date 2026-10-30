"""color_node — 검사 자리(ROI)의 색 판별. 판별 함수는 프로젝트 2 와 같은 soarm.color.classify.

    ros2 run so101_ros color_node --ros-args -p config:=<so101-lab>/project/color.json

구독 /camera/image_raw
발행 /color/detected     std_msgs/String   red · blue · none (영상마다)
     /color/debug_image  sensor_msgs/Image ROI 상자와 결과를 그린 영상 (rqt_image_view 로 본다)
     /color/debug_image/compressed  같은 영상의 JPEG. Wi-Fi 너머 노트북에서 볼 때는 이쪽
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from soarm.color import CONFIG_FILE, ColorConfig, classify, draw


class ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")
        path = self.declare_parameter("config", str(CONFIG_FILE)).value
        self.cfg = ColorConfig.load(path)
        self.pub = self.create_publisher(String, "/color/detected", 10)
        self.debug = self.create_publisher(Image, "/color/debug_image", 2)
        self.debug_jpg = self.create_publisher(CompressedImage, "/color/debug_image/compressed", 2)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 5)
        self.last = None
        self.get_logger().info(f"설정 {path}: ROI {self.cfg.roi}, min_ratio {self.cfg.min_ratio}")

    def on_image(self, msg: Image):
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":          # camera_node · HiWonder 차 카메라는 bgr8. RGB 로 보내는 카메라면 순서를 바꾼다
            frame = np.ascontiguousarray(frame[:, :, ::-1])
        color, ratios = classify(frame, self.cfg)
        self.pub.publish(String(data=color))
        if color != self.last:
            self.get_logger().info(f"{color}  {ratios}")
            self.last = color
        raw, jpg = self.debug.get_subscription_count(), self.debug_jpg.get_subscription_count()
        if raw or jpg:
            out = draw(frame, self.cfg, color, ratios)
        if raw:
            self.debug.publish(Image(header=msg.header, height=msg.height, width=msg.width, encoding="bgr8",
                                     step=msg.width * 3, data=out.tobytes()))
        if jpg:
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.debug_jpg.publish(CompressedImage(header=msg.header, format="jpeg", data=buf.tobytes()))


def main():
    rclpy.init()
    node = ColorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
