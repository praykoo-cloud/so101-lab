"""camera_node — 카메라 영상을 토픽으로.

    ros2 run so101_ros camera_node --ros-args -p device:=0 -p fps:=10.0

발행 /camera/image_raw  sensor_msgs/Image (bgr8)
cv_bridge 없이 배열을 메시지에 바로 담는다 (설치할 것이 적다).
device 에 영상 파일 경로를 주면 끝까지 재생한 뒤 처음부터 다시 (카메라 없이 시험할 때).
"""
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from soarm.color import open_camera


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        dev = self.declare_parameter("device", "0").value
        self.width = int(self.declare_parameter("width", 640).value)
        self.height = int(self.declare_parameter("height", 480).value)
        fps = float(self.declare_parameter("fps", 10.0).value)
        dev = int(dev) if str(dev).isdigit() else dev          # "0" → 0, "/dev/video0" 는 그대로
        self.is_file = isinstance(dev, str) and not dev.startswith("/dev/")   # 시험용 영상 파일
        self.cap = open_camera(dev, self.width, self.height)
        self.pub = self.create_publisher(Image, "/camera/image_raw", 5)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"카메라 {dev}, {fps} fps")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok and self.is_file:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)      # 영상 파일이면 처음부터 다시
            ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("영상 받기 실패")
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise          # Ctrl+C 로 끝나는 중에 타이머가 한 번 더 발행하다 나는 오류는 숨긴다
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()
