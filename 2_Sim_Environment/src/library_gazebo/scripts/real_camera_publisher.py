#!/usr/bin/env python3
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class RealCameraPublisher(Node):
    def __init__(self):
        super().__init__("real_camera_publisher")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("frame_id", "low_angle_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("show_preview", False)

        self.device = str(self.get_parameter("device").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/low_angle_camera/image_raw", 10)
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera device: {self.device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        period = 1.0 / max(1.0, self.fps)
        self.create_timer(period, self.publish_frame)
        self.get_logger().info(f"Publishing real camera {self.device} to /low_angle_camera/image_raw")

    def publish_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera frame read failed")
            time.sleep(0.1)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

        if self.show_preview:
            cv2.imshow("Real Low Angle Camera", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
