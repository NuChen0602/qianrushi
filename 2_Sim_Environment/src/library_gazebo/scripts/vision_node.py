#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionPatrolNode(Node):
    def __init__(self):
        super().__init__('vision_patrol_node')
        # 修改为正确的真实话题
        self.subscription = self.create_subscription(
            Image, '/low_angle_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('👁️ 视觉巡检节点已启动！正在监控 /camera/image_raw ...')

    def image_callback(self, msg):
        try:
            # 1. 将 ROS 图像转化为 OpenCV 矩阵
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 2. Task 2: 检测红色违规插座
            lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
            lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
            mask_red = cv2.bitwise_or(cv2.inRange(hsv_frame, lower_red1, upper_red1),
                                      cv2.inRange(hsv_frame, lower_red2, upper_red2))

            # 3. Task 3: 检测蓝色校园卡
            lower_blue, upper_blue = np.array([100, 150, 0]), np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)

            # 4. 逻辑判断与终端报警
            if cv2.countNonZero(mask_red) > 500:
                self.get_logger().warn('🚨 [安全警报] 发现违规发热插座！位置已记录。')
            if cv2.countNonZero(mask_blue) > 200:
                self.get_logger().info('📘 [失物招领] 发现疑似校园卡！正在生成工单...')

            # 5. 实时渲染 AI 视角
            cv2.imshow("AI Patrol View", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"处理图像时发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionPatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()