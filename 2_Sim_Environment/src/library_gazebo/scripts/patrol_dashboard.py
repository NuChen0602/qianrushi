#!/usr/bin/env python3
import json
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class PatrolDashboard(Node):
    def __init__(self):
        super().__init__("patrol_dashboard")
        self.events = []
        self.goal = None
        self.pose = None
        self.create_subscription(String, "/patrol/events", self.event_callback, 10)
        self.create_subscription(String, "/patrol/mission_status", self.status_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_timer(1.0, self.render)
        self.get_logger().info("中控终端看板已启动，监听 /patrol/events /goal_pose /odom。")

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            event = {"type": "raw", "message": msg.data}
        event["received_at"] = datetime.now().strftime("%H:%M:%S")
        self.events.insert(0, event)
        self.events = self.events[:8]

    def status_callback(self, msg):
        self.event_callback(msg)

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def odom_callback(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def render(self):
        os.system("clear")
        print("Library Patrol 中控看板")
        print("=" * 72)
        if self.pose:
            print(f"机器人当前位置: x={self.pose[0]:.2f}, y={self.pose[1]:.2f}")
        else:
            print("机器人当前位置: 等待 /odom")
        if self.goal:
            print(f"当前导航目标: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}")
        else:
            print("当前导航目标: 等待 /goal_pose")
        print("-" * 72)
        print("最新业务事件")
        if not self.events:
            print("暂无事件。")
            return
        for event in self.events:
            event_type = event.get("type", "unknown")
            message = event.get("message", "")
            x = event.get("x")
            y = event.get("y")
            location = f" @ ({x:.2f}, {y:.2f})" if isinstance(x, (int, float)) and isinstance(y, (int, float)) else ""
            print(f"[{event.get('received_at', '--:--:--')}] {event_type}: {message}{location}")


def main(args=None):
    rclpy.init(args=args)
    node = PatrolDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
