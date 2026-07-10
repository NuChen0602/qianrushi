#!/usr/bin/env python3
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from patrol_knowledge import load_library_map


def default_route_config_path():
    return os.path.join(
        get_package_share_directory("library_gazebo"),
        "config",
        "real_route_primitives.json",
    )


class RealGoalDriver(Node):
    def __init__(self):
        super().__init__("real_goal_driver")
        self.declare_parameter("route_config", default_route_config_path())
        self.route_config_path = str(self.get_parameter("route_config").value)

        self.map_data = load_library_map()
        self.areas = list(self.map_data.get("areas", []))
        self.route_config = self.load_route_config(self.route_config_path)

        self.cmd_pub = self.create_publisher(String, "/patrol/real_motion_cmd", 10)
        self.status_pub = self.create_publisher(String, "/patrol/mission_status", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.worker_lock = threading.Lock()
        self.worker = None
        self.cancel_flag = False
        self.get_logger().info(f"Real goal driver using route config: {self.route_config_path}")

    def load_route_config(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"Route config load failed, using fallback: {exc}")
            return {"default_goal": [{"cmd": "MOVE forward 25", "wait_sec": 4.0}], "areas": {}}

    def publish_status(self, event_type, message, **extra):
        payload = {"type": event_type, "message": message}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def goal_callback(self, msg):
        area = self.find_nearest_area(msg.pose.position.x, msg.pose.position.y)
        if area is None:
            self.publish_status("real_goal_error", "No semantic area matches the goal")
            return

        with self.worker_lock:
            self.cancel_flag = True
            if self.worker and self.worker.is_alive():
                self.publish_motion_cmd("STOP")
                time.sleep(0.2)
            self.cancel_flag = False
            self.worker = threading.Thread(target=self.run_area_sequence, args=(area,), daemon=True)
            self.worker.start()

    def find_nearest_area(self, x, y):
        best = None
        best_dist = None
        for area in self.areas:
            dist = math.hypot(float(area["x"]) - x, float(area["y"]) - y)
            if best_dist is None or dist < best_dist:
                best = area
                best_dist = dist
        return best

    def sequence_for_area(self, area):
        by_area = self.route_config.get("areas", {})
        return by_area.get(area["id"], self.route_config.get("default_goal", []))

    def publish_motion_cmd(self, line):
        msg = String()
        msg.data = line
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"motion cmd: {line}")

    def run_area_sequence(self, area):
        sequence = self.sequence_for_area(area)
        self.publish_status(
            "real_goal_start",
            f"Real robot moving toward {area['name']}: {area['label']}",
            area_id=area["id"],
            x=float(area["x"]),
            y=float(area["y"]),
        )
        for step in sequence:
            if self.cancel_flag or not rclpy.ok():
                self.publish_status("real_goal_cancelled", f"Cancelled {area['name']}", area_id=area["id"])
                return
            line = str(step.get("cmd", "")).strip()
            if not line:
                continue
            wait_sec = float(step.get("wait_sec", 3.0))
            self.publish_motion_cmd(line)
            time.sleep(max(0.1, wait_sec))
        self.publish_status("real_goal_done", f"Arrived near {area['name']}", area_id=area["id"])


def main(args=None):
    rclpy.init(args=args)
    node = RealGoalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
