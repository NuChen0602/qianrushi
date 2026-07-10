#!/usr/bin/env python3
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from patrol_knowledge import patrol_route_targets


class PatrolMission(Node):
    def __init__(self):
        super().__init__("patrol_mission")
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "/patrol/mission_status", 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.route = patrol_route_targets()

    def publish_status(self, event_type, message, target=None):
        payload = {"type": event_type, "message": message}
        if target:
            payload.update({"x": target.x, "y": target.y, "area_id": target.area_id})
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def run_route(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.publish_status("mission_error", "Nav2 navigate_to_pose action server 未就绪。")
            return

        for target in self.route:
            self.publish_status("mission_waypoint", target.speech, target)
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = target.x
            goal_msg.pose.position.y = target.y
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)

            goal = NavigateToPose.Goal()
            goal.pose = goal_msg
            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.publish_status("mission_error", f"Nav2 拒绝目标点：{target.name}", target)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.publish_status("mission_arrived", f"已到达{target.name}，开始低视角扫描。", target)

        self.publish_status("mission_done", "巡检路线执行完成。")


def main(args=None):
    rclpy.init(args=args)
    node = PatrolMission()
    try:
        node.run_route()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
