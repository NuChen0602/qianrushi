#!/usr/bin/env python3
import json
import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def parse_kv_tokens(tokens):
    data = {}
    for token in tokens:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        data[key] = value
    return data


class RealMotionBridge(Node):
    def __init__(self):
        super().__init__("real_motion_bridge")
        self.declare_parameter("board_host", "192.168.2.77")
        self.declare_parameter("board_port", 15000)
        self.declare_parameter("local_port", 15001)
        self.declare_parameter("counts_per_meter", 800.0)
        self.declare_parameter("turn_yaw_rad", 0.35)
        self.declare_parameter("cmd_vel_counts_scale", 30.0)
        self.declare_parameter("cmd_vel_min_period", 0.7)
        self.declare_parameter("linear_deadband", 0.05)
        self.declare_parameter("angular_deadband", 0.10)

        self.board_host = str(self.get_parameter("board_host").value)
        self.board_port = int(self.get_parameter("board_port").value)
        self.local_port = int(self.get_parameter("local_port").value)
        self.counts_per_meter = float(self.get_parameter("counts_per_meter").value)
        self.turn_yaw_rad = float(self.get_parameter("turn_yaw_rad").value)
        self.cmd_vel_counts_scale = float(self.get_parameter("cmd_vel_counts_scale").value)
        self.cmd_vel_min_period = float(self.get_parameter("cmd_vel_min_period").value)
        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.status_pub = self.create_publisher(String, "/patrol/mission_status", 10)
        self.create_subscription(String, "/patrol/real_motion_cmd", self.command_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.local_port))
        self.sock.settimeout(0.2)
        self.sock_lock = threading.Lock()
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_motion = None
        self.last_cmd_vel_send = 0.0
        self.last_ping = 0.0

        self.create_timer(0.1, self.publish_odom)
        self.create_timer(2.0, self.ping_board)
        self.publish_status("real_bridge_start", f"UDP bridge ready for {self.board_host}:{self.board_port}")
        self.get_logger().info(f"Real motion bridge listening on UDP :{self.local_port}, board={self.board_host}:{self.board_port}")

    def publish_status(self, event_type, message, **extra):
        payload = {"type": event_type, "message": message}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def send_board(self, line):
        if not self.board_host:
            self.get_logger().warn("board_host is empty; command not sent")
            return
        payload = (line.strip() + "\n").encode("utf-8")
        with self.sock_lock:
            self.sock.sendto(payload, (self.board_host, self.board_port))
        self.get_logger().info(f"board <= {line.strip()}")
        self.remember_motion(line.strip())

    def remember_motion(self, line):
        words = line.split()
        if not words:
            return
        op = words[0].lower()
        motion = None
        counts = None
        if op in {"move", "motion"} and len(words) >= 2:
            motion = words[1].lower()
            counts = int(words[2]) if len(words) >= 3 and words[2].lstrip("-").isdigit() else None
        elif op == "turn" and len(words) >= 2:
            motion = words[1].lower()
            counts = int(words[2]) if len(words) >= 3 and words[2].lstrip("-").isdigit() else None
        elif op in {"forward", "back", "left", "right"}:
            motion = op
            counts = int(words[1]) if len(words) >= 2 and words[1].lstrip("-").isdigit() else None
        if motion:
            self.last_motion = {"motion": motion, "counts": counts, "line": line, "time": time.time()}

    def command_callback(self, msg):
        self.send_board(msg.data)

    def cmd_vel_callback(self, msg):
        now = time.time()
        if now - self.last_cmd_vel_send < self.cmd_vel_min_period:
            return
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if abs(linear) >= self.linear_deadband:
            counts = max(1, min(80, int(abs(linear) * self.cmd_vel_counts_scale)))
            self.send_board(f"MOVE {'forward' if linear > 0 else 'back'} {counts}")
            self.last_cmd_vel_send = now
        elif abs(angular) >= self.angular_deadband:
            counts = max(1, min(35, int(abs(angular) * self.cmd_vel_counts_scale)))
            self.send_board(f"TURN {'left' if angular > 0 else 'right'} {counts}")
            self.last_cmd_vel_send = now

    def ping_board(self):
        self.send_board("PING")

    def rx_loop(self):
        while self.running:
            try:
                data, peer = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            text = data.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            self.get_logger().info(f"board => {text}")
            self.handle_board_line(text, peer)

    def handle_board_line(self, text, peer):
        words = text.split()
        if not words:
            return
        kind = words[0].lower()
        if kind == "done":
            kv = parse_kv_tokens(words[2:])
            motion = words[1].lower() if len(words) > 1 else self.last_motion_name()
            enc1 = int(kv.get("enc1", "0"))
            enc2 = int(kv.get("enc2", "0"))
            self.apply_motion_estimate(motion, enc1, enc2)
            self.publish_status(
                "real_motion_done",
                f"Board completed {motion}",
                x=self.x,
                y=self.y,
                yaw=self.yaw,
                enc1=enc1,
                enc2=enc2,
            )
        elif kind == "state":
            self.publish_status("real_bridge_state", text)
        elif kind in {"ack", "pong", "busy", "err"}:
            self.publish_status("real_bridge_reply", text)

    def last_motion_name(self):
        return self.last_motion["motion"] if self.last_motion else "unknown"

    def apply_motion_estimate(self, motion, enc1, enc2):
        counts = max(abs(enc1), abs(enc2))
        if counts == 0 and self.last_motion and self.last_motion.get("counts"):
            counts = abs(int(self.last_motion["counts"]))
        distance = counts / max(1.0, self.counts_per_meter)

        if motion == "forward":
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)
        elif motion == "back":
            self.x -= distance * math.cos(self.yaw)
            self.y -= distance * math.sin(self.yaw)
        elif motion == "left":
            self.yaw += self.turn_yaw_rad
        elif motion == "right":
            self.yaw -= self.turn_yaw_rad
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def publish_odom(self):
        msg = Odometry()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = yaw_to_quaternion(self.yaw)
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        self.odom_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealMotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
