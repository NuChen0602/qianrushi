import socket
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class Ips200MapStream(Node):
    def __init__(self):
        super().__init__('ips200_map_stream')
        self.declare_parameter('host', '192.168.123.70')
        self.declare_parameter('port', 2370)
        self.declare_parameter('frame_width', 240)
        self.declare_parameter('frame_height', 180)
        self.declare_parameter('max_fps', 2.0)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.min_interval = 1.0 / max(float(self.get_parameter('max_fps').value), 0.2)

        self.sock = None
        self.last_send = 0.0
        self.last_warn = 0.0

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1,
        )
        self.get_logger().info(
            f'Streaming /map to IPS200 at {self.host}:{self.port}')

    def destroy_node(self):
        self.close_socket()
        super().destroy_node()

    def close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def connect(self):
        if self.sock is not None:
            return True

        try:
            sock = socket.create_connection((self.host, self.port), timeout=0.5)
            sock.settimeout(0.5)
        except OSError as exc:
            now = time.monotonic()
            if now - self.last_warn > 5.0:
                self.get_logger().warn(
                    f'Cannot connect to IPS200 map display: {exc}')
                self.last_warn = now
            return False

        self.sock = sock
        self.get_logger().info('Connected to IPS200 map display')
        return True

    @staticmethod
    def rgb565_to_le(color):
        return bytes((color & 0xFF, (color >> 8) & 0xFF))

    def map_callback(self, msg):
        now = time.monotonic()
        if now - self.last_send < self.min_interval:
            return
        self.last_send = now

        if msg.info.width == 0 or msg.info.height == 0:
            return
        if not self.connect():
            return

        frame = self.render_map(msg)
        try:
            self.sock.sendall(frame)
        except OSError as exc:
            self.get_logger().warn(f'IPS200 map stream disconnected: {exc}')
            self.close_socket()

    def render_map(self, msg):
        map_width = int(msg.info.width)
        map_height = int(msg.info.height)
        data = msg.data

        scale = min(self.frame_width / map_width, self.frame_height / map_height)
        display_width = max(1, int(map_width * scale))
        display_height = max(1, int(map_height * scale))
        pad_x = (self.frame_width - display_width) // 2
        pad_y = (self.frame_height - display_height) // 2

        background = 0x39E7
        unknown = 0x8410
        free = 0xFFFF
        occupied = 0x0000

        frame = bytearray(self.frame_width * self.frame_height * 2)
        bg_le = self.rgb565_to_le(background)
        unknown_le = self.rgb565_to_le(unknown)
        free_le = self.rgb565_to_le(free)
        occupied_le = self.rgb565_to_le(occupied)

        for y in range(self.frame_height):
            in_y = pad_y <= y < pad_y + display_height
            for x in range(self.frame_width):
                offset = (y * self.frame_width + x) * 2
                if not in_y or x < pad_x or x >= pad_x + display_width:
                    frame[offset:offset + 2] = bg_le
                    continue

                map_x = int((x - pad_x) / scale)
                visual_y = int((y - pad_y) / scale)
                map_y = map_height - 1 - visual_y
                map_x = min(max(map_x, 0), map_width - 1)
                map_y = min(max(map_y, 0), map_height - 1)
                value = data[map_y * map_width + map_x]

                if value < 0:
                    color = unknown_le
                elif value >= 50:
                    color = occupied_le
                else:
                    color = free_le
                frame[offset:offset + 2] = color

        return bytes(frame)


def main(args=None):
    rclpy.init(args=args)
    node = Ips200MapStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
