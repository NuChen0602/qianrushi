import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MappingKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('mapping_keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('initial_speed_mps', 0.20)
        self.declare_parameter('min_speed_mps', 0.16)
        self.declare_parameter('max_speed_mps', 0.28)
        self.declare_parameter('speed_step_mps', 0.02)
        self.declare_parameter('full_steer_speed_scale', 0.80)
        self.declare_parameter('steering_step', 0.1)
        self.declare_parameter('deadman_timeout_sec', 2.0)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.selected_speed = float(
            self.get_parameter('initial_speed_mps').value)
        self.min_speed = float(self.get_parameter('min_speed_mps').value)
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.speed_step = float(self.get_parameter('speed_step_mps').value)
        self.full_steer_speed_scale = max(
            0.5,
            min(
                1.0,
                float(self.get_parameter('full_steer_speed_scale').value)))
        self.steering_step = float(
            self.get_parameter('steering_step').value)
        self.deadman_timeout = float(
            self.get_parameter('deadman_timeout_sec').value)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.drive_direction = 0.0
        self.steering = 0.0
        self.last_drive_key = 0.0
        self.running = True

    def publish_command(self):
        now = time.monotonic()
        if (self.drive_direction != 0.0 and
                now - self.last_drive_key > self.deadman_timeout):
            self.drive_direction = 0.0
            self.show_status('deadman stop')

        message = Twist()
        message.linear.x = self.current_linear_speed()
        message.angular.z = self.steering
        self.publisher.publish(message)

    def current_linear_speed(self):
        steering_scale = (
            1.0 -
            (1.0 - self.full_steer_speed_scale) * abs(self.steering))
        return self.drive_direction * self.selected_speed * steering_scale

    def show_status(self, reason='command'):
        linear = self.current_linear_speed()
        print(
            f'\r{reason}: speed={linear:+.2f}m/s '
            f'steering={self.steering:+.1f}    ',
            end='',
            flush=True)

    def handle_key(self, key):
        key = key.lower()
        refresh_motion = False
        if key == 'w':
            self.drive_direction = 1.0
            refresh_motion = True
        elif key == 's':
            self.drive_direction = -1.0
            refresh_motion = True
        elif key == 'a':
            self.steering = min(1.0, self.steering + self.steering_step)
            refresh_motion = True
        elif key == 'd':
            self.steering = max(-1.0, self.steering - self.steering_step)
            refresh_motion = True
        elif key == 'c':
            self.steering = 0.0
            refresh_motion = True
        elif key in ('+', '='):
            self.selected_speed = min(
                self.max_speed, self.selected_speed + self.speed_step)
            refresh_motion = True
        elif key in ('-', '_'):
            self.selected_speed = max(
                self.min_speed, self.selected_speed - self.speed_step)
            refresh_motion = True
        elif key in (' ', 'x'):
            self.drive_direction = 0.0
        elif key == 'q':
            self.drive_direction = 0.0
            self.steering = 0.0
            self.running = False
        else:
            return
        if refresh_motion:
            self.last_drive_key = time.monotonic()
        self.show_status()

    def send_stop(self):
        self.drive_direction = 0.0
        self.steering = 0.0
        for _ in range(5):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.03)


def main(args=None):
    rclpy.init(args=args)
    node = MappingKeyboardTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error(
            'keyboard teleop requires an interactive terminal')
        node.destroy_node()
        rclpy.shutdown()
        return

    print('键盘建图遥控：W/S或上下方向键行驶，A/D或左右方向键转向，C回正，+/-调速，空格停车，Q保存并退出')
    original = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    try:
        node.show_status('ready')
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                keys = sys.stdin.read(1)
                if keys == '\x1b':
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        keys += sys.stdin.read(1)
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        keys += sys.stdin.read(1)
                    arrow_keys = {
                        '\x1b[A': 'w',
                        '\x1b[B': 's',
                        '\x1b[D': 'a',
                        '\x1b[C': 'd',
                    }
                    keys = arrow_keys.get(keys, keys)
                if keys:
                    node.handle_key(keys)
            node.publish_command()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        print('\n键盘遥控已停车。')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
