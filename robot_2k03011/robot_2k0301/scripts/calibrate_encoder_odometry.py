#!/usr/bin/env python3
import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(
        description='Accumulate wheel encoder counts over a measured straight distance.')
    parser.add_argument('--host', default='192.168.123.70')
    parser.add_argument('--port', type=int, default=2369)
    parser.add_argument('--distance', type=float, required=True, help='Measured distance in meters')
    args = parser.parse_args()
    if args.distance <= 0.0:
        raise SystemExit('--distance must be positive')

    left_total = 0
    right_total = 0
    print('连接成功后，沿直线推动小车指定距离，结束时按 Ctrl+C。')
    try:
        with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
            sock.settimeout(None)
            with sock.makefile('r', encoding='ascii') as stream:
                for line in stream:
                    packet = json.loads(line)
                    left_total += int(packet.get('left', 0))
                    right_total += int(packet.get('right', 0))
                    print(
                        f'\rleft={left_total} right={right_total} '
                        f'average={(left_total + right_total) / 2.0:.1f}',
                        end='',
                        flush=True)
    except KeyboardInterrupt:
        pass

    average = 0.5 * (left_total + right_total)
    counts_per_meter = abs(average) / args.distance
    print(f'\ncounts_per_meter: {counts_per_meter:.3f}')
    print('将该值写入 ros2_ws/src/robot_lidar_bridge/config/odometry.yaml')


if __name__ == '__main__':
    main()
