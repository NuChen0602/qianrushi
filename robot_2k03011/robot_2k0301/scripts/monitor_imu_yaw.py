#!/usr/bin/env python3

import argparse
import json
import math
import socket
import time


def normalize_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser(
        description='Display integrated IMU Z-axis yaw from the board odometry stream.')
    parser.add_argument('--host', default='192.168.123.70')
    parser.add_argument('--port', type=int, default=2369)
    parser.add_argument('--sign', type=float, default=1.0)
    parser.add_argument('--display-hz', type=float, default=10.0)
    args = parser.parse_args()

    yaw_total = 0.0
    next_display = 0.0
    print('IMU yaw monitor: keep the car still at startup, then rotate it by hand.')
    print('Press Ctrl+C to stop. Positive/negative direction depends on IMU mounting.')

    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.settimeout(2.0)
        with sock.makefile('r', encoding='ascii') as stream:
            for line in stream:
                packet = json.loads(line)
                if packet.get('imu_ready') is not True:
                    raise RuntimeError('board reports that the IMU is not ready')
                dt = float(packet['dt'])
                gyro_z_dps = float(packet['gyro_z_dps']) * args.sign
                if not (math.isfinite(dt) and math.isfinite(gyro_z_dps)):
                    continue
                if not 0.0 < dt <= 0.2:
                    continue
                yaw_total += gyro_z_dps * dt

                now = time.monotonic()
                if now >= next_display:
                    print(
                        f'\ryaw_total={yaw_total:+9.2f} deg  '
                        f'yaw_norm={normalize_degrees(yaw_total):+8.2f} deg  '
                        f'gyro_z={gyro_z_dps:+8.2f} deg/s',
                        end='',
                        flush=True)
                    next_display = now + 1.0 / max(args.display_hz, 1.0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.')
