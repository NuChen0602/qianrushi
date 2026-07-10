#!/usr/bin/env python3
import argparse
import json
import math
import socket
import time


def receive_packets(host, port, required, timeout_sec):
    packets = []
    deadline = time.monotonic() + timeout_sec
    buffer = b''
    with socket.create_connection((host, port), timeout=3.0) as sock:
        sock.settimeout(0.5)
        while len(packets) < required and time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buffer += chunk
            while b'\n' in buffer and len(packets) < required:
                line, buffer = buffer.split(b'\n', 1)
                if not line:
                    continue
                packets.append(json.loads(line.decode('ascii')))
    if len(packets) < required:
        raise RuntimeError(
            f'{host}:{port} provided {len(packets)}/{required} packets')
    return packets


def check_sequence(packets, label):
    previous = None
    previous_mono_ns = None
    for packet in packets:
        sequence = int(packet.get('seq', -1))
        if sequence < 0:
            raise RuntimeError(f'{label}: missing sequence number')
        if previous is not None and sequence <= previous:
            raise RuntimeError(
                f'{label}: non-increasing sequence {previous} -> {sequence}')
        previous = sequence
        mono_ns = int(packet.get('mono_ns', 0))
        if mono_ns <= 0:
            raise RuntimeError(f'{label}: missing board monotonic timestamp')
        if previous_mono_ns is not None and mono_ns <= previous_mono_ns:
            raise RuntimeError(
                f'{label}: non-increasing timestamp '
                f'{previous_mono_ns} -> {mono_ns}')
        previous_mono_ns = mono_ns


def check_lidar(args):
    packets = receive_packets(
        args.host, args.lidar_port, args.lidar_samples, args.timeout)
    check_sequence(packets, 'lidar')

    frequencies = []
    point_counts = []
    for packet in packets:
        frequency = float(packet.get('hz', 0.0))
        if not math.isfinite(frequency):
            raise RuntimeError('lidar: non-finite scan frequency')
        if not args.min_lidar_hz <= frequency <= args.max_lidar_hz:
            raise RuntimeError(
                f'lidar: frequency {frequency:.2f} Hz outside '
                f'[{args.min_lidar_hz}, {args.max_lidar_hz}]')

        points = packet.get('points')
        if not isinstance(points, list):
            raise RuntimeError('lidar: points is not a list')
        valid_points = 0
        for point in points:
            if not isinstance(point, list) or len(point) < 2:
                continue
            angle = float(point[0])
            distance_mm = float(point[1])
            if (math.isfinite(angle) and math.isfinite(distance_mm) and
                    0.0 <= angle < 360.0 and 20.0 <= distance_mm <= 30000.0):
                valid_points += 1
        if valid_points < args.min_lidar_points:
            raise RuntimeError(
                f'lidar: only {valid_points} valid points, '
                f'minimum is {args.min_lidar_points}')
        frequencies.append(frequency)
        point_counts.append(valid_points)

    print(
        'LIDAR OK: '
        f'samples={len(packets)} '
        f'avg_hz={sum(frequencies) / len(frequencies):.2f} '
        f'min_points={min(point_counts)}')


def check_odometry(args):
    packets = receive_packets(
        args.host, args.odom_port, args.odom_samples + 1, args.timeout)
    # The board emits one initialization packet immediately after accept().
    # Its dt is intentionally near zero because no complete 20 ms cycle has
    # elapsed yet, so it must not be judged as a normal odometry sample.
    packets = packets[1:]
    check_sequence(packets, 'odometry')

    dts = []
    count_magnitudes = []
    gyros = []
    for packet in packets:
        if packet.get('imu_ready') is not True:
            raise RuntimeError('odometry: IMU is not ready')
        dt = float(packet.get('dt', 0.0))
        gyro_z_dps = float(packet.get('gyro_z_dps', math.nan))
        left = int(packet.get('left', 0))
        right = int(packet.get('right', 0))
        if not args.min_odom_dt <= dt <= args.max_odom_dt:
            raise RuntimeError(
                f'odometry: dt={dt:.4f}s outside '
                f'[{args.min_odom_dt}, {args.max_odom_dt}]')
        if (abs(left) > args.max_encoder_count or
                abs(right) > args.max_encoder_count):
            raise RuntimeError(
                f'odometry: encoder spike left={left} right={right}')
        if (not math.isfinite(gyro_z_dps) or
                abs(gyro_z_dps) > args.max_gyro_dps):
            raise RuntimeError(
                f'odometry: invalid gyro_z_dps={gyro_z_dps}')
        dts.append(dt)
        count_magnitudes.append(0.5 * (abs(left) + abs(right)))
        gyros.append(abs(gyro_z_dps))

    average_count = sum(count_magnitudes) / len(count_magnitudes)
    if average_count > args.max_stationary_count:
        raise RuntimeError(
            'odometry: car must remain still during preflight; '
            f'average encoder magnitude is {average_count:.1f}')
    average_gyro = sum(gyros) / len(gyros)
    if average_gyro > args.max_stationary_gyro_dps:
        raise RuntimeError(
            'odometry: IMU heading would drift while stationary; '
            f'average gyro magnitude is {average_gyro:.2f} dps')

    print(
        'ODOMETRY OK: '
        f'samples={len(packets)} '
        f'avg_dt={sum(dts) / len(dts):.4f}s '
        f'avg_stationary_count={average_count:.2f} '
        f'avg_abs_gyro={average_gyro:.2f}dps '
        f'max_abs_gyro={max(gyros):.2f}dps')


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Validate raw lidar and odometry streams before ROS2 '
            'SLAM starts.'))
    parser.add_argument('--host', default='192.168.123.70')
    parser.add_argument('--lidar-port', type=int, default=2368)
    parser.add_argument('--odom-port', type=int, default=2369)
    parser.add_argument('--lidar-samples', type=int, default=5)
    parser.add_argument('--odom-samples', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--min-lidar-hz', type=float, default=5.0)
    parser.add_argument('--max-lidar-hz', type=float, default=15.0)
    parser.add_argument('--min-lidar-points', type=int, default=30)
    parser.add_argument('--min-odom-dt', type=float, default=0.005)
    parser.add_argument('--max-odom-dt', type=float, default=0.100)
    parser.add_argument('--max-encoder-count', type=int, default=2000)
    parser.add_argument('--max-stationary-count', type=float, default=100.0)
    parser.add_argument('--max-stationary-gyro-dps', type=float, default=3.0)
    parser.add_argument('--max-gyro-dps', type=float, default=500.0)
    args = parser.parse_args()

    try:
        check_lidar(args)
        check_odometry(args)
    except (OSError, ValueError, json.JSONDecodeError, RuntimeError) as exc:
        print(f'MAPPING PREFLIGHT FAILED: {exc}')
        return 1

    print('MAPPING PREFLIGHT PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
