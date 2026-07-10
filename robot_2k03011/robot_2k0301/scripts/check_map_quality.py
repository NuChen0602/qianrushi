#!/usr/bin/env python3

import argparse
import json
import math
from collections import deque
from pathlib import Path


def parse_simple_yaml(path):
    values = {}
    for raw_line in path.read_text(encoding='utf-8').splitlines():
        line = raw_line.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        key, value = line.split(':', 1)
        values[key.strip()] = value.strip()
    return values


def read_token(stream):
    token = bytearray()
    while True:
        byte = stream.read(1)
        if not byte:
            raise ValueError('unexpected end of PGM header')
        if byte == b'#':
            stream.readline()
            continue
        if not byte.isspace():
            token.extend(byte)
            break
    while True:
        byte = stream.read(1)
        if not byte or byte.isspace():
            return bytes(token)
        token.extend(byte)


def read_pgm(path):
    with path.open('rb') as stream:
        magic = read_token(stream)
        if magic != b'P5':
            raise ValueError(f'unsupported PGM format {magic!r}, expected P5')
        width = int(read_token(stream))
        height = int(read_token(stream))
        maximum = int(read_token(stream))
        if maximum != 255:
            raise ValueError(
                f'unsupported PGM maximum {maximum}, expected 255')
        pixels = stream.read(width * height)
    if len(pixels) != width * height:
        raise ValueError(
            f'incomplete PGM data: expected {width * height} bytes, '
            f'got {len(pixels)}')
    return width, height, pixels


def occupied_bounds(width, pixels):
    occupied = [index for index, value in enumerate(pixels) if value < 100]
    if not occupied:
        raise ValueError('map contains no occupied cells')
    xs = [index % width for index in occupied]
    ys = [index // width for index in occupied]
    return (
        len(occupied),
        min(xs),
        min(ys),
        max(xs),
        max(ys),
    )


def largest_free_component(width, height, pixels):
    free = bytearray(value > 250 for value in pixels)
    visited = bytearray(width * height)
    largest = []

    for start in range(width * height):
        if not free[start] or visited[start]:
            continue
        queue = deque([start])
        visited[start] = 1
        component = []
        while queue:
            index = queue.popleft()
            x = index % width
            y = index // width
            component.append((x, y))
            neighbours = []
            if x > 0:
                neighbours.append(index - 1)
            if x + 1 < width:
                neighbours.append(index + 1)
            if y > 0:
                neighbours.append(index - width)
            if y + 1 < height:
                neighbours.append(index + width)
            for neighbour in neighbours:
                if free[neighbour] and not visited[neighbour]:
                    visited[neighbour] = 1
                    queue.append(neighbour)
        if len(component) > len(largest):
            largest = component

    if not largest:
        raise ValueError('map contains no connected free-space region')
    return largest


def percentile(sorted_values, fraction):
    index = round((len(sorted_values) - 1) * fraction)
    return sorted_values[max(0, min(index, len(sorted_values) - 1))]


def interior_free_span(width, height, pixels, resolution):
    points = largest_free_component(width, height, pixels)
    best = None

    # Search the dominant room orientation. Trimming 0.5% at each edge keeps
    # a doorway or a few scan-matching outliers from changing the room size.
    for half_degree in range(180):
        angle = math.radians(half_degree * 0.5)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        axis_a = sorted(x * cosine + y * sine for x, y in points)
        axis_b = sorted(-x * sine + y * cosine for x, y in points)
        span_a = (
            percentile(axis_a, 0.995) - percentile(axis_a, 0.005) + 1.0)
        span_b = (
            percentile(axis_b, 0.995) - percentile(axis_b, 0.005) + 1.0)
        candidate = (span_a * span_b, span_a, span_b, angle)
        if best is None or candidate[0] < best[0]:
            best = candidate

    _, span_a, span_b, angle = best
    return (
        len(points),
        span_a * resolution,
        span_b * resolution,
        math.degrees(angle),
    )


def inspect_map(
        pgm_path,
        resolution,
        expected_width,
        expected_height,
        size_tolerance,
        max_outer_excess,
        max_unknown_ratio):
    width, height, pixels = read_pgm(pgm_path)
    occupied_count, min_x, min_y, max_x, max_y = occupied_bounds(
        width, pixels)
    unknown_count = sum(100 <= value <= 250 for value in pixels)
    total_cells = width * height
    unknown_ratio = unknown_count / total_cells
    occupied_width = (max_x - min_x + 1) * resolution
    occupied_height = (max_y - min_y + 1) * resolution
    (
        interior_free_cells,
        interior_width,
        interior_height,
        interior_angle_deg,
    ) = interior_free_span(width, height, pixels, resolution)

    # The orientation search may exchange the two room axes. Match them to
    # the expected dimensions using the lower total absolute error.
    direct_error = (
        abs(interior_width - expected_width) +
        abs(interior_height - expected_height))
    swapped_error = (
        abs(interior_height - expected_width) +
        abs(interior_width - expected_height))
    if swapped_error < direct_error:
        interior_width, interior_height = interior_height, interior_width

    minimum_width = expected_width - size_tolerance
    maximum_width = expected_width + size_tolerance
    minimum_height = expected_height - size_tolerance
    maximum_height = expected_height + size_tolerance
    reasons = []
    if not minimum_width <= interior_width <= maximum_width:
        reasons.append(
            f'interior width {interior_width:.2f} m is outside '
            f'[{minimum_width:.2f}, {maximum_width:.2f}] m')
    if not minimum_height <= interior_height <= maximum_height:
        reasons.append(
            f'interior height {interior_height:.2f} m is outside '
            f'[{minimum_height:.2f}, {maximum_height:.2f}] m')
    if occupied_width > expected_width + max_outer_excess:
        reasons.append(
            f'occupied outer width {occupied_width:.2f} m exceeds '
            f'{expected_width + max_outer_excess:.2f} m; possible ghost wall')
    if occupied_height > expected_height + max_outer_excess:
        reasons.append(
            f'occupied outer height {occupied_height:.2f} m exceeds '
            f'{expected_height + max_outer_excess:.2f} m; possible ghost wall')
    if unknown_ratio > max_unknown_ratio:
        reasons.append(
            f'unknown-cell ratio {unknown_ratio:.1%} exceeds '
            f'{max_unknown_ratio:.1%}')
    if occupied_count < 100:
        reasons.append(
            f'only {occupied_count} occupied cells were recorded')

    return {
        'passed': not reasons,
        'pgm': str(pgm_path),
        'resolution_m': resolution,
        'grid_width_cells': width,
        'grid_height_cells': height,
        'grid_width_m': width * resolution,
        'grid_height_m': height * resolution,
        'occupied_width_m': occupied_width,
        'occupied_height_m': occupied_height,
        'occupied_cells': occupied_count,
        'interior_width_m': interior_width,
        'interior_height_m': interior_height,
        'interior_angle_deg': interior_angle_deg,
        'interior_free_cells': interior_free_cells,
        'unknown_ratio': unknown_ratio,
        'expected_width_m': expected_width,
        'expected_height_m': expected_height,
        'size_tolerance_m': size_tolerance,
        'max_outer_excess_m': max_outer_excess,
        'reasons': reasons,
    }


def resolve_inputs(args):
    if args.map_yaml:
        yaml_path = Path(args.map_yaml).resolve()
        values = parse_simple_yaml(yaml_path)
        image = values.get('image')
        if not image:
            raise ValueError(f'missing image entry in {yaml_path}')
        resolution = float(values.get('resolution', 'nan'))
        pgm_path = (yaml_path.parent / image).resolve()
    else:
        pgm_path = Path(args.pgm).resolve()
        resolution = args.resolution
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('map resolution must be a positive finite value')
    return pgm_path, resolution


def main():
    parser = argparse.ArgumentParser(
        description='Reject stretched, incomplete, or mostly unknown maps.')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--map-yaml')
    source.add_argument('--pgm')
    parser.add_argument('--resolution', type=float, default=math.nan)
    parser.add_argument('--expected-width', type=float, default=1.8)
    parser.add_argument('--expected-height', type=float, default=1.8)
    parser.add_argument('--size-tolerance', type=float, default=0.20)
    parser.add_argument('--max-outer-excess', type=float, default=0.60)
    parser.add_argument('--max-unknown-ratio', type=float, default=0.50)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()

    try:
        pgm_path, resolution = resolve_inputs(args)
        result = inspect_map(
            pgm_path=pgm_path,
            resolution=resolution,
            expected_width=args.expected_width,
            expected_height=args.expected_height,
            size_tolerance=args.size_tolerance,
            max_outer_excess=args.max_outer_excess,
            max_unknown_ratio=args.max_unknown_ratio,
        )
    except (OSError, ValueError) as exc:
        result = {'passed': False, 'reasons': [str(exc)]}

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        state = 'PASS' if result['passed'] else 'FAIL'
        print(f'map quality: {state}')
        if 'interior_width_m' in result:
            print(
                '  estimated interior: '
                f'{result["interior_width_m"]:.2f} m x '
                f'{result["interior_height_m"]:.2f} m '
                f'(map angle {result["interior_angle_deg"]:.1f} deg)')
            print(
                '  expected interior: '
                f'{result["expected_width_m"]:.2f} m x '
                f'{result["expected_height_m"]:.2f} m '
                f'(tolerance +/-{result["size_tolerance_m"]:.2f} m)')
            print(
                '  occupied outer span: '
                f'{result["occupied_width_m"]:.2f} m x '
                f'{result["occupied_height_m"]:.2f} m')
            print(
                f'  grid: {result["grid_width_cells"]} x '
                f'{result["grid_height_cells"]} cells at '
                f'{result["resolution_m"]:.3f} m/cell')
            print(f'  unknown cells: {result["unknown_ratio"]:.1%}')
        for reason in result['reasons']:
            print(f'  - {reason}')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
