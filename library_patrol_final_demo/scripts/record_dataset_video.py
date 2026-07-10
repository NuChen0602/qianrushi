#!/usr/bin/env python3
"""Record the board length-prefixed JPEG stream for dataset collection."""

import argparse
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def read_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("camera stream closed")
        data.extend(chunk)
    return bytes(data)


def read_jpeg(sock, max_frame_bytes):
    size = struct.unpack("!I", read_exact(sock, 4))[0]
    if size <= 0 or size > max_frame_bytes:
        raise ValueError(f"invalid JPEG frame size: {size}")
    jpeg = read_exact(sock, size)
    if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
        raise ValueError("incomplete JPEG frame")
    return jpeg


def decode_jpeg(jpeg):
    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("failed to decode JPEG frame")
    return frame


def create_writer(session_dir, width, height, fps):
    mp4_path = session_dir / "capture.mp4"
    writer = cv2.VideoWriter(
        str(mp4_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if writer.isOpened():
        return writer, mp4_path

    writer.release()
    avi_path = session_dir / "capture.avi"
    writer = cv2.VideoWriter(
        str(avi_path),
        cv2.VideoWriter_fourcc(*"MJPG"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV cannot create MP4 or AVI video")
    return writer, avi_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record target_bundle/background sessions without LabelImg.",
    )
    parser.add_argument("--host", default="192.168.43.192")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--label", choices=("target_bundle", "background", "preview"), required=True)
    parser.add_argument("--duration", type=float, default=60.0, help="wall-clock seconds")
    parser.add_argument("--video-fps", type=float, default=15.0)
    parser.add_argument("--sample-fps", type=float, default=3.0)
    parser.add_argument(
        "--output-root",
        default=str(Path(__file__).resolve().parents[1] / "dataset_recordings"),
    )
    parser.add_argument("--max-frame-bytes", type=int, default=5 * 1024 * 1024)
    parser.add_argument("--snapshot-only", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0 or args.video_fps <= 0 or args.sample_fps < 0:
        raise SystemExit("duration/video-fps must be positive and sample-fps cannot be negative")

    session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = Path(args.output_root).expanduser().resolve() / args.label / session_name
    frames_dir = session_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=False)

    print(f"[record] connecting to {args.host}:{args.port}")
    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.settimeout(5.0)
        first_jpeg = read_jpeg(sock, args.max_frame_bytes)
        first_frame = decode_jpeg(first_jpeg)
        height, width = first_frame.shape[:2]

        if args.snapshot_only:
            snapshot_path = session_dir / "snapshot.jpg"
            snapshot_path.write_bytes(first_jpeg)
            print(f"[record] snapshot={snapshot_path}")
            print(f"[record] resolution={width}x{height}")
            return

        writer, video_path = create_writer(session_dir, width, height, args.video_fps)
        started_at = time.monotonic()
        next_video_at = started_at
        next_sample_at = started_at
        video_period = 1.0 / args.video_fps
        sample_period = 1.0 / args.sample_fps if args.sample_fps > 0 else None
        received = 0
        written = 0
        sampled = 0
        jpeg = first_jpeg
        frame = first_frame

        print(
            f"[record] label={args.label} resolution={width}x{height} "
            f"duration={args.duration:.1f}s video_fps={args.video_fps:.1f} sample_fps={args.sample_fps:.1f}"
        )
        try:
            while True:
                now = time.monotonic()
                if now - started_at >= args.duration:
                    break
                received += 1

                if now >= next_video_at:
                    writer.write(frame)
                    written += 1
                    while next_video_at <= now:
                        next_video_at += video_period

                if sample_period is not None and now >= next_sample_at:
                    frame_path = frames_dir / f"frame_{sampled:05d}.jpg"
                    frame_path.write_bytes(jpeg)
                    sampled += 1
                    while next_sample_at <= now:
                        next_sample_at += sample_period

                jpeg = read_jpeg(sock, args.max_frame_bytes)
                frame = decode_jpeg(jpeg)
        except KeyboardInterrupt:
            print("\n[record] interrupted; keeping recorded data")
        finally:
            writer.release()

    elapsed = max(0.001, time.monotonic() - started_at)
    print(f"[record] video={video_path}")
    print(f"[record] sampled_frames={frames_dir}")
    print(
        f"[record] received={received} ({received / elapsed:.1f} FPS) "
        f"video_frames={written} samples={sampled} elapsed={elapsed:.1f}s"
    )


if __name__ == "__main__":
    main()
