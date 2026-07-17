
#!/usr/bin/env python3
import argparse
import socket
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive board JPEG frames and detect ArUco book markers."
    )
    parser.add_argument("--host", default="192.168.43.192")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--dict", default="5x5_250")
    parser.add_argument(
        "--expected",
        choices=("engineering", "science", "liberal", "unknown"),
        default="engineering",
    )
    parser.add_argument("--save-dir", default="outputs")
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.6,
        help="Keep a marker for N seconds after it disappears to reduce flicker.",
    )
    return parser.parse_args()


def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("board stream closed the TCP connection")
        data.extend(chunk)
    return bytes(data)


def marker_category(marker_id):
    if 101 <= marker_id <= 105:
        return "engineering"
    if 151 <= marker_id <= 155:
        return "science"
    if 201 <= marker_id <= 205:
        return "liberal"
    return "unknown"


def short_category_name(category):
    return {
        "engineering": "ENG",
        "science": "SCI",
        "liberal": "LIB",
        "unknown": "UNK",
    }.get(category, "UNK")


def create_aruco_detector(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install it with: "
            "pip install opencv-contrib-python"
        )

    normalized = dictionary_name.lower().replace("dict_", "")
    dictionaries = {
        "5x5_250": cv2.aruco.DICT_5X5_250,
    }

    if normalized not in dictionaries:
        raise ValueError(
            f"Unsupported ArUco dictionary '{dictionary_name}'. "
            "Supported value: 5x5_250"
        )

    dictionary = cv2.aruco.getPredefinedDictionary(dictionaries[normalized])

    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers

    parameters = cv2.aruco.DetectorParameters_create()

    def detect_legacy(image):
        return cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)

    return detect_legacy


def draw_text(image, text, origin, color, scale=0.52, thickness=1):
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def annotate_frame(frame, markers, expected):
    overlay = frame.copy()

    markers = list(markers)
    markers.sort(key=lambda item: item["center_x"])

    misplaced = []
    summaries = []

    for index, marker in enumerate(markers, start=1):
        marker_id = marker["id"]
        category = marker_category(marker_id)
        wrong = expected != "unknown" and category != expected

        if wrong:
            misplaced.append(index)

        polygon = np.rint(marker["points"]).astype(np.int32).reshape(-1, 1, 2)
        color = (0, 0, 255) if wrong else (0, 220, 0)

        cv2.polylines(overlay, [polygon], True, color, 2, cv2.LINE_AA)

        text_x = int(np.min(marker["points"][:, 0]))
        text_y = max(18, int(np.min(marker["points"][:, 1])) - 8)

        label = f"#{index} ID{marker_id} {short_category_name(category)}"
        if wrong:
            label += " WRONG"

        draw_text(overlay, label, (text_x, text_y), color, scale=0.48, thickness=1)
        summaries.append(f"{index}:ID{marker_id}:{category}")

    panel_height = 68
    cv2.rectangle(
        overlay,
        (0, 0),
        (overlay.shape[1], panel_height),
        (20, 20, 20),
        -1,
    )

    draw_text(overlay, f"expected={expected}", (10, 22), (255, 255, 255), 0.58, 1)
    draw_text(overlay, f"detected={len(markers)}", (10, 43), (255, 255, 255))
    draw_text(overlay, f"misplaced={misplaced}", (10, 63), (0, 180, 255))

    return overlay, misplaced, summaries


def update_stable_markers(stable_markers, corners, ids, hold_seconds):
    now = time.time()

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            points = marker_corners.reshape(4, 2)

            stable_markers[marker_id] = {
                "id": marker_id,
                "points": points.copy(),
                "center_x": float(np.mean(points[:, 0])),
                "center_y": float(np.mean(points[:, 1])),
                "last_seen": now,
            }

    expired_ids = [
        marker_id
        for marker_id, marker in stable_markers.items()
        if now - marker["last_seen"] > hold_seconds
    ]

    for marker_id in expired_ids:
        del stable_markers[marker_id]

    return list(stable_markers.values())


def main():
    args = parse_args()

    try:
        detect_markers = create_aruco_detector(args.dict)
    except (RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    expected = args.expected
    stable_markers = {}

    print(f"Connecting to {args.host}:{args.port} ...")

    try:
        with socket.create_connection((args.host, args.port), timeout=10) as sock:
            sock.settimeout(None)
            print(
                "Connected. Keys: 1=engineering 2=science 3=liberal "
                "0=unknown s=save q/ESC=quit"
            )

            while True:
                jpeg_size = struct.unpack("!I", recv_exact(sock, 4))[0]

                if jpeg_size == 0 or jpeg_size > 64 * 1024 * 1024:
                    raise RuntimeError(f"invalid JPEG frame size: {jpeg_size}")

                jpeg_data = recv_exact(sock, jpeg_size)
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )

                if frame is None:
                    print("Warning: failed to decode a JPEG frame", file=sys.stderr)
                    continue

                corners, ids, _ = detect_markers(frame)

                stable_list = update_stable_markers(
                    stable_markers,
                    corners,
                    ids,
                    args.hold_seconds,
                )

                overlay, misplaced, summaries = annotate_frame(
                    frame,
                    stable_list,
                    expected,
                )

                cv2.imshow("PC ArUco Board Stream", overlay)

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), 27):
                    break
                if key == ord("1"):
                    expected = "engineering"
                elif key == ord("2"):
                    expected = "science"
                elif key == ord("3"):
                    expected = "liberal"
                elif key == ord("0"):
                    expected = "unknown"
                elif key == ord("s"):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    output_path = save_dir / f"aruco_stream_{timestamp}.png"

                    if cv2.imwrite(str(output_path), overlay):
                        print(f"Saved {output_path}")
                    else:
                        print(
                            f"Warning: failed to save {output_path}",
                            file=sys.stderr,
                        )

                _ = misplaced, summaries

    except (ConnectionError, OSError, RuntimeError) as error:
        print(f"Stream error: {error}", file=sys.stderr)
        return 1
    finally:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
