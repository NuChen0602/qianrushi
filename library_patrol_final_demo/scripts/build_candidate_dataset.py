#!/usr/bin/env python3
"""Build a no-LabelImg crop dataset from controlled recording sessions."""

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "scripts"))

from lost_item_visual_api import box_iou, detect_candidate_boxes  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description="Auto-crop target/background candidates.")
    parser.add_argument(
        "--recordings-root",
        default=str(PROJECT_DIR / "dataset_recordings"),
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_DIR / "candidate_datasets"),
    )
    parser.add_argument(
        "--val-positive-index",
        type=int,
        default=1,
        help="sorted target session index reserved for validation",
    )
    parser.add_argument(
        "--val-background-index",
        type=int,
        default=-1,
        help="sorted background session index reserved for validation",
    )
    parser.add_argument("--padding", type=float, default=0.12)
    return parser.parse_args()


def frame_paths(session):
    return sorted((session / "frames").glob("frame_*.jpg"))


def box_center(box):
    return box[0] + box[2] / 2.0, box[1] + box[3] / 2.0


def initial_target_score(box, image_shape):
    height, width = image_shape[:2]
    center_x, center_y = box_center(box)
    anchor_x, anchor_y = width * 0.5, height * 0.48
    distance = math.hypot(center_x - anchor_x, center_y - anchor_y)
    diagonal = math.hypot(width, height)
    area_ratio = (box[2] * box[3]) / max(1.0, width * height)
    # Controlled positive sessions begin with the target near the image center.
    return 1.0 - distance / diagonal - max(0.0, area_ratio - 0.12) * 2.0


def temporal_target_score(previous_box, box, image_shape):
    height, width = image_shape[:2]
    previous_center = box_center(previous_box)
    center = box_center(box)
    distance = math.hypot(center[0] - previous_center[0], center[1] - previous_center[1])
    diagonal = math.hypot(width, height)
    overlap = box_iou(previous_box, box)
    previous_area = max(1.0, previous_box[2] * previous_box[3])
    area = max(1.0, box[2] * box[3])
    size_similarity = min(previous_area, area) / max(previous_area, area)
    return overlap * 0.50 + (1.0 - distance / diagonal) * 0.35 + size_similarity * 0.15


def select_target_box(boxes, previous_box, image_shape):
    if not boxes:
        return None, 0.0
    if previous_box is None:
        scored = [(initial_target_score(box, image_shape), box) for box in boxes]
    else:
        scored = [(temporal_target_score(previous_box, box, image_shape), box) for box in boxes]
    score, box = max(scored, key=lambda item: item[0])
    return box, float(score)


def expand_box(box, image_shape, padding):
    height, width = image_shape[:2]
    x, y, box_width, box_height = box
    pad_x = box_width * padding
    pad_y = box_height * padding
    x1 = max(0, int(math.floor(x - pad_x)))
    y1 = max(0, int(math.floor(y - pad_y)))
    x2 = min(width, int(math.ceil(x + box_width + pad_x)))
    y2 = min(height, int(math.ceil(y + box_height + pad_y)))
    return x1, y1, max(1, x2 - x1), max(1, y2 - y1)


def crop_image(image, box):
    x, y, width, height = [int(value) for value in box]
    return image[y:y + height, x:x + width]


def write_crop(output_dir, prefix, index, image):
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{prefix}_{index:05d}.jpg"
    if image.size == 0 or not cv2.imwrite(str(output_path), image, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise RuntimeError(f"failed to write {output_path}")
    return output_path


def choose_validation_session(sessions, index):
    if not sessions:
        return None
    resolved = index if index >= 0 else len(sessions) + index
    resolved = max(0, min(len(sessions) - 1, resolved))
    return sessions[resolved]


def build_contact_sheet(paths, output_path, title, columns=8, tile_size=128, limit=64):
    selected = list(paths)[:limit]
    if not selected:
        return
    rows = math.ceil(len(selected) / columns)
    sheet = np.full((rows * tile_size + 30, columns * tile_size, 3), 238, dtype=np.uint8)
    cv2.putText(sheet, title, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
    for index, path in enumerate(selected):
        image = cv2.imread(str(path))
        if image is None:
            continue
        height, width = image.shape[:2]
        scale = min((tile_size - 8) / max(1, width), (tile_size - 8) / max(1, height))
        resized = cv2.resize(
            image,
            (max(1, int(width * scale)), max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
        row, column = divmod(index, columns)
        x = column * tile_size + (tile_size - resized.shape[1]) // 2
        y = 30 + row * tile_size + (tile_size - resized.shape[0]) // 2
        sheet[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    cv2.imwrite(str(output_path), sheet)


def main():
    args = parse_args()
    recordings_root = Path(args.recordings_root).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()
    build_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = output_root / build_id
    output_dir.mkdir(parents=True, exist_ok=False)

    positive_sessions = sorted(
        session for session in (recordings_root / "target_bundle").glob("*")
        if frame_paths(session)
    )
    background_sessions = sorted(
        session for session in (recordings_root / "background").glob("*")
        if frame_paths(session)
    )
    if not positive_sessions or not background_sessions:
        raise SystemExit("target_bundle and background recording sessions are both required")

    val_positive = choose_validation_session(positive_sessions, args.val_positive_index)
    val_background = choose_validation_session(background_sessions, args.val_background_index)
    manifest_rows = []
    counters = {
        (split, label): 0
        for split in ("train", "val")
        for label in ("target_bundle", "background")
    }

    for session in positive_sessions:
        split = "val" if session == val_positive else "train"
        previous_box = None
        for source_path in frame_paths(session):
            image = cv2.imread(str(source_path))
            if image is None:
                continue
            boxes = detect_candidate_boxes(image)
            selected_box, track_score = select_target_box(boxes, previous_box, image.shape)
            if selected_box is None:
                manifest_rows.append({
                    "split": split,
                    "label": "target_bundle",
                    "source": str(source_path),
                    "crop": "",
                    "bbox": "",
                    "track_score": 0.0,
                    "status": "no_candidate",
                })
                previous_box = None
                continue
            previous_box = selected_box
            crop_box = expand_box(selected_box, image.shape, args.padding)
            crop = crop_image(image, crop_box)
            index = counters[(split, "target_bundle")]
            counters[(split, "target_bundle")] += 1
            output_path = write_crop(
                output_dir / split / "target_bundle",
                session.name,
                index,
                crop,
            )
            manifest_rows.append({
                "split": split,
                "label": "target_bundle",
                "source": str(source_path),
                "crop": str(output_path),
                "bbox": json.dumps(crop_box),
                "track_score": round(track_score, 4),
                "status": "ok",
            })

    for session in background_sessions:
        split = "val" if session == val_background else "train"
        for source_path in frame_paths(session):
            image = cv2.imread(str(source_path))
            if image is None:
                continue
            for candidate_index, box in enumerate(detect_candidate_boxes(image)):
                crop_box = expand_box(box, image.shape, args.padding)
                crop = crop_image(image, crop_box)
                index = counters[(split, "background")]
                counters[(split, "background")] += 1
                output_path = write_crop(
                    output_dir / split / "background",
                    f"{session.name}_{candidate_index}",
                    index,
                    crop,
                )
                manifest_rows.append({
                    "split": split,
                    "label": "background",
                    "source": str(source_path),
                    "crop": str(output_path),
                    "bbox": json.dumps(crop_box),
                    "track_score": "",
                    "status": "ok",
                })

    manifest_path = output_dir / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("split", "label", "source", "crop", "bbox", "track_score", "status"),
        )
        writer.writeheader()
        writer.writerows(manifest_rows)

    summary = {
        "build_id": build_id,
        "positive_sessions": [session.name for session in positive_sessions],
        "background_sessions": [session.name for session in background_sessions],
        "validation_positive_session": val_positive.name,
        "validation_background_session": val_background.name,
        "counts": {f"{split}/{label}": count for (split, label), count in counters.items()},
        "no_candidate_positive_frames": sum(
            row["status"] == "no_candidate" and row["label"] == "target_bundle"
            for row in manifest_rows
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    for split in ("train", "val"):
        for label in ("target_bundle", "background"):
            paths = sorted((output_dir / split / label).glob("*.jpg"))
            build_contact_sheet(
                paths,
                output_dir / f"contact_{split}_{label}.jpg",
                f"{split}/{label}",
            )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"dataset={output_dir}")


if __name__ == "__main__":
    main()
