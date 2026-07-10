#!/usr/bin/env python3
"""Purify candidate crops and train a tiny OpenCV HOG+SVM classifier."""

import argparse
import csv
import json
import math
import shutil
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
HOG = cv2.HOGDescriptor((96, 96), (16, 16), (8, 8), (8, 8), 9)


def parse_args():
    parser = argparse.ArgumentParser(description="Train target_bundle/background HOG+SVM.")
    parser.add_argument("--dataset", help="candidate dataset build; defaults to latest")
    parser.add_argument("--seed-positive-count", type=int, default=128)
    parser.add_argument("--min-seed-margin", type=float, default=0.0)
    return parser.parse_args()


def latest_dataset():
    builds = sorted(
        path for path in (PROJECT_DIR / "candidate_datasets").glob("20*")
        if (path / "manifest.csv").is_file() and (path / "summary.json").is_file()
    )
    if not builds:
        raise SystemExit("no candidate dataset found; run build_candidate_dataset.py first")
    return builds[-1]


def letterbox(image, size=96, content_size=88):
    height, width = image.shape[:2]
    scale = min(content_size / max(1, width), content_size / max(1, height))
    resized = cv2.resize(
        image,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
    )
    canvas = np.full((size, size, 3), 127, dtype=np.uint8)
    x = (size - resized.shape[1]) // 2
    y = (size - resized.shape[0]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return canvas


def feature(path):
    image = cv2.imread(str(path))
    if image is None:
        raise RuntimeError(f"cannot read {path}")
    return HOG.compute(letterbox(image)).reshape(-1)


def train_svm(positive_paths, negative_paths, c_value=0.5):
    features = np.float32([feature(path) for path in positive_paths + negative_paths])
    labels = np.int32([1] * len(positive_paths) + [0] * len(negative_paths))
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setC(float(c_value))
    if not svm.train(features, cv2.ml.ROW_SAMPLE, labels):
        raise RuntimeError("OpenCV SVM training failed")
    _, raw = svm.predict(features, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
    positive_mean = float(raw[:len(positive_paths)].mean())
    negative_mean = float(raw[len(positive_paths):].mean())
    raw_sign = 1.0 if positive_mean > negative_mean else -1.0
    return svm, raw_sign


def predict(svm, raw_sign, paths):
    if not paths:
        return np.empty((0,), np.int32), np.empty((0,), np.float32)
    features = np.float32([feature(path) for path in paths])
    _, labels = svm.predict(features)
    _, raw = svm.predict(features, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
    return labels.ravel().astype(np.int32), raw.ravel().astype(np.float32) * raw_sign


def session_from_source(source):
    return Path(source).parent.parent.name


def evenly_sample(paths, limit=80):
    paths = list(paths)
    if len(paths) <= limit:
        return paths
    indices = np.linspace(0, len(paths) - 1, limit).astype(int)
    return [paths[index] for index in indices]


def contact_sheet(paths, output_path, title, columns=8, tile=128):
    paths = evenly_sample(paths)
    if not paths:
        return
    rows = math.ceil(len(paths) / columns)
    sheet = np.full((rows * tile + 30, columns * tile, 3), 238, np.uint8)
    cv2.putText(sheet, title, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (25, 25, 25), 1, cv2.LINE_AA)
    for index, path in enumerate(paths):
        image = cv2.imread(str(path))
        if image is None:
            continue
        height, width = image.shape[:2]
        scale = min((tile - 8) / max(1, width), (tile - 8) / max(1, height))
        resized = cv2.resize(image, (max(1, int(width * scale)), max(1, int(height * scale))))
        row, column = divmod(index, columns)
        x = column * tile + (tile - resized.shape[1]) // 2
        y = 30 + row * tile + (tile - resized.shape[0]) // 2
        sheet[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    cv2.imwrite(str(output_path), sheet)


def metrics(expected, predicted):
    expected = np.asarray(expected, dtype=np.int32)
    predicted = np.asarray(predicted, dtype=np.int32)
    tp = int(np.sum((expected == 1) & (predicted == 1)))
    tn = int(np.sum((expected == 0) & (predicted == 0)))
    fp = int(np.sum((expected == 0) & (predicted == 1)))
    fn = int(np.sum((expected == 1) & (predicted == 0)))
    return {
        "tp": tp,
        "tn": tn,
        "fp": fp,
        "fn": fn,
        "accuracy": (tp + tn) / max(1, len(expected)),
        "precision": tp / max(1, tp + fp),
        "recall": tp / max(1, tp + fn),
    }


def main():
    args = parse_args()
    source_dataset = Path(args.dataset).resolve() if args.dataset else latest_dataset()
    summary = json.loads((source_dataset / "summary.json").read_text(encoding="utf-8"))
    with (source_dataset / "manifest.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    positive_sessions = summary["positive_sessions"]
    background_sessions = summary["background_sessions"]
    if len(positive_sessions) < 3 or len(background_sessions) < 2:
        raise SystemExit("at least 3 positive sessions and 2 background sessions are required")

    seed_positive_session = positive_sessions[1]
    validation_positive_session = positive_sessions[-1]
    seed_background_session = background_sessions[0]
    validation_background_session = background_sessions[-1]

    seed_positive = [
        Path(row["crop"])
        for row in rows
        if row["label"] == "target_bundle"
        and row["status"] == "ok"
        and session_from_source(row["source"]) == seed_positive_session
    ][:args.seed_positive_count]
    seed_negative = [
        Path(row["crop"])
        for row in rows
        if row["label"] == "background"
        and row["status"] == "ok"
        and session_from_source(row["source"]) == seed_background_session
    ]
    if len(seed_positive) < 32 or len(seed_negative) < 32:
        raise SystemExit("not enough trusted seed crops")

    seed_svm, seed_raw_sign = train_svm(seed_positive, seed_negative)
    positive_rows = [row for row in rows if row["label"] == "target_bundle" and row["status"] == "ok"]
    positive_paths = [Path(row["crop"]) for row in positive_rows]
    positive_labels, positive_margins = predict(seed_svm, seed_raw_sign, positive_paths)

    build_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_purified"
    purified_dir = PROJECT_DIR / "candidate_datasets" / build_id
    model_dir = PROJECT_DIR / "local_models"
    purified_dir.mkdir(parents=True, exist_ok=False)
    model_dir.mkdir(parents=True, exist_ok=True)

    copied = {(split, label): [] for split in ("train", "val") for label in ("target_bundle", "background")}
    rejected = []
    for row, predicted, margin in zip(positive_rows, positive_labels, positive_margins):
        session = session_from_source(row["source"])
        split = "val" if session == validation_positive_session else "train"
        source = Path(row["crop"])
        if predicted != 1 or float(margin) < args.min_seed_margin:
            rejected.append({"source": str(source), "session": session, "margin": float(margin)})
            continue
        destination = purified_dir / split / "target_bundle" / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied[(split, "target_bundle")].append(destination)

    for row in rows:
        if row["label"] != "background" or row["status"] != "ok":
            continue
        session = session_from_source(row["source"])
        split = "val" if session == validation_background_session else "train"
        source = Path(row["crop"])
        destination = purified_dir / split / "background" / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied[(split, "background")].append(destination)

    # Evaluate on held-out sessions first.
    train_positive = copied[("train", "target_bundle")]
    train_negative = copied[("train", "background")]
    evaluation_svm, evaluation_raw_sign = train_svm(train_positive, train_negative)
    validation_paths = copied[("val", "target_bundle")] + copied[("val", "background")]
    expected = [1] * len(copied[("val", "target_bundle")]) + [0] * len(copied[("val", "background")])
    predicted, margins = predict(evaluation_svm, evaluation_raw_sign, validation_paths)
    validation_metrics = metrics(expected, predicted)

    # After measuring held-out performance, retrain the deployable model on all
    # purified sessions so close/far examples from validation are not discarded.
    deploy_positive = train_positive + copied[("val", "target_bundle")]
    deploy_negative = train_negative + copied[("val", "background")]
    final_svm, final_raw_sign = train_svm(deploy_positive, deploy_negative)

    model_path = model_dir / "lost_item_hog_svm.xml"
    metadata_path = model_dir / "lost_item_hog_svm.json"
    final_svm.save(str(model_path))
    metadata = {
        "model_type": "opencv_hog_linear_svm",
        "input_size": [96, 96],
        "classes": {"0": "background", "1": "target_bundle"},
        "raw_sign": final_raw_sign,
        "decision_threshold": -0.30,
        "near_min_aspect": 2.2,
        "near_width_reference": 145.0,
        "near_threshold_slope": 0.006,
        "near_threshold_floor": -0.95,
        "source_dataset": str(source_dataset),
        "purified_dataset": str(purified_dir),
        "seed_positive_session": seed_positive_session,
        "seed_background_session": seed_background_session,
        "validation_positive_session": validation_positive_session,
        "validation_background_session": validation_background_session,
        "counts": {f"{split}/{label}": len(paths) for (split, label), paths in copied.items()},
        "deploy_counts": {
            "target_bundle": len(deploy_positive),
            "background": len(deploy_negative),
        },
        "seed_rejected_positive_crops": len(rejected),
        "validation": validation_metrics,
        "validation_margin": {
            "positive_mean": float(margins[:len(copied[("val", "target_bundle")])].mean()),
            "negative_mean": float(margins[len(copied[("val", "target_bundle")]):].mean()),
        },
    }
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    (purified_dir / "rejected_positive.json").write_text(
        json.dumps(rejected, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    for split in ("train", "val"):
        for label in ("target_bundle", "background"):
            contact_sheet(
                copied[(split, label)],
                purified_dir / f"contact_{split}_{label}.jpg",
                f"{split}/{label}",
            )

    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(f"model={model_path}")
    print(f"metadata={metadata_path}")


if __name__ == "__main__":
    main()
