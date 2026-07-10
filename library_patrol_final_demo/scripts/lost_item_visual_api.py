#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Low-latency lost-item vision service.

OpenCV keeps candidate boxes and track IDs current at camera speed. Qwen-VL runs
asynchronously on a cropped candidate and only adds semantics to an existing
track; it never owns the live bounding-box coordinates.
"""

import base64
import json
import os
import queue
import threading
import time
import urllib.request
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ENV_PATH = ROOT / ".env"

WEB_STATE_URL = os.getenv("LOST_ITEM_STATE_URL", "http://127.0.0.1:8090/api/demo/state")
CAMERA_URL = os.getenv("LOST_ITEM_CAMERA_URL", "http://127.0.0.1:8090/camera.jpg")


def load_env():
    if not ENV_PATH.exists():
        return
    for line in ENV_PATH.read_text(errors="ignore").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        if key and value and key not in os.environ:
            os.environ[key] = value


def fetch_json(url, timeout=0.5):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8", "ignore"))
    except Exception:
        return {}


def fetch_camera_jpeg():
    request = urllib.request.Request(CAMERA_URL, headers={"Cache-Control": "no-cache"})
    with urllib.request.urlopen(request, timeout=1.5) as response:
        content_type = str(response.headers.get("Content-Type", ""))
        data = response.read()
        if "image/jpeg" not in content_type:
            raise RuntimeError(f"camera frame is not JPEG: {content_type}")
        frame_id = response.headers.get("X-Frame-ID")
        frame_timestamp = response.headers.get("X-Frame-Timestamp")

    if not frame_id:
        frame_id = str(zlib.crc32(data) & 0xFFFFFFFF)
    try:
        frame_timestamp = float(frame_timestamp)
    except (TypeError, ValueError):
        frame_timestamp = time.time()
    return data, str(frame_id), frame_timestamp


def jpg_to_bgr(data):
    arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def resize_model_crop(img, min_side=256, max_side=384):
    height, width = img.shape[:2]
    longest = max(height, width)
    if longest <= 0:
        raise RuntimeError("empty candidate crop")
    target = min(max_side, max(min_side, longest))
    scale = target / longest
    if abs(scale - 1.0) < 0.02:
        return img
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(
        img,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=interpolation,
    )


def bgr_to_data_url(img):
    ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 84])
    if not ok:
        raise RuntimeError("failed to encode candidate crop")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return "data:image/jpeg;base64," + payload


def state_is_lost_task(state):
    orchestrator = state.get("orchestrator") if isinstance(state, dict) else {}
    work_order = state.get("work_order") if isinstance(state, dict) else {}
    orchestrator = orchestrator if isinstance(orchestrator, dict) else {}
    work_order = work_order if isinstance(work_order, dict) else {}

    active_id = str(orchestrator.get("active") or "")
    active_state = str(orchestrator.get("state") or "")
    task_id = str(work_order.get("current_task") or "")
    title = str(work_order.get("current_title") or "")
    stage = str(work_order.get("stage") or "")
    is_lost = (
        active_id == "LOST_ITEM_PATROL"
        or task_id == "LOST_ITEM_PATROL"
        or "遗失物巡检" in title
        or "LOST" in stage.upper()
    )
    # Keep scanning after the patrol route completes so the operator can still
    # inspect the target and its live box. A new mission changes current_task
    # and naturally disables this detector.
    return is_lost


def box_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    union = aw * ah + bw * bh - intersection
    return intersection / union if union else 0.0


def merge_boxes(boxes):
    merged = []
    for box in boxes:
        for index, current in enumerate(merged):
            if box_iou(box, current) > 0.12:
                x1 = min(box[0], current[0])
                y1 = min(box[1], current[1])
                x2 = max(box[0] + box[2], current[0] + current[2])
                y2 = max(box[1] + box[3], current[1] + current[3])
                merged[index] = (x1, y1, x2 - x1, y2 - y1)
                break
        else:
            merged.append(box)
    return merged


def detect_candidate_boxes(img, max_candidates=5):
    """Find small foreground-like regions in the lower part of the image."""
    height, width = img.shape[:2]
    y0 = int(height * 0.30)
    roi = img[y0:height, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 35, 110)
    saturation_mask = cv2.inRange(hsv[:, :, 1], 32, 255)
    background = cv2.GaussianBlur(gray, (35, 35), 0)
    difference = cv2.absdiff(gray, background)
    _, difference_mask = cv2.threshold(difference, 16, 255, cv2.THRESH_BINARY)

    mask = cv2.bitwise_or(edges, saturation_mask)
    mask = cv2.bitwise_or(mask, difference_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    image_area = width * height
    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        area = box_width * box_height
        if area < 300 or area > image_area * 0.55:
            continue
        if box_width < 16 or box_height < 12:
            continue
        if box_width > int(width * 0.95) or box_height > int(height * 0.75):
            continue

        # A fixed 18 px border worked for far targets but made a close target
        # occupy nearly the entire classifier crop. Scale the context border
        # with the candidate while keeping the old minimum for small objects.
        padding = max(18, int(round(max(box_width, box_height) * 0.08)))
        x1 = max(0, x - padding)
        y1 = max(0, y + y0 - padding)
        x2 = min(width, x + box_width + padding)
        y2 = min(height, y + y0 + box_height + padding)
        boxes.append((x1, y1, x2 - x1, y2 - y1))

    boxes = merge_boxes(boxes)
    boxes.sort(key=lambda box: (-(box[2] * box[3]), -box[1]))
    return boxes[:max_candidates]


def extract_json(text):
    text = str(text or "").strip()
    if text.startswith("```"):
        text = text.replace("```json", "").replace("```", "").strip()
    start = text.find("{")
    end = text.rfind("}")
    if start >= 0 and end >= start:
        text = text[start:end + 1]
    return json.loads(text)


def qwen_classify_crop(client, model, crop, max_tokens=80):
    crop = resize_model_crop(crop, min_side=256, max_side=384)
    prompt = """
你是图书馆巡检机器人的遗失物确认模块。图中是本地视觉截取的一个候选区域。
判断主体是否为可能遗失的钥匙、校园卡、手机、钱包、眼镜、耳机、U盘或其他随身小物。
只返回紧凑 JSON，不要 Markdown：
{"has_lost_item":true,"type":"keychain/campus_card/phone/wallet/glasses/earphones/u_disk/other","confidence":0.0,"description":"简短中文描述","speech":"一句中文提醒"}
不是遗失物时返回：
{"has_lost_item":false,"type":"none","confidence":0.0,"description":"非遗失物","speech":""}
""".strip()
    completion = client.chat.completions.create(
        model=model,
        messages=[{
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": bgr_to_data_url(crop)}},
                {"type": "text", "text": prompt},
            ],
        }],
        temperature=0,
        max_tokens=max_tokens,
    )
    return extract_json(completion.choices[0].message.content)


def normalize_qwen_result(payload):
    payload = payload if isinstance(payload, dict) else {}
    items = payload.get("items") if isinstance(payload.get("items"), list) else []
    item = items[0] if items and isinstance(items[0], dict) else payload
    has_lost_item = bool(payload.get("has_lost_item"))
    item_type = str(item.get("type") or "other")
    try:
        confidence = float(item.get("confidence") or 0.0)
    except (TypeError, ValueError):
        confidence = 0.0
    description = str(item.get("description") or payload.get("description") or "发现疑似遗失物")
    speech = str(payload.get("speech") or description)
    return {
        "has_lost_item": has_lost_item,
        "type": item_type,
        "confidence": max(0.0, min(1.0, confidence)),
        "description": description,
        "speech": speech,
    }


def label_cn(item_type):
    return {
        "keychain": "钥匙",
        "campus_card": "校园卡",
        "phone": "手机",
        "wallet": "钱包",
        "glasses": "眼镜",
        "earphones": "耳机",
        "u_disk": "U盘",
        "other": "遗失物",
    }.get(item_type, "遗失物")


def crop_box(image, box):
    height, width = image.shape[:2]
    x, y, box_width, box_height = [int(value) for value in box]
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(width, x + box_width)
    y2 = min(height, y + box_height)
    if x2 <= x1 or y2 <= y1:
        return None
    return image[y1:y2, x1:x2].copy()


class LocalBundleClassifier:
    """Tiny OpenCV-only HOG+SVM classifier for the fixed target bundle."""

    def __init__(self, model_path, metadata_path):
        self.model_path = Path(model_path)
        self.metadata_path = Path(metadata_path)
        self.svm = cv2.ml.SVM_load(str(self.model_path))
        self.metadata = json.loads(self.metadata_path.read_text(encoding="utf-8"))
        self.raw_sign = float(self.metadata.get("raw_sign", -1.0))
        self.decision_threshold = float(os.getenv(
            "LOST_ITEM_LOCAL_THRESHOLD",
            str(self.metadata.get("decision_threshold", -0.30)),
        ))
        self.near_min_aspect = float(self.metadata.get("near_min_aspect", 2.2))
        self.near_width_reference = float(self.metadata.get("near_width_reference", 145.0))
        self.near_threshold_slope = float(self.metadata.get("near_threshold_slope", 0.006))
        self.near_threshold_floor = float(self.metadata.get("near_threshold_floor", -0.95))
        self.hog = cv2.HOGDescriptor((96, 96), (16, 16), (8, 8), (8, 8), 9)

    @staticmethod
    def _letterbox(image, size=96, content_size=88):
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

    def threshold_for_crop(self, crop):
        height, width = crop.shape[:2]
        aspect = width / max(1.0, float(height))
        if aspect < self.near_min_aspect:
            return self.decision_threshold
        size_adjustment = self.near_threshold_slope * max(
            0.0,
            float(width) - self.near_width_reference,
        )
        return max(self.near_threshold_floor, self.decision_threshold - size_adjustment)

    def predict(self, crop):
        feature = self.hog.compute(self._letterbox(crop)).reshape(1, -1).astype(np.float32)
        _, raw = self.svm.predict(feature, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
        margin = float(raw[0, 0]) * self.raw_sign
        return int(margin >= self.threshold_for_crop(crop)), margin


class LostItemVisionRuntime:
    def __init__(self):
        load_env()
        self.poll_hz = max(1.0, float(os.getenv("LOST_ITEM_LOCAL_FPS", "10")))
        self.track_ttl_sec = max(0.5, float(os.getenv("LOST_ITEM_TRACK_TTL_SEC", "1.5")))
        self.max_missed_frames = max(1, int(os.getenv("LOST_ITEM_MAX_MISSED_FRAMES", "4")))
        self.model = os.getenv("QWEN_VL_MODEL", "qwen3-vl-flash")
        self.max_tokens = max(32, int(os.getenv("QWEN_VL_MAX_TOKENS", "80")))
        self.api_key = os.getenv("DASHSCOPE_API_KEY", "")
        self.base_url = os.getenv("DASHSCOPE_BASE_URL", "")
        local_model_dir = ROOT / "local_models"
        local_model_path = Path(os.getenv(
            "LOST_ITEM_LOCAL_MODEL",
            str(local_model_dir / "lost_item_hog_svm.xml"),
        ))
        local_metadata_path = Path(os.getenv(
            "LOST_ITEM_LOCAL_MODEL_METADATA",
            str(local_model_dir / "lost_item_hog_svm.json"),
        ))
        self.local_classifier = None
        self.local_model_error = ""
        try:
            if local_model_path.is_file() and local_metadata_path.is_file():
                self.local_classifier = LocalBundleClassifier(local_model_path, local_metadata_path)
        except Exception as exc:
            self.local_model_error = str(exc)

        self.lock = threading.RLock()
        self.running = False
        self.active = False
        self.force_until = 0.0
        self.tracks = {}
        self.next_track_id = 1
        self.last_frame_id = None
        self.last_frame_timestamp = 0.0
        self.image_size = {"width": 0, "height": 0}
        self.last_error = ""
        self.last_qwen_error = ""
        self.last_qwen_latency_ms = None
        self.qwen_inflight = None
        self.qwen_queue = queue.Queue(maxsize=1)
        self.capture_thread = None
        self.qwen_thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.qwen_thread = threading.Thread(target=self._qwen_loop, daemon=True)
        self.capture_thread.start()
        self.qwen_thread.start()

    def stop(self):
        self.running = False

    def force(self, seconds=15.0):
        with self.lock:
            self.force_until = max(self.force_until, time.time() + seconds)
            self.active = True

    def reset(self):
        with self.lock:
            self.tracks.clear()
            self.last_frame_id = None
            self.last_error = ""
            self.last_qwen_error = ""
        self._drain_qwen_queue()

    def _drain_qwen_queue(self):
        while True:
            try:
                self.qwen_queue.get_nowait()
                self.qwen_queue.task_done()
            except queue.Empty:
                return

    def _capture_loop(self):
        period = 1.0 / self.poll_hz
        last_state_check = 0.0
        while self.running:
            started_at = time.monotonic()
            now = time.time()
            if now - last_state_check >= 0.5:
                state = fetch_json(WEB_STATE_URL)
                with self.lock:
                    self.active = state_is_lost_task(state) or now < self.force_until
                last_state_check = now

            with self.lock:
                active = self.active
            if not active:
                with self.lock:
                    self.tracks.clear()
                    self.last_error = ""
                self._drain_qwen_queue()
                self._sleep_remaining(started_at, period)
                continue

            try:
                jpeg, frame_id, frame_timestamp = fetch_camera_jpeg()
                if frame_id == self.last_frame_id:
                    self._expire_tracks()
                    self._sleep_remaining(started_at, period)
                    continue
                image = jpg_to_bgr(jpeg)
                if image is None:
                    raise RuntimeError("failed to decode camera JPEG")

                boxes = detect_candidate_boxes(image)
                self._update_tracks(boxes, frame_id, frame_timestamp, image)
                with self.lock:
                    self.last_frame_id = frame_id
                    self.last_frame_timestamp = frame_timestamp
                    self.image_size = {"width": int(image.shape[1]), "height": int(image.shape[0])}
                    self.last_error = ""
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                self._expire_tracks()

            self._sleep_remaining(started_at, period)

    @staticmethod
    def _sleep_remaining(started_at, period):
        remaining = period - (time.monotonic() - started_at)
        if remaining > 0:
            time.sleep(remaining)

    def _expire_tracks(self):
        now = time.monotonic()
        with self.lock:
            stale = [
                track_id for track_id, track in self.tracks.items()
                if now - track["last_seen"] > self.track_ttl_sec
            ]
            for track_id in stale:
                self.tracks.pop(track_id, None)

    @staticmethod
    def _match_score(old_box, new_box):
        overlap = box_iou(old_box, new_box)
        old_cx = old_box[0] + old_box[2] / 2.0
        old_cy = old_box[1] + old_box[3] / 2.0
        new_cx = new_box[0] + new_box[2] / 2.0
        new_cy = new_box[1] + new_box[3] / 2.0
        distance = ((old_cx - new_cx) ** 2 + (old_cy - new_cy) ** 2) ** 0.5
        scale = max(old_box[2], old_box[3], new_box[2], new_box[3], 1)
        proximity = max(0.0, 1.0 - distance / (scale * 1.5))
        if overlap < 0.05 and proximity < 0.45:
            return -1.0
        return overlap * 0.75 + proximity * 0.25

    def _update_tracks(self, boxes, frame_id, frame_timestamp, image):
        now_mono = time.monotonic()
        now_wall = time.time()
        matched_tracks = set()
        observations = []
        for box in boxes:
            local_label = None
            local_margin = None
            if self.local_classifier is not None:
                candidate_crop = crop_box(image, box)
                if candidate_crop is not None and candidate_crop.size:
                    try:
                        local_threshold = self.local_classifier.threshold_for_crop(candidate_crop)
                        local_label, local_margin = self.local_classifier.predict(candidate_crop)
                    except Exception as exc:
                        self.local_model_error = str(exc)
                        local_threshold = None
                else:
                    local_threshold = None
            else:
                local_threshold = None
            observations.append({
                "box": box,
                "local_label": local_label,
                "local_margin": local_margin,
                "local_threshold": local_threshold,
            })
        local_predictions = []

        with self.lock:
            for observation in observations:
                box = observation["box"]
                best_track_id = None
                best_score = -1.0
                for track_id, track in self.tracks.items():
                    if track_id in matched_tracks:
                        continue
                    # Do not merge a local target candidate into a known
                    # background track (or vice versa) merely because it moved
                    # near that box between two frames.
                    if (
                        observation["local_label"] is not None
                        and track.get("local_label") is not None
                        and int(observation["local_label"]) != int(track["local_label"])
                    ):
                        continue
                    score = self._match_score(track["bbox"], box)
                    if score > best_score:
                        best_score = score
                        best_track_id = track_id

                if best_track_id is None or best_score < 0:
                    best_track_id = f"lost-{self.next_track_id}"
                    self.next_track_id += 1
                    self.tracks[best_track_id] = {
                        "track_id": best_track_id,
                        "bbox": tuple(float(value) for value in box),
                        "created_at": now_wall,
                        "last_seen": now_mono,
                        "last_seen_wall": now_wall,
                        "missed_frames": 0,
                        "frame_id": frame_id,
                        "frame_timestamp": frame_timestamp,
                        "qwen_state": "waiting",
                        "local_positive_streak": 0,
                        "local_negative_streak": 0,
                        "local_label": None,
                        "local_margin": None,
                        "retry_after": 0.0,
                        "label": "疑似遗失物",
                        "score": 0.0,
                        "message": "等待 Qwen-VL 确认",
                    }
                else:
                    track = self.tracks[best_track_id]
                    old_box = track["bbox"]
                    # Heavier weight on the newest local detection keeps the box responsive.
                    track["bbox"] = tuple(
                        old_value * 0.25 + float(new_value) * 0.75
                        for old_value, new_value in zip(old_box, box)
                    )
                    track["last_seen"] = now_mono
                    track["last_seen_wall"] = now_wall
                    track["missed_frames"] = 0
                    track["frame_id"] = frame_id
                    track["frame_timestamp"] = frame_timestamp
                matched_tracks.add(best_track_id)
                if observation["local_label"] is not None:
                    local_predictions.append((
                        best_track_id,
                        observation["local_label"],
                        observation["local_margin"],
                        observation["local_threshold"],
                    ))

            for track_id, track in self.tracks.items():
                if track_id not in matched_tracks:
                    track["missed_frames"] = track.get("missed_frames", 0) + 1

            stale = [
                track_id for track_id, track in self.tracks.items()
                if now_mono - track["last_seen"] > self.track_ttl_sec
                or track.get("missed_frames", 0) >= self.max_missed_frames
            ]
            for track_id in stale:
                self.tracks.pop(track_id, None)

        for track_id, local_label, local_margin, local_threshold in local_predictions:
            self._apply_local_prediction(track_id, local_label, local_margin, local_threshold)

        with self.lock:
            candidates = [
                track for track in self.tracks.values()
                if track["qwen_state"] in {"waiting", "queued", "error"}
                and now_wall >= track.get("retry_after", 0.0)
                and track["track_id"] in matched_tracks
            ]
            candidates.sort(key=lambda track: track["created_at"])
            # Qwen is now a fallback. A loaded local model owns the real-time decision.
            candidate = dict(candidates[0]) if candidates and self.local_classifier is None else None

        if candidate:
            crop = crop_box(image, candidate["bbox"])
            if crop is not None and crop.size:
                self._replace_queued_task({
                    "track_id": candidate["track_id"],
                    "frame_id": frame_id,
                    "frame_timestamp": frame_timestamp,
                    "crop": crop,
                    "submitted_at": time.time(),
                })

    def _apply_local_prediction(self, track_id, label, margin, threshold=None):
        with self.lock:
            track = self.tracks.get(track_id)
            if not track:
                return
            track["local_label"] = int(label)
            track["local_margin"] = round(float(margin), 4)
            track["local_threshold"] = (
                round(float(threshold), 4) if threshold is not None else None
            )
            if int(label) == 1:
                track["local_positive_streak"] = track.get("local_positive_streak", 0) + 1
                track["local_negative_streak"] = 0
                if track["local_positive_streak"] >= 3:
                    track["qwen_state"] = "confirmed"
                    track["confirmation_source"] = "local_hog_svm"
                    track["label"] = "钥匙与校园卡组合"
                    track["score"] = 0.0
                    track["message"] = "本地模型连续3帧确认目标钥匙串"
                elif track.get("qwen_state") == "local_rejected":
                    track["qwen_state"] = "waiting"
                    track["label"] = "疑似目标钥匙串"
                    track["message"] = "本地模型正在连续帧确认"
            else:
                track["local_negative_streak"] = track.get("local_negative_streak", 0) + 1
                track["local_positive_streak"] = 0
                if track["local_negative_streak"] >= 3:
                    track["qwen_state"] = "local_rejected"
                    track["confirmation_source"] = "local_hog_svm"
                    track["label"] = "背景"
                    track["score"] = 0.0
                    track["message"] = "本地模型判定为背景候选"

    def _replace_queued_task(self, task):
        try:
            dropped = self.qwen_queue.get_nowait()
            self.qwen_queue.task_done()
            with self.lock:
                dropped_track = self.tracks.get(dropped["track_id"])
                if dropped_track and dropped_track.get("qwen_state") == "queued":
                    dropped_track["qwen_state"] = "waiting"
        except queue.Empty:
            pass

        with self.lock:
            track = self.tracks.get(task["track_id"])
            if not track or track.get("qwen_state") in {"analyzing", "confirmed", "rejected"}:
                return
            track["qwen_state"] = "queued"
            track["queued_frame_id"] = task["frame_id"]
        try:
            self.qwen_queue.put_nowait(task)
        except queue.Full:
            with self.lock:
                track = self.tracks.get(task["track_id"])
                if track and track.get("qwen_state") == "queued":
                    track["qwen_state"] = "waiting"

    def _qwen_loop(self):
        client = None
        while self.running:
            try:
                task = self.qwen_queue.get(timeout=0.25)
            except queue.Empty:
                continue

            track_id = task["track_id"]
            with self.lock:
                track = self.tracks.get(track_id)
                if not track:
                    self.qwen_queue.task_done()
                    continue
                track["qwen_state"] = "analyzing"
                self.qwen_inflight = {
                    "track_id": track_id,
                    "frame_id": task["frame_id"],
                    "started_at": time.time(),
                }

            started_at = time.monotonic()
            try:
                if not self.api_key or not self.base_url:
                    raise RuntimeError("DASHSCOPE_API_KEY 或 DASHSCOPE_BASE_URL 未配置")
                if client is None:
                    from openai import OpenAI
                    client = OpenAI(api_key=self.api_key, base_url=self.base_url)
                raw_result = qwen_classify_crop(
                    client,
                    self.model,
                    task["crop"],
                    max_tokens=self.max_tokens,
                )
                result = normalize_qwen_result(raw_result)
                latency_ms = int((time.monotonic() - started_at) * 1000)

                with self.lock:
                    track = self.tracks.get(track_id)
                    if track:
                        if result["has_lost_item"]:
                            track["qwen_state"] = "confirmed"
                            track["label"] = label_cn(result["type"])
                            track["score"] = result["confidence"]
                            track["message"] = result["description"]
                            track["speech"] = result["speech"]
                            track["semantic"] = result
                        else:
                            track["qwen_state"] = "rejected"
                            track["label"] = "非遗失物"
                            track["score"] = result["confidence"]
                            track["message"] = result["description"]
                    self.last_qwen_latency_ms = latency_ms
                    self.last_qwen_error = ""
            except Exception as exc:
                with self.lock:
                    track = self.tracks.get(track_id)
                    if track:
                        track["qwen_state"] = "error"
                        track["retry_after"] = time.time() + 5.0
                        track["message"] = "Qwen-VL 暂不可用，本地候选框继续跟踪"
                    self.last_qwen_error = str(exc)
            finally:
                with self.lock:
                    self.qwen_inflight = None
                self.qwen_queue.task_done()

    def snapshot(self):
        self._expire_tracks()
        now = time.time()
        with self.lock:
            detections = []
            for track in self.tracks.values():
                if track["qwen_state"] in {"rejected", "local_rejected"}:
                    continue
                if (
                    self.local_classifier is not None
                    and track.get("local_label") == 0
                    and track.get("qwen_state") != "confirmed"
                ):
                    continue
                age_ms = max(0, int((now - track["last_seen_wall"]) * 1000))
                if age_ms > int(self.track_ttl_sec * 1000):
                    continue
                x, y, width, height = track["bbox"]
                confirmed = track["qwen_state"] == "confirmed"
                detections.append({
                    "track_id": track["track_id"],
                    "frame_id": track["frame_id"],
                    "frame_timestamp": track["frame_timestamp"],
                    "age_ms": age_ms,
                    "state": "confirmed" if confirmed else "candidate",
                    "color": "red" if confirmed else "yellow",
                    "qwen_state": track["qwen_state"],
                    "confirmation_source": track.get("confirmation_source"),
                    "local_label": track.get("local_label"),
                    "local_margin": track.get("local_margin"),
                    "local_threshold": track.get("local_threshold"),
                    "label": track["label"],
                    "score": round(float(track.get("score", 0.0)), 3),
                    "message": track["message"],
                    "bbox": {
                        "x": int(round(x)),
                        "y": int(round(y)),
                        "w": int(round(width)),
                        "h": int(round(height)),
                    },
                })

            # This deployment has exactly one physical key/card bundle. Keep
            # multiple internal tracks for robust association, but never draw
            # duplicate boxes for split contours of the same target.
            detections.sort(key=lambda item: (
                item["state"] == "confirmed",
                float(item.get("local_margin") or -999.0),
                -int(item.get("age_ms") or 0),
            ), reverse=True)
            detections = detections[:1]
            confirmed_count = sum(item["state"] == "confirmed" for item in detections)
            if not self.active:
                status = "idle"
            elif confirmed_count:
                status = "detected"
            elif self.qwen_inflight or not self.qwen_queue.empty():
                status = "analyzing"
            elif detections:
                status = "tracking"
            elif self.last_error:
                status = "camera_error"
            else:
                status = "scanning"

            return {
                "ok": True,
                "available": True,
                "active": self.active,
                "status": status,
                "mode": (
                    "local_tracking_hog_svm"
                    if self.local_classifier is not None
                    else "local_tracking_async_crop_qwen_vl"
                ),
                "model": self.model,
                "frame_id": self.last_frame_id,
                "frame_timestamp": self.last_frame_timestamp,
                "image_size": dict(self.image_size),
                "track_ttl_sec": self.track_ttl_sec,
                "max_missed_frames": self.max_missed_frames,
                "candidate_count": len(detections),
                "confirmed_count": confirmed_count,
                "detections": detections,
                "qwen": {
                    "inflight": dict(self.qwen_inflight) if self.qwen_inflight else None,
                    "queue_depth": self.qwen_queue.qsize(),
                    "last_latency_ms": self.last_qwen_latency_ms,
                    "last_error": self.last_qwen_error,
                    "max_tokens": self.max_tokens,
                    "crop_max_side": 384,
                },
                "local_model": {
                    "available": self.local_classifier is not None,
                    "type": "opencv_hog_linear_svm",
                    "confirm_frames": 3,
                    "decision_threshold": (
                        self.local_classifier.decision_threshold
                        if self.local_classifier is not None
                        else None
                    ),
                    "near_threshold_floor": (
                        self.local_classifier.near_threshold_floor
                        if self.local_classifier is not None
                        else None
                    ),
                    "last_error": self.local_model_error,
                },
                "error": self.last_error,
            }


class Handler(BaseHTTPRequestHandler):
    @property
    def runtime(self):
        return self.server.runtime

    def _send_json(self, payload, code=200):
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def do_OPTIONS(self):
        self._send_json({"ok": True})

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path != "/api/vision/lost_items":
            self._send_json({"ok": False, "error": "not found"}, 404)
            return

        query = parse_qs(parsed.query)
        if query.get("force", ["0"])[0] == "1":
            self.runtime.force()
        if query.get("refresh", ["0"])[0] == "1":
            self.runtime.reset()
        self._send_json(self.runtime.snapshot())

    def log_message(self, fmt, *args):
        return


class VisionHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, runtime):
        super().__init__(address, Handler)
        self.runtime = runtime


def main():
    runtime = LostItemVisionRuntime()
    runtime.start()
    server = VisionHttpServer(("127.0.0.1", 8091), runtime)
    print("[lost-item-vision] http://127.0.0.1:8091/api/vision/lost_items")
    print(
        "[lost-item-vision] local_fps=%.1f track_ttl=%.1fs model=%s crop<=384 max_tokens=%d"
        % (runtime.poll_hz, runtime.track_ttl_sec, runtime.model, runtime.max_tokens)
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()
        server.server_close()


if __name__ == "__main__":
    main()
