#!/usr/bin/env python3
import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive board JPEG frames, detect ArUco book markers, and find books."
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
    parser.add_argument("--book-db", default="book_database.json")
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


def load_book_database(path):
    db_path = Path(path)
    if not db_path.exists():
        raise FileNotFoundError(f"book database not found: {db_path}")

    with db_path.open("r", encoding="utf-8") as f:
        raw_db = json.load(f)

    book_db = {}

    for marker_id_str, info in raw_db.items():
        marker_id = int(marker_id_str)

        book_db[marker_id] = {
            "id": marker_id,
            "title": info.get("title", f"ID{marker_id}"),
            "category": info.get("category", marker_category(marker_id)),
            "category_cn": info.get("category_cn", ""),
            "shelf_id": info.get("shelf_id", ""),
            "summary": info.get("summary", ""),
        }

    return book_db


def normalize_text(text):
    text = str(text).lower().strip()

    remove_chars = [
        "《", "》",
        "（", "）",
        "(", ")",
        " ", "\t", "\n",
        "-", "_",
        "，", ",",
        "。", ".",
        "：", ":",
        "；", ";",
        "、",
    ]

    for ch in remove_chars:
        text = text.replace(ch, "")

    text = text.replace("上册", "上")
    text = text.replace("下册", "下")

    return text


def find_book_by_query(query, book_db):
    q = normalize_text(query)

    if not q:
        return None

    # 1. 完全匹配
    for book in book_db.values():
        title_norm = normalize_text(book["title"])
        if q == title_norm:
            return book

    # 2. 包含匹配
    for book in book_db.values():
        title_norm = normalize_text(book["title"])
        if q in title_norm or title_norm in q:
            return book

    return None


def build_visible_books(stable_markers, book_db):
    markers = list(stable_markers)
    markers.sort(key=lambda item: item["center_x"])

    visible_books = []

    for index, marker in enumerate(markers, start=1):
        marker_id = int(marker["id"])
        book = book_db.get(marker_id)

        if book is None:
            book = {
                "id": marker_id,
                "title": f"未知书籍 ID{marker_id}",
                "category": marker_category(marker_id),
                "category_cn": "",
                "shelf_id": "",
                "summary": "",
            }

        visible_books.append(
            {
                "index": index,
                "marker_id": marker_id,
                "title": book["title"],
                "category": book["category"],
                "category_cn": book["category_cn"],
                "shelf_id": book["shelf_id"],
                "summary": book["summary"],
            }
        )

    return visible_books


def find_book_in_view(query, stable_markers, book_db):
    target_book = find_book_by_query(query, book_db)

    if target_book is None:
        return {
            "found_in_db": False,
            "found_in_view": False,
            "book": None,
            "index": None,
            "message": f"数据库中没有找到与“{query}”匹配的书籍。",
        }

    visible_books = build_visible_books(stable_markers, book_db)

    for visible_book in visible_books:
        if visible_book["marker_id"] == target_book["id"]:
            return {
                "found_in_db": True,
                "found_in_view": True,
                "book": target_book,
                "index": visible_book["index"],
                "message": (
                    f"找到了，《{target_book['title']}》"
                    f"在当前画面中从左往右数第 {visible_book['index']} 本。"
                ),
            }

    shelf_id = target_book.get("shelf_id", "")
    category_cn = target_book.get("category_cn", "")

    location_hint = ""
    if shelf_id or category_cn:
        location_hint = f"它登记在 {shelf_id} {category_cn}书架，"

    return {
        "found_in_db": True,
        "found_in_view": False,
        "book": target_book,
        "index": None,
        "message": (
            f"数据库中有《{target_book['title']}》，"
            f"{location_hint}但当前画面中没有识别到这本书。"
        ),
    }


def build_book_intro(book):
    if book is None:
        return "当前还没有选中的书。请先按 f 查找一本书。"

    summary = book.get("summary", "")
    if not summary:
        return f"《{book['title']}》暂时没有录入简介。"

    return summary


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


def annotate_frame(frame, markers, expected, book_db):
    overlay = frame.copy()

    markers = list(markers)
    markers.sort(key=lambda item: item["center_x"])

    misplaced = []
    summaries = []

    for index, marker in enumerate(markers, start=1):
        marker_id = int(marker["id"])
        book = book_db.get(marker_id)

        if book is not None:
            category = book.get("category", marker_category(marker_id))
            title = book.get("title", f"ID{marker_id}")
        else:
            category = marker_category(marker_id)
            title = f"未知书籍 ID{marker_id}"

        wrong = expected != "unknown" and category != expected

        if wrong:
            misplaced.append(index)

        polygon = np.rint(marker["points"]).astype(np.int32).reshape(-1, 1, 2)
        color = (0, 0, 255) if wrong else (0, 220, 0)

        cv2.polylines(overlay, [polygon], True, color, 2, cv2.LINE_AA)

        text_x = int(np.min(marker["points"][:, 0]))
        text_y = max(18, int(np.min(marker["points"][:, 1])) - 8)

        # OpenCV putText 不适合直接显示中文，所以画面上保留短标签；
        # 书名和简介在终端输出，后续接语音播报。
        label = f"#{index} ID{marker_id} {short_category_name(category)}"
        if wrong:
            label += " WRONG"

        draw_text(overlay, label, (text_x, text_y), color, scale=0.48, thickness=1)

        summaries.append(
            {
                "index": index,
                "id": marker_id,
                "title": title,
                "category": category,
                "wrong": wrong,
            }
        )

    panel_height = 88
    cv2.rectangle(
        overlay,
        (0, 0),
        (overlay.shape[1], panel_height),
        (20, 20, 20),
        -1,
    )

    draw_text(overlay, f"expected={expected}", (10, 22), (255, 255, 255), 0.58, 1)
    draw_text(overlay, f"detected={len(markers)}", (10, 43), (255, 255, 255))
    draw_text(overlay, f"misplaced={misplaced}", (10, 64), (0, 180, 255))
    draw_text(overlay, "keys: f=find  i=intro  v=visible  s=save  q=quit", (10, 84), (180, 220, 255), 0.45, 1)

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


def print_visible_books(stable_markers, book_db):
    visible_books = build_visible_books(stable_markers, book_db)

    if not visible_books:
        print("当前画面中没有稳定识别到书籍。")
        return

    print("\n当前画面识别到的书籍：")
    for book in visible_books:
        category_cn = book["category_cn"] or book["category"]
        print(
            f"  第 {book['index']} 本："
            f"ID{book['marker_id']}，"
            f"《{book['title']}》，"
            f"{category_cn}"
        )


def main():
    args = parse_args()

    try:
        detect_markers = create_aruco_detector(args.dict)
    except (RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2

    try:
        book_db = load_book_database(args.book_db)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error: failed to load book database: {error}", file=sys.stderr)
        return 2

    print(f"Loaded {len(book_db)} books from {args.book_db}")

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    expected = args.expected
    stable_markers = {}
    last_found_book = None

    print(f"Connecting to {args.host}:{args.port} ...")

    try:
        with socket.create_connection((args.host, args.port), timeout=10) as sock:
            sock.settimeout(None)
            print(
                "Connected.\n"
                "Keys:\n"
                "  1=engineering  2=science  3=liberal  0=unknown\n"
                "  f=find book     i=introduce last found book\n"
                "  v=print visible books\n"
                "  s=save image    q/ESC=quit"
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
                    book_db,
                )

                cv2.imshow("PC Book Finder Board Stream", overlay)

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), 27):
                    break

                if key == ord("1"):
                    expected = "engineering"
                    print("expected = engineering / 工科技术类")
                elif key == ord("2"):
                    expected = "science"
                    print("expected = science / 理学科普类")
                elif key == ord("3"):
                    expected = "liberal"
                    print("expected = liberal / 文学历史类")
                elif key == ord("0"):
                    expected = "unknown"
                    print("expected = unknown")

                elif key == ord("f"):
                    query = input("\n请输入要查找的书名关键词：").strip()
                    result = find_book_in_view(query, stable_list, book_db)
                    print(result["message"])

                    if result.get("book") is not None:
                        last_found_book = result["book"]

                elif key == ord("i"):
                    intro = build_book_intro(last_found_book)
                    print(intro)

                elif key == ord("v"):
                    print_visible_books(stable_list, book_db)

                elif key == ord("s"):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    output_path = save_dir / f"book_finder_{timestamp}.png"

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
