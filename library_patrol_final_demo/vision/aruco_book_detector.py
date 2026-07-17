import json
from pathlib import Path
from typing import Dict, Any, Optional

from app.utils import load_config

_BOOKS = load_config(Path(__file__).resolve().parents[1] / "config", "books").get("books", [])
BOOK_ID_TO_NAME = {int(book["id"]): book["name"] for book in _BOOKS}
BOOK_ID_TO_SHELF = {int(book["id"]): str(book.get("shelf_id", "")) for book in _BOOKS}
SHELF_ID_TO_NAME = {
    str(shelf_id): str(shelf.get("name", shelf_id))
    for shelf_id, shelf in load_config(
        Path(__file__).resolve().parents[1] / "config", "books").get(
            "shelves", {}).items()
}
SHELF_POINT_TO_ID = {
    str(book.get("shelf_point", "")): str(book.get("shelf_id", ""))
    for book in _BOOKS
    if book.get("shelf_point") and book.get("shelf_id")
}
SHELF_BOOK_IDS = {}
for _book in _BOOKS:
    SHELF_BOOK_IDS.setdefault(str(_book.get("shelf_id", "")), set()).add(int(_book["id"]))


def shelf_id_for_point(point_id: str) -> Optional[str]:
    """Return the configured shelf represented by a navigation point."""
    return SHELF_POINT_TO_ID.get(str(point_id))


def misplaced_books_for_shelf(books, observed_shelf_id: str):
    """Return detected books whose configured shelf differs from this shelf.

    Marker IDs unknown to the formal 15-book catalogue are ignored: they
    cannot support a reliable misplaced-book conclusion.
    """
    observed_shelf_id = str(observed_shelf_id)
    misplaced = []
    seen = set()
    for book in books or ():
        try:
            book_id = int(book.get("id"))
        except (AttributeError, TypeError, ValueError):
            continue
        expected_shelf_id = BOOK_ID_TO_SHELF.get(book_id)
        if (not expected_shelf_id or expected_shelf_id == observed_shelf_id or
                book_id in seen):
            continue
        seen.add(book_id)
        misplaced.append({
            "id": book_id,
            "name": str(book.get("name") or BOOK_ID_TO_NAME[book_id]),
            "observed_shelf_id": observed_shelf_id,
            "observed_shelf_name": SHELF_ID_TO_NAME.get(
                observed_shelf_id, observed_shelf_id),
            "expected_shelf_id": expected_shelf_id,
            "expected_shelf_name": SHELF_ID_TO_NAME.get(
                expected_shelf_id, expected_shelf_id),
        })
    return sorted(misplaced, key=lambda item: item["id"])


def misplaced_books_message(misplaced) -> str:
    """Build a spoken report from confirmed visual misplaced-book results."""
    if not misplaced:
        return "视觉检测未发现错放图书。"
    if len(misplaced) == 1:
        book = misplaced[0]
        return (
            f"视觉检测发现《{book['name']}》疑似错放在"
            f"{book['observed_shelf_name']}，应归位至"
            f"{book['expected_shelf_name']}。")
    details = "；".join(
        f"《{book['name']}》应归位至{book['expected_shelf_name']}"
        for book in misplaced)
    return f"视觉检测发现{len(misplaced)}本疑似错放图书：{details}。"


def live_position_message(target: Dict[str, Any]) -> str:
    """Describe a target's left-to-right position from this camera frame.

    A configured catalog rank is not a physical position: books may be moved.
    Only ArUco centres detected in the current frame determine this message.
    """
    rank = int(target["rank"])
    visible = int(target["visible_shelf_book_count"])
    name = str(target["name"])
    if target.get("shelf_position_complete"):
        return f"已识别到《{name}》，在书架上从左往右第{rank}本"
    return (
        f"已识别到《{name}》，在当前画面已识别到的{visible}本同书架图书中，"
        f"从左往右第{rank}本"
    )


def detect_books_stub(expected_id: int = 203) -> Dict[str, Any]:
    name = BOOK_ID_TO_NAME.get(int(expected_id), f"ID{expected_id}")
    return {
        "found": True,
        "books": [
            {"id": int(expected_id), "name": name, "rank": None}
        ],
        "expected_id": int(expected_id),
        "target": {"id": int(expected_id), "name": name, "rank": None},
        "message": f"已识别到《{name}》，但未获取真实书架位置",
    }


def _load_cv2():
    try:
        import cv2
        return cv2
    except Exception as exc:
        raise RuntimeError(f"OpenCV 不可用：{exc}") from exc


def _get_aruco_detector(cv2, dict_name="DICT_5X5_250"):
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dict_name, aruco.DICT_5X5_250)
    dictionary = aruco.getPredefinedDictionary(dictionary_id)

    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            return detector.detectMarkers(gray)

        return detect

    params = aruco.DetectorParameters_create()

    def detect(gray):
        return aruco.detectMarkers(gray, dictionary, parameters=params)

    return detect


def detect_books_in_jpeg(
    jpeg_bytes: bytes,
    expected_id: Optional[int] = 203,
    dict_name: str = "DICT_5X5_250",
) -> Dict[str, Any]:
    cv2 = _load_cv2()
    import numpy as np

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("JPEG 解码失败")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detect = _get_aruco_detector(cv2, dict_name)
    corners, ids, _rejected = detect(gray)

    books = []
    if ids is not None:
        for marker_corners, marker_id_arr in zip(corners, ids):
            marker_id = int(marker_id_arr[0])
            pts = marker_corners.reshape(-1, 2)
            x_min = float(pts[:, 0].min())
            x_max = float(pts[:, 0].max())
            y_min = float(pts[:, 1].min())
            y_max = float(pts[:, 1].max())
            cx = (x_min + x_max) / 2.0

            name = BOOK_ID_TO_NAME.get(marker_id, f"ID{marker_id}")
            books.append({
                "id": marker_id,
                "name": name,
                "bbox": [x_min, y_min, x_max, y_max],
                "cx": cx,
            })

    # 书脊位置必须由当前画面决定；配置中的 rank 仅可用作静态资料，
    # 绝不能用于寻找结果，否则换书后仍会播报旧位置。
    books.sort(key=lambda b: b["cx"])
    for idx, book in enumerate(books, start=1):
        book["visual_rank"] = idx

    books_by_shelf = {}
    for book in books:
        shelf_id = BOOK_ID_TO_SHELF.get(int(book["id"]), "")
        books_by_shelf.setdefault(shelf_id, []).append(book)
    for shelf_id, shelf_books in books_by_shelf.items():
        shelf_books.sort(key=lambda book: book["cx"])
        visible_ids = {int(book["id"]) for book in shelf_books}
        expected_ids = SHELF_BOOK_IDS.get(shelf_id, set())
        complete = bool(expected_ids) and expected_ids.issubset(visible_ids)
        for idx, book in enumerate(shelf_books, start=1):
            book["rank"] = idx
            book["visible_shelf_book_count"] = len(shelf_books)
            book["shelf_position_complete"] = complete

    found_expected = None
    if expected_id is not None:
        for book in books:
            if int(book["id"]) == int(expected_id):
                found_expected = book
                break

    # 画框
    for book in books:
        x1, y1, x2, y2 = [int(v) for v in book["bbox"]]
        is_expected = expected_id is not None and int(book["id"]) == int(expected_id)

        color = (0, 0, 255) if is_expected else (0, 180, 0)
        thickness = 3 if is_expected else 2

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        # OpenCV putText 不支持中文，摄像头画面上只画 ASCII 标签。
        # 中文结果显示在右侧工单中。
        if is_expected:
            label = f'ID{book["id"]} R{book["rank"]} TARGET'
        else:
            label = f'ID{book["id"]} R{book["rank"]}'

        cv2.putText(
            frame,
            label,
            (x1, max(25, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    if found_expected:
        message = live_position_message(found_expected)
        found = True
    elif books:
        message = "已识别到书籍标签，但未找到目标书籍"
        found = False
    else:
        message = "未识别到书籍标签"
        found = False

    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise RuntimeError("标注图像 JPEG 编码失败")

    return {
        "found": found,
        "expected_id": expected_id,
        "target": found_expected,
        "books": books,
        "image_size": {
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
        },
        "message": message,
        "annotated_jpeg": encoded.tobytes(),
    }


def detect_books_in_jpeg_json(jpeg_bytes: bytes, expected_id: Optional[int] = 203) -> Dict[str, Any]:
    result = detect_books_in_jpeg(jpeg_bytes, expected_id=expected_id)
    result = dict(result)
    result.pop("annotated_jpeg", None)
    return result
