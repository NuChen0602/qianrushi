import json
from typing import Dict, Any, Optional


BOOK_ID_TO_NAME = {
    201: "苏东坡传",
    202: "霍乱时期的爱情",
    203: "百年孤独",
    204: "瓦尔登湖",
    205: "大国崛起",
    101: "工程控制论",
}

# 文学书架预设顺序，用于视频展示。
# 不按“当前识别到几个码”临时排名，避免漏检 202 时把 203 错判成第 2 本。
BOOK_ID_TO_FIXED_RANK = {
    201: 1,
    202: 2,
    203: 3,
    204: 4,
    205: 5,
}


def detect_books_stub() -> Dict[str, Any]:
    return {
        "found": True,
        "books": [
            {"id": 203, "name": "百年孤独", "rank": 3}
        ],
        "message": "已识别到《百年孤独》，从左往右第三本",
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

    # 按画面从左到右排序用于显示，但 rank 优先使用书架预设顺序。
    books.sort(key=lambda b: b["cx"])
    for idx, book in enumerate(books, start=1):
        book["visual_rank"] = idx
        book["rank"] = BOOK_ID_TO_FIXED_RANK.get(int(book["id"]), idx)

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
        message = f'已识别到《{found_expected["name"]}》，从左往右第{found_expected["rank"]}本'
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
