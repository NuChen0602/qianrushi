WAKE_FRAME = "AA 55 03 00 FB"
SLEEP_FRAME = "AA 55 02 6F FB"
SLEEP_ACK_FRAME = "AA 55 02 00 FB"
SLEEP_FRAMES = {SLEEP_FRAME, SLEEP_ACK_FRAME}


BOOK_FRAME_BY_ID = {
    203: "AA 55 00 A2 FB",  # 保留原视频“寻找百年孤独”直接导航帧
    101: "AA 55 00 B0 FB",
    102: "AA 55 00 B1 FB",
    103: "AA 55 00 B2 FB",
    104: "AA 55 00 B3 FB",
    105: "AA 55 00 B4 FB",
    151: "AA 55 00 B5 FB",
    152: "AA 55 00 B6 FB",
    153: "AA 55 00 B7 FB",
    154: "AA 55 00 B8 FB",
    155: "AA 55 00 B9 FB",
    201: "AA 55 00 BA FB",
    202: "AA 55 00 BB FB",
    204: "AA 55 00 BC FB",
    205: "AA 55 00 BD FB",
}


FIXED_COMMANDS = [
    {"id": "RECOMMEND_BOOKS", "phrase": "推荐文学小说", "frame": "AA 55 00 A1 FB", "kind": "mission", "mission": "RECOMMEND_BOOKS"},
    {"id": "INTRO_CURRENT", "phrase": "介绍这本书", "frame": "AA 55 00 A3 FB", "kind": "introduce_current"},
    {"id": "SHELF_CHECK", "phrase": "检查当前书架", "frame": "AA 55 00 A4 FB", "kind": "mission", "mission": "SHELF_CHECK"},
    {"id": "LOST_ITEM_PATROL", "phrase": "扫描遗失物", "frame": "AA 55 00 A5 FB", "kind": "mission", "mission": "LOST_ITEM_PATROL"},
    {"id": "HAZARD_CHECK", "phrase": "检查高危点位", "frame": "AA 55 00 A6 FB", "kind": "mission", "mission": "HAZARD_CHECK"},
    {"id": "FULL_PATROL", "phrase": "开始全图巡检", "frame": "AA 55 00 A7 FB", "kind": "mission", "mission": "FULL_PATROL"},
    {"id": "RETURN_HOME", "phrase": "返回起点", "frame": "AA 55 00 A8 FB", "kind": "mission", "mission": "RETURN_HOME"},
    {"id": "CANCEL", "phrase": "停止当前任务", "frame": "AA 55 00 A9 FB", "kind": "cancel"},
    {"id": "EXIT_DIALOGUE", "phrase": "退出对话", "frame": "AA 55 00 BE FB", "kind": "sleep"},
]


def build_ci302_commands(catalog):
    commands = list(FIXED_COMMANDS)
    for book in catalog.books:
        commands.append({
            "id": f"FIND_{book['id']}",
            "phrase": f"寻找{book['name']}",
            "frame": BOOK_FRAME_BY_ID[int(book["id"])],
            "kind": "find_book",
            "book_id": int(book["id"]),
        })
    return commands


def commands_by_frame(catalog):
    return {command["frame"]: command for command in build_ci302_commands(catalog)}
