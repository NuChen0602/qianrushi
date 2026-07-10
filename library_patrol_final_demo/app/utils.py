import json
import time
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_config(config_dir, name):
    """Load the JSON config used by the first demo version."""
    path = Path(config_dir) / f"{name}.json"
    if not path.exists():
        raise FileNotFoundError(
            f"missing {path}; this demo reads JSON configs. "
            "YAML copies are kept for humans only."
        )
    return load_json(path)


def now_iso():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def json_response(handler, payload, status=200):
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(data)


def read_json_body(handler, limit=65536):
    length = int(handler.headers.get("Content-Length", "0"))
    if length <= 0 or length > limit:
        raise ValueError("invalid JSON body size")
    return json.loads(handler.rfile.read(length).decode("utf-8"))


def point_payload(point_id, point):
    return {
        "id": point_id,
        "name": point.get("name", point_id),
        "x": float(point["x"]),
        "y": float(point["y"]),
        "yaw": float(point.get("yaw", 0.0)),
    }


def simplify_navigation_state(state):
    if not isinstance(state, dict):
        return {"available": False, "error": "navigation state is not an object"}
    return {
        "available": True,
        "robot": state.get("robot"),
        "robot_connected": state.get("robot_connected"),
        "goal": state.get("goal"),
        "path": state.get("path", []),
        "planner": state.get("planner", {}),
        "navigation": state.get("navigation", {}),
        "localization": state.get("localization", {}),
        "obstacle": state.get("obstacle", {}),
        "drive": state.get("drive", {}),
        "patrol": state.get("patrol", {}),
        "emergency_stop": state.get("emergency_stop", False),
        "map_version": state.get("map_version"),
    }
