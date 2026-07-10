import json
import urllib.error
import urllib.request
from urllib.parse import urljoin


class NavigationClient:
    def __init__(self, dashboard_url, timeout=1.5, endpoints=None):
        self.dashboard_url = dashboard_url.rstrip("/") + "/"
        self.timeout = timeout
        self.endpoints = {
            "map": "/api/map",
            "state": "/api/state",
            "goal": "/api/goal",
            "patrol": "/api/patrol/start",
            "cancel": "/api/cancel",
            "emergency_stop": "/api/emergency-stop",
            "emergency_release": "/api/emergency-release",
        }
        if endpoints:
            self.endpoints.update(endpoints)
        self.last_error = ""

    def _url(self, endpoint):
        return urljoin(self.dashboard_url, endpoint.lstrip("/"))

    def _request_json(self, method, endpoint, payload=None):
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"
        request = urllib.request.Request(
            self._url(endpoint),
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                raw = response.read()
            self.last_error = ""
            return json.loads(raw.decode("utf-8")) if raw else {"ok": True}
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            self.last_error = str(exc)
            raise RuntimeError(f"navigation API unavailable: {exc}") from exc

    def get_state(self):
        return self._request_json("GET", self.endpoints["state"])

    def get_map(self):
        return self._request_json("GET", self.endpoints["map"])

    def send_goal(self, point_dict):
        payload = {
            "x": float(point_dict["x"]),
            "y": float(point_dict["y"]),
            "yaw": float(point_dict.get("yaw", 0.0)),
            "name": str(point_dict.get("name", "目标点")),
        }
        return self._request_json("POST", self.endpoints["goal"], payload)

    def start_patrol(self, waypoints):
        payload = {
            "repeat": False,
            "waypoints": [
                {
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "yaw": float(item.get("yaw", 0.0)),
                    "name": str(item.get("name", "巡检点")),
                }
                for item in waypoints
            ],
        }
        return self._request_json("POST", self.endpoints["patrol"], payload)

    def cancel(self):
        return self._request_json("POST", self.endpoints["cancel"], {})

    def emergency_stop(self):
        return self._request_json("POST", self.endpoints["emergency_stop"], {})

    def emergency_release(self):
        return self._request_json("POST", self.endpoints["emergency_release"], {})

    def is_available(self):
        try:
            self.get_state()
            return True
        except RuntimeError:
            return False
