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

    def _request_json(self, method, endpoint, payload=None, timeout=None):
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
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                raw = response.read()
            self.last_error = ""
            result = json.loads(raw.decode("utf-8")) if raw else {"ok": True}
            if isinstance(result, dict) and result.get("ok") is False:
                raise RuntimeError(f"navigation API rejected request: {result.get('error') or result.get('reason') or result}")
            return result
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
        return self._request_json("POST", self.endpoints["goal"], payload, timeout=3.0)

    def start_patrol(self, waypoints, pause_seconds=None):
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
        if pause_seconds is not None:
            payload["pause_seconds"] = max(0.0, float(pause_seconds))
        return self._request_json("POST", self.endpoints["patrol"], payload, timeout=3.0)

    def cancel(self):
        try:
            return self._request_json("POST", self.endpoints["cancel"], {}, timeout=1.0)
        except RuntimeError as cancel_error:
            # Some deployed dashboard versions reject an idempotent cancel with
            # HTTP 400 when no goal exists.  Never treat every 400 as success:
            # confirm from a fresh state snapshot that both navigation and
            # patrol are already stopped before accepting the no-op cancel.
            if "HTTP Error 400" not in str(cancel_error):
                raise
            try:
                state = self.get_state()
            except RuntimeError:
                raise cancel_error
            navigation = state.get("navigation", {}) if isinstance(state, dict) else {}
            patrol = state.get("patrol", {}) if isinstance(state, dict) else {}
            navigation_state = str(navigation.get("state", "")).lower()
            patrol_state = str(patrol.get("state", "idle")).lower()
            navigation_idle = navigation_state in {
                "idle", "reached", "approached", "cancelled", "canceled",
                "failed", "emergency_stopped",
            }
            patrol_idle = not bool(patrol.get("active", False)) and patrol_state in {
                "", "idle", "waiting", "completed", "cancelled", "canceled",
                "failed", "emergency_stopped",
            }
            if navigation_idle and patrol_idle:
                return {
                    "ok": True,
                    "state": "idle",
                    "cancelled": True,
                    "reason": "already_idle_after_cancel_rejection",
                }
            raise cancel_error

    def emergency_stop(self):
        return self._request_json("POST", self.endpoints["emergency_stop"], {}, timeout=0.8)

    def emergency_release(self):
        return self._request_json("POST", self.endpoints["emergency_release"], {}, timeout=1.5)

    def is_available(self):
        try:
            self.get_state()
            return True
        except RuntimeError:
            return False
