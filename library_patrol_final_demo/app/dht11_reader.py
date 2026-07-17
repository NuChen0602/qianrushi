import threading
import time
import subprocess
import select
from datetime import datetime, timezone
from pathlib import Path


class Dht11Reader:
    """Read the kernel DHT11 IIO device with stale-value fallback."""

    SENSOR_NAMES = {"dht11", "dht11_sensor"}

    def __init__(self, iio_root="/sys/bus/iio/devices", board_host=None,
                 board_user="root", poll_interval=1.5):
        self.iio_root = Path(iio_root)
        self.board_host = str(board_host or "")
        self.board_user = str(board_user)
        self.poll_interval = max(0.5, float(poll_interval))
        self.lock = threading.Lock()
        self.device_dir = None
        self.sensor_name = None
        self.last_success = None
        self.last_result = {
            "ok": False, "temperature_c": None, "humidity_rh": None,
            "raw_temperature": None, "raw_humidity": None,
            "sensor_name": None, "device": None, "timestamp": self._now(),
            "age_ms": None, "stale": False, "error": "等待板端 DHT11 检查",
        }
        self.next_poll = time.monotonic() + 12.0
        self.stop_event = threading.Event()
        self.thread = None
        self.ssh_process = None
        if self.board_host:
            self.thread = threading.Thread(
                target=self._remote_loop, daemon=True, name="dht11-reader")
            self.thread.start()

    def read(self):
        if self.board_host:
            with self.lock:
                result = dict(self.last_result)
                if self.last_success:
                    saved_at, _payload = self.last_success
                    result["age_ms"] = int((time.monotonic() - saved_at) * 1000)
                return result
        with self.lock:
            now = time.monotonic()
            if self.last_result is not None and now < self.next_poll:
                result = dict(self.last_result)
                if self.last_success:
                    saved_at, _payload = self.last_success
                    result["age_ms"] = int((now - saved_at) * 1000)
                return result
            self.next_poll = now + self.poll_interval
            try:
                device = self._find_device()
                temperature = self._read_int(device / "in_temp_input")
                humidity = self._read_int(device / "in_humidityrelative_input")
                payload = {
                    "ok": True, "temperature_c": temperature / 1000.0,
                    "humidity_rh": humidity / 1000.0,
                    "raw_temperature": temperature, "raw_humidity": humidity,
                    "sensor_name": self.sensor_name, "device": str(device),
                    "timestamp": self._now(), "age_ms": 0, "stale": False,
                    "error": None,
                }
                self.last_success = (now, dict(payload))
                self.last_result = dict(payload)
                return payload
            except Exception as exc:
                self.next_poll = now + (15.0 if isinstance(exc, FileNotFoundError) else 10.0)
                if self.last_success:
                    saved_at, payload = self.last_success
                    result = dict(payload)
                    result.update(ok=True, stale=True,
                                  age_ms=int((now - saved_at) * 1000), error=str(exc))
                    self.last_result = dict(result)
                    return result
                result = {"ok": False, "temperature_c": None, "humidity_rh": None,
                        "raw_temperature": None, "raw_humidity": None,
                        "sensor_name": self.sensor_name,
                        "device": str(self.device_dir) if self.device_dir else None,
                        "timestamp": self._now(), "age_ms": None, "stale": False,
                        "error": str(exc)}
                self.last_result = dict(result)
                return result

    def close(self):
        self.stop_event.set()
        self._close_remote_reader()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _remote_loop(self):
        # Let the persistent TTS and MQ-2 sessions establish first.
        if self.stop_event.wait(12.0):
            return
        while not self.stop_event.is_set():
            now = time.monotonic()
            try:
                device, sensor_name, temperature, humidity = self._read_remote()
                payload = {
                    "ok": True, "temperature_c": temperature / 1000.0,
                    "humidity_rh": humidity / 1000.0,
                    "raw_temperature": temperature, "raw_humidity": humidity,
                    "sensor_name": sensor_name, "device": device,
                    "timestamp": self._now(), "age_ms": 0, "stale": False,
                    "error": None,
                }
                with self.lock:
                    self.device_dir = Path(device)
                    self.sensor_name = sensor_name
                    self.last_success = (now, dict(payload))
                    self.last_result = dict(payload)
                delay = self.poll_interval
            except Exception as exc:
                with self.lock:
                    if self.last_success:
                        saved_at, payload = self.last_success
                        result = dict(payload)
                        result.update(
                            ok=True, stale=True,
                            age_ms=int((now - saved_at) * 1000), error=str(exc))
                    else:
                        result = {
                            "ok": False, "temperature_c": None, "humidity_rh": None,
                            "raw_temperature": None, "raw_humidity": None,
                            "sensor_name": self.sensor_name,
                            "device": str(self.device_dir) if self.device_dir else None,
                            "timestamp": self._now(), "age_ms": None, "stale": False,
                            "error": str(exc),
                        }
                    self.last_result = result
                delay = 30.0
            self.stop_event.wait(delay)

    def _read_remote(self):
        command = (
            "while :; do found=0; "
            f"for d in {self.iio_root}/iio:device*; do "
            "name=$(cat \"$d/name\" 2>/dev/null) || continue; "
            "case \"$name\" in dht11|dht11_sensor) "
            "temp=$(cat \"$d/in_temp_input\" 2>/dev/null) && "
            "hum=$(cat \"$d/in_humidityrelative_input\" 2>/dev/null) && "
            "printf '%s\\t%s\\t%s\\t%s\\n' \"$d\" \"$name\" \"$temp\" \"$hum\"; "
            "found=1; break;; esac; done; "
            "test \"$found\" = 1 || exit 2; sleep 2; done"
        )
        process = self.ssh_process
        if process is None or process.poll() is not None:
            self._close_remote_reader()
            process = subprocess.Popen(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
                 "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                 f"{self.board_user}@{self.board_host}", command],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1,
            )
            self.ssh_process = process
        readable, _, _ = select.select([process.stdout], [], [], 20.0)
        if not readable:
            self._close_remote_reader()
            raise TimeoutError("板端 DHT11 持久连接读取超时")
        line = process.stdout.readline()
        if not line:
            returncode = process.poll()
            self._close_remote_reader()
            if returncode == 2:
                raise FileNotFoundError("板端 DHT11 IIO 设备未注册")
            raise RuntimeError("板端 DHT11 读取失败")
        values = line.rstrip("\n").split("\t")
        if len(values) < 4:
            raise ValueError("板端 DHT11 返回数据不完整")
        return values[0], values[1], int(values[2]), int(values[3])

    def _close_remote_reader(self):
        process = self.ssh_process
        self.ssh_process = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def _find_device(self):
        if self.device_dir and self._matches(self.device_dir):
            return self.device_dir
        for name_file in sorted(self.iio_root.glob("iio:device*/name")):
            if self._matches(name_file.parent):
                return self.device_dir
        self.device_dir = self.sensor_name = None
        raise FileNotFoundError("DHT11 IIO device not found")

    def _matches(self, device):
        try:
            name = (device / "name").read_text(encoding="utf-8").strip()
        except OSError:
            return False
        if name in self.SENSOR_NAMES:
            self.device_dir, self.sensor_name = device, name
            return True
        return False

    @staticmethod
    def _read_int(path):
        return int(path.read_text(encoding="utf-8").strip())

    @staticmethod
    def _now():
        return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
