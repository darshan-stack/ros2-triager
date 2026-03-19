# export/alert_webhook.py
from __future__ import annotations
import json
import threading
import time
from ..state.state_bus import Alert


def post_alert(url: str, alert: Alert, robot_name: str = "robot") -> bool:
    """
    POST a single alert to a webhook URL.
    Returns True on success, False on failure.
    Non-blocking wrapper calls this in a daemon thread.
    """
    try:
        import urllib.request
        payload = {
            "robot": robot_name,
            "timestamp": alert.timestamp,
            "severity": alert.severity,
            "source": alert.source,
            "message": alert.message,
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status < 400
    except Exception:
        return False


def post_alert_async(url: str, alert: Alert, robot_name: str = "robot") -> None:
    """Fire-and-forget webhook post in a daemon thread."""
    t = threading.Thread(
        target=post_alert,
        args=(url, alert, robot_name),
        daemon=True,
    )
    t.start()


class AlertWebhookPublisher:
    """
    Watches the StateBus alert deque and posts new alerts to a webhook URL.
    Run in a background thread.
    """

    def __init__(self, url: str, robot_name: str = "robot", poll_sec: float = 1.0):
        self._url = url
        self._robot_name = robot_name
        self._poll_sec = poll_sec
        self._last_ts: float = time.time()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self, bus) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, args=(bus,), daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _loop(self, bus) -> None:
        while self._running:
            try:
                with bus._lock:
                    new_alerts = [
                        a for a in bus.alerts
                        if a.timestamp > self._last_ts
                    ]
                for alert in reversed(new_alerts):
                    post_alert_async(self._url, alert, self._robot_name)
                if new_alerts:
                    self._last_ts = new_alerts[0].timestamp
            except Exception:
                pass
            time.sleep(self._poll_sec)
