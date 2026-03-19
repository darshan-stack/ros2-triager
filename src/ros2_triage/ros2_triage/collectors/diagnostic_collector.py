# collectors/diagnostic_collector.py
from __future__ import annotations
import time
import rclpy.qos as qos
from rclpy.node import Node
from ..state.state_bus import StateBus, DiagItem, DiagState
from .base_collector import BaseCollector

try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    _HAS_DIAG = True
except ImportError:
    _HAS_DIAG = False


class DiagnosticCollector(BaseCollector):
    """
    Subscribes to /diagnostics AND /diagnostics_agg.
    Converts DiagnosticStatus into DiagItem in StateBus.
    Also converts to Finding objects for compatibility with existing checks/.
    """

    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=99999)  # event-driven
        self._sub = None
        self._sub_agg = None

    def start(self) -> None:
        if not _HAS_DIAG:
            self._node.get_logger().warn(
                "diagnostic_msgs not available — diagnostics disabled"
            )
            return
        self._sub = self._node.create_subscription(
            DiagnosticArray, "/diagnostics",
            self._callback, qos.qos_profile_sensor_data
        )
        try:
            self._sub_agg = self._node.create_subscription(
                DiagnosticArray, "/diagnostics_agg",
                self._callback, qos.qos_profile_sensor_data
            )
        except Exception:
            pass

    def collect(self) -> None:
        pass  # callback-driven

    def _callback(self, msg: "DiagnosticArray") -> None:
        items: dict[str, DiagItem] = {}
        errors = warns = 0
        for status in msg.status:
            key = f"{status.hardware_id}/{status.name}"
            level = int(status.level)
            if level >= DiagnosticStatus.ERROR:
                errors += 1
            elif level == DiagnosticStatus.WARN:
                warns += 1
            items[key] = DiagItem(
                hardware_id=status.hardware_id,
                name=status.name,
                level=level,
                message=status.message,
                values={kv.key: kv.value for kv in status.values},
                stamp=time.time(),
            )
        with self._bus._lock:
            self._bus.diag = DiagState(
                items={**self._bus.diag.items, **items},
                error_count=errors,
                warn_count=warns,
            )
