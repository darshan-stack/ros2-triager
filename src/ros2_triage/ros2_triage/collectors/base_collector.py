# collectors/base_collector.py
from __future__ import annotations
import threading
import time
from abc import ABC, abstractmethod
from rclpy.node import Node
from ..state.state_bus import StateBus


class BaseCollector(ABC):
    """
    Abstract base. Each subclass runs in a daemon thread,
    collecting data and writing to StateBus under bus._lock.
    """

    def __init__(self, node: Node, bus: StateBus, interval_sec: float = 1.0):
        self._node = node
        self._bus = bus
        self._interval = interval_sec
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        while self._running:
            try:
                self.collect()
            except Exception as e:
                self._node.get_logger().warn(
                    f"{self.__class__.__name__} error: {e}"
                )
            time.sleep(self._interval)

    @abstractmethod
    def collect(self) -> None:
        """Override: read from ROS2 graph, write results to self._bus under self._bus._lock."""
        ...
