# ros_interface/ros_node_manager.py
from __future__ import annotations
import threading
import rclpy
from rclpy.node import Node


class RosNodeManager:
    """
    Manages the shared rclpy node used by all TUI collectors.
    Handles initialization, spinning in a background thread, and cleanup.
    """

    def __init__(self, node_name: str = "_ros2_triager_node"):
        self._node_name = node_name
        self._node: Node | None = None
        self._spin_thread: threading.Thread | None = None
        self._running = False

    def initialize(self, domain_id: int = 0) -> Node:
        """Initialize rclpy, create node, start background spin thread."""
        import os
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(self._node_name)
        return self._node

    def start_spin(self) -> None:
        """Start spinning the node in a background daemon thread."""
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True
        )
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                break

    def shutdown(self) -> None:
        """Stop spinning and clean up."""
        self._running = False
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    @property
    def node(self) -> Node | None:
        return self._node
