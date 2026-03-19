# collectors/tf_collector.py
from __future__ import annotations
import time
from rclpy.node import Node
from ..state.state_bus import StateBus, TFState
from .base_collector import BaseCollector


class TFCollector(BaseCollector):
    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=0.5)
        self._tf_buffer = None
        self._tf_listener = None
        self._init_tf()

    def _init_tf(self) -> None:
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self._node
            )
        except ImportError:
            self._node.get_logger().warn(
                "tf2_ros not available — TF checks disabled"
            )

    def collect(self) -> None:
        if not self._tf_buffer:
            return
        try:
            raw = self._tf_buffer.all_frames_as_yaml()
            frames, stale = self._parse_frames(raw)
            with self._bus._lock:
                self._bus.tf = TFState(
                    frames=frames,
                    stale_frames=stale,
                    broken_chains=self._find_broken_chains(frames),
                    last_update=time.monotonic(),
                )
        except Exception as e:
            self._node.get_logger().debug(f"TFCollector: {e}")

    def _parse_frames(self, yaml_str: str) -> tuple[dict[str, str], list[str]]:
        """Parse tf2 YAML output into child→parent dict + stale list."""
        import yaml
        frames: dict[str, str] = {}
        stale: list[str] = []
        try:
            data = yaml.safe_load(yaml_str) or {}
            for frame_id, info in data.items():
                parent = info.get("parent", "")
                frames[frame_id] = parent
                age = info.get("most_recent_transform", 0.0)
                if age > 0.5:
                    stale.append(frame_id)
        except Exception:
            pass
        return frames, stale

    def _find_broken_chains(self, frames: dict[str, str]) -> list[str]:
        """Find frames whose parent chain never reaches a root."""
        roots = {f for f, p in frames.items() if not p or p not in frames}
        broken = []
        for frame in frames:
            visited = set()
            cur = frame
            while cur in frames and cur not in roots:
                if cur in visited:
                    broken.append(frame)
                    break
                visited.add(cur)
                cur = frames[cur]
        return broken
