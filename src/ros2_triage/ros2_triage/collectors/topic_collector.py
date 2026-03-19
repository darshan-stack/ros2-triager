# collectors/topic_collector.py
from __future__ import annotations
import time
from collections import deque
from rclpy.node import Node
from ..state.state_bus import StateBus, TopicState
from .base_collector import BaseCollector


class TopicCollector(BaseCollector):
    """
    Auto-discovers ALL topics via get_topic_names_and_types().
    Measures actual Hz using rolling deque of message timestamps.
    Measures bandwidth using serialized message size.
    """

    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=1.0)
        self._timestamps: dict[str, deque] = {}  # topic → deque of float
        self._bytes: dict[str, int] = {}
        self._subscriptions: dict[str, object] = {}
        self._known_topics: set[str] = set()

    def collect(self) -> None:
        # Discover new topics
        current = {
            name: types[0] if types else "unknown"
            for name, types in self._node.get_topic_names_and_types()
        }
        for topic, msg_type in current.items():
            if topic not in self._known_topics:
                self._known_topics.add(topic)
                self._timestamps[topic] = deque(maxlen=100)
                self._bytes[topic] = 0
                self._subscribe(topic, msg_type)

        # Update state for all known topics
        with self._bus._lock:
            for topic in self._known_topics:
                ts = self._timestamps.get(topic, deque())
                hz = self._calc_hz(ts)
                pub_count = self._node.count_publishers(topic)
                sub_count = self._node.count_subscribers(topic)
                last_time = ts[-1] if ts else 0.0
                age = time.monotonic() - last_time if last_time else 9999.0

                # Accurate status — see §DEAD TOPIC RULE
                if pub_count == 0:
                    status = "NO_PUB"
                elif hz == 0.0 and age > 5.0:
                    status = "DEAD"
                elif hz > 0:
                    existing = self._bus.topics.get(topic)
                    exp = existing.expected_hz if existing else None
                    if exp and abs(hz - exp) / exp > 0.5:
                        status = "LOW_HZ"
                    else:
                        status = "OK"
                else:
                    status = "OK"

                existing = self._bus.topics.get(topic)
                bw = self._calc_bw(topic, ts)
                if existing:
                    existing.actual_hz = hz
                    existing.publisher_count = pub_count
                    existing.subscriber_count = sub_count
                    existing.last_msg_time = last_time
                    existing.status = status
                    existing.bandwidth_kbps = bw
                    existing.hz_history.append(hz)
                else:
                    self._bus.topics[topic] = TopicState(
                        name=topic,
                        msg_type=current.get(topic, "unknown"),
                        actual_hz=hz,
                        publisher_count=pub_count,
                        subscriber_count=sub_count,
                        last_msg_time=last_time,
                        status=status,
                        bandwidth_kbps=bw,
                    )

    def _subscribe(self, topic: str, msg_type_str: str) -> None:
        """Create a generic subscription using AnyMsg to track timestamps."""
        try:
            from rosidl_runtime_py.utilities import get_message
            import rclpy.qos as qos
            MsgType = get_message(msg_type_str)
            qos_profile = qos.qos_profile_sensor_data

            def cb(msg, t=topic):
                now = time.monotonic()
                self._timestamps[t].append(now)
                self._bytes[t] = self._bytes.get(t, 0) + len(str(msg))

            sub = self._node.create_subscription(MsgType, topic, cb, qos_profile)
            self._subscriptions[topic] = sub
        except Exception:
            pass  # Unknown type — will still track pub/sub counts

    def _calc_hz(self, ts: deque) -> float:
        if len(ts) < 2:
            return 0.0
        window = min(len(ts), 10)
        recent = list(ts)[-window:]
        elapsed = recent[-1] - recent[0]
        return (len(recent) - 1) / elapsed if elapsed > 0 else 0.0

    def _calc_bw(self, topic: str, ts: deque) -> float:
        if len(ts) < 2:
            return 0.0
        elapsed = ts[-1] - ts[0]
        if elapsed <= 0:
            return 0.0
        return (self._bytes.get(topic, 0) / elapsed) / 1024.0  # KB/s
