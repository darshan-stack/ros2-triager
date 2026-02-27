# Copyright 2024 darshan — Apache-2.0
"""
hz_check.py — Measures topic publish rates and flags topics that are
publishing too slowly (or not at all) compared to a known expected rate.

Without configuration, it flags topics that are publishing but at a
suspiciously low rate (< 0.5 Hz for control topics, < 1 Hz for sensors).
With --expected-hz YAML, it compares against user-defined thresholds.
"""

import time
import collections
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from .finding import Finding, CRITICAL_TOPICS, classify_topic

# Topics that are OUTPUTS / commands — silent when robot is idle, that's normal.
# We should NEVER flag these for being silent; only sensors should be rate-checked.
COMMAND_TOPICS = (
    'cmd_vel', 'twist_cmd', 'cmd_vel_nav', 'cmd_vel_joy',
    'cmd_vel_teleop', 'cmd_vel_unstamped', 'cmd_vel_mux',
)

# Suffixes/prefixes that indicate slam / nav2 visualisation outputs — skip
HZ_SKIP_SUFFIXES = (
    '/scan_visualization',   # slam_toolbox viz
    '/waypoints',            # nav2 waypoint marker array
    '/voxel_grid',
    '/voxel_marked_cloud',
    '/clearing_endpoints',
    '/costmap_raw',
    '/plan_smoothed',
    '/received_global_plan',
    '/transformed_global_plan',
)

# Default minimum expected rates (Hz) by topic keyword
# SENSOR topics only — never put command/output topics here
RATE_RULES = {
    # keyword      (min_hz, severity, label)
    'scan':        (5.0,  3, 'lidar scan'),
    'laser':       (5.0,  3, 'laser scan'),
    'pointcloud':  (2.0,  2, 'point cloud'),    # exact word — avoids /waypoints
    'odom':        (10.0, 3, 'odometry'),
    'imu':         (10.0, 2, 'IMU data'),
    'image_raw':   (5.0,  2, 'camera image'),
    'image_color': (5.0,  2, 'camera image'),
    'joint_state': (10.0, 2, 'joint states'),
    'battery':     (0.5,  2, 'battery state'),
    'gps':         (1.0,  2, 'GPS fix'),
    'fix':         (1.0,  2, 'GPS fix'),        # sensor_msgs/NavSatFix
}

# Measurement window in seconds
MEASURE_WINDOW = 3.0


class _RateCounter(Node):
    """Temporary node that counts messages on multiple topics."""

    def __init__(self, topics_and_types: list):
        super().__init__('_ros2_triage_hz_checker_')
        self._counts: dict = collections.defaultdict(int)
        self._lock = threading.Lock()

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._subs = []
        for topic, types in topics_and_types:
            if not types:
                continue
            msg_type = types[0]
            try:
                # Use AnyMsg to avoid importing every message type
                from rclpy.serialization import deserialize_message
                sub = self.create_subscription(
                    self._get_msg_class(msg_type),
                    topic,
                    self._make_callback(topic),
                    best_effort_qos,
                )
                self._subs.append(sub)
            except Exception:
                pass  # skip topics whose type we can't import

    def _make_callback(self, topic: str):
        def cb(msg):
            with self._lock:
                self._counts[topic] += 1
        return cb

    def _get_msg_class(self, type_str: str):
        """Dynamically import a message class from its type string."""
        # e.g. "sensor_msgs/msg/LaserScan" → sensor_msgs.msg.LaserScan
        parts = type_str.replace('/', '.').rsplit('.', 1)
        if len(parts) != 2:
            raise ImportError(f'Cannot parse type: {type_str}')
        module_path, class_name = parts
        import importlib
        mod = importlib.import_module(module_path)
        return getattr(mod, class_name)

    def get_counts(self) -> dict:
        with self._lock:
            return dict(self._counts)


def check_hz(graph: dict,
             expected_hz: Optional[dict] = None,
             window: float = MEASURE_WINDOW,
             ignore_set: set = None) -> list:
    """
    Measure publish rates of all active topics and flag anomalies.

    Parameters
    ----------
    graph        : dict  — from graph_utils.build_topic_graph()
    expected_hz  : dict  — {topic: min_hz} from --expected-hz YAML
    window       : float — measurement window in seconds
    ignore_set   : set   — topics to skip

    Returns
    -------
    list[Finding]
    """
    findings = []
    ignore_set = ignore_set or set()

    def _should_skip(topic: str) -> bool:
        """Return True if this topic should never be rate-checked."""
        low = topic.lower()
        # Skip command/output topics — silent when robot is idle, that's normal
        if any(cmd in low for cmd in COMMAND_TOPICS):
            return True
        # Skip slam / nav2 visualization suffixes
        if any(topic.endswith(sfx) or low.endswith(sfx) for sfx in HZ_SKIP_SUFFIXES):
            return True
        # Skip infra / viz / sim topics
        cls = classify_topic(topic)
        if cls in ('skip', 'sim', 'viz'):
            return True
        return False

    # Only measure topics that have publishers and are worth checking
    measurable = {
        topic: info for topic, info in graph.items()
        if len(info['publishers']) > 0
        and topic not in ignore_set
        and not _should_skip(topic)
    }

    if not measurable:
        return findings

    # Subscribe to every publishable topic and count messages
    need_init = not rclpy.ok()
    if need_init:
        rclpy.init()

    counter = _RateCounter(
        [(t, info['types']) for t, info in measurable.items()]
    )

    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(counter)

    t_start = time.time()
    while time.time() - t_start < window:
        executor.spin_once(timeout_sec=0.1)

    counts = counter.get_counts()
    counter.destroy_node()

    if need_init:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Evaluate rates
    for topic in measurable:
        count = counts.get(topic, 0)
        hz = count / window

        # Determine expected minimum rate
        min_hz = None
        severity = 2
        label = 'topic'

        if expected_hz and topic in expected_hz:
            min_hz = expected_hz[topic]
            severity = 3
            label = 'user-defined'
        else:
            low = topic.lower()
            for kw, (rate, sev, lbl) in RATE_RULES.items():
                if kw in low:
                    min_hz = rate
                    severity = sev
                    label = lbl
                    break

        if min_hz is None:
            continue  # no rule applies

        if hz < min_hz:
            actual_str = f'{hz:.1f} Hz' if hz > 0 else '0 Hz (SILENT)'
            findings.append(Finding(
                check='hz',
                topic=topic,
                severity=severity if hz == 0 else max(1, severity - (1 if hz > min_hz * 0.5 else 0)),
                message=(
                    f'{label} topic {topic} is publishing at '
                    f'{actual_str} — expected ≥ {min_hz:.0f} Hz.'
                ),
                suggestion=(
                    f'Check the node publishing {topic} is not CPU-starved or crashed.\n'
                    f'Run: `ros2 topic hz {topic}` to confirm.\n'
                    f'Common causes: sensor driver stalling, timer drift, process overload.'
                ),
                extra={'measured_hz': round(hz, 2), 'expected_hz': min_hz},
            ))

    findings.sort(key=lambda f: (-f.severity, f.topic))
    return findings
