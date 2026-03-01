# Copyright 2024 darshan - Apache-2.0
"""
latency_engine.py - Latency and Jitter Analysis Engine

Calculates timing metrics for stamped messages:
  - Latency: T_arrival - T_header_stamp (transport + processing delay)
  - Jitter: Variance in inter-message timing
  - Drift: Systematic timing offset over time

Used to detect:
  - Network congestion
  - Processing bottlenecks
  - Clock synchronization issues
  - Real-time constraint violations
"""

import time
import threading
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.time import Time
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

from .finding import Finding


@dataclass
class LatencyStats:
    """Statistics for a single topic's latency measurements."""
    topic: str
    sample_count: int = 0
    min_latency_ms: float = float('inf')
    max_latency_ms: float = 0.0
    mean_latency_ms: float = 0.0
    std_latency_ms: float = 0.0
    jitter_ms: float = 0.0  # Mean absolute deviation of inter-arrival times
    samples: List[float] = field(default_factory=list)
    inter_arrival_times: List[float] = field(default_factory=list)
    
    def update(self, latency_ms: float, inter_arrival_ms: float = None):
        """Add a new latency sample."""
        self.samples.append(latency_ms)
        self.sample_count = len(self.samples)
        
        if inter_arrival_ms is not None and inter_arrival_ms > 0:
            self.inter_arrival_times.append(inter_arrival_ms)
        
        # Update statistics
        self.min_latency_ms = min(self.min_latency_ms, latency_ms)
        self.max_latency_ms = max(self.max_latency_ms, latency_ms)
        self.mean_latency_ms = statistics.mean(self.samples)
        
        if len(self.samples) >= 2:
            self.std_latency_ms = statistics.stdev(self.samples)
        
        if len(self.inter_arrival_times) >= 2:
            mean_iat = statistics.mean(self.inter_arrival_times)
            deviations = [abs(t - mean_iat) for t in self.inter_arrival_times]
            self.jitter_ms = statistics.mean(deviations)


@dataclass
class LatencyThreshold:
    """Threshold configuration for latency alerts."""
    warn_ms: float = 50.0   # Warning threshold in milliseconds
    crit_ms: float = 100.0  # Critical threshold in milliseconds
    jitter_warn_ms: float = 20.0
    jitter_crit_ms: float = 50.0


# Default thresholds by topic type
DEFAULT_THRESHOLDS: Dict[str, LatencyThreshold] = {
    'scan': LatencyThreshold(warn_ms=30, crit_ms=80, jitter_warn_ms=10, jitter_crit_ms=30),
    'odom': LatencyThreshold(warn_ms=20, crit_ms=50, jitter_warn_ms=5, jitter_crit_ms=15),
    'imu': LatencyThreshold(warn_ms=10, crit_ms=30, jitter_warn_ms=3, jitter_crit_ms=10),
    'cmd_vel': LatencyThreshold(warn_ms=20, crit_ms=50, jitter_warn_ms=5, jitter_crit_ms=15),
    'image': LatencyThreshold(warn_ms=100, crit_ms=200, jitter_warn_ms=30, jitter_crit_ms=60),
    'pointcloud': LatencyThreshold(warn_ms=100, crit_ms=200, jitter_warn_ms=30, jitter_crit_ms=60),
    'default': LatencyThreshold(warn_ms=50, crit_ms=100, jitter_warn_ms=20, jitter_crit_ms=50),
}


class LatencyMeasurer:
    """
    Measures latency (T_arrival - T_header_stamp) for stamped messages.
    
    Only works with messages that have a std_msgs/Header with stamp field.
    Supported message types:
      - sensor_msgs/LaserScan
      - nav_msgs/Odometry
      - sensor_msgs/Imu
      - sensor_msgs/PointCloud2
      - sensor_msgs/Image
      - geometry_msgs/TwistStamped
    """
    
    # Message types with headers (type_name -> module path)
    STAMPED_TYPES = {
        'sensor_msgs/msg/LaserScan': 'sensor_msgs.msg',
        'sensor_msgs/msg/Imu': 'sensor_msgs.msg',
        'sensor_msgs/msg/PointCloud2': 'sensor_msgs.msg',
        'sensor_msgs/msg/Image': 'sensor_msgs.msg',
        'nav_msgs/msg/Odometry': 'nav_msgs.msg',
        'geometry_msgs/msg/TwistStamped': 'geometry_msgs.msg',
        'geometry_msgs/msg/PoseStamped': 'geometry_msgs.msg',
        'tf2_msgs/msg/TFMessage': 'tf2_msgs.msg',
    }
    
    def __init__(self, window_sec: float = 5.0, max_samples: int = 100):
        self.window_sec = window_sec
        self.max_samples = max_samples
        self._stats: Dict[str, LatencyStats] = {}
        self._last_arrival: Dict[str, float] = {}
        self._lock = threading.Lock()
    
    def measure_latency(
        self,
        topic: str,
        header_stamp_sec: float,
        header_stamp_nsec: int
    ) -> Optional[float]:
        """
        Calculate latency between message timestamp and arrival time.
        
        Args:
            topic: Topic name
            header_stamp_sec: Header stamp seconds
            header_stamp_nsec: Header stamp nanoseconds
            
        Returns:
            Latency in milliseconds, or None if calculation failed
        """
        arrival_time = time.time()
        message_time = header_stamp_sec + (header_stamp_nsec / 1e9)
        
        # Sanity check: message time should not be in the future
        if message_time > arrival_time + 1.0:
            # Clock not synchronized or invalid timestamp
            return None
        
        latency_ms = (arrival_time - message_time) * 1000.0
        
        # Calculate inter-arrival time
        inter_arrival_ms = None
        with self._lock:
            if topic in self._last_arrival:
                inter_arrival_ms = (arrival_time - self._last_arrival[topic]) * 1000.0
            self._last_arrival[topic] = arrival_time
            
            # Update stats
            if topic not in self._stats:
                self._stats[topic] = LatencyStats(topic=topic)
            
            self._stats[topic].update(latency_ms, inter_arrival_ms)
            
            # Limit samples
            if len(self._stats[topic].samples) > self.max_samples:
                self._stats[topic].samples = self._stats[topic].samples[-self.max_samples:]
                self._stats[topic].inter_arrival_times = \
                    self._stats[topic].inter_arrival_times[-self.max_samples:]
        
        return latency_ms
    
    def get_stats(self, topic: str) -> Optional[LatencyStats]:
        """Get latency statistics for a topic."""
        with self._lock:
            return self._stats.get(topic)
    
    def get_all_stats(self) -> Dict[str, LatencyStats]:
        """Get all latency statistics."""
        with self._lock:
            return dict(self._stats)
    
    def clear(self):
        """Clear all measurements."""
        with self._lock:
            self._stats.clear()
            self._last_arrival.clear()


class _LatencyNode(Node):
    """Temporary node for measuring message latency."""
    
    def __init__(self, topics_and_types: List[Tuple[str, List[str]]]):
        super().__init__('_ros2_triage_latency_checker_')
        self.measurer = LatencyMeasurer()
        self._subs = []
        
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        
        for topic, types in topics_and_types:
            if not types:
                continue
            msg_type_str = types[0]
            
            # Check if this is a stamped message type
            msg_class = self._get_stamped_msg_class(msg_type_str)
            if msg_class is None:
                continue
            
            try:
                sub = self.create_subscription(
                    msg_class,
                    topic,
                    self._make_callback(topic),
                    best_effort_qos,
                )
                self._subs.append(sub)
            except Exception:
                pass
    
    def _get_stamped_msg_class(self, type_str: str):
        """Get message class if it's a stamped type."""
        if type_str not in LatencyMeasurer.STAMPED_TYPES:
            return None
        
        try:
            # Parse type string: "sensor_msgs/msg/LaserScan"
            parts = type_str.split('/')
            if len(parts) != 3:
                return None
            
            pkg, _, msg_name = parts
            module = __import__(f'{pkg}.msg', fromlist=[msg_name])
            return getattr(module, msg_name)
        except (ImportError, AttributeError):
            return None
    
    def _make_callback(self, topic: str):
        def cb(msg):
            # Extract header stamp
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                stamp = msg.header.stamp
                self.measurer.measure_latency(
                    topic,
                    float(stamp.sec),
                    stamp.nanosec
                )
            elif hasattr(msg, 'transforms') and msg.transforms:
                # TF message
                stamp = msg.transforms[0].header.stamp
                self.measurer.measure_latency(
                    topic,
                    float(stamp.sec),
                    stamp.nanosec
                )
        return cb


def _get_threshold_for_topic(topic: str) -> LatencyThreshold:
    """Get appropriate threshold for a topic based on its name."""
    topic_lower = topic.lower()
    
    for keyword, threshold in DEFAULT_THRESHOLDS.items():
        if keyword != 'default' and keyword in topic_lower:
            return threshold
    
    return DEFAULT_THRESHOLDS['default']


def check_latency(
    graph: dict,
    window_sec: float = 5.0,
    ignore_set: set = None
) -> List[Finding]:
    """
    Measure latency and jitter for stamped topics.
    
    Args:
        graph: Topic graph from graph_utils.build_topic_graph()
        window_sec: Measurement window in seconds
        ignore_set: Topics to ignore
        
    Returns:
        List of findings for topics with latency/jitter issues
    """
    if not HAS_RCLPY:
        return [Finding(
            check='latency',
            topic='N/A',
            severity=1,
            message='rclpy not available - latency check skipped',
            suggestion='Source ROS 2 environment: source /opt/ros/humble/setup.bash'
        )]
    
    ignore_set = ignore_set or set()
    findings = []
    
    # Filter topics to only stamped types
    topics_to_check = []
    for topic, info in graph.items():
        if topic in ignore_set:
            continue
        
        types = info.get('types', [])
        if not types:
            continue
        
        # Check if any type is stamped
        for t in types:
            if t in LatencyMeasurer.STAMPED_TYPES:
                topics_to_check.append((topic, types))
                break
    
    if not topics_to_check:
        return []
    
    # Create temporary node and measure
    try:
        if not rclpy.ok():
            rclpy.init()
        
        node = _LatencyNode(topics_to_check)
        
        # Spin for measurement window
        start = time.time()
        while (time.time() - start) < window_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Analyze results
        all_stats = node.measurer.get_all_stats()
        
        for topic, stats in all_stats.items():
            if stats.sample_count < 3:
                continue  # Not enough samples
            
            threshold = _get_threshold_for_topic(topic)
            
            # Check mean latency
            if stats.mean_latency_ms >= threshold.crit_ms:
                findings.append(Finding(
                    check='latency',
                    topic=topic,
                    severity=3,
                    message=(
                        f'High latency: {stats.mean_latency_ms:.1f}ms mean '
                        f'(threshold: {threshold.crit_ms:.0f}ms). '
                        f'Range: {stats.min_latency_ms:.1f}-{stats.max_latency_ms:.1f}ms'
                    ),
                    suggestion=(
                        'Check for network congestion, processing delays, or '
                        'clock synchronization issues. '
                        'Consider: chronyc sources, ros2 topic hz, htop'
                    ),
                    extra={
                        'mean_latency_ms': round(stats.mean_latency_ms, 2),
                        'max_latency_ms': round(stats.max_latency_ms, 2),
                        'samples': stats.sample_count,
                    }
                ))
            elif stats.mean_latency_ms >= threshold.warn_ms:
                findings.append(Finding(
                    check='latency',
                    topic=topic,
                    severity=2,
                    message=(
                        f'Elevated latency: {stats.mean_latency_ms:.1f}ms mean '
                        f'(threshold: {threshold.warn_ms:.0f}ms). '
                        f'Range: {stats.min_latency_ms:.1f}-{stats.max_latency_ms:.1f}ms'
                    ),
                    suggestion=(
                        'Monitor for increasing latency. May indicate early '
                        'signs of network or processing issues.'
                    ),
                    extra={
                        'mean_latency_ms': round(stats.mean_latency_ms, 2),
                        'samples': stats.sample_count,
                    }
                ))
            
            # Check jitter
            if stats.jitter_ms >= threshold.jitter_crit_ms:
                findings.append(Finding(
                    check='latency',
                    topic=topic,
                    severity=3,
                    message=(
                        f'High jitter: {stats.jitter_ms:.1f}ms '
                        f'(threshold: {threshold.jitter_crit_ms:.0f}ms). '
                        f'Timing variance may cause control instability.'
                    ),
                    suggestion=(
                        'Jitter indicates inconsistent timing. Check: '
                        'CPU load (htop), real-time scheduling, '
                        'network congestion, publisher stability.'
                    ),
                    extra={
                        'jitter_ms': round(stats.jitter_ms, 2),
                        'std_latency_ms': round(stats.std_latency_ms, 2),
                    }
                ))
            elif stats.jitter_ms >= threshold.jitter_warn_ms:
                findings.append(Finding(
                    check='latency',
                    topic=topic,
                    severity=2,
                    message=(
                        f'Moderate jitter: {stats.jitter_ms:.1f}ms '
                        f'(threshold: {threshold.jitter_warn_ms:.0f}ms)'
                    ),
                    suggestion='Monitor timing consistency under load.',
                    extra={'jitter_ms': round(stats.jitter_ms, 2)}
                ))
        
        node.destroy_node()
        
    except Exception as e:
        findings.append(Finding(
            check='latency',
            topic='N/A',
            severity=1,
            message=f'Latency check failed: {e}',
            suggestion='Check ROS 2 environment and topic availability.'
        ))
    
    return findings


def get_latency_summary(stats: Dict[str, LatencyStats]) -> dict:
    """
    Generate summary statistics across all measured topics.
    
    Returns:
        Dict with overall latency health metrics
    """
    if not stats:
        return {'status': 'no_data', 'topics_measured': 0}
    
    total_samples = sum(s.sample_count for s in stats.values())
    all_means = [s.mean_latency_ms for s in stats.values() if s.sample_count > 0]
    all_jitters = [s.jitter_ms for s in stats.values() if s.jitter_ms > 0]
    
    return {
        'status': 'ok',
        'topics_measured': len(stats),
        'total_samples': total_samples,
        'avg_latency_ms': round(statistics.mean(all_means), 2) if all_means else 0,
        'max_latency_ms': round(max(s.max_latency_ms for s in stats.values()), 2),
        'avg_jitter_ms': round(statistics.mean(all_jitters), 2) if all_jitters else 0,
    }
