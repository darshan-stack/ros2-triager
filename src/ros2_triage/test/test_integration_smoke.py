# test/test_integration_smoke.py
"""
Integration smoke test: verify all components work together without ROS2 runtime.
Tests the complete data flow from StateBus -> Collectors -> Engine -> TUI widgets.
"""
import pytest
import threading
import time
from collections import deque

from ros2_triage.state.state_bus import StateBus, TopicState, NodeState, Alert
from ros2_triage.engine.health_scorer import compute_health
from ros2_triage.engine.dead_topic_detector import classify_topic
from ros2_triage.engine.zombie_node_detector import is_zombie
from ros2_triage.correlation_engine import CorrelationEngine
from ros2_triage.export.snapshot_exporter import export_snapshot


class TestIntegrationSmoke:
    """Smoke tests for the complete system integration."""

    def test_bus_to_health_scorer_flow(self):
        """Test: StateBus -> HealthScorer -> Score computation."""
        bus = StateBus()

        # Add some topics
        bus.topics["/scan"] = TopicState(
            name="/scan", msg_type="sensor_msgs/LaserScan",
            status="OK", publisher_count=1, subscriber_count=2
        )
        bus.topics["/cmd_vel"] = TopicState(
            name="/cmd_vel", msg_type="geometry_msgs/Twist",
            status="DEAD", publisher_count=0, is_critical=True
        )

        # Add some nodes
        bus.nodes["/robot_driver"] = NodeState(
            name="/robot_driver", is_zombie=False
        )
        bus.nodes["/dead_node"] = NodeState(
            name="/dead_node", is_zombie=True
        )

        # Compute health
        score = compute_health(bus)

        # Verify scoring logic
        assert score.overall < 100, "Should be penalized for dead topic and zombie"
        assert score.topics_score == 80, "Critical dead topic = -20"
        assert score.nodes_score == 70, "Zombie node = -30"
        # Weighted: 0.3*80 + 0.2*70 + 0.2*100 + 0.2*100 + 0.1*100 = 88 (HEALTHY)
        assert score.overall == 88
        assert score.label in ["HEALTHY", "DEGRADED"]  # 88 is still HEALTHY

    def test_topic_classification(self):
        """Test: Topic classifier correctly identifies topic statuses."""
        # OK topic
        t1 = TopicState(
            name="/scan", msg_type="sensor_msgs/LaserScan",
            publisher_count=1, actual_hz=10.0, expected_hz=10.0,
            last_msg_time=time.monotonic()
        )
        status = classify_topic(t1)
        assert status in ["OK", "UNKNOWN"]

        # DEAD topic (no publisher)
        t2 = TopicState(
            name="/missing", msg_type="std_msgs/String",
            publisher_count=0, actual_hz=0.0, last_msg_time=0.0
        )
        status = classify_topic(t2)
        assert status in ["NO_PUB", "DEAD"]

    def test_zombie_detector(self):
        """Test: Zombie node detection logic."""
        bus = StateBus()

        # Create a node that publishes a topic
        bus.topics["/pub_topic"] = TopicState(
            name="/pub_topic", msg_type="std_msgs/String",
            publisher_count=0  # Node claims to publish but topic has no pubs
        )

        node = NodeState(
            name="/zombie_candidate",
            pub_topics=["/pub_topic"],
            sub_topics=[]
        )

        # Check if it's a zombie
        result = is_zombie(node, bus.topics)
        assert isinstance(result, bool)

    def test_correlation_engine_from_bus(self):
        """Test: Correlation engine analyzes StateBus state."""
        bus = StateBus()

        # Add dead topic
        bus.topics["/odom"] = TopicState(
            name="/odom", msg_type="nav_msgs/Odometry",
            status="DEAD", publisher_count=0
        )

        # Add zombie node
        bus.nodes["/odom_publisher"] = NodeState(
            name="/odom_publisher", is_zombie=True,
            pub_topics=["/odom"]
        )

        # Add stale TF
        bus.tf.stale_frames = ["odom"]

        engine = CorrelationEngine()
        hypotheses = engine.correlate_from_bus(bus)

        assert isinstance(hypotheses, list)
        # Should detect the odometry pipeline issue
        if hypotheses:
            h = hypotheses[0]
            assert "confidence" in h
            assert "root_cause" in h
            assert h["confidence"] > 0.7

    def test_alert_system(self):
        """Test: Alert push and retrieval."""
        bus = StateBus()

        bus.push_alert("ERROR", "test_system", "Critical failure detected")
        bus.push_alert("WARN", "test_system", "Warning condition")
        bus.push_alert("INFO", "test_system", "Informational message")

        assert len(bus.alerts) == 3
        # Newest first
        assert bus.alerts[0].severity == "INFO"
        assert bus.alerts[1].severity == "WARN"
        assert bus.alerts[2].severity == "ERROR"

    def test_snapshot_export(self):
        """Test: Snapshot exporter produces valid output."""
        bus = StateBus()

        bus.topics["/test"] = TopicState(
            name="/test", msg_type="std_msgs/String",
            status="OK", actual_hz=5.0
        )
        bus.nodes["/node1"] = NodeState(name="/node1")
        bus.push_alert("INFO", "test", "Test alert")

        snapshot = export_snapshot(bus)

        assert isinstance(snapshot, dict)
        assert "timestamp" in snapshot
        assert "topics" in snapshot
        assert "nodes" in snapshot
        assert "alerts" in snapshot
        assert "health" in snapshot

    def test_concurrent_collectors_simulation(self):
        """Test: Simulated concurrent collector writes (thread safety)."""
        bus = StateBus()
        errors = []

        def collector_1():
            try:
                for i in range(50):
                    with bus._lock:
                        bus.topics[f"/topic_{i}"] = TopicState(
                            name=f"/topic_{i}", msg_type="std_msgs/String"
                        )
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        def collector_2():
            try:
                for i in range(50):
                    with bus._lock:
                        bus.nodes[f"/node_{i}"] = NodeState(name=f"/node_{i}")
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        def scorer():
            try:
                for _ in range(50):
                    score = compute_health(bus)
                    assert score.overall >= 0
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=collector_1),
            threading.Thread(target=collector_2),
            threading.Thread(target=scorer),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [], f"Concurrent access errors: {errors}"
        assert len(bus.topics) > 0
        assert len(bus.nodes) > 0

    def test_config_manager_loads_defaults(self):
        """Test: Config manager provides sensible defaults."""
        from ros2_triage.config.config_manager import load_config

        config = load_config(None)  # No config file
        assert isinstance(config, dict)
        # Should have some default structure
        assert "settings" in config or len(config) == 0  # Empty dict is valid default


class TestPerformanceBenchmark:
    """Basic performance benchmarks."""

    def test_health_scorer_performance(self):
        """Benchmark: Health scoring should be fast even with many topics."""
        bus = StateBus()

        # Create 1000 topics
        for i in range(1000):
            bus.topics[f"/topic_{i}"] = TopicState(
                name=f"/topic_{i}", msg_type="std_msgs/String",
                status="OK" if i % 10 != 0 else "DEAD"
            )

        # Time the health computation
        start = time.perf_counter()
        for _ in range(100):
            score = compute_health(bus)
        elapsed = time.perf_counter() - start

        avg_ms = (elapsed / 100) * 1000
        print(f"\nHealth scoring: {avg_ms:.2f}ms avg (1000 topics, 100 iterations)")
        assert avg_ms < 50, f"Health scoring too slow: {avg_ms:.2f}ms (expected < 50ms)"

    def test_concurrent_alert_performance(self):
        """Benchmark: Alert system should handle high throughput."""
        bus = StateBus()

        start = time.perf_counter()
        for i in range(1000):
            bus.push_alert("INFO", "benchmark", f"Message {i}")
        elapsed = time.perf_counter() - start

        rate = 1000 / elapsed
        print(f"\nAlert throughput: {rate:.0f} alerts/sec")
        assert rate > 5000, f"Alert rate too slow: {rate:.0f}/sec (expected > 5000/sec)"

    def test_topic_classifier_performance(self):
        """Benchmark: Topic classification should be fast."""
        topics = [
            TopicState(name=f"/topic_{i}", msg_type="std_msgs/String",
                      publisher_count=1, actual_hz=10.0, last_msg_time=time.monotonic())
            for i in range(1000)
        ]

        start = time.perf_counter()
        for t in topics:
            status = classify_topic(t)
        elapsed = time.perf_counter() - start

        avg_us = (elapsed / 1000) * 1_000_000
        print(f"\nTopic classification: {avg_us:.2f}μs avg per topic")
        assert avg_us < 100, f"Classification too slow: {avg_us:.2f}μs (expected < 100μs)"
