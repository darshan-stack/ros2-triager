# test/test_state_bus.py
"""
Unit tests for state.state_bus.StateBus.
No ROS2 runtime needed — pure Python.
"""
import threading
import time
import pytest
from ros2_triage.state.state_bus import (
    StateBus, TopicState, NodeState, TFState, DiagState, DiagItem,
    LifecycleState, OdomState, Alert, HealthScore,
)


class TestStateBusInit:
    def test_default_state_empty(self):
        bus = StateBus()
        assert bus.topics == {}
        assert bus.nodes == {}
        assert bus.lifecycle == {}
        assert len(bus.alerts) == 0
        assert bus.diag.error_count == 0
        assert bus.diag.warn_count == 0

    def test_health_starts_at_100(self):
        bus = StateBus()
        assert bus.health.overall == 100
        assert bus.health.topics_score == 100
        assert bus.health.nodes_score == 100

    def test_uptime_increases(self):
        bus = StateBus()
        t0 = bus.uptime_seconds()
        time.sleep(0.05)
        t1 = bus.uptime_seconds()
        assert t1 > t0

    def test_odom_default_zero(self):
        bus = StateBus()
        assert bus.odom.x == 0.0
        assert bus.odom.y == 0.0
        assert bus.odom.qw == 1.0  # unit quaternion


class TestStateBusPushAlert:
    def test_push_alert_adds_to_deque(self):
        bus = StateBus()
        bus.push_alert("ERROR", "test", "something failed")
        assert len(bus.alerts) == 1
        alert = bus.alerts[0]
        assert alert.severity == "ERROR"
        assert alert.source == "test"
        assert alert.message == "something failed"
        assert alert.timestamp > 0

    def test_push_alert_newest_first(self):
        bus = StateBus()
        bus.push_alert("INFO", "src1", "first")
        bus.push_alert("WARN", "src2", "second")
        assert bus.alerts[0].message == "second"
        assert bus.alerts[1].message == "first"

    def test_alert_deque_maxlen_500(self):
        bus = StateBus()
        for i in range(510):
            bus.push_alert("INFO", "test", f"msg_{i}")
        assert len(bus.alerts) == 500


class TestStateBusLockThreadSafety:
    def test_concurrent_writes_no_crash(self):
        """Writes from multiple threads must not crash or corrupt state."""
        bus = StateBus()
        errors = []

        def writer(tid: int):
            try:
                for i in range(100):
                    with bus._lock:
                        bus.topics[f"/topic_{tid}_{i}"] = TopicState(
                            name=f"/topic_{tid}_{i}", msg_type="std_msgs/String"
                        )
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [], f"Thread errors: {errors}"
        assert len(bus.topics) > 0

    def test_lock_is_reentrant(self):
        """RLock allows acquiring the same lock twice in the same thread."""
        bus = StateBus()
        with bus._lock:
            with bus._lock:  # Should NOT deadlock
                bus.push_alert("INFO", "test", "nested lock OK")
        assert len(bus.alerts) == 1

    def test_concurrent_push_alert_no_crash(self):
        bus = StateBus()
        errors = []

        def pusher(tid: int):
            try:
                for i in range(50):
                    bus.push_alert("WARN", f"src_{tid}", f"msg_{tid}_{i}")
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=pusher, args=(i,)) for i in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == [], f"Thread errors: {errors}"
        assert len(bus.alerts) <= 500  # Bounded by maxlen


class TestTopicState:
    def test_hz_history_maxlen_60(self):
        ts = TopicState(name="/foo", msg_type="std_msgs/String")
        for i in range(70):
            ts.hz_history.append(float(i))
        assert len(ts.hz_history) == 60

    def test_status_defaults_unknown(self):
        ts = TopicState(name="/foo", msg_type="std_msgs/String")
        assert ts.status == "UNKNOWN"


class TestHealthScoreDataclass:
    def test_overall_0_is_critical(self):
        h = HealthScore(overall=0)
        assert h.label == "CRITICAL"
        assert h.color == "red"

    def test_overall_100_is_healthy(self):
        h = HealthScore(overall=100)
        assert h.label == "HEALTHY"
        assert h.color == "green"
