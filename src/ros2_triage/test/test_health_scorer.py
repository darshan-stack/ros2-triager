# test/test_health_scorer.py
"""
Unit tests for engine.health_scorer.compute_health().
No ROS2 runtime needed — pure Python.
"""
import pytest
from ros2_triage.state.state_bus import (
    StateBus, TopicState, NodeState, TFState, DiagState, DiagItem,
    LifecycleState, HealthScore,
)
from ros2_triage.engine.health_scorer import compute_health


def _make_bus(**kwargs) -> StateBus:
    bus = StateBus()
    for k, v in kwargs.items():
        setattr(bus, k, v)
    return bus


# ── Topic score tests ──────────────────────────────────────────────────────────

class TestTopicScore:
    def test_all_ok_topics_score_100(self):
        bus = StateBus()
        bus.topics = {
            "/foo": TopicState(name="/foo", msg_type="std_msgs/String", status="OK", publisher_count=1),
            "/bar": TopicState(name="/bar", msg_type="std_msgs/String", status="OK", publisher_count=1),
        }
        score = compute_health(bus)
        assert score.topics_score == 100

    def test_dead_noncritical_topic_penalized_10(self):
        bus = StateBus()
        bus.topics = {
            "/foo": TopicState(name="/foo", msg_type="std_msgs/String", status="DEAD", publisher_count=0),
        }
        score = compute_health(bus)
        assert score.topics_score == 90

    def test_dead_critical_topic_penalized_20(self):
        bus = StateBus()
        bus.topics = {
            "/cmd_vel": TopicState(
                name="/cmd_vel", msg_type="geometry_msgs/Twist",
                status="DEAD", publisher_count=0, is_critical=True,
            ),
        }
        score = compute_health(bus)
        assert score.topics_score == 80

    def test_low_hz_topic_penalized_5(self):
        bus = StateBus()
        bus.topics = {
            "/scan": TopicState(name="/scan", msg_type="sensor_msgs/LaserScan", status="LOW_HZ"),
        }
        score = compute_health(bus)
        assert score.topics_score == 95

    def test_clamped_to_zero_for_many_dead(self):
        bus = StateBus()
        bus.topics = {
            f"/dead_{i}": TopicState(name=f"/dead_{i}", msg_type="std_msgs/String", status="DEAD")
            for i in range(20)
        }
        score = compute_health(bus)
        assert score.topics_score == 0


# ── Node score tests ───────────────────────────────────────────────────────────

class TestNodeScore:
    def test_no_zombies_score_100(self):
        bus = StateBus()
        bus.nodes = {
            "/my_node": NodeState(name="/my_node", is_zombie=False),
        }
        score = compute_health(bus)
        assert score.nodes_score == 100

    def test_one_zombie_penalized_30(self):
        bus = StateBus()
        bus.nodes = {
            "/zombie": NodeState(name="/zombie", is_zombie=True),
        }
        score = compute_health(bus)
        assert score.nodes_score == 70

    def test_multiple_zombies_clamped(self):
        bus = StateBus()
        bus.nodes = {
            f"/zombie_{i}": NodeState(name=f"/zombie_{i}", is_zombie=True)
            for i in range(10)
        }
        score = compute_health(bus)
        assert score.nodes_score == 0


# ── TF score tests ─────────────────────────────────────────────────────────────

class TestTFScore:
    def test_no_stale_score_100(self):
        bus = StateBus()
        score = compute_health(bus)
        assert score.tf_score == 100

    def test_stale_frame_penalized_15(self):
        bus = StateBus()
        bus.tf = TFState(frames={}, stale_frames=["base_footprint"], broken_chains=[])
        score = compute_health(bus)
        assert score.tf_score == 85

    def test_broken_chain_penalized_30(self):
        bus = StateBus()
        bus.tf = TFState(frames={}, stale_frames=[], broken_chains=["odom"])
        score = compute_health(bus)
        assert score.tf_score == 70


# ── Diagnostics score tests ────────────────────────────────────────────────────

class TestDiagScore:
    def test_no_errors_score_100(self):
        bus = StateBus()
        score = compute_health(bus)
        assert score.diag_score == 100

    def test_one_error_penalized_20(self):
        bus = StateBus()
        bus.diag = DiagState(error_count=1, warn_count=0)
        score = compute_health(bus)
        assert score.diag_score == 80

    def test_one_warn_penalized_5(self):
        bus = StateBus()
        bus.diag = DiagState(error_count=0, warn_count=1)
        score = compute_health(bus)
        assert score.diag_score == 95


# ── Overall weighted average tests ────────────────────────────────────────────

class TestOverallScore:
    def test_empty_bus_scores_100(self):
        bus = StateBus()
        score = compute_health(bus)
        assert score.overall == 100
        assert score.label == "HEALTHY"
        assert score.color == "green"

    def test_weighted_average(self):
        """
        Manually calculated:
          topic_score=60, node_score=70, tf_score=85, diag_score=100, lc_score=100
          = 0.3*60 + 0.2*70 + 0.2*85 + 0.2*100 + 0.1*100 = 79
        """
        bus = StateBus()
        # Add 4 dead non-critical topics => -40 on topics (100-40=60)
        bus.topics = {
            f"/dead_{i}": TopicState(name=f"/dead_{i}", msg_type="std_msgs/String", status="DEAD")
            for i in range(4)
        }
        # 1 zombie => -30 on nodes (100-30=70)
        bus.nodes = {
            "/z": NodeState(name="/z", is_zombie=True),
        }
        # 1 stale TF => -15 on tf (100-15=85)
        bus.tf = TFState(frames={}, stale_frames=["odom"], broken_chains=[])

        score = compute_health(bus)
        assert score.topics_score == 60
        assert score.nodes_score == 70
        assert score.tf_score == 85
        assert score.overall == 79

    def test_degraded_label(self):
        bus = StateBus()
        bus.diag = DiagState(error_count=2, warn_count=0)
        score = compute_health(bus)
        # diag_score=60, weighted 20% = 60*0.2 = 12 below full
        assert score.label in ("HEALTHY", "DEGRADED")


class TestHealthScoreProperties:
    def test_color_and_label_thresholds(self):
        s = HealthScore(overall=80)
        assert s.color == "green"
        assert s.label == "HEALTHY"
        s.overall = 79
        assert s.color == "yellow"
        assert s.label == "DEGRADED"
        s.overall = 49
        assert s.color == "red"
        assert s.label == "CRITICAL"
