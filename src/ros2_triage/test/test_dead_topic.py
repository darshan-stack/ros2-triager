# Copyright 2024 darshan — Apache-2.0
"""
Unit tests for checks/dead_topic.py
Tests use synthetic graph data — no live ROS system required.
"""

import pytest
from ros2_triage.checks.dead_topic import check_dead_topics


# ─── Helpers ──────────────────────────────────────────────────────────────────

class FakeEndpoint:
    """Minimal mock of rclpy TopicEndpointInfo."""
    def __init__(self, node_name: str):
        self.node_name = node_name


def _graph(topic: str, pub_nodes: list, sub_nodes: list) -> dict:
    """Build a minimal synthetic graph dict for a single topic."""
    return {
        topic: {
            'types': ['std_msgs/msg/String'],
            'publishers':  [FakeEndpoint(n) for n in pub_nodes],
            'subscribers': [FakeEndpoint(n) for n in sub_nodes],
        }
    }


# ─── Tests ────────────────────────────────────────────────────────────────────

class TestUnpublishedTopics:
    """Topics with subscribers but no publishers."""

    def test_critical_topic_gets_severity_3(self):
        graph = _graph('/cmd_vel', pub_nodes=[], sub_nodes=['nav2_node'])
        findings = check_dead_topics(graph)
        assert len(findings) == 1
        assert findings[0].severity == 3
        assert findings[0].topic == '/cmd_vel'
        assert 'UNPUBLISHED' in findings[0].message

    def test_unknown_topic_gets_severity_2(self):
        """Unknown custom topics get WARN (2) — any dead topic in a real robot warrants attention."""
        graph = _graph('/my_custom_topic', pub_nodes=[], sub_nodes=['my_node'])
        findings = check_dead_topics(graph)
        assert len(findings) == 1
        assert findings[0].severity == 2  # WARN — better safe than sorry on real robots

    def test_nav_keyword_gets_severity_2(self):
        graph = _graph('/navigation_status', pub_nodes=[], sub_nodes=['rviz'])
        findings = check_dead_topics(graph)
        assert len(findings) == 1
        assert findings[0].severity == 2

    def test_subscriber_node_name_in_message(self):
        graph = _graph('/scan', pub_nodes=[], sub_nodes=['lidar_processor'])
        findings = check_dead_topics(graph)
        assert 'lidar_processor' in findings[0].message


class TestUnsubscribedTopics:
    """Topics with publishers but no subscribers."""

    def test_unsubscribed_topic_detected(self):
        graph = _graph('/sensor_data', pub_nodes=['sensor_node'], sub_nodes=[])
        findings = check_dead_topics(graph)
        assert len(findings) == 1
        assert 'UNSUBSCRIBED' in findings[0].message

    def test_publisher_node_name_in_message(self):
        graph = _graph('/my_output', pub_nodes=['processor_node'], sub_nodes=[])
        findings = check_dead_topics(graph)
        assert 'processor_node' in findings[0].message


class TestHealthyTopics:
    """Topics with both publishers and subscribers — no findings expected."""

    def test_healthy_topic_no_findings(self):
        graph = _graph('/cmd_vel', pub_nodes=['joy_node'], sub_nodes=['base_node'])
        findings = check_dead_topics(graph)
        assert findings == []

    def test_multiple_pubs_and_subs_no_findings(self):
        graph = _graph('/scan',
                       pub_nodes=['lidar_a', 'lidar_b'],
                       sub_nodes=['slam_node', 'nav_node'])
        findings = check_dead_topics(graph)
        assert findings == []


class TestNoisyTopics:
    """Infrastructure topics should be skipped by default."""

    def test_rosout_skipped(self):
        graph = _graph('/rosout', pub_nodes=[], sub_nodes=['some_node'])
        findings = check_dead_topics(graph, skip_noisy=True)
        assert findings == []

    def test_parameter_events_skipped(self):
        graph = _graph('/parameter_events', pub_nodes=['node_a'], sub_nodes=[])
        findings = check_dead_topics(graph, skip_noisy=True)
        assert findings == []

    def test_noisy_topic_included_when_skip_false(self):
        graph = _graph('/rosout', pub_nodes=[], sub_nodes=['node_a'])
        findings = check_dead_topics(graph, skip_noisy=False)
        assert len(findings) == 1


class TestSeveritySorting:
    """Findings should be sorted by severity descending."""

    def test_higher_severity_first(self):
        graph = {}
        graph['/cmd_vel'] = {
            'types': ['std_msgs/msg/Twist'],
            'publishers': [],
            'subscribers': [FakeEndpoint('nav')],
        }
        graph['/my_debug'] = {
            'types': ['std_msgs/msg/String'],
            'publishers': [],
            'subscribers': [FakeEndpoint('debug_node')],
        }
        findings = check_dead_topics(graph)
        # /cmd_vel (sev 3) must come before /my_debug (sev 1)
        sev_list = [f.severity for f in findings]
        assert sev_list == sorted(sev_list, reverse=True)


class TestFindingStructure:
    """Verify Finding fields are populated correctly."""

    def test_finding_check_name(self):
        graph = _graph('/scan', pub_nodes=[], sub_nodes=['slam_node'])
        findings = check_dead_topics(graph)
        assert findings[0].check == 'dead_topics'

    def test_finding_has_suggestion(self):
        graph = _graph('/scan', pub_nodes=[], sub_nodes=['slam_node'])
        findings = check_dead_topics(graph)
        assert len(findings[0].suggestion) > 0

    def test_finding_to_dict(self):
        graph = _graph('/scan', pub_nodes=[], sub_nodes=['slam_node'])
        findings = check_dead_topics(graph)
        d = findings[0].to_dict()
        assert 'check' in d
        assert 'topic' in d
        assert 'severity' in d
        assert 'message' in d
        assert 'suggestion' in d
