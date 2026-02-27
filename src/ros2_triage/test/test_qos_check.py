# Copyright 2024 darshan — Apache-2.0
"""Unit tests for checks/qos_check.py — no live ROS system required."""

import pytest
from unittest.mock import MagicMock
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from ros2_triage.checks.qos_check import check_qos


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _make_endpoint(node_name: str, reliability, durability):
    ep = MagicMock()
    ep.node_name = node_name
    ep.qos_profile.reliability = reliability
    ep.qos_profile.durability = durability
    return ep


def _graph_with_qos(topic, pub_rel, pub_dur, sub_rel, sub_dur):
    pub = _make_endpoint('pub_node', pub_rel, pub_dur)
    sub = _make_endpoint('sub_node', sub_rel, sub_dur)
    return {
        topic: {
            'types': ['std_msgs/msg/String'],
            'publishers': [pub],
            'subscribers': [sub],
        }
    }


# ─── Tests ────────────────────────────────────────────────────────────────────

class TestReliabilityMismatch:

    def test_best_effort_pub_reliable_sub_is_incompatible(self):
        graph = _graph_with_qos(
            '/test_topic',
            ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE,
            ReliabilityPolicy.RELIABLE,    DurabilityPolicy.VOLATILE,
        )
        findings = check_qos(graph)
        rel_findings = [f for f in findings if 'Reliability' in f.message]
        assert len(rel_findings) == 1
        assert rel_findings[0].severity == 3

    def test_reliable_pub_best_effort_sub_is_compatible(self):
        """RELIABLE pub + BEST_EFFORT sub is actually fine in ROS 2."""
        graph = _graph_with_qos(
            '/test_topic',
            ReliabilityPolicy.RELIABLE,    DurabilityPolicy.VOLATILE,
            ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE,
        )
        findings = check_qos(graph)
        rel_findings = [f for f in findings if 'Reliability' in f.message]
        assert len(rel_findings) == 0

    def test_reliable_pub_reliable_sub_no_findings(self):
        graph = _graph_with_qos(
            '/test_topic',
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE,
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE,
        )
        findings = check_qos(graph)
        assert findings == []


class TestDurabilityMismatch:

    def test_volatile_pub_transient_sub_is_incompatible(self):
        graph = _graph_with_qos(
            '/test_topic',
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE,
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL,
        )
        findings = check_qos(graph)
        dur_findings = [f for f in findings if 'Durability' in f.message]
        assert len(dur_findings) == 1
        assert dur_findings[0].severity == 2

    def test_transient_pub_volatile_sub_is_compatible(self):
        graph = _graph_with_qos(
            '/test_topic',
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL,
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE,
        )
        findings = check_qos(graph)
        dur_findings = [f for f in findings if 'Durability' in f.message]
        assert len(dur_findings) == 0


class TestNoFindings:

    def test_fully_compatible_qos(self):
        graph = _graph_with_qos(
            '/healthy_topic',
            ReliabilityPolicy.RELIABLE,     DurabilityPolicy.TRANSIENT_LOCAL,
            ReliabilityPolicy.RELIABLE,     DurabilityPolicy.TRANSIENT_LOCAL,
        )
        findings = check_qos(graph)
        assert findings == []

    def test_topic_with_no_subscribers_skipped(self):
        """Dead topics are handled by dead_topic.py, not qos_check."""
        graph = {
            '/no_sub': {
                'types': ['std_msgs/msg/String'],
                'publishers': [_make_endpoint('pub', ReliabilityPolicy.RELIABLE,
                                              DurabilityPolicy.VOLATILE)],
                'subscribers': [],
            }
        }
        findings = check_qos(graph)
        assert findings == []
