# Copyright 2024 darshan — Apache-2.0
"""Tests for the Finding dataclass — JSON serialization and field validation."""

import json
import pytest
from ros2_triage.checks.finding import Finding


class TestFindingDataclass:

    def test_creation(self):
        f = Finding(
            check='dead_topics',
            topic='/cmd_vel',
            severity=3,
            message='No publishers',
            suggestion='Check launch files',
        )
        assert f.check == 'dead_topics'
        assert f.topic == '/cmd_vel'
        assert f.severity == 3

    def test_to_dict_includes_required_fields(self):
        f = Finding(
            check='qos', topic='/scan', severity=2,
            message='QoS mismatch', suggestion='Fix it',
        )
        d = f.to_dict()
        assert set(d.keys()) >= {'check', 'topic', 'severity', 'message', 'suggestion'}

    def test_to_dict_no_extra_field_when_empty(self):
        f = Finding(
            check='tf', topic='tf:odom', severity=1,
            message='Frame missing', suggestion='Run robot_state_publisher',
        )
        d = f.to_dict()
        assert 'extra' not in d

    def test_to_dict_extra_field_when_populated(self):
        f = Finding(
            check='qos', topic='/scan', severity=2,
            message='msg', suggestion='fix',
            extra={'pub_node': 'lidar', 'sub_node': 'slam'},
        )
        d = f.to_dict()
        assert 'extra' in d
        assert d['extra']['pub_node'] == 'lidar'

    def test_json_serializable(self):
        f = Finding(
            check='dead_topics', topic='/odom', severity=3,
            message='No publishers', suggestion='Check localization nodes',
        )
        json_str = json.dumps(f.to_dict())
        assert '/odom' in json_str
        assert '3' in json_str


class TestSeverityConstants:
    def test_severity_labels(self):
        from ros2_triage.checks.finding import SEVERITY_LABEL
        assert SEVERITY_LABEL[1] == 'INFO'
        assert SEVERITY_LABEL[2] == 'WARN'
        assert SEVERITY_LABEL[3] == 'CRIT'

    def test_critical_topics_contains_cmd_vel(self):
        from ros2_triage.checks.finding import CRITICAL_TOPICS
        assert '/cmd_vel' in CRITICAL_TOPICS

    def test_noisy_topics_contains_rosout(self):
        from ros2_triage.checks.finding import NOISY_TOPICS
        assert '/rosout' in NOISY_TOPICS
