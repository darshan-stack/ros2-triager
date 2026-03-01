# Copyright 2024 darshan - Apache-2.0
"""Unit tests for correlation_engine.py - hypothesis generation and evidence scoring."""

import pytest
from unittest.mock import MagicMock, patch
from ros2_triage.correlation_engine import (
    Evidence,
    Hypothesis,
    ProcessMonitor,
    LogGrepper,
    CorrelationEngine,
)


class TestEvidence:
    """Test Evidence dataclass and scoring."""

    def test_score_calculation(self):
        e = Evidence(
            source='graph',
            signal='topic_silent',
            value=1.0,
            weight=0.4,
            details='Topic has no publishers'
        )
        assert e.score() == pytest.approx(0.4)

    def test_score_with_partial_value(self):
        e = Evidence(
            source='os',
            signal='cpu_high',
            value=0.5,
            weight=0.3,
            details='CPU at 50%'
        )
        assert e.score() == pytest.approx(0.15)

    def test_score_zero_weight(self):
        e = Evidence(
            source='logs',
            signal='log_error',
            value=1.0,
            weight=0.0,
            details='Error found'
        )
        assert e.score() == 0.0


class TestHypothesis:
    """Test Hypothesis confidence calculations."""

    def test_confidence_empty_evidence(self):
        h = Hypothesis(
            root_cause='unknown',
            topic='/test',
            severity=2,
            evidence=[]
        )
        assert h.confidence() == 0.0
        assert not h.is_high_confidence()

    def test_confidence_single_evidence(self):
        h = Hypothesis(
            root_cause='process_crash',
            topic='/cmd_vel',
            severity=3,
            evidence=[
                Evidence('graph', 'topic_silent', 1.0, 0.4, 'No publishers')
            ]
        )
        assert h.confidence() == pytest.approx(0.4)
        assert not h.is_high_confidence()

    def test_confidence_multiple_evidence(self):
        h = Hypothesis(
            root_cause='process_crash',
            topic='/cmd_vel',
            severity=3,
            evidence=[
                Evidence('graph', 'topic_silent', 1.0, 0.4, 'No publishers'),
                Evidence('os', 'process_missing', 1.0, 0.5, 'Node not running'),
            ]
        )
        assert h.confidence() == pytest.approx(0.9)
        assert h.is_high_confidence()

    def test_confidence_capped_at_1(self):
        h = Hypothesis(
            root_cause='catastrophic_failure',
            topic='/scan',
            severity=3,
            evidence=[
                Evidence('graph', 'topic_silent', 1.0, 0.5, 'No publishers'),
                Evidence('os', 'process_missing', 1.0, 0.5, 'Node not running'),
                Evidence('logs', 'process_crash', 1.0, 0.5, 'Segfault in logs'),
            ]
        )
        assert h.confidence() == 1.0

    def test_high_confidence_threshold(self):
        h = Hypothesis(
            root_cause='resource_starvation',
            topic='/odom',
            severity=2,
            evidence=[
                Evidence('graph', 'topic_slow', 0.8, 0.4, 'Publishing slowly'),
                Evidence('os', 'cpu_saturation', 0.9, 0.5, 'CPU at 95%'),
            ]
        )
        # 0.8*0.4 + 0.9*0.5 = 0.32 + 0.45 = 0.77
        assert h.confidence() == pytest.approx(0.77)
        assert not h.is_high_confidence()  # below 0.85


class TestProcessMonitor:
    """Test ProcessMonitor without requiring psutil."""

    def test_find_node_pid_without_psutil(self):
        with patch('ros2_triage.correlation_engine.HAS_PSUTIL', False):
            monitor = ProcessMonitor()
            pid = monitor.find_node_pid('some_node')
            assert pid is None

    def test_get_process_snapshot_without_psutil(self):
        with patch('ros2_triage.correlation_engine.HAS_PSUTIL', False):
            monitor = ProcessMonitor()
            snapshot = monitor.get_process_snapshot(12345)
            assert snapshot is None


class TestLogGrepper:
    """Test LogGrepper pattern matching."""

    def test_error_patterns_defined(self):
        grepper = LogGrepper()
        assert len(grepper.ERROR_PATTERNS) > 0
        # Check pattern format: (regex, type, confidence)
        for pattern, error_type, confidence in grepper.ERROR_PATTERNS:
            assert isinstance(pattern, str)
            assert isinstance(error_type, str)
            assert 0.0 <= confidence <= 1.0

    @patch('subprocess.run')
    def test_search_logs_handles_timeout(self, mock_run):
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired(cmd='journalctl', timeout=2)
        
        grepper = LogGrepper()
        findings = grepper.search_logs('some_node')
        assert findings == []

    @patch('subprocess.run')
    def test_search_logs_handles_missing_journalctl(self, mock_run):
        mock_run.side_effect = FileNotFoundError()
        
        grepper = LogGrepper()
        findings = grepper.search_logs('some_node')
        assert findings == []


class TestCorrelationEngine:
    """Test CorrelationEngine hypothesis generation."""

    def test_analyze_unpublished_topic_returns_hypothesis(self):
        engine = CorrelationEngine()
        graph = {
            '/cmd_vel': {
                'types': ['geometry_msgs/msg/Twist'],
                'publishers': [],
                'subscribers': [MagicMock(node_name='nav2_controller')],
            }
        }
        
        hyp = engine.analyze_unpublished_topic(
            '/cmd_vel',
            ['nav2_controller'],
            graph
        )
        
        assert isinstance(hyp, Hypothesis)
        assert hyp.topic == '/cmd_vel'
        assert len(hyp.evidence) >= 1
        # Should have at least the graph evidence
        graph_evidence = [e for e in hyp.evidence if e.source == 'graph']
        assert len(graph_evidence) >= 1

    def test_analyze_qos_mismatch_returns_hypothesis(self):
        engine = CorrelationEngine()
        
        hyp = engine.analyze_qos_mismatch(
            '/scan',
            'lidar_node',
            'slam_node',
            'RELIABILITY'
        )
        
        assert isinstance(hyp, Hypothesis)
        assert hyp.topic == '/scan'
        assert hyp.root_cause == 'qos_configuration_error'
        assert hyp.severity == 3
        # Should have QoS mismatch evidence
        qos_evidence = [e for e in hyp.evidence if e.signal == 'qos_incompatible']
        assert len(qos_evidence) == 1

    def test_process_monitor_integration(self):
        engine = CorrelationEngine()
        assert isinstance(engine.process_monitor, ProcessMonitor)
        assert isinstance(engine.log_grepper, LogGrepper)
