# Copyright 2024 darshan - Apache-2.0
"""Unit tests for reporter.py - output formatting."""

import json
import io
import pytest
from ros2_triage.reporter import (
    print_human,
    print_json,
    print_report,
    SECTION,
    _group,
    HAS_RICH,
    HAS_COLOR,
)
from ros2_triage.checks.finding import Finding


class TestGroupFunction:
    """Test finding grouping by check name."""

    def test_empty_findings(self):
        groups = _group([])
        assert groups == {}

    def test_single_finding(self):
        f = Finding('dead_topics', '/scan', 3, 'msg', 'fix')
        groups = _group([f])
        assert 'dead_topics' in groups
        assert len(groups['dead_topics']) == 1

    def test_multiple_checks(self):
        findings = [
            Finding('dead_topics', '/scan', 3, 'msg', 'fix'),
            Finding('qos', '/odom', 2, 'msg', 'fix'),
            Finding('dead_topics', '/cmd_vel', 3, 'msg', 'fix'),
        ]
        groups = _group(findings)
        assert len(groups['dead_topics']) == 2
        assert len(groups['qos']) == 1


class TestSectionMetadata:
    """Test section header definitions."""

    def test_dead_topics_section(self):
        assert 'dead_topics' in SECTION
        header, desc = SECTION['dead_topics']
        assert 'DEAD TOPICS' in header

    def test_qos_section(self):
        assert 'qos' in SECTION
        header, desc = SECTION['qos']
        assert 'QoS' in header

    def test_tf_section(self):
        assert 'tf' in SECTION
        header, desc = SECTION['tf']
        assert 'TF' in header

    def test_latency_section(self):
        assert 'latency' in SECTION
        header, desc = SECTION['latency']
        assert 'LATENCY' in header

    def test_dds_domain_section(self):
        assert 'dds_domain' in SECTION
        header, desc = SECTION['dds_domain']
        assert 'DDS' in header


class TestPrintHuman:
    """Test human-readable output."""

    def test_no_findings_shows_ok(self):
        stream = io.StringIO()
        print_human([], stream=stream)
        output = stream.getvalue()
        # Should indicate no issues
        assert 'No issues' in output or 'OK' in output

    def test_finding_appears_in_output(self):
        findings = [Finding('dead_topics', '/scan', 3, 'No publishers', 'Check launch')]
        stream = io.StringIO()
        print_human(findings, stream=stream)
        output = stream.getvalue()
        assert '/scan' in output
        assert 'No publishers' in output

    def test_severity_filter(self):
        findings = [
            Finding('dead_topics', '/scan', 1, 'INFO level', 'fix'),
            Finding('qos', '/odom', 3, 'CRIT level', 'fix'),
        ]
        stream = io.StringIO()
        print_human(findings, severity_threshold=3, stream=stream)
        output = stream.getvalue()
        # Only severity 3 should appear
        assert 'CRIT level' in output
        # INFO should be filtered
        assert 'INFO level' not in output

    def test_simulation_mode_tag(self):
        findings = [Finding('dead_topics', '/scan', 3, 'msg', 'fix')]
        stream = io.StringIO()
        print_human(findings, simulation_mode=True, stream=stream)
        output = stream.getvalue()
        assert 'SIMULATION' in output

    def test_ignored_topics_shown(self):
        stream = io.StringIO()
        print_human([], ignored={'/ignored_topic'}, stream=stream)
        output = stream.getvalue()
        assert 'ignored' in output.lower()


class TestPrintJson:
    """Test JSON output format."""

    def test_valid_json(self):
        findings = [Finding('dead_topics', '/scan', 3, 'No pub', 'Fix it')]
        # Capture stdout
        import sys
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            print_json(findings)
            output = sys.stdout.getvalue()
        finally:
            sys.stdout = old_stdout
        
        # Should be valid JSON
        data = json.loads(output)
        assert 'findings' in data

    def test_findings_in_json(self):
        findings = [
            Finding('dead_topics', '/scan', 3, 'No pub', 'Fix'),
            Finding('qos', '/odom', 2, 'Mismatch', 'Fix'),
        ]
        import sys
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            print_json(findings)
            output = sys.stdout.getvalue()
        finally:
            sys.stdout = old_stdout
        
        data = json.loads(output)
        assert len(data['findings']) == 2

    def test_severity_filter_json(self):
        findings = [
            Finding('dead_topics', '/scan', 1, 'INFO', 'fix'),
            Finding('qos', '/odom', 3, 'CRIT', 'fix'),
        ]
        import sys
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            print_json(findings, severity_threshold=3)
            output = sys.stdout.getvalue()
        finally:
            sys.stdout = old_stdout
        
        data = json.loads(output)
        # Only severity 3 should be included
        assert len(data['findings']) == 1
        assert data['findings'][0]['severity'] == 3


class TestPrintReport:
    """Test print_report dispatcher."""

    def test_fallback_to_human(self):
        # print_report should not crash
        findings = [Finding('dead_topics', '/scan', 3, 'msg', 'fix')]
        # Just verify it doesn't crash
        import sys
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            print_report(findings, use_rich=False)
        finally:
            sys.stdout = old_stdout


class TestLibraryDetection:
    """Test optional library detection."""

    def test_has_rich_is_boolean(self):
        assert isinstance(HAS_RICH, bool)

    def test_has_color_is_boolean(self):
        assert isinstance(HAS_COLOR, bool)
