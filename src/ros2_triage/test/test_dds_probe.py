# Copyright 2024 darshan - Apache-2.0
"""Unit tests for dds_probe.py - DDS domain conflict detection."""

import pytest
import os
from unittest.mock import MagicMock, patch
from ros2_triage.checks.dds_probe import (
    DDSDomainInfo,
    DDS_PORT_BASE,
    DDS_DOMAIN_GAIN,
    calculate_dds_ports,
    get_ros_domain_id,
    check_port_in_use,
    get_port_processes,
    probe_domain,
    scan_domains,
    check_dds_domain,
)


class TestDDSPortCalculation:
    """Test DDS port formula implementation."""

    def test_domain_0_discovery_multicast(self):
        ports = calculate_dds_ports(0)
        # PB + DG * d + d0 = 7400 + 250 * 0 + 0 = 7400
        assert ports['discovery_multicast'] == 7400

    def test_domain_0_user_multicast(self):
        ports = calculate_dds_ports(0)
        # PB + DG * d + d2 = 7400 + 250 * 0 + 1 = 7401
        assert ports['user_multicast'] == 7401

    def test_domain_1_ports(self):
        ports = calculate_dds_ports(1)
        # Discovery multicast: 7400 + 250 * 1 + 0 = 7650
        assert ports['discovery_multicast'] == 7650
        # User multicast: 7400 + 250 * 1 + 1 = 7651
        assert ports['user_multicast'] == 7651

    def test_domain_42_ports(self):
        ports = calculate_dds_ports(42)
        # Discovery multicast: 7400 + 250 * 42 + 0 = 17900
        assert ports['discovery_multicast'] == 17900

    def test_participant_id_offset(self):
        ports_p0 = calculate_dds_ports(0, participant_id=0)
        ports_p1 = calculate_dds_ports(0, participant_id=1)
        # Participant gain = 2
        assert ports_p1['discovery_unicast'] == ports_p0['discovery_unicast'] + 2
        assert ports_p1['user_unicast'] == ports_p0['user_unicast'] + 2


class TestROSDomainID:
    """Test ROS_DOMAIN_ID environment variable handling."""

    def test_default_domain_id(self):
        with patch.dict(os.environ, {}, clear=True):
            if 'ROS_DOMAIN_ID' in os.environ:
                del os.environ['ROS_DOMAIN_ID']
            assert get_ros_domain_id() == 0

    def test_custom_domain_id(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '42'}):
            assert get_ros_domain_id() == 42

    def test_invalid_domain_id_falls_back(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': 'not_a_number'}):
            assert get_ros_domain_id() == 0


class TestDDSDomainInfo:
    """Test DDSDomainInfo dataclass."""

    def test_creation(self):
        info = DDSDomainInfo(
            domain_id=0,
            discovery_port=7400,
            user_port_base=7401,
        )
        assert info.domain_id == 0
        assert info.is_active is False
        assert info.processes == []

    def test_active_domain(self):
        info = DDSDomainInfo(
            domain_id=1,
            discovery_port=7650,
            user_port_base=7651,
            is_active=True,
            processes=['12345/ros2'],
        )
        assert info.is_active is True
        assert '12345/ros2' in info.processes


class TestPortChecking:
    """Test port availability checking."""

    @patch('socket.socket')
    def test_check_port_not_in_use(self, mock_socket_class):
        mock_sock = MagicMock()
        mock_socket_class.return_value.__enter__ = MagicMock(return_value=mock_sock)
        mock_socket_class.return_value.__exit__ = MagicMock(return_value=False)
        mock_sock.bind.return_value = None
        
        # Port should be available if bind succeeds
        result = check_port_in_use(12345, 'udp')
        # Function returns False when bind succeeds (port not in use)
        assert result is False

    @patch('subprocess.run')
    def test_get_port_processes_handles_timeout(self, mock_run):
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired(cmd='netstat', timeout=5)
        
        processes = get_port_processes(7400)
        assert processes == []

    @patch('subprocess.run')
    def test_get_port_processes_handles_missing_netstat(self, mock_run):
        mock_run.side_effect = FileNotFoundError()
        
        processes = get_port_processes(7400)
        assert processes == []


class TestProbeDomain:
    """Test domain probing functionality."""

    def test_probe_returns_domain_info(self):
        info = probe_domain(0)
        assert isinstance(info, DDSDomainInfo)
        assert info.domain_id == 0
        assert info.discovery_port == 7400

    def test_probe_different_domains(self):
        info0 = probe_domain(0)
        info1 = probe_domain(1)
        assert info0.discovery_port != info1.discovery_port


class TestScanDomains:
    """Test multi-domain scanning."""

    def test_scan_returns_list(self):
        results = scan_domains(max_domains=3)
        assert isinstance(results, list)
        assert len(results) <= 3

    def test_scan_includes_current_domain(self):
        with patch('ros2_triage.checks.dds_probe.get_ros_domain_id', return_value=5):
            results = scan_domains(max_domains=10)
            domain_ids = [r.domain_id for r in results]
            assert 5 in domain_ids


class TestCheckDDSDomain:
    """Test check_dds_domain finding generation."""

    def test_returns_list_of_findings(self):
        findings = check_dds_domain()
        assert isinstance(findings, list)
        # All items should be Finding objects
        from ros2_triage.checks.finding import Finding
        for f in findings:
            assert isinstance(f, Finding)

    def test_respects_ignore_set(self):
        # Should not crash with ignore set
        findings = check_dds_domain(ignore_set={'dds_probe'})
        assert isinstance(findings, list)


class TestDDSConstants:
    """Test DDS protocol constants."""

    def test_port_base(self):
        assert DDS_PORT_BASE == 7400

    def test_domain_gain(self):
        assert DDS_DOMAIN_GAIN == 250
