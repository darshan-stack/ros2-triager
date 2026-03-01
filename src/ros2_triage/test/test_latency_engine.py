# Copyright 2024 darshan - Apache-2.0
"""Unit tests for latency_engine.py - timing analysis for stamped messages."""

import pytest
import time
from unittest.mock import MagicMock, patch
from ros2_triage.checks.latency_engine import (
    LatencyStats,
    LatencyThreshold,
    LatencyMeasurer,
    DEFAULT_THRESHOLDS,
)


class TestLatencyStats:
    """Test LatencyStats dataclass and statistics updates."""

    def test_initial_state(self):
        stats = LatencyStats(topic='/test')
        assert stats.topic == '/test'
        assert stats.sample_count == 0
        assert stats.min_latency_ms == float('inf')
        assert stats.max_latency_ms == 0.0
        assert stats.mean_latency_ms == 0.0
        assert stats.samples == []

    def test_single_update(self):
        stats = LatencyStats(topic='/scan')
        stats.update(10.0)
        
        assert stats.sample_count == 1
        assert stats.min_latency_ms == 10.0
        assert stats.max_latency_ms == 10.0
        assert stats.mean_latency_ms == 10.0

    def test_multiple_updates(self):
        stats = LatencyStats(topic='/odom')
        stats.update(10.0)
        stats.update(20.0)
        stats.update(30.0)
        
        assert stats.sample_count == 3
        assert stats.min_latency_ms == 10.0
        assert stats.max_latency_ms == 30.0
        assert stats.mean_latency_ms == pytest.approx(20.0)

    def test_std_deviation(self):
        stats = LatencyStats(topic='/imu')
        stats.update(10.0)
        stats.update(20.0)
        stats.update(30.0)
        
        # std of [10, 20, 30] = 10.0
        assert stats.std_latency_ms == pytest.approx(10.0)

    def test_inter_arrival_times(self):
        stats = LatencyStats(topic='/cmd_vel')
        stats.update(10.0, inter_arrival_ms=100.0)
        stats.update(15.0, inter_arrival_ms=105.0)
        stats.update(12.0, inter_arrival_ms=95.0)
        
        assert len(stats.inter_arrival_times) == 3
        assert 100.0 in stats.inter_arrival_times

    def test_jitter_calculation(self):
        stats = LatencyStats(topic='/pointcloud')
        # Inter-arrival times: 100, 100, 100 (no jitter)
        stats.update(10.0, inter_arrival_ms=100.0)
        stats.update(10.0, inter_arrival_ms=100.0)
        stats.update(10.0, inter_arrival_ms=100.0)
        
        # Mean = 100, all deviations = 0, so jitter = 0
        assert stats.jitter_ms == pytest.approx(0.0)

    def test_jitter_with_variance(self):
        stats = LatencyStats(topic='/image')
        # Inter-arrival times: 100, 120, 80 -> mean = 100, deviations: 0, 20, 20
        stats.update(10.0, inter_arrival_ms=100.0)
        stats.update(10.0, inter_arrival_ms=120.0)
        stats.update(10.0, inter_arrival_ms=80.0)
        
        # Mean jitter = mean([0, 20, 20]) = 40/3 = 13.33
        assert stats.jitter_ms == pytest.approx(40.0 / 3.0)


class TestLatencyThreshold:
    """Test LatencyThreshold configuration."""

    def test_default_values(self):
        t = LatencyThreshold()
        assert t.warn_ms == 50.0
        assert t.crit_ms == 100.0
        assert t.jitter_warn_ms == 20.0
        assert t.jitter_crit_ms == 50.0

    def test_custom_values(self):
        t = LatencyThreshold(
            warn_ms=10.0,
            crit_ms=30.0,
            jitter_warn_ms=3.0,
            jitter_crit_ms=10.0
        )
        assert t.warn_ms == 10.0
        assert t.crit_ms == 30.0


class TestDefaultThresholds:
    """Test default threshold configurations."""

    def test_scan_threshold_exists(self):
        assert 'scan' in DEFAULT_THRESHOLDS
        assert DEFAULT_THRESHOLDS['scan'].warn_ms == 30

    def test_odom_threshold_exists(self):
        assert 'odom' in DEFAULT_THRESHOLDS
        assert DEFAULT_THRESHOLDS['odom'].warn_ms == 20

    def test_imu_threshold_exists(self):
        assert 'imu' in DEFAULT_THRESHOLDS
        assert DEFAULT_THRESHOLDS['imu'].warn_ms == 10

    def test_default_threshold_exists(self):
        assert 'default' in DEFAULT_THRESHOLDS
        assert DEFAULT_THRESHOLDS['default'].warn_ms == 50


class TestLatencyMeasurer:
    """Test LatencyMeasurer timestamp processing."""

    def test_creation(self):
        measurer = LatencyMeasurer()
        assert measurer.window_sec == 5.0
        assert measurer.max_samples == 100
        assert measurer._stats == {}

    def test_custom_parameters(self):
        measurer = LatencyMeasurer(window_sec=10.0, max_samples=50)
        assert measurer.window_sec == 10.0
        assert measurer.max_samples == 50

    def test_measure_latency_basic(self):
        measurer = LatencyMeasurer()
        now = time.time()
        
        # Message sent 10ms ago
        msg_time = now - 0.010
        msg_sec = int(msg_time)
        msg_nsec = int((msg_time - msg_sec) * 1e9)
        
        latency = measurer.measure_latency('/test', msg_sec, msg_nsec)
        
        # Should be approximately 10ms (allow for timing variance)
        assert latency is not None
        assert latency >= 10.0  # At least 10ms

    def test_measure_latency_future_timestamp_rejected(self):
        measurer = LatencyMeasurer()
        now = time.time()
        
        # Message timestamp 10 seconds in the future (clock sync issue)
        future_time = now + 10.0
        msg_sec = int(future_time)
        msg_nsec = int((future_time - msg_sec) * 1e9)
        
        latency = measurer.measure_latency('/test', msg_sec, msg_nsec)
        
        # Should reject future timestamps
        assert latency is None

    def test_stats_accumulation(self):
        measurer = LatencyMeasurer()
        now = time.time()
        
        # Simulate multiple messages
        for i in range(5):
            msg_time = now - (0.010 + i * 0.001)  # 10-14ms ago
            msg_sec = int(msg_time)
            msg_nsec = int((msg_time - msg_sec) * 1e9)
            measurer.measure_latency('/scan', msg_sec, msg_nsec)
        
        stats = measurer.get_stats('/scan')
        assert stats is not None
        assert stats.sample_count == 5

    def test_get_all_stats(self):
        measurer = LatencyMeasurer()
        now = time.time()
        
        # Measure on two topics
        msg_time = now - 0.010
        msg_sec = int(msg_time)
        msg_nsec = int((msg_time - msg_sec) * 1e9)
        
        measurer.measure_latency('/scan', msg_sec, msg_nsec)
        measurer.measure_latency('/odom', msg_sec, msg_nsec)
        
        all_stats = measurer.get_all_stats()
        assert '/scan' in all_stats
        assert '/odom' in all_stats

    def test_clear(self):
        measurer = LatencyMeasurer()
        now = time.time()
        msg_time = now - 0.010
        msg_sec = int(msg_time)
        msg_nsec = int((msg_time - msg_sec) * 1e9)
        
        measurer.measure_latency('/scan', msg_sec, msg_nsec)
        assert measurer.get_stats('/scan') is not None
        
        measurer.clear()
        assert measurer.get_stats('/scan') is None
        assert measurer.get_all_stats() == {}

    def test_sample_limit(self):
        measurer = LatencyMeasurer(max_samples=10)
        now = time.time()
        
        # Add 20 samples
        for i in range(20):
            msg_time = now - 0.010
            msg_sec = int(msg_time)
            msg_nsec = int((msg_time - msg_sec) * 1e9)
            measurer.measure_latency('/test', msg_sec, msg_nsec)
        
        stats = measurer.get_stats('/test')
        # Should be limited to max_samples
        assert len(stats.samples) <= 10


class TestStampedTypes:
    """Test supported stamped message types."""

    def test_stamped_types_defined(self):
        measurer = LatencyMeasurer()
        assert len(measurer.STAMPED_TYPES) > 0

    def test_laser_scan_supported(self):
        measurer = LatencyMeasurer()
        assert 'sensor_msgs/msg/LaserScan' in measurer.STAMPED_TYPES

    def test_odometry_supported(self):
        measurer = LatencyMeasurer()
        assert 'nav_msgs/msg/Odometry' in measurer.STAMPED_TYPES

    def test_imu_supported(self):
        measurer = LatencyMeasurer()
        assert 'sensor_msgs/msg/Imu' in measurer.STAMPED_TYPES
