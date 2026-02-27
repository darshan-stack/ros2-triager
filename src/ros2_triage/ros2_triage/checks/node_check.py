# Copyright 2024 darshan — Apache-2.0
"""
node_check.py — Detects expected nodes that are NOT running.

Load an expected-nodes YAML file and compare against the live ROS 2
node list. Reports missing nodes with severity 3 (CRIT) and unexpected
nodes (running but not in the expected list) with severity 1 (INFO).

YAML format:
  nodes:
    - /slam_toolbox
    - /controller_server
    - /amcl
    - /map_server
  optional:
    - /joy_node
    - /foxglove_bridge
"""

import yaml
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
import time

from .finding import Finding


def _get_running_nodes(timeout: float = 2.0) -> list:
    """Return list of fully-qualified node names currently in the graph."""
    need_init = not rclpy.ok()
    if need_init:
        rclpy.init()

    sentinel = Node('_ros2_triage_node_checker_')
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(sentinel, timeout_sec=0.1)

        raw = sentinel.get_node_names_and_namespaces()
        nodes = []
        for name, ns in raw:
            if ns.endswith('/'):
                fqn = ns + name
            else:
                fqn = ns + '/' + name
            if not fqn.startswith('/'):
                fqn = '/' + fqn
            # skip our own inspector node
            if '_ros2_triage_' not in name:
                nodes.append(fqn)
        return nodes
    finally:
        sentinel.destroy_node()
        if need_init:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def load_expected_nodes(yaml_path: str) -> tuple:
    """
    Load expected node list from YAML.

    Returns
    -------
    (required: set, optional: set)
    """
    path = Path(yaml_path)
    if not path.exists():
        raise FileNotFoundError(f'Expected-nodes file not found: {yaml_path}')

    with open(path) as f:
        data = yaml.safe_load(f) or {}

    required = set(data.get('nodes', []))
    optional = set(data.get('optional', []))
    return required, optional


def check_nodes(expected_yaml: Optional[str] = None,
                running_nodes: Optional[list] = None,
                ignore_set: set = None) -> list:
    """
    Compare running nodes against expected node list.

    Parameters
    ----------
    expected_yaml : str  — path to expected_nodes YAML file
    running_nodes : list — pre-fetched node list (avoids double rclpy.init)
    ignore_set    : set  — nodes to skip

    Returns
    -------
    list[Finding]
    """
    findings = []
    ignore_set = ignore_set or set()

    # Get running nodes
    if running_nodes is None:
        running_nodes = _get_running_nodes()

    running_set = set(running_nodes) - ignore_set

    # If no YAML, just report what's running (informational)
    if not expected_yaml:
        return findings

    required, optional = load_expected_nodes(expected_yaml)

    # Check required nodes
    for node in required:
        if node in ignore_set:
            continue
        if node not in running_set:
            findings.append(Finding(
                check='nodes',
                topic=node,
                severity=3,
                message=(
                    f'Required node {node} is NOT running. '
                    f'It was declared in your expected_nodes config but '
                    f'is missing from the live graph.'
                ),
                suggestion=(
                    f'Check your launch file includes {node}.\n'
                    f'Try starting it manually: `ros2 run <pkg> <executable>`\n'
                    f'Check for crash logs: `ros2 node info {node}`'
                ),
                extra={'status': 'MISSING', 'required': True},
            ))

    # Check optional nodes (INFO only)
    for node in optional:
        if node in ignore_set:
            continue
        if node not in running_set:
            findings.append(Finding(
                check='nodes',
                topic=node,
                severity=1,
                message=(
                    f'Optional node {node} is not running. '
                    f'This is non-critical but may limit functionality.'
                ),
                suggestion=(
                    f'Start {node} if you need its functionality.\n'
                    f'It is marked optional in your expected_nodes config.'
                ),
                extra={'status': 'MISSING', 'required': False},
            ))

    # Unexpected nodes (running but not expected)
    declared = required | optional
    for node in sorted(running_set):
        # Skip standard ROS 2 infrastructure nodes
        if any(skip in node for skip in ('_ros2_triage_', 'rviz', 'gazebo',
                                          'transform_listener', '_ros2cli_')):
            continue
        if node not in declared:
            findings.append(Finding(
                check='nodes',
                topic=node,
                severity=1,
                message=(
                    f'Node {node} is running but not listed in '
                    f'your expected_nodes config (unexpected).'
                ),
                suggestion=(
                    f'If this node is intentional, add it to expected_nodes.yaml.\n'
                    f'If not, check if it was accidentally started: '
                    f'`ros2 node info {node}`'
                ),
                extra={'status': 'UNEXPECTED'},
            ))

    findings.sort(key=lambda f: (-f.severity, f.topic))
    return findings
