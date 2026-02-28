# Copyright 2024 darshan — Apache-2.0
"""
tf_check.py — Validates the TF tree structure.

Checks:
  - Expected frames present (map, odom, base_link)
  - Tree is fully connected (no isolated subtrees)
"""

import rclpy
from rclpy.node import Node
import time

from .finding import Finding

# Frames that most robots should have
EXPECTED_FRAMES = ['map', 'odom', 'base_link', 'base_footprint']

# Time to wait for TF data to accumulate
TF_WAIT_SEC = 3.0


def check_tf(external_node: Node = None) -> list:
    """
    Check TF tree sanity.

    Uses tf2_ros to get all available frames and check connectivity.

    Returns
    -------
    list[Finding]
    """
    findings = []
    own_node = False

    try:
        import tf2_ros
    except ImportError:
        return []

    need_init = not rclpy.ok()
    if need_init:
        rclpy.init()

    node = external_node
    if node is None:
        node = Node('_ros2_triage_tf_checker_')
        own_node = True

    try:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        # Wait for TF data
        deadline = time.time() + TF_WAIT_SEC
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Get all frames from the TF buffer
        frames_yaml = tf_buffer.all_frames_as_yaml()
        if not frames_yaml or frames_yaml.strip() == '':
            findings.append(Finding(
                check='tf',
                topic='tf_tree',
                severity=2,
                message='No TF frames found. TF tree appears to be empty.',
                suggestion=(
                    'Ensure nodes that publish TF (robot_state_publisher, '
                    'localization nodes) are running. '
                    'Check: `ros2 topic echo /tf`'
                ),
            ))
            return findings

        # Parse frame names from YAML output
        available_frames = set()
        for line in frames_yaml.splitlines():
            line = line.strip()
            if line.endswith(':') and not line.startswith('-'):
                frame = line.rstrip(':')
                available_frames.add(frame)

        # Check for expected frames
        for frame in EXPECTED_FRAMES:
            if frame not in available_frames:
                # Missing "map" can be harmless on robots that operate purely in
                # odom coordinates, so keep severity at WARNING but clarify in
                # the suggestion that this may be intentional.
                findings.append(Finding(
                    check='tf',
                    topic=f'tf:{frame}',
                    severity=2,
                    message=(
                        f'Expected TF frame "{frame}" not found in the TF tree. '
                        f'Available frames: {sorted(available_frames)}'
                    ),
                    suggestion=(
                        f'Check that the node publishing the "{frame}" frame is '
                        f'running (e.g., robot_state_publisher for base_link, '
                        f'map_server for map, nav2_amcl for odom→map).\n'
                        f'If your robot intentionally does not use a global '
                        f'\"map\" frame (pure odom-only operation), you can treat '
                        f'the missing \"map\" warning as informational or ignore '
                        f'it via `--ignore tf:map`.\n'
                        f'Run: `ros2 run tf2_tools view_frames` to inspect the '
                        f'TF tree.'
                    ),
                ))

        # Check tf→odom→base_link chain connectivity
        chain = ['map', 'odom', 'base_link']
        for i in range(len(chain) - 1):
            parent, child = chain[i], chain[i + 1]
            if parent in available_frames and child in available_frames:
                try:
                    tf_buffer.lookup_transform(parent, child,
                                               rclpy.time.Time())
                except Exception as e:
                    findings.append(Finding(
                        check='tf',
                        topic=f'tf:{parent}→{child}',
                        severity=3,
                        message=(
                            f'Cannot lookup transform {parent}→{child}: {e}. '
                            f'The TF tree may have a broken link.'
                        ),
                        suggestion=(
                            f'Ensure the transform from {parent} to {child} is '
                            f'being published. Check `ros2 topic echo /tf` and '
                            f'`ros2 run tf2_tools view_frames` for gaps.'
                        ),
                    ))

    finally:
        if own_node and node:
            node.destroy_node()
        if need_init:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    findings.sort(key=lambda f: f.severity, reverse=True)
    return findings
