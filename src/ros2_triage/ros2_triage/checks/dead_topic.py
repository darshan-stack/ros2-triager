# Copyright 2024 darshan — Apache-2.0
"""
dead_topic.py — Detects topics with publishers but no subscribers (UNSUBSCRIBED)
or subscribers but no publishers (UNPUBLISHED).

Uses classify_topic() to eliminate false positives from Gazebo, Rviz,
Nav2 infrastructure topics, camera streams, and interactive operator topics.
"""

from .finding import Finding, SEVERITY_LABEL, CRITICAL_TOPICS, classify_topic


def _severity(topic: str, classification: str) -> int:
    """
    Severity based on topic classification:
      critical → 3 (real robot control / sensor pipeline broken)
      normal   → 2 (application topic with no counterpart)
      sim/viz  → 1 (simulation or visualisation artefact — informational only)
    """
    if classification == 'critical':
        return 3
    if classification in ('sim', 'viz'):
        return 1

    # Keyword heuristics for 'normal' topics
    low = topic.lower()
    # Strong control/sensor keywords → raise to WARNING
    STRONG_KEYWORDS = ('cmd_vel', 'twist', 'velocity', 'odom', 'scan',
                       'lidar', 'laser', 'joint_state', 'imu', 'gps',
                       'battery', 'emergency', 'estop', 'safety', 'fault',
                       'error', 'status', 'state')
    if any(kw in low for kw in STRONG_KEYWORDS):
        return 2

    return 2  # default all unknown topics to WARNING — important in real robots


def check_dead_topics(graph: dict,
                      skip_noisy: bool = True,
                      extra_ignore: set = None,
                      simulation_mode: bool = False) -> list:
    """
    Analyse the topic graph for dead-end connections.

    Parameters
    ----------
    graph          : dict — output of graph_utils.build_topic_graph()
    skip_noisy     : bool — skip well-known infrastructure / sim / viz topics
    extra_ignore   : set  — additional topics to ignore (from --ignore flag)
    simulation_mode: bool — if True, downgrade sim/viz topics to INFO and skip them

    Returns
    -------
    list[Finding]
    """
    findings = []
    ignore_set = extra_ignore or set()

    for topic, info in graph.items():
        # User-supplied ignore
        if topic in ignore_set:
            continue

        pub_count = len(info['publishers'])
        sub_count = len(info['subscribers'])

        # Classify the topic
        cls = classify_topic(topic)

        # Always skip pure infra / interactive / nav2-internal
        if skip_noisy and cls == 'skip':
            continue

        # In --simulation mode, also skip sim and viz topics entirely
        if simulation_mode and cls in ('sim', 'viz'):
            continue

        pub_nodes = [ep.node_name for ep in info['publishers']]
        sub_nodes = [ep.node_name for ep in info['subscribers']]

        # ── Case 1: subscribers present, no publishers ──────────────────────
        if sub_count > 0 and pub_count == 0:
            sev = _severity(topic, cls)
            findings.append(Finding(
                check='dead_topics',
                topic=topic,
                severity=sev,
                message=(
                    f'{sub_count} subscriber(s) [{", ".join(sub_nodes)}] '
                    f'but 0 publishers — topic is UNPUBLISHED.'
                ),
                suggestion=_unpublished_suggestion(topic, sub_nodes),
                extra={'classification': cls,
                       'subscribers': sub_nodes,
                       'publishers': []},
            ))

        # ── Case 2: publishers present, no subscribers ──────────────────────
        elif pub_count > 0 and sub_count == 0:
            sev = _severity(topic, cls)

            # Sim / viz publishers with no subscriber are almost never bugs
            # in real projects — downgrade to INFO so they don't pollute report
            if cls in ('sim', 'viz') and not simulation_mode:
                sev = 1

            findings.append(Finding(
                check='dead_topics',
                topic=topic,
                severity=sev,
                message=(
                    f'{pub_count} publisher(s) [{", ".join(pub_nodes)}] '
                    f'but 0 subscribers — topic is UNSUBSCRIBED.'
                ),
                suggestion=_unsubscribed_suggestion(topic, pub_nodes, cls),
                extra={'classification': cls,
                       'publishers': pub_nodes,
                       'subscribers': []},
            ))

    # Add higher-level pairwise findings for important topic families
    findings.extend(_pairwise_topic_family_findings(graph))

    # Sort by severity descending, then by topic name
    findings.sort(key=lambda f: (-f.severity, f.topic))
    return findings


# ── Suggestion generators ────────────────────────────────────────────────────

def _unpublished_suggestion(topic: str, sub_nodes: list) -> str:
    low = topic.lower()
    if 'cmd_vel' in low or 'twist' in low:
        return (
            f'No publisher is visible on {topic} in the current ROS 2 graph. '
            'If your velocity commands are published on a different topic '
            '(for example a namespaced /cmd_vel or /cmd_vel_nav), or are '
            'remapped in your launch files, verify that the remapping is '
            'correct and that the subscriber really listens to this topic. '
            'Otherwise check that your teleoperation node, navigation '
            'controller, or joystick driver is running. '
            'You can inject a test command with:\n'
            f'  ros2 topic pub --once {topic} geometry_msgs/msg/Twist "{{}}"'
        )
    if 'map' in low:
        return (
            f'No publisher is visible on {topic} in the current ROS 2 graph. '
            'If your global map is provided on a differently named topic or '
            'via remapping, confirm that your navigation stack is using the '
            'same topic name. Otherwise check that map_server, '
            'nav2_map_server, or slam_toolbox is running and included in '
            'your launch file.'
        )
    if 'odom' in low:
        return (
            f'No publisher is visible on {topic} in the current ROS 2 graph. '
            'If odometry is being published on an alternative topic such as '
            '/diff_cont/odom or /odometry/filtered, confirm that your '
            'navigation and localization nodes are remapped accordingly. '
            'If not, check that your robot base driver or EKF node is running.'
        )
    if 'scan' in low or 'laser' in low or 'lidar' in low:
        return (
            f'No lidar/laser data on {topic}. '
            'Check the sensor driver node or its USB/serial connection.'
        )
    return (
        f'{", ".join(sub_nodes)} is waiting for data on {topic}, '
        'but no publisher exists. '
        'Verify the publishing node is in your launch file and running: '
        '`ros2 node list`'
    )


def _unsubscribed_suggestion(topic: str, pub_nodes: list, cls: str) -> str:
    if cls in ('sim', 'viz'):
        return (
            f'{topic} is a simulation/visualisation topic with no current listener. '
            'This is usually harmless — Rviz or other tools may subscribe on demand.'
        )
    if cls == 'skip':
        return 'Infrastructure topic — this finding should not normally appear.'
    low = topic.lower()
    if 'scan' in low or 'laser' in low:
        return (
            f'Your sensor is publishing {topic} but nothing is consuming it. '
            'Check that slam_toolbox, nav2, or your mapping node is running.'
        )
    if 'imu' in low:
        return (
            f'{topic} has data but no consumer. '
            'Check that robot_localization (EKF) or your fuser node is running.'
        )
    if 'image' in low or 'camera' in low:
        return (
            f'Camera data on {topic} has no subscriber. '
            'If Rviz is displaying it, this is normal. '
            'Otherwise verify your perception pipeline is running.'
        )
    return (
        f'{", ".join(pub_nodes)} is publishing to {topic} but nothing subscribes. '
        'Verify your consumer node is running: `ros2 node list`. '
        'You can inspect the data with: `ros2 topic echo ' + topic + '`'
    )


def _pairwise_topic_family_findings(graph: dict) -> list:
    """
    Look for pairs of related topics in the same semantic family where one
    has only publishers and the other has only subscribers.

    Example: odometry published on /diff_cont/odom but consumed on /odom.
    In such cases we emit a single, high-level Finding that explains the
    mismatch in terms of the observed ROS 2 graph, without making assumptions
    about the physical robot state.
    """
    from .finding import classify_topic  # local import to avoid cycles

    results = []

    def _family_members(keyword: str):
        members = []
        for name, info in graph.items():
            low = name.lower()
            if keyword not in low:
                continue
            pub_count = len(info['publishers'])
            sub_count = len(info['subscribers'])
            if pub_count == 0 and sub_count == 0:
                continue
            members.append((name, pub_count, sub_count))
        return members

    def _add_family_pair(keyword: str, label: str, severity: int):
        members = _family_members(keyword)
        if not members:
            return

        pubs_only = [(t, p, s) for t, p, s in members if p > 0 and s == 0]
        subs_only = [(t, p, s) for t, p, s in members if s > 0 and p == 0]

        if not pubs_only or not subs_only:
            return

        # Take the most "significant" publisher-only and subscriber-only topics
        pub_topic, pub_p, _ = sorted(
            pubs_only, key=lambda x: (-x[1], x[0])
        )[0]
        sub_topic, _, sub_s = sorted(
            subs_only, key=lambda x: (-x[2], x[0])
        )[0]

        # Skip if either side is classified as pure infra/sim/viz noise
        if classify_topic(pub_topic) in ('skip', 'sim', 'viz'):
            return
        if classify_topic(sub_topic) in ('skip', 'sim', 'viz'):
            return

        results.append(Finding(
            check='dead_topics',
            topic=f'{label}:{pub_topic}→{sub_topic}',
            severity=severity,
            message=(
                f'{label.capitalize()} appears to be PUBLISHED on {pub_topic} '
                f'({pub_p} publisher(s)) while nodes SUBSCRIBE on {sub_topic} '
                f'({sub_s} subscriber(s)) with no visible publisher there in '
                'the current ROS 2 graph.'
            ),
            suggestion=(
                f'Check your remappings and launch configuration so that the '
                f'{label} source and consumers agree on the same topic name.\n'
                f'For example, if {pub_topic} is the true source but your '
                f'navigation or localization stack subscribes to {sub_topic}, '
                f'you likely need a remap or to adjust parameters.\n'
                f'Use `ros2 topic info {pub_topic}` and `ros2 topic info '
                f'{sub_topic}` to inspect endpoints.'
            ),
            extra={
                'family': label,
                'publisher_topic': pub_topic,
                'subscriber_topic': sub_topic,
                'publisher_only_count': pub_p,
                'subscriber_only_count': sub_s,
            },
        ))

    # Odometry / map / cmd_vel families are especially critical in real robots.
    _add_family_pair('odom', 'odometry', severity=3)
    _add_family_pair('map', 'map', severity=2)
    _add_family_pair('cmd_vel', 'cmd_vel', severity=3)

    return results
