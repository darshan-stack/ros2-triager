# Copyright 2024 darshan — Apache-2.0
"""
Shared Finding dataclass and topic classification constants.

classify_topic() returns one of:
  'skip'     — always ignored (infra, Nav2 internal, ros2_control lifecycle)
  'sim'      — Gazebo simulation topic  (severity cap: INFO in sim mode)
  'viz'      — Rviz/visualisation topic (severity cap: INFO)
  'critical' — known critical robot topic (severity: CRIT)
  'normal'   — standard app topic (severity: WARN or CRIT by keyword)
"""

from dataclasses import dataclass, field, asdict


@dataclass
class Finding:
    """Represents a single diagnostic finding."""
    check: str          # "dead_topics" | "qos" | "tf"
    topic: str          # topic or frame name
    severity: int       # 1=INFO, 2=WARN, 3=CRIT
    message: str
    suggestion: str
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = asdict(self)
        if not d['extra']:
            del d['extra']
        return d


# ── Severity labels ──────────────────────────────────────────────────────────
SEVERITY_LABEL = {1: 'INFO', 2: 'WARN', 3: 'CRIT'}

# ── 1. Pure infrastructure — always skip ─────────────────────────────────────
NOISY_TOPICS = {
    '/rosout', '/parameter_events', '/clock',
    '/tf', '/tf_static',
    '/diagnostics', '/diagnostics_agg', '/diagnostics_toplevel_state',
    '/robot_description', '/bond',
    '/lifecycle_state', '/lifecycle_manager_navigation/transition_event',
    '/lifecycle_manager_localization/transition_event',
    '/lifecycle_manager/transition_event',
}

# ── 2. Topic SUFFIXES that are always infrastructure noise ───────────────────
#    Matches: /any_node/transition_event, /local_costmap/local_costmap/transition_event
#    Matches: /navigate_to_pose/_action/status, /drive_on_heading/_action/feedback
NOISY_SUFFIXES = (
    '/transition_event',          # ros2 lifecycle events (never subscribed to)
    '/_action/status',            # Nav2 / action server status streams
    '/_action/feedback',          # Nav2 / action server feedback streams
    '/_action/result',            # Nav2 / action server results
    '/_action/cancel_goal',       # Nav2 / action server cancel
    '/_action/get_result',        # Nav2 / action server get result
    '/_action/send_goal',         # Nav2 / action server send goal
)

# ── 3. Exact topic names from Nav2 / ros2_control / SLAM that are ─────────────
#    intentionally one-directional or debug-only in normal operation
NAV2_DEBUG_TOPICS = {
    # Nav2 BT & planner debug
    '/behavior_tree_log',
    '/evaluation',
    '/cost_cloud',
    '/received_global_plan',
    '/transformed_global_plan',
    '/plan_smoothed',
    '/pose',                        # slam_toolbox internal pose
    '/slam_toolbox/update',
    '/slam_toolbox/feedback',

    # ros2_control debug
    '/dynamic_joint_states',

    # Gazebo metrics
    '/performance_metrics',
    '/world_stats',

    # Nav2 costmap debug streams (Rviz subscribes on demand)
    '/global_costmap/voxel_marked_cloud',
    '/local_costmap/voxel_grid',
    '/local_costmap/voxel_marked_cloud',
    '/local_costmap/clearing_endpoints',
    '/downsampled_costmap',
    '/downsampled_costmap_updates',

    # AMCL / localization visualization
    '/particle_cloud',
    '/particlecloud',

    # Map updates (Rviz subscribes on demand)
    '/map_updates',
    '/map_metadata',
}

# ── 4. Interactive topics — published only when operator acts ─────────────────
INTERACTIVE_TOPICS = {
    '/initialpose',
    '/goal_pose',
    '/clicked_point',
    '/move_base_simple/goal',
    '/global_costmap/footprint',   # set by operator / config, not runtime
    '/local_costmap/footprint',    # set by operator / config, not runtime
    '/preempt_teleop',             # published by external operator tool
    '/speed_limit',                # published by speed_filter if configured
}

# ── 5. Simulation (Gazebo) topic prefixes ─────────────────────────────────────
SIMULATION_PREFIXES = (
    '/gazebo/', '/gz/', '/world/', '/model/',
    '/physics', '/joint_state_controller',
    '/gazebo_ros', '/simulation', '/sim/',
)

# ── 6. Visualisation topic prefixes / keywords ────────────────────────────────
VISUALIZATION_PREFIXES = (
    '/rviz', '/rviz2', '/visualization', '/display_', '/rviz_grid',
)
VISUALIZATION_SUFFIXES_TOPIC = (
    '/visualization_marker', '/visualization_marker_array',
    '/display_trajectory', '/planned_path',
    '_display', '_markers', '_marker',
    '/collision_object', '/monitored_planning_scene',
    '/motion_plan_request',
    # Mobile base / Kobuki / Turtlebot sensor display topics
    '/bumper_pointcloud', '/proximity/pointcloud',
    '/sensors/bumper_pointcloud', '/sensors/cliff_pointcloud',
)

# ── 7. Critical topics — missing counterpart is a real robot problem ──────────
CRITICAL_TOPICS = {
    '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_mux', '/cmd_vel_smoothed',
    '/scan', '/scan_filtered', '/laser_scan',
    '/points', '/points2',
    '/odom', '/odometry/filtered',
    '/map',
    '/emergency_stop', '/e_stop',
    '/joint_states',
}

# ── Combined fast-lookup set for 'skip' classification ───────────────────────
ALL_SKIP_EXACT = NOISY_TOPICS | NAV2_DEBUG_TOPICS | INTERACTIVE_TOPICS


def classify_topic(topic: str) -> str:
    """
    Classify a topic name for severity/filtering purposes.

    Returns: 'skip' | 'sim' | 'viz' | 'critical' | 'normal'
    """
    # 1. Exact match against all skip sets
    if topic in ALL_SKIP_EXACT:
        return 'skip'

    # 2. Suffix-based noise patterns (transition_event, _action/status, etc.)
    if any(topic.endswith(s) for s in NOISY_SUFFIXES):
        return 'skip'

    # 3. Nav2 costmap sub-namespaces that are always internal
    #    e.g. /global_costmap/global_costmap/anything
    #         /local_costmap/local_costmap/anything
    low = topic.lower()
    if ('global_costmap/global_costmap' in low or
            'local_costmap/local_costmap' in low):
        return 'skip'

    # 4. Simulation prefixes
    if any(low.startswith(p) or ('/' + p.strip('/')) in low
           for p in SIMULATION_PREFIXES):
        return 'sim'

    # 5. Visualisation prefixes / suffixes
    if any(low.startswith(p) for p in VISUALIZATION_PREFIXES):
        return 'viz'
    if any(low.endswith(s) for s in VISUALIZATION_SUFFIXES_TOPIC):
        return 'viz'
    if 'marker' in low or 'rviz' in low or 'visualization' in low:
        return 'viz'

    # 6. Camera / image topics — Rviz displays on demand
    if any(s in low for s in ('/image_raw', '/image_color', '/image_rect',
                               '/camera_info', '/compressed', '/theora',
                               '/depth/image')):
        return 'viz'

    # 7. Critical topics
    if topic in CRITICAL_TOPICS:
        return 'critical'

    return 'normal'
