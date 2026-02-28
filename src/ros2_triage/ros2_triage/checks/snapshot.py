# Copyright 2024 darshan — Apache-2.0
"""
snapshot.py — Save a known-good system state and diff against it later.

Workflow:
  # When robot works correctly:
  ros2 triage --snapshot-save healthy.json

  # Later (after a change, during debugging):
  ros2 triage --snapshot-diff healthy.json

The diff reports:
  - Topics that disappeared since the snapshot (CRIT if critical topic)
  - Topics that appeared since the snapshot (INFO)
  - Nodes that disappeared (CRIT)
  - Nodes that appeared (INFO)
"""

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from .finding import Finding, classify_topic


# ── Snapshot format ────────────────────────────────────────────────────────────

def build_snapshot(graph: dict, running_nodes: list) -> dict:
    """Build a JSON-serializable snapshot of the current system state."""
    topics = {}
    for topic, info in graph.items():
        topics[topic] = {
            'types': info.get('types', []),
            'publisher_count': len(info['publishers']),
            'subscriber_count': len(info['subscribers']),
            'publisher_nodes': [ep.node_name for ep in info['publishers']],
            'subscriber_nodes': [ep.node_name for ep in info['subscribers']],
        }
    return {
        'schema_version': '1.0',
        'timestamp': datetime.now().isoformat(),
        'ros2_triage': 'snapshot',
        'topics': topics,
        'nodes': sorted(running_nodes),
    }


def save_snapshot(graph: dict,
                  running_nodes: list,
                  output_path: str) -> None:
    """Save snapshot to a JSON file."""
    snap = build_snapshot(graph, running_nodes)
    path = Path(output_path)
    with open(path, 'w') as f:
        json.dump(snap, f, indent=2)
    print(f'\nSnapshot saved: {path}')
    print(f'    Topics captured : {len(snap["topics"])}')
    print(f'    Nodes captured  : {len(snap["nodes"])}')
    print(f'    Timestamp       : {snap["timestamp"]}')
    print(f'\n  To compare later: ros2 triage --snapshot-diff {path}\n')


def load_snapshot(path: str) -> dict:
    """Load a snapshot from a JSON file."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f'Snapshot file not found: {path}')
    with open(p) as f:
        return json.load(f)


def diff_snapshot(graph: dict,
                  running_nodes: list,
                  snapshot_path: str,
                  ignore_set: set = None) -> list:
    """
    Compare current system state against a saved snapshot.

    Returns
    -------
    list[Finding] — changes since the snapshot
    """
    ignore_set = ignore_set or set()
    findings = []

    snap = load_snapshot(snapshot_path)
    snap_topics = snap.get('topics', {})
    snap_nodes = set(snap.get('nodes', []))
    snap_time = snap.get('timestamp', 'unknown')

    current_topics = set(graph.keys()) - ignore_set
    current_nodes = set(running_nodes)

    # ── Topic diffs ──────────────────────────────────────────────────────────

    # Topics that existed before but are GONE now
    disappeared_topics = set(snap_topics.keys()) - current_topics
    for topic in sorted(disappeared_topics):
        if topic in ignore_set:
            continue
        cls = classify_topic(topic)
        if cls == 'skip':
            continue
        sev = 3 if cls == 'critical' else 2
        old_info = snap_topics[topic]
        findings.append(Finding(
            check='snapshot',
            topic=topic,
            severity=sev,
            message=(
                f'Topic {topic} existed in snapshot ({snap_time}) '
                f'[{old_info["publisher_count"]} pub, {old_info["subscriber_count"]} sub] '
                f'but is GONE from the current graph.'
            ),
            suggestion=(
                f'Check if the node publishing {topic} was stopped or crashed.\n'
                f'Snapshot publishers were: {old_info["publisher_nodes"]}\n'
                f'Run: `ros2 node list` to see what changed.'
            ),
            extra={'change': 'DISAPPEARED', 'snapshot_time': snap_time},
        ))

    # Topics that are NEW since the snapshot
    new_topics = current_topics - set(snap_topics.keys())
    for topic in sorted(new_topics):
        cls = classify_topic(topic)
        if cls in ('skip', 'sim', 'viz'):
            continue
        findings.append(Finding(
            check='snapshot',
            topic=topic,
            severity=1,
            message=(
                f'Topic {topic} is NEW since snapshot ({snap_time}). '
                f'It was not present in the baseline.'
            ),
            suggestion=(
                f'A new node or plugin has started publishing {topic}. '
                f'This may be intentional. If not, check: `ros2 node list`'
            ),
            extra={'change': 'APPEARED', 'snapshot_time': snap_time},
        ))

    # Topics where pub/sub count changed significantly
    for topic in current_topics & set(snap_topics.keys()):
        if topic in ignore_set:
            continue
        cls = classify_topic(topic)
        if cls in ('skip', 'sim', 'viz'):
            continue

        cur_info = graph[topic]
        old_info = snap_topics[topic]
        cur_pub = len(cur_info['publishers'])
        cur_sub = len(cur_info['subscribers'])
        old_pub = old_info['publisher_count']
        old_sub = old_info['subscriber_count']

        # Publisher dropped to 0 (was > 0)
        if old_pub > 0 and cur_pub == 0:
            sev = 3 if classify_topic(topic) == 'critical' else 2
            findings.append(Finding(
                check='snapshot',
                topic=topic,
                severity=sev,
                message=(
                    f'Topic {topic}: publishers dropped from '
                    f'{old_pub} → {cur_pub} since snapshot ({snap_time}).'
                ),
                suggestion=(
                    f'Node(s) that were publishing {topic} have stopped.\n'
                    f'Snapshot publishers: {old_info["publisher_nodes"]}'
                ),
                extra={'change': 'PUBLISHER_LOST',
                       'old_pub': old_pub, 'cur_pub': cur_pub},
            ))

    # ── Node diffs ───────────────────────────────────────────────────────────

    # Nodes that existed before but are gone
    disappeared_nodes = snap_nodes - current_nodes
    for node in sorted(disappeared_nodes):
        if any(skip in node for skip in ('_ros2_triage_', 'rviz', 'gazebo')):
            continue
        findings.append(Finding(
            check='snapshot',
            topic=node,
            severity=2,
            message=(
                f'Node {node} was running at snapshot time ({snap_time}) '
                f'but is NOT running now.'
            ),
            suggestion=(
                f'Node {node} may have crashed or been stopped.\n'
                f'Check: `ros2 node list` and your process manager / systemd / launch.'
            ),
            extra={'change': 'NODE_DISAPPEARED', 'snapshot_time': snap_time},
        ))

    findings.sort(key=lambda f: (-f.severity, f.topic))
    return findings
