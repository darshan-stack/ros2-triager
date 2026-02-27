# Copyright 2024 darshan — Apache-2.0
"""
qos_check.py — Detects QoS incompatibilities between publishers and
subscribers sharing a topic.

Checks:
  - Reliability mismatch  (RELIABLE pub ↔ BEST_EFFORT sub, or vice versa)
  - Durability mismatch   (TRANSIENT_LOCAL pub ↔ VOLATILE sub, or vice versa)
"""

from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from .finding import Finding, NOISY_TOPICS


def _reliability_name(r) -> str:
    try:
        return ReliabilityPolicy(r).name
    except Exception:
        return str(r)


def _durability_name(d) -> str:
    try:
        return DurabilityPolicy(d).name
    except Exception:
        return str(d)


def _reliability_compat(pub_rel, sub_rel) -> bool:
    """
    ROS 2 QoS reliability compatibility rule:
      A BEST_EFFORT publisher is incompatible with a RELIABLE subscriber.
      All other combinations are compatible.
    """
    try:
        p = ReliabilityPolicy(pub_rel)
        s = ReliabilityPolicy(sub_rel)
    except Exception:
        return True  # can't determine → skip

    if p == ReliabilityPolicy.BEST_EFFORT and s == ReliabilityPolicy.RELIABLE:
        return False
    return True


def _durability_compat(pub_dur, sub_dur) -> bool:
    """
    ROS 2 QoS durability compatibility rule:
      A VOLATILE publisher is incompatible with a TRANSIENT_LOCAL subscriber.
    """
    try:
        p = DurabilityPolicy(pub_dur)
        s = DurabilityPolicy(sub_dur)
    except Exception:
        return True

    if p == DurabilityPolicy.VOLATILE and s == DurabilityPolicy.TRANSIENT_LOCAL:
        return False
    return True


def check_qos(graph: dict, ignore_set: set = None) -> list:
    """
    For each topic, compare every publisher↔subscriber QoS pair and
    report incompatibilities.

    Parameters
    ----------
    graph      : dict — output of graph_utils.build_topic_graph()
    ignore_set : set  — additional topics to skip (from --ignore flag)

    Returns
    -------
    list[Finding]
    """
    from .finding import classify_topic
    findings = []
    ignore_set = ignore_set or set()

    for topic, info in graph.items():
        if topic in ignore_set:
            continue

        # Skip infra / sim / viz topics — QoS mismatches there don't matter
        cls = classify_topic(topic)
        if cls in ('skip', 'sim', 'viz'):
            continue

        pubs = info['publishers']
        subs = info['subscribers']

        if not pubs or not subs:
            continue   # dead-topic checker handles these

        for pub in pubs:
            pub_qos = pub.qos_profile
            pub_node = pub.node_name

            for sub in subs:
                sub_qos = sub.qos_profile
                sub_node = sub.node_name

                # Reliability check
                if not _reliability_compat(pub_qos.reliability,
                                           sub_qos.reliability):
                    findings.append(Finding(
                        check='qos',
                        topic=topic,
                        severity=3,
                        message=(
                            f'Reliability mismatch on {topic}: '
                            f'publisher [{pub_node}] = '
                            f'{_reliability_name(pub_qos.reliability)}, '
                            f'subscriber [{sub_node}] = '
                            f'{_reliability_name(sub_qos.reliability)}. '
                            f'Messages will be DROPPED silently.'
                        ),
                        suggestion=(
                            f'Align QoS on both sides. Options:\n'
                            f'  • Change {sub_node} reliability to BEST_EFFORT\n'
                            f'  • Change {pub_node} reliability to RELIABLE\n'
                            f'Run `ros2 topic info -v {topic}` to inspect current profiles.'
                        ),
                    ))

                # Durability check
                if not _durability_compat(pub_qos.durability,
                                          sub_qos.durability):
                    findings.append(Finding(
                        check='qos',
                        topic=topic,
                        severity=2,
                        message=(
                            f'Durability mismatch on {topic}: '
                            f'publisher [{pub_node}] = '
                            f'{_durability_name(pub_qos.durability)}, '
                            f'subscriber [{sub_node}] = '
                            f'{_durability_name(sub_qos.durability)}. '
                            f'Late-joining subscriber may miss messages.'
                        ),
                        suggestion=(
                            f'Change {pub_node} publisher durability to TRANSIENT_LOCAL\n'
                            f'so {sub_node} receives messages published before it started.\n'
                            f'Run `ros2 topic info -v {topic}` to inspect current profiles.'
                        ),
                    ))

    findings.sort(key=lambda f: (-f.severity, f.topic))
    return findings

