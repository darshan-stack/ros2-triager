# Copyright 2024 darshan — Apache-2.0
"""
graph_utils.py — Inspects the live ROS 2 graph using rclpy.

Returns:
  graph : dict  topic → {types, publishers, subscribers}
  nodes : list  fully-qualified node names currently running
"""

import time
import rclpy
from rclpy.node import Node


def build_topic_graph(timeout_sec: float = 3.0) -> tuple:
    """
    Spin a temporary node, wait for the graph to settle, then snapshot
    publishers, subscribers, and running nodes.

    Parameters
    ----------
    timeout_sec : float — seconds to wait for graph to propagate

    Returns
    -------
    (graph, nodes)
      graph : dict  topic → {"types": list[str],
                              "publishers": list[TopicEndpointInfo],
                              "subscribers": list[TopicEndpointInfo]}
      nodes : list  sorted fully-qualified node names
    """
    need_init = not rclpy.ok()
    if need_init:
        rclpy.init()

    node = Node('_ros2_triage_inspector_')
    try:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        # ── Topic graph ───────────────────────────────────────────────────
        topics_and_types = node.get_topic_names_and_types()
        graph: dict = {}
        for topic, types in topics_and_types:
            pubs = node.get_publishers_info_by_topic(topic)
            subs = node.get_subscriptions_info_by_topic(topic)
            graph[topic] = {
                'types': types,
                'publishers': pubs,
                'subscribers': subs,
            }

        # ── Node list ─────────────────────────────────────────────────────
        raw_nodes = node.get_node_names_and_namespaces()
        nodes = []
        for name, ns in raw_nodes:
            fqn = (ns.rstrip('/') + '/' + name) if ns != '/' else ('/' + name)
            if '_ros2_triage_' not in name:
                nodes.append(fqn)

        return graph, sorted(nodes)

    finally:
        node.destroy_node()
        if need_init:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def node_list_from_graph(graph: dict) -> list:
    """Return unique node names seen across all publisher/subscriber endpoints."""
    nodes: set = set()
    for info in graph.values():
        for ep in info['publishers'] + info['subscribers']:
            nodes.add(ep.node_name)
    return sorted(nodes)
