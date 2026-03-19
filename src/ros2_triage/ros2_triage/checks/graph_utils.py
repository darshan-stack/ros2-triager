# Copyright 2024 darshan - Apache-2.0
"""
graph_utils.py - Inspects the live ROS 2 graph using rclpy.

Returns:
  graph : dict  topic -> {types, publishers, subscribers}
  nodes : list  fully-qualified node names currently running
"""

import time
import rclpy
from rclpy.node import Node


class InspectorNode(Node):
    """
    A longer-lived inspector node that can be reused across calls.
    Adds richer graph query methods (§6.3).
    """

    def __init__(self, name: str = '_ros2_triage_inspector_'):
        super().__init__(name)

    def get_all_topics_with_info(self) -> dict:
        """Returns {topic_name: {type, pub_count, sub_count}}"""
        result = {}
        for name, types in self.get_topic_names_and_types():
            result[name] = {
                "type": types[0] if types else "unknown",
                "pub_count": self.count_publishers(name),
                "sub_count": self.count_subscribers(name),
            }
        return result

    def get_all_nodes_with_info(self) -> list[dict]:
        """Returns [{name, namespace, pub_topics, sub_topics, services}]"""
        result = []
        for name, ns in self.get_node_names_and_namespaces():
            try:
                pubs = [
                    t for t, _ in
                    self.get_publisher_names_and_types_by_node(name, ns)
                ]
                subs = [
                    t for t, _ in
                    self.get_subscriber_names_and_types_by_node(name, ns)
                ]
                svcs = [
                    s for s, _ in
                    self.get_service_names_and_types_by_node(name, ns)
                ]
                result.append({
                    "name": name,
                    "namespace": ns,
                    "pub_topics": pubs,
                    "sub_topics": subs,
                    "services": svcs,
                })
            except Exception:
                result.append({
                    "name": name,
                    "namespace": ns,
                    "pub_topics": [],
                    "sub_topics": [],
                    "services": [],
                })
        return result


def build_topic_graph(timeout_sec: float = 3.0) -> tuple:
    """
    Spin a temporary node, wait for the graph to settle, then snapshot
    publishers, subscribers, and running nodes.

    Parameters
    ----------
    timeout_sec : float - seconds to wait for graph to propagate

    Returns
    -------
    (graph, nodes)
      graph : dict  topic -> {"types": list[str],
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
