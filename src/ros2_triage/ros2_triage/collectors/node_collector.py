# collectors/node_collector.py
from __future__ import annotations
from rclpy.node import Node
from ..state.state_bus import StateBus, NodeState
from .base_collector import BaseCollector


class NodeCollector(BaseCollector):
    """
    Discovers all running nodes via get_node_names_and_namespaces().
    Populates NodeState with publishers, subscribers, and services.
    """

    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=2.0)

    def collect(self) -> None:
        try:
            all_nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return

        updated: dict[str, NodeState] = {}
        for name, ns in all_nodes:
            # Skip our own internal node
            if "_ros2_triager" in name:
                continue
            full = (ns.rstrip("/") + "/" + name).replace("//", "/")
            if not full.startswith("/"):
                full = "/" + full
            try:
                pubs = [
                    t for t, _ in
                    self._node.get_publisher_names_and_types_by_node(name, ns)
                ]
            except Exception:
                pubs = []
            try:
                subs = [
                    t for t, _ in
                    self._node.get_subscriber_names_and_types_by_node(name, ns)
                ]
            except Exception:
                subs = []
            try:
                svcs = [
                    s for s, _ in
                    self._node.get_service_names_and_types_by_node(name, ns)
                ]
            except Exception:
                svcs = []

            updated[full] = NodeState(
                name=full,
                namespace=ns,
                pub_topics=pubs,
                sub_topics=subs,
                services=svcs,
            )

        with self._bus._lock:
            # Preserve zombie flags when updating
            for full, state in updated.items():
                existing = self._bus.nodes.get(full)
                if existing:
                    state.is_zombie = existing.is_zombie
                    state.status = existing.status
            self._bus.nodes = updated
