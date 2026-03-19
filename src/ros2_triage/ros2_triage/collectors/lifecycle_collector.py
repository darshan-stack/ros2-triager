# collectors/lifecycle_collector.py
from __future__ import annotations
import time
from rclpy.node import Node
from ..state.state_bus import StateBus, LifecycleState
from .base_collector import BaseCollector

LIFECYCLE_LABELS = {
    1: "UNCONFIGURED",
    2: "INACTIVE",
    3: "ACTIVE",
    4: "FINALIZED",
}


class LifecycleCollector(BaseCollector):
    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=2.0)

    def collect(self) -> None:
        # Discover lifecycle nodes by checking for /get_state service
        try:
            all_nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return

        all_services = []
        try:
            all_services = [s for s, _ in self._node.get_service_names_and_types()]
        except Exception:
            pass

        lifecycle_nodes = []
        for name, ns in all_nodes:
            full = (ns.rstrip("/") + "/" + name).replace("//", "/")
            if not full.startswith("/"):
                full = "/" + full
            svc = f"{full}/get_state"
            if svc in all_services:
                lifecycle_nodes.append((name, ns, full))

        updated = {}
        for name, ns, full in lifecycle_nodes:
            try:
                from lifecycle_msgs.srv import GetState
                cli = self._node.create_client(GetState, f"{full}/get_state")
                if cli.wait_for_service(timeout_sec=0.3):
                    future = cli.call_async(GetState.Request())
                    import rclpy
                    rclpy.spin_until_future_complete(
                        self._node, future, timeout_sec=0.5
                    )
                    if future.result():
                        sid = future.result().current_state.id
                        updated[full] = LifecycleState(
                            name=full,
                            state_id=sid,
                            state_label=LIFECYCLE_LABELS.get(sid, "UNKNOWN"),
                            last_transition=time.monotonic(),
                        )
                self._node.destroy_client(cli)
            except Exception:
                pass

        with self._bus._lock:
            self._bus.lifecycle.update(updated)
