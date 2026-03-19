# tui/app.py
from __future__ import annotations
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Header, Footer, TabbedContent, TabPane
from ..state.state_bus import StateBus
from ..engine.health_scorer import compute_health
from ..engine.dead_topic_detector import classify_topic
from ..engine.zombie_node_detector import is_zombie
from .widgets.health_banner import HealthBanner
from .widgets.topics_table import TopicsTable
from .widgets.nodes_table import NodesTable
from .widgets.tf_tree import TFTreeWidget
from .widgets.diagnostics_panel import DiagnosticsPanel
from .widgets.lifecycle_panel import LifecyclePanel
from .widgets.alerts_log import AlertsLog
from .widgets.graph_panel import GraphPanel
from .widgets.odom_map_panel import OdomMapPanel
from .widgets.inspect_panel import InspectPanel
from .widgets.preflight_panel import PreflightPanel
from .screens.help_screen import HelpScreen


class TriagerApp(App):
    CSS_PATH = "theme.tcss"
    TITLE = "ros2-triager v2"
    SUB_TITLE = "ROS2 Diagnostic TUI"

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("h", "show_help", "Help"),
        Binding("s", "snapshot", "Snapshot"),
        Binding("r", "reload", "Reload"),
        Binding("p", "preflight", "Preflight"),
        Binding("f", "filter", "Filter"),
        Binding("d", "change_domain", "Domain"),
        Binding("a", "switch_tab('alerts')", "Alerts"),
        Binding("1", "switch_tab('topics')", "Topics"),
        Binding("2", "switch_tab('nodes')", "Nodes"),
        Binding("3", "switch_tab('tf')", "TF"),
        Binding("4", "switch_tab('diagnostics')", "Diagnostics"),
        Binding("5", "switch_tab('lifecycle')", "Lifecycle"),
        Binding("6", "switch_tab('graph')", "Graph"),
        Binding("7", "switch_tab('odom')", "Odom/Map"),
        Binding("8", "switch_tab('alerts')", "Alerts"),
        Binding("9", "switch_tab('preflight')", "Preflight"),
        Binding("+", "faster_refresh", "Faster", show=False),
        Binding("-", "slower_refresh", "Slower", show=False),
    ]

    SCREENS = {"help": HelpScreen}

    def __init__(self, bus: StateBus, config: dict | None = None):
        super().__init__()
        self.bus = bus
        self.config = config or {}

    def compose(self) -> ComposeResult:
        yield Header()
        yield HealthBanner(self.bus)
        with TabbedContent(initial="topics"):
            with TabPane("Topics", id="topics"):
                yield TopicsTable(self.bus)
            with TabPane("Nodes", id="nodes"):
                yield NodesTable(self.bus)
            with TabPane("TF Tree", id="tf"):
                yield TFTreeWidget(self.bus)
            with TabPane("Diagnostics", id="diagnostics"):
                yield DiagnosticsPanel(self.bus)
            with TabPane("Lifecycle", id="lifecycle"):
                yield LifecyclePanel(self.bus)
            with TabPane("Graph", id="graph"):
                yield GraphPanel(self.bus)
            with TabPane("Odom/Map", id="odom"):
                yield OdomMapPanel(self.bus)
            with TabPane("Alerts", id="alerts"):
                yield AlertsLog(self.bus)
            with TabPane("Preflight", id="preflight"):
                yield PreflightPanel(self.bus, self.config)
        yield Footer()

    def on_mount(self) -> None:
        # Refresh TUI every 1 second from bus state
        self._refresh_interval: float = 1.0
        self._refresh_timer = self.set_interval(self._refresh_interval, self._refresh_all)

    def _refresh_all(self) -> None:
        # Recompute health score
        score = compute_health(self.bus)
        with self.bus._lock:
            self.bus.health = score

        # Reclassify all topics
        with self.bus._lock:
            for t in self.bus.topics.values():
                t.status = classify_topic(t)

        # Reclassify zombie nodes
        with self.bus._lock:
            for n in self.bus.nodes.values():
                n.is_zombie = is_zombie(n, self.bus.topics)

        # Notify all widgets to refresh
        try:
            self.query_one(HealthBanner).update_health()
        except Exception:
            pass
        for widget_cls in [
            TopicsTable, NodesTable, TFTreeWidget,
            DiagnosticsPanel, LifecyclePanel, GraphPanel,
            OdomMapPanel, AlertsLog, PreflightPanel,
        ]:
            try:
                self.query_one(widget_cls).refresh_data()
            except Exception:
                pass

    def action_snapshot(self) -> None:
        from ..export.snapshot_exporter import export_snapshot
        import json
        import datetime
        path = (
            f"/tmp/ros2_triage_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        with open(path, "w") as f:
            json.dump(export_snapshot(self.bus), f, indent=2, default=str)
        self.notify(f"Snapshot saved: {path}")

    def action_switch_tab(self, tab_id: str) -> None:
        try:
            self.query_one(TabbedContent).active = tab_id
        except Exception:
            pass

    def action_filter(self) -> None:
        try:
            inp = self.query_one("#topic-filter")
            inp.focus()
        except Exception:
            pass

    def action_preflight(self) -> None:
        try:
            panel = self.query_one(PreflightPanel)
            panel.run_check()
            self.action_switch_tab("preflight")
        except Exception:
            pass

    def action_reload(self) -> None:
        self.notify("Reloading — topics/nodes will rediscover within 2s")

    def action_show_help(self) -> None:
        """Show the help overlay screen."""
        self.push_screen(HelpScreen())

    def action_change_domain(self) -> None:
        """Placeholder: change ROS_DOMAIN_ID (requires restart)."""
        import os
        current = os.environ.get("ROS_DOMAIN_ID", "0")
        self.notify(f"Current ROS_DOMAIN_ID={current}. Restart with --domain N to change.")

    def action_faster_refresh(self) -> None:
        """Speed up TUI refresh rate (min 0.25s)."""
        self._refresh_interval = max(0.25, self._refresh_interval / 2)
        self._refresh_timer.stop()
        self._refresh_timer = self.set_interval(self._refresh_interval, self._refresh_all)
        self.notify(f"Refresh: every {self._refresh_interval:.2f}s")

    def action_slower_refresh(self) -> None:
        """Slow down TUI refresh rate (max 10s)."""
        self._refresh_interval = min(10.0, self._refresh_interval * 2)
        self._refresh_timer.stop()
        self._refresh_timer = self.set_interval(self._refresh_interval, self._refresh_all)
        self.notify(f"Refresh: every {self._refresh_interval:.2f}s")
