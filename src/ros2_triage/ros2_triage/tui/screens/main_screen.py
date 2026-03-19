# tui/screens/main_screen.py
from textual.app import ComposeResult
from textual.screen import Screen
from textual.widgets import Header, Footer, TabbedContent, TabPane
from ..widgets.health_banner import HealthBanner
from ..widgets.topics_table import TopicsTable
from ..widgets.nodes_table import NodesTable
from ..widgets.tf_tree import TFTreeWidget
from ..widgets.diagnostics_panel import DiagnosticsPanel
from ..widgets.lifecycle_panel import LifecyclePanel
from ..widgets.graph_panel import GraphPanel
from ..widgets.odom_map_panel import OdomMapPanel
from ..widgets.alerts_log import AlertsLog
from ..widgets.preflight_panel import PreflightPanel
from ...state.state_bus import StateBus


class MainScreen(Screen):
    """The primary tabbed dashboard screen."""

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
