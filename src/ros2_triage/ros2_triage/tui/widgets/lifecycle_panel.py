# tui/widgets/lifecycle_panel.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import DataTable
from ...state.state_bus import StateBus

STATE_COLORS = {
    "ACTIVE": "green",
    "INACTIVE": "yellow",
    "UNCONFIGURED": "red",
    "FINALIZED": "orange",
    "UNKNOWN": "white",
}

STATE_ICONS = {
    "ACTIVE": "●",
    "INACTIVE": "◑",
    "UNCONFIGURED": "○",
    "FINALIZED": "✖",
    "UNKNOWN": "?",
}


class LifecyclePanel(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        tbl = DataTable(id="lc-dt")
        tbl.add_columns("", "Node", "State", "State ID")
        yield tbl

    def refresh_data(self) -> None:
        try:
            tbl = self.query_one("#lc-dt", DataTable)
        except Exception:
            return
        tbl.clear()
        with self.bus._lock:
            nodes = sorted(
                self.bus.lifecycle.values(),
                key=lambda n: (3 if n.state_label == "ACTIVE" else 0, n.name),
            )
        for n in nodes:
            color = STATE_COLORS.get(n.state_label, "white")
            icon = STATE_ICONS.get(n.state_label, "?")
            tbl.add_row(
                f"[{color}]{icon}[/{color}]",
                n.name,
                f"[{color}]{n.state_label}[/{color}]",
                str(n.state_id),
                key=n.name,
            )
