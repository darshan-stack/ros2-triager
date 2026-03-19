# tui/widgets/diagnostics_panel.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import DataTable
from ...state.state_bus import StateBus

LEVEL_LABELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
LEVEL_COLORS = {0: "green", 1: "yellow", 2: "red", 3: "orange"}


class DiagnosticsPanel(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        tbl = DataTable(id="diag-dt")
        tbl.add_columns("Level", "Hardware ID", "Name", "Message")
        yield tbl

    def refresh_data(self) -> None:
        try:
            tbl = self.query_one("#diag-dt", DataTable)
        except Exception:
            return
        tbl.clear()
        with self.bus._lock:
            items = sorted(
                self.bus.diag.items.values(),
                key=lambda i: (-i.level, i.hardware_id, i.name),
            )
        for item in items:
            level_str = LEVEL_LABELS.get(item.level, "???")
            color = LEVEL_COLORS.get(item.level, "white")
            tbl.add_row(
                f"[{color}]{level_str}[/{color}]",
                item.hardware_id or "—",
                item.name,
                item.message[:80],
                key=f"{item.hardware_id}/{item.name}",
            )
