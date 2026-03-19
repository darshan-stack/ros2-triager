# tui/widgets/alerts_log.py
import time
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import DataTable
from ...state.state_bus import StateBus

SEVERITY_COLORS = {
    "ERROR": "red",
    "WARN": "yellow",
    "INFO": "cyan",
}


class AlertsLog(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        tbl = DataTable(id="alerts-dt")
        tbl.add_columns("Time", "Sev", "Source", "Message")
        yield tbl

    def refresh_data(self) -> None:
        try:
            tbl = self.query_one("#alerts-dt", DataTable)
        except Exception:
            return
        tbl.clear()
        with self.bus._lock:
            alerts = list(self.bus.alerts)[:100]
        for a in alerts:
            color = SEVERITY_COLORS.get(a.severity, "white")
            ts = time.strftime("%H:%M:%S", time.localtime(a.timestamp))
            tbl.add_row(
                ts,
                f"[{color}]{a.severity}[/{color}]",
                a.source,
                a.message[:80],
            )
