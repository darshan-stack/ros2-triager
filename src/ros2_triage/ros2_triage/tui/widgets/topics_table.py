# tui/widgets/topics_table.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import DataTable, Input
from ...state.state_bus import StateBus
from .sparkline import hz_sparkline

STATUS_ICON = {
    "OK": "●",
    "LOW_HZ": "▲",
    "WARN_HZ": "▲",
    "DEAD": "✖",
    "NO_PUB": "✖",
    "STALE": "◌",
    "UNKNOWN": "?",
}

STATUS_CLASS = {
    "OK": "status-ok",
    "LOW_HZ": "status-warn",
    "WARN_HZ": "status-warn",
    "DEAD": "status-dead",
    "NO_PUB": "status-dead",
    "STALE": "status-stale",
}


class TopicsTable(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus
        self._filter = ""

    def compose(self) -> ComposeResult:
        yield Input(placeholder="filter topics…", id="topic-filter")
        tbl = DataTable(id="topics-dt")
        tbl.add_columns("", "Topic", "Hz", "Exp", "BW KB/s", "Pub", "Trend", "Status")
        yield tbl

    def on_input_changed(self, event: Input.Changed) -> None:
        self._filter = event.value.lower()
        self.refresh_data()

    def refresh_data(self) -> None:
        try:
            tbl = self.query_one("#topics-dt", DataTable)
        except Exception:
            return
        tbl.clear()
        with self.bus._lock:
            topics = sorted(
                self.bus.topics.values(),
                key=lambda t: (
                    0 if t.status == "DEAD" else 1 if "WARN" in t.status else 2,
                    t.name,
                ),
            )
        for t in topics:
            if self._filter and self._filter not in t.name.lower():
                continue
            icon = STATUS_ICON.get(t.status, "?")
            exp = f"{t.expected_hz:.1f}" if t.expected_hz else "—"
            spark = hz_sparkline(list(t.hz_history))
            tbl.add_row(
                icon,
                t.name,
                f"{t.actual_hz:.1f}",
                exp,
                f"{t.bandwidth_kbps:.1f}",
                str(t.publisher_count),
                spark,
                t.status,
                key=t.name,
            )
