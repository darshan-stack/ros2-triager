# tui/widgets/nodes_table.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import DataTable
from ...state.state_bus import StateBus

STATUS_ICON = {"OK": "●", "ZOMBIE": "☠", "MISSING": "✖"}


class NodesTable(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        tbl = DataTable(id="nodes-dt")
        tbl.add_columns("", "Node", "Namespace", "Pubs", "Subs", "Svcs", "Status")
        yield tbl

    def refresh_data(self) -> None:
        try:
            tbl = self.query_one("#nodes-dt", DataTable)
        except Exception:
            return
        tbl.clear()
        with self.bus._lock:
            nodes = sorted(
                self.bus.nodes.values(),
                key=lambda n: (0 if n.is_zombie else 1, n.name),
            )
        for n in nodes:
            status = "ZOMBIE" if n.is_zombie else n.status
            icon = STATUS_ICON.get(status, "●")
            tbl.add_row(
                icon,
                n.name,
                n.namespace,
                str(len(n.pub_topics)),
                str(len(n.sub_topics)),
                str(len(n.services)),
                status,
                key=n.name,
            )
