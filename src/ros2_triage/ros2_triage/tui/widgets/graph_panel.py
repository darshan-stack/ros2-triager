# tui/widgets/graph_panel.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static
from ...state.state_bus import StateBus


def _build_ascii_graph(nodes: list, topics: dict) -> str:
    """Build a simple ASCII representation of the ROS graph."""
    lines = []
    lines.append("[bold]Node → Topic Connections[/bold]\n")
    for node in nodes[:20]:  # Limit to first 20 nodes to avoid overflow
        if node.is_zombie:
            lines.append(f"  [red]☠ {node.name}[/red]")
        else:
            lines.append(f"  [green]● {node.name}[/green]")
        for t in node.pub_topics[:5]:
            ts = topics.get(t)
            if ts:
                status_color = "red" if ts.status in ("DEAD", "NO_PUB") else "yellow" if "WARN" in ts.status else "green"
                lines.append(
                    f"      ├── [dim]pub→[/dim] [{status_color}]{t}[/{status_color}] "
                    f"[dim]({ts.actual_hz:.1f}Hz)[/dim]"
                )
        for t in node.sub_topics[:5]:
            lines.append(f"      └── [dim]sub←[/dim] [blue]{t}[/blue]")
    if not nodes:
        lines.append("[dim]  No nodes discovered yet...[/dim]")
    return "\n".join(lines)


class GraphPanel(Widget):
    """Displays a simplified ASCII node graph."""

    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        yield Static(id="graph-display")

    def refresh_data(self) -> None:
        try:
            display = self.query_one("#graph-display", Static)
        except Exception:
            return
        with self.bus._lock:
            nodes = sorted(
                self.bus.nodes.values(),
                key=lambda n: (0 if n.is_zombie else 1, n.name),
            )
            topics = dict(self.bus.topics)
        display.update(_build_ascii_graph(nodes, topics))
