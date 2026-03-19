# tui/widgets/inspect_panel.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static
from ...state.state_bus import StateBus


class InspectPanel(Widget):
    """Detailed inspection panel for a selected topic or node."""

    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus
        self._subject: str | None = None
        self._subject_type: str = "topic"  # "topic" | "node"

    def inspect_topic(self, topic_name: str) -> None:
        self._subject = topic_name
        self._subject_type = "topic"
        self.refresh_data()

    def inspect_node(self, node_name: str) -> None:
        self._subject = node_name
        self._subject_type = "node"
        self.refresh_data()

    def compose(self) -> ComposeResult:
        yield Static(id="inspect-display")

    def refresh_data(self) -> None:
        try:
            display = self.query_one("#inspect-display", Static)
        except Exception:
            return

        if not self._subject:
            display.update("[dim]Press Enter on a row to inspect[/dim]")
            return

        if self._subject_type == "topic":
            self._render_topic(display)
        else:
            self._render_node(display)

    def _render_topic(self, display: Static) -> None:
        with self.bus._lock:
            t = self.bus.topics.get(self._subject)
        if not t:
            display.update(f"[red]Topic '{self._subject}' not found[/red]")
            return

        from .sparkline import hz_sparkline
        spark = hz_sparkline(list(t.hz_history), width=30)
        lines = [
            f"[bold]Topic:[/bold] [cyan]{t.name}[/cyan]",
            f"[bold]Type:[/bold]  {t.msg_type}",
            f"[bold]Status:[/bold] [{'red' if t.status in ('DEAD','NO_PUB') else 'yellow' if 'WARN' in t.status else 'green'}]{t.status}[/]",
            f"",
            f"[bold]Hz:[/bold]    {t.actual_hz:.3f} Hz",
            f"[bold]Exp Hz:[/bold] {t.expected_hz:.1f} Hz" if t.expected_hz else "[bold]Exp Hz:[/bold] —",
            f"[bold]BW:[/bold]    {t.bandwidth_kbps:.2f} KB/s",
            f"",
            f"[bold]Publishers:[/bold]  {t.publisher_count}",
            f"[bold]Subscribers:[/bold] {t.subscriber_count}",
            f"[bold]Critical:[/bold]    {'✓' if t.is_critical else '✗'}",
            f"",
            f"[bold]Hz trend:[/bold] {spark}",
        ]
        display.update("\n".join(lines))

    def _render_node(self, display: Static) -> None:
        with self.bus._lock:
            n = self.bus.nodes.get(self._subject)
        if not n:
            display.update(f"[red]Node '{self._subject}' not found[/red]")
            return

        zombie_str = "[red]ZOMBIE[/red]" if n.is_zombie else "[green]OK[/green]"
        pub_list = "\n".join(f"    - {t}" for t in n.pub_topics[:20]) or "    (none)"
        sub_list = "\n".join(f"    - {t}" for t in n.sub_topics[:20]) or "    (none)"
        lines = [
            f"[bold]Node:[/bold]      [cyan]{n.name}[/cyan]",
            f"[bold]Namespace:[/bold] {n.namespace}",
            f"[bold]Status:[/bold]    {zombie_str}",
            f"",
            f"[bold]Publishes ({len(n.pub_topics)}):[/bold]",
            pub_list,
            f"",
            f"[bold]Subscribes ({len(n.sub_topics)}):[/bold]",
            sub_list,
            f"",
            f"[bold]Services ({len(n.services)}):[/bold]",
            "\n".join(f"    - {s}" for s in n.services[:10]) or "    (none)",
        ]
        display.update("\n".join(lines))
