# tui/screens/config_screen.py
from __future__ import annotations
from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Static, Label
from textual.binding import Binding


class ConfigScreen(ModalScreen):
    """Config screen — shows active configuration key-value pairs."""

    BINDINGS = [Binding("escape,q", "dismiss", "Close")]

    def __init__(self, config: dict | None = None):
        super().__init__()
        self.config = config or {}

    def compose(self) -> ComposeResult:
        settings = self.config.get("settings", {})
        lines = ["[bold cyan]Active Configuration[/bold cyan]\n"]
        if settings:
            for k, v in settings.items():
                lines.append(f"  [yellow]{k}[/yellow]: {v}")
        else:
            lines.append("  [dim]No custom config loaded (zero-config mode)[/dim]")

        monitored = self.config.get("monitored_topics", [])
        if monitored:
            lines.append(f"\n[bold]Monitored Topics ({len(monitored)}):[/bold]")
            for t in monitored[:10]:
                name = t.get("name", t.get("topic", "?"))
                hz = t.get("expected_hz", "?")
                lines.append(f"  {name}  [{hz} Hz]")

        lines.append("\n[dim]Press Escape or q to close[/dim]")
        yield Static("\n".join(lines), id="config-content")

    def on_key(self, event) -> None:
        self.dismiss()
