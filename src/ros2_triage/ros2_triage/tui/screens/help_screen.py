# tui/screens/help_screen.py
from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Static
from textual.binding import Binding

HELP_TEXT = """\
[bold cyan]ros2-triager TUI — Keyboard Reference[/bold cyan]

[bold]Navigation[/bold]
  [yellow]1–9[/yellow]        Switch tab directly (Topics, Nodes, TF…)
  [yellow]Tab / Shift+Tab[/yellow]  Cycle panel focus
  [yellow]↑ ↓[/yellow]        Navigate rows in tables
  [yellow]Enter[/yellow]      Inspect selected row

[bold]Actions[/bold]
  [yellow]s[/yellow]          Export JSON snapshot to /tmp/
  [yellow]r[/yellow]          Reload config + rediscover all topics/nodes
  [yellow]p[/yellow]          Run preflight health check
  [yellow]f[/yellow]          Toggle filter bar (Topics tab)
  [yellow]a[/yellow]          Jump to Alerts log tab
  [yellow]+[/yellow] / [yellow]-[/yellow]     Faster / slower refresh rate

[bold]Display[/bold]
  [yellow]d[/yellow]          Change ROS_DOMAIN_ID
  [yellow]h[/yellow]          Toggle this help overlay
  [yellow]q[/yellow]          Quit

[bold]Status Icons[/bold]
  [green]●[/green] OK         [yellow]▲[/yellow] WARN_HZ / LOW_HZ
  [red]✖[/red] DEAD/NO_PUB  [dim]◌[/dim] STALE / UNKNOWN

[bold]Health Score Thresholds[/bold]
  [green]80–100[/green]  HEALTHY   [yellow]50–79[/yellow]  DEGRADED   [red]0–49[/red]  CRITICAL

[dim]Press any key to close[/dim]
"""


class HelpScreen(ModalScreen):
    """Help overlay screen with keyboard reference."""

    BINDINGS = [Binding("escape,h,q", "dismiss", "Close")]

    def compose(self) -> ComposeResult:
        yield Static(HELP_TEXT, id="help-content")

    def on_key(self, event) -> None:
        self.dismiss()
