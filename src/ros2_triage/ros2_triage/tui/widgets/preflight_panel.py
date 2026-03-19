# tui/widgets/preflight_panel.py
import time
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static
from ...state.state_bus import StateBus
from ...engine.health_scorer import compute_health


class PreflightPanel(Widget):
    """Run a preflight health check and display pass/fail status."""

    def __init__(self, bus: StateBus, config: dict | None = None):
        super().__init__()
        self.bus = bus
        self.config = config or {}
        self._last_result: dict | None = None

    def compose(self) -> ComposeResult:
        yield Static(id="preflight-display")

    def run_check(self) -> dict:
        """Run a synchronous preflight check. Returns result dict."""
        score = compute_health(self.bus)
        min_health = self.config.get("settings", {}).get("preflight_min_health", 80)

        with self.bus._lock:
            topics = list(self.bus.topics.values())
            nodes = list(self.bus.nodes.values())
            tf = self.bus.tf

        dead_topics = [t.name for t in topics if t.status == "DEAD"]
        zombie_nodes = [n.name for n in nodes if n.is_zombie]

        passed = score.overall >= min_health
        result = {
            "passed": passed,
            "score": score.overall,
            "min_required": min_health,
            "timestamp": time.time(),
            "checks": {
                "topics_score": score.topics_score,
                "nodes_score": score.nodes_score,
                "tf_score": score.tf_score,
                "diag_score": score.diag_score,
                "lifecycle_score": score.lifecycle_score,
            },
            "issues": {
                "dead_topics": dead_topics,
                "zombie_nodes": zombie_nodes,
                "stale_tf_frames": tf.stale_frames,
                "broken_tf_chains": tf.broken_chains,
            },
        }
        self._last_result = result
        self.refresh_data()
        return result

    def refresh_data(self) -> None:
        try:
            display = self.query_one("#preflight-display", Static)
        except Exception:
            return

        if not self._last_result:
            display.update(
                "[dim]Press [bold]p[/bold] to run preflight check[/dim]"
            )
            return

        r = self._last_result
        status_color = "green" if r["passed"] else "red"
        status_label = "✔ PASSED" if r["passed"] else "✖ FAILED"
        ts = time.strftime("%H:%M:%S", time.localtime(r["timestamp"]))

        lines = [
            f"[bold]Preflight Check — {ts}[/bold]\n",
            f"[{status_color}]{status_label}[/{status_color}] — Score: "
            f"[bold]{r['score']}/100[/bold] (required ≥ {r['min_required']})\n",
            f"[bold]Subsystem Scores:[/bold]",
            f"  Topics:    {r['checks']['topics_score']:3d}/100",
            f"  Nodes:     {r['checks']['nodes_score']:3d}/100",
            f"  TF:        {r['checks']['tf_score']:3d}/100",
            f"  Diag:      {r['checks']['diag_score']:3d}/100",
            f"  Lifecycle: {r['checks']['lifecycle_score']:3d}/100",
        ]

        issues = r["issues"]
        if any(v for v in issues.values()):
            lines.append("\n[bold]Issues:[/bold]")
            if issues["dead_topics"]:
                lines.append(f"  [red]Dead topics ({len(issues['dead_topics'])}):[/red]")
                for t in issues["dead_topics"][:5]:
                    lines.append(f"    - {t}")
            if issues["zombie_nodes"]:
                lines.append(f"  [red]Zombie nodes ({len(issues['zombie_nodes'])}):[/red]")
                for n in issues["zombie_nodes"][:5]:
                    lines.append(f"    - {n}")
            if issues["stale_tf_frames"]:
                lines.append(f"  [yellow]Stale TF ({len(issues['stale_tf_frames'])}):[/yellow]")
                for f in issues["stale_tf_frames"][:5]:
                    lines.append(f"    - {f}")
        else:
            lines.append("\n[green]No issues detected.[/green]")

        display.update("\n".join(lines))
