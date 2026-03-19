# tui/widgets/health_banner.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static
from ...state.state_bus import StateBus


class HealthCard(Static):
    def __init__(self, label: str, value: str = "—", css_class: str = "ok"):
        super().__init__()
        self._label = label
        self._value = value
        self._css = css_class

    def render(self) -> str:
        return f"[dim]{self._label}[/]\n[bold]{self._value}[/]"

    def update_value(self, value: str, css_class: str = "ok") -> None:
        self._value = value
        self._css = css_class
        self.remove_class("ok", "degraded", "critical")
        self.add_class(css_class)
        self.refresh()


class HealthBanner(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        yield HealthCard("SYSTEM HEALTH", "—", "ok")
        yield HealthCard("TOPICS", "—")
        yield HealthCard("NODES", "—")
        yield HealthCard("TF FRAMES", "—")
        yield HealthCard("/diagnostics", "—")
        yield HealthCard("LIFECYCLE", "—")

    def update_health(self) -> None:
        with self.bus._lock:
            h = self.bus.health
            topics = list(self.bus.topics.values())
            nodes = list(self.bus.nodes.values())
            tf = self.bus.tf
            diag = self.bus.diag
            lc = list(self.bus.lifecycle.values())

        cards = list(self.query(HealthCard))
        if not cards:
            return

        # Overall health
        cards[0].update_value(f"{h.overall}/100 [{h.label}]", h.color)

        # Topics
        dead = sum(1 for t in topics if t.status == "DEAD")
        cards[1].update_value(
            f"{len(topics) - dead}/{len(topics)}",
            "ok" if dead == 0 else "critical" if dead > 2 else "degraded",
        )

        # Nodes
        zombies = sum(1 for n in nodes if n.is_zombie)
        cards[2].update_value(
            f"{len(nodes) - zombies}/{len(nodes)}",
            "ok" if zombies == 0 else "critical",
        )

        # TF
        stale = len(tf.stale_frames)
        cards[3].update_value(
            f"{len(tf.frames)} frames",
            "ok" if stale == 0 else "degraded",
        )

        # Diagnostics
        cards[4].update_value(
            f"{diag.error_count} ERR {diag.warn_count} WARN",
            "critical" if diag.error_count > 0 else "degraded" if diag.warn_count > 0 else "ok",
        )

        # Lifecycle
        inactive = sum(1 for l in lc if l.state_id != 3)
        cards[5].update_value(
            f"{len(lc) - inactive}/{len(lc)} ACTIVE",
            "ok" if inactive == 0 else "degraded",
        )
