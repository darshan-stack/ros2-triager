# tui/widgets/odom_map_panel.py
import math
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static
from ...state.state_bus import StateBus


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle in degrees."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class OdomMapPanel(Widget):
    """Displays odometry data and basic map info."""

    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        yield Static(id="odom-display")

    def refresh_data(self) -> None:
        try:
            display = self.query_one("#odom-display", Static)
        except Exception:
            return
        with self.bus._lock:
            o = self.bus.odom

        if o.last_update == 0.0:
            display.update("[dim]No odometry data — /odom not publishing[/dim]")
            return

        yaw = _quat_to_yaw(o.qx, o.qy, o.qz, o.qw)
        text = (
            f"[bold]Position[/bold]\n"
            f"  X: [cyan]{o.x:.3f}[/cyan] m\n"
            f"  Y: [cyan]{o.y:.3f}[/cyan] m\n"
            f"  Z: [cyan]{o.z:.3f}[/cyan] m\n"
            f"  Yaw: [cyan]{yaw:.1f}°[/cyan]\n\n"
            f"[bold]Velocity[/bold]\n"
            f"  Linear:  [yellow]{o.linear_vel:.3f}[/yellow] m/s\n"
            f"  Angular: [yellow]{o.angular_vel:.3f}[/yellow] rad/s\n\n"
            f"[bold]Quaternion[/bold]\n"
            f"  ({o.qx:.3f}, {o.qy:.3f}, {o.qz:.3f}, {o.qw:.3f})\n"
        )
        display.update(text)
