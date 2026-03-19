# Copyright 2024 darshan - Apache-2.0
"""
interactive_tui.py - Interactive Terminal User Interface (DEPRECATED)

.. deprecated::
    This module is superseded by the Textual-based TUI in ``ros2_triage.tui.app``.
    Use ``ros2 triage tui`` to launch the new Textual TUI.
    This file is kept for backwards compatibility only and may be removed in a
    future release.

Legacy Rich-based keyboard navigation TUI:
  - Arrow keys: Navigate between findings
  - Enter: Expand/collapse finding details
  - q: Quit
  - r: Refresh
  - h: Help
  - l: Toggle live log tail
  - d: Toggle deep dive panel
  - s: Toggle system metrics

Performance target: <50ms refresh rate
"""
import warnings as _warnings
_warnings.warn(
    "interactive_tui.py is deprecated. Use 'ros2 triage tui' for the new Textual TUI.",
    DeprecationWarning,
    stacklevel=2,
)

import sys
import time
import threading
from typing import List, Optional, Callable
from dataclasses import dataclass, field

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from rich.live import Live
    from rich import box
    HAS_RICH = True
except ImportError:
    HAS_RICH = False

try:
    import termios
    import tty
    import select
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

from .checks.finding import Finding, SEVERITY_LABEL
from .correlation_engine import Hypothesis


@dataclass
class TUIState:
    """Current state of the interactive TUI."""
    selected_index: int = 0
    expanded_index: int = -1  # -1 means none expanded
    show_help: bool = False
    show_deep_dive: bool = True
    show_logs: bool = False
    show_metrics: bool = True
    scroll_offset: int = 0
    max_visible: int = 10
    
    # Data
    findings: List[Finding] = field(default_factory=list)
    hypotheses: List[Hypothesis] = field(default_factory=list)
    system_metrics: dict = field(default_factory=dict)
    log_lines: List[str] = field(default_factory=list)
    
    # Callbacks
    on_refresh: Optional[Callable] = None
    on_quit: Optional[Callable] = None


class KeyboardHandler:
    """Non-blocking keyboard input handler."""
    
    def __init__(self):
        self._stop_event = threading.Event()
        self._key_buffer = []
        self._lock = threading.Lock()
        self._thread = None
        self._old_settings = None
    
    def start(self):
        """Start listening for keyboard input."""
        if not HAS_TERMIOS:
            return
        
        try:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            return
        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_keys, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Stop listening for keyboard input."""
        self._stop_event.set()
        
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
        
        if self._thread is not None:
            self._thread.join(timeout=0.5)
    
    def _read_keys(self):
        """Background thread to read keyboard input."""
        while not self._stop_event.is_set():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    # Handle escape sequences (arrow keys)
                    if key == '\x1b':
                        # Read additional characters for escape sequence
                        if select.select([sys.stdin], [], [], 0.05)[0]:
                            key += sys.stdin.read(1)
                            if select.select([sys.stdin], [], [], 0.05)[0]:
                                key += sys.stdin.read(1)
                    
                    with self._lock:
                        self._key_buffer.append(key)
            except Exception:
                break
    
    def get_key(self) -> Optional[str]:
        """Get next key from buffer, or None if empty."""
        with self._lock:
            if self._key_buffer:
                return self._key_buffer.pop(0)
        return None
    
    def parse_key(self, key: str) -> str:
        """Parse raw key into readable name."""
        if key == '\x1b[A':
            return 'up'
        elif key == '\x1b[B':
            return 'down'
        elif key == '\x1b[C':
            return 'right'
        elif key == '\x1b[D':
            return 'left'
        elif key == '\r' or key == '\n':
            return 'enter'
        elif key == '\x1b':
            return 'escape'
        elif key == ' ':
            return 'space'
        else:
            return key.lower()


class InteractiveTUI:
    """Interactive terminal UI with keyboard navigation."""
    
    def __init__(self, refresh_callback: Callable = None):
        self.console = Console() if HAS_RICH else None
        self.state = TUIState()
        self.keyboard = KeyboardHandler()
        self._running = False
        self._refresh_callback = refresh_callback
        self._last_refresh = 0
        self._min_refresh_interval = 0.05  # 50ms target
    
    def update_data(
        self,
        findings: List[Finding] = None,
        hypotheses: List[Hypothesis] = None,
        system_metrics: dict = None,
        log_lines: List[str] = None
    ):
        """Update TUI data without blocking."""
        if findings is not None:
            self.state.findings = findings
            # Reset selection if out of bounds
            if self.state.selected_index >= len(findings):
                self.state.selected_index = max(0, len(findings) - 1)
        
        if hypotheses is not None:
            self.state.hypotheses = hypotheses
        
        if system_metrics is not None:
            self.state.system_metrics = system_metrics
        
        if log_lines is not None:
            self.state.log_lines = log_lines
    
    def _handle_input(self, key: str):
        """Handle parsed keyboard input."""
        if key == 'q':
            self._running = False
            if self.state.on_quit:
                self.state.on_quit()
        
        elif key == 'up':
            if self.state.selected_index > 0:
                self.state.selected_index -= 1
                self._adjust_scroll()
        
        elif key == 'down':
            if self.state.selected_index < len(self.state.findings) - 1:
                self.state.selected_index += 1
                self._adjust_scroll()
        
        elif key == 'enter' or key == 'space':
            if self.state.expanded_index == self.state.selected_index:
                self.state.expanded_index = -1  # Collapse
            else:
                self.state.expanded_index = self.state.selected_index  # Expand
        
        elif key == 'r':
            if self._refresh_callback:
                self._refresh_callback()
        
        elif key == 'h':
            self.state.show_help = not self.state.show_help
        
        elif key == 'l':
            self.state.show_logs = not self.state.show_logs
        
        elif key == 'd':
            self.state.show_deep_dive = not self.state.show_deep_dive
        
        elif key == 's':
            self.state.show_metrics = not self.state.show_metrics
        
        elif key == 'escape':
            self.state.show_help = False
            self.state.expanded_index = -1
    
    def _adjust_scroll(self):
        """Adjust scroll offset to keep selected item visible."""
        if self.state.selected_index < self.state.scroll_offset:
            self.state.scroll_offset = self.state.selected_index
        elif self.state.selected_index >= self.state.scroll_offset + self.state.max_visible:
            self.state.scroll_offset = self.state.selected_index - self.state.max_visible + 1
    
    def _build_layout(self) -> Layout:
        """Build the Rich layout for current state."""
        layout = Layout()
        
        # Main structure
        layout.split_column(
            Layout(name="header", size=4),
            Layout(name="body"),
            Layout(name="footer", size=3)
        )
        
        # Body split based on toggles
        if self.state.show_help:
            layout["body"].update(self._build_help_panel())
        elif self.state.show_deep_dive and self.state.show_logs:
            layout["body"].split_row(
                Layout(name="findings", ratio=2),
                Layout(name="sidebar", ratio=1)
            )
            layout["body"]["sidebar"].split_column(
                Layout(name="details", ratio=1),
                Layout(name="logs", ratio=1)
            )
            layout["body"]["findings"].update(self._build_findings_table())
            layout["body"]["sidebar"]["details"].update(self._build_details_panel())
            layout["body"]["sidebar"]["logs"].update(self._build_logs_panel())
        elif self.state.show_deep_dive:
            layout["body"].split_row(
                Layout(name="findings", ratio=2),
                Layout(name="details", ratio=1)
            )
            layout["body"]["findings"].update(self._build_findings_table())
            layout["body"]["details"].update(self._build_details_panel())
        elif self.state.show_logs:
            layout["body"].split_row(
                Layout(name="findings", ratio=2),
                Layout(name="logs", ratio=1)
            )
            layout["body"]["findings"].update(self._build_findings_table())
            layout["body"]["logs"].update(self._build_logs_panel())
        else:
            layout["body"].update(self._build_findings_table())
        
        layout["header"].update(self._build_header())
        layout["footer"].update(self._build_footer())
        
        return layout
    
    def _build_header(self) -> Panel:
        """Build header with system metrics."""
        text = Text()
        text.append("ROS 2 Triager", style="bold cyan")
        text.append(" - Interactive Dashboard\n", style="dim")
        
        if self.state.show_metrics and self.state.system_metrics:
            metrics = self.state.system_metrics
            text.append(f"  CPU: {metrics.get('avg_cpu', 0):.0f}%", 
                       style="green" if metrics.get('avg_cpu', 0) < 70 else "yellow")
            text.append(f"  |  Hz: {metrics.get('avg_hz', 0):.1f}", style="cyan")
            text.append(f"  |  Findings: {len(self.state.findings)}", style="white")
        
        return Panel(text, box=box.ROUNDED, border_style="cyan")
    
    def _build_findings_table(self) -> Panel:
        """Build findings table with selection highlight."""
        if not self.state.findings:
            return Panel(
                Text("No findings", style="bold green"),
                title="FINDINGS",
                box=box.ROUNDED,
                border_style="green"
            )
        
        table = Table(
            box=box.SIMPLE,
            show_header=True,
            header_style="bold",
            expand=True
        )
        table.add_column("", width=2)  # Selection indicator
        table.add_column("SEV", width=5)
        table.add_column("TOPIC", style="cyan", ratio=1)
        table.add_column("MESSAGE", ratio=2)
        
        # Calculate visible range
        start = self.state.scroll_offset
        end = min(start + self.state.max_visible, len(self.state.findings))
        
        for i in range(start, end):
            finding = self.state.findings[i]
            
            # Selection indicator
            if i == self.state.selected_index:
                indicator = ">"
                row_style = "reverse"
            else:
                indicator = " "
                row_style = ""
            
            # Severity styling
            sev_style = {3: "bold red", 2: "bold yellow", 1: "bold cyan"}.get(
                finding.severity, "white"
            )
            sev_label = SEVERITY_LABEL.get(finding.severity, "????")
            
            # Message (truncate if not expanded)
            message = finding.message
            if i == self.state.expanded_index:
                message += f"\n  Suggestion: {finding.suggestion}"
            elif len(message) > 50:
                message = message[:50] + "..."
            
            table.add_row(
                Text(indicator, style="bold green" if i == self.state.selected_index else ""),
                Text(sev_label, style=sev_style),
                finding.topic,
                message,
                style=row_style
            )
        
        # Count by severity
        crits = sum(1 for f in self.state.findings if f.severity == 3)
        warns = sum(1 for f in self.state.findings if f.severity == 2)
        infos = sum(1 for f in self.state.findings if f.severity == 1)
        
        title = f"FINDINGS [{crits}C {warns}W {infos}I] ({start+1}-{end}/{len(self.state.findings)})"
        border_color = "red" if crits > 0 else ("yellow" if warns > 0 else "cyan")
        
        return Panel(table, title=title, box=box.ROUNDED, border_style=border_color)
    
    def _build_details_panel(self) -> Panel:
        """Build detailed view for selected finding."""
        if self.state.selected_index < 0 or self.state.selected_index >= len(self.state.findings):
            return Panel(
                Text("Select a finding to view details", style="dim"),
                title="DETAILS",
                box=box.ROUNDED,
                border_style="blue"
            )
        
        finding = self.state.findings[self.state.selected_index]
        
        text = Text()
        text.append(f"Topic: {finding.topic}\n", style="bold cyan")
        text.append(f"Check: {finding.check}\n", style="dim")
        text.append(f"Severity: {SEVERITY_LABEL.get(finding.severity, '?')}\n\n", 
                   style="bold red" if finding.severity == 3 else 
                         "bold yellow" if finding.severity == 2 else "bold cyan")
        
        text.append("Message:\n", style="bold white")
        text.append(f"  {finding.message}\n\n", style="white")
        
        text.append("Suggestion:\n", style="bold green")
        text.append(f"  {finding.suggestion}\n", style="green")
        
        # Show correlation evidence if available
        if self.state.hypotheses:
            matching = [h for h in self.state.hypotheses if h.topic == finding.topic]
            if matching:
                hyp = matching[0]
                text.append(f"\nCorrelation: {int(hyp.confidence() * 100)}%\n", style="bold magenta")
                text.append(f"Root Cause: {hyp.root_cause}\n", style="magenta")
        
        return Panel(text, title="DETAILS", box=box.ROUNDED, border_style="blue")
    
    def _build_logs_panel(self) -> Panel:
        """Build live log tail panel."""
        if not self.state.log_lines:
            text = Text("No log data available\n", style="dim")
            text.append("Run with journalctl access\nfor live log streaming.", style="dim")
        else:
            # Show last 10 lines
            text = Text()
            for line in self.state.log_lines[-10:]:
                # Color based on log level
                if 'error' in line.lower() or 'fatal' in line.lower():
                    style = "red"
                elif 'warn' in line.lower():
                    style = "yellow"
                else:
                    style = "dim"
                text.append(f"{line}\n", style=style)
        
        return Panel(text, title="LOGS", box=box.ROUNDED, border_style="magenta")
    
    def _build_help_panel(self) -> Panel:
        """Build help overlay panel."""
        text = Text()
        text.append("Keyboard Controls\n\n", style="bold cyan")
        
        controls = [
            ("Up/Down", "Navigate findings"),
            ("Enter/Space", "Expand/collapse finding"),
            ("r", "Refresh data"),
            ("d", "Toggle deep dive panel"),
            ("l", "Toggle log panel"),
            ("s", "Toggle system metrics"),
            ("h", "Toggle this help"),
            ("q", "Quit"),
            ("Esc", "Close panels"),
        ]
        
        for key, desc in controls:
            text.append(f"  {key:12}", style="bold green")
            text.append(f" {desc}\n", style="white")
        
        text.append("\n\nPress any key to close help...", style="dim")
        
        return Panel(text, title="HELP", box=box.DOUBLE, border_style="cyan")
    
    def _build_footer(self) -> Panel:
        """Build footer with key hints."""
        hints = "[q]uit  [r]efresh  [h]elp  [d]etails  [l]ogs  [s]tats"
        
        # Status indicator
        crits = sum(1 for f in self.state.findings if f.severity == 3)
        if crits > 0:
            status = Text("CRITICAL", style="bold red")
        elif any(f.severity == 2 for f in self.state.findings):
            status = Text("WARNING", style="bold yellow")
        else:
            status = Text("HEALTHY", style="bold green")
        
        text = Text()
        text.append("Status: ")
        text.append_text(status)
        text.append(f"  |  {hints}", style="dim")
        
        return Panel(text, box=box.ROUNDED, border_style="white")
    
    def run(self, refresh_interval: float = 1.0):
        """
        Run the interactive TUI.
        
        Args:
            refresh_interval: Seconds between auto-refresh
        """
        if not HAS_RICH:
            print("Rich library not available. Install with: pip install rich")
            return
        
        self._running = True
        self.keyboard.start()
        
        try:
            with Live(
                self._build_layout(),
                console=self.console,
                refresh_per_second=20,  # ~50ms
                screen=True
            ) as live:
                last_auto_refresh = time.time()
                
                while self._running:
                    # Handle keyboard input
                    key = self.keyboard.get_key()
                    if key:
                        parsed = self.keyboard.parse_key(key)
                        self._handle_input(parsed)
                    
                    # Auto-refresh
                    now = time.time()
                    if self._refresh_callback and (now - last_auto_refresh) >= refresh_interval:
                        self._refresh_callback()
                        last_auto_refresh = now
                    
                    # Update display (respecting refresh rate limit)
                    if (now - self._last_refresh) >= self._min_refresh_interval:
                        live.update(self._build_layout())
                        self._last_refresh = now
                    
                    # Small sleep to prevent CPU spin
                    time.sleep(0.01)
        
        finally:
            self.keyboard.stop()
    
    def render_once(self) -> None:
        """Render the TUI once without interaction."""
        if not HAS_RICH:
            self._print_basic()
            return
        
        self.console.print(self._build_layout())
    
    def _print_basic(self):
        """Fallback basic text output."""
        print("\n" + "=" * 60)
        print("  ROS 2 Triager - Diagnostic Report")
        print("=" * 60 + "\n")
        
        if not self.state.findings:
            print("  [OK] No issues found\n")
            return
        
        for f in self.state.findings:
            sev_label = SEVERITY_LABEL.get(f.severity, "????")
            print(f"  [{sev_label}] {f.topic}")
            print(f"         {f.message}")
            print(f"         Suggestion: {f.suggestion}\n")


def run_interactive(
    findings: List[Finding],
    hypotheses: List[Hypothesis] = None,
    system_metrics: dict = None,
    refresh_callback: Callable = None,
    refresh_interval: float = 5.0
):
    """
    Convenience function to run interactive TUI.
    
    Args:
        findings: Initial findings list
        hypotheses: Correlation hypotheses
        system_metrics: System metrics dict
        refresh_callback: Function to call for data refresh
        refresh_interval: Auto-refresh interval in seconds
    """
    tui = InteractiveTUI(refresh_callback=refresh_callback)
    tui.update_data(findings, hypotheses, system_metrics)
    tui.run(refresh_interval=refresh_interval)
