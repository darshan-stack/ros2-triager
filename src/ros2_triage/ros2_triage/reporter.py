# Copyright 2024 darshan - Apache-2.0
"""
reporter.py - Formats and prints triage findings.

Provides multiple output formats:
  print_human()  - Coloured terminal output (Rich or colorama fallback)
  print_json()   - Structured JSON for CI pipelines
  print_rich()   - Rich library dashboard (if available)

The reporter auto-detects available libraries and uses the best option.
"""

import json
import sys
from datetime import datetime
from typing import List, Optional

from .checks.finding import Finding, SEVERITY_LABEL

# Try Rich first (preferred), then colorama as fallback
try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.text import Text
    from rich import box
    HAS_RICH = True
except ImportError:
    HAS_RICH = False

try:
    import colorama
    from colorama import Fore, Style, Back
    colorama.init(autoreset=True)
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = False


# ── Color helpers (colorama fallback) ─────────────────────────────────────────

def _c(color_code: str, text: str) -> str:
    if HAS_COLOR:
        return color_code + text + Style.RESET_ALL
    return text


def _bold(text: str) -> str:
    if HAS_COLOR:
        return Style.BRIGHT + text + Style.RESET_ALL
    return text


def _sev_color(severity: int) -> str:
    if not HAS_COLOR:
        return ''
    return {3: Fore.RED, 2: Fore.YELLOW, 1: Fore.CYAN}.get(severity, '')


def _ok() -> str:
    if HAS_COLOR:
        return Fore.GREEN + '[OK]' + Style.RESET_ALL
    return '[OK]'


def _sev_badge(severity: int) -> str:
    label = SEVERITY_LABEL.get(severity, '????')
    color = _sev_color(severity)
    return _c(color, f'[{label}]')


# ── Group findings by check name ──────────────────────────────────────────────

def _group(findings: List[Finding]) -> dict:
    groups: dict = {}
    for f in findings:
        groups.setdefault(f.check, []).append(f)
    return groups


# ── Section metadata ──────────────────────────────────────────────────────────

SECTION = {
    'dead_topics': ('DEAD TOPICS',
                    'Topics with a missing publisher or subscriber'),
    'qos':         ('QoS MISMATCHES',
                    'Publisher <-> Subscriber QoS incompatibilities (messages may be dropped)'),
    'tf':          ('TF TREE ISSUES',
                    'TF frame connectivity problems'),
    'hz':          ('RATE ANOMALIES',
                    'Topics publishing slower than expected (sensor/control latency risk)'),
    'nodes':       ('NODE STATUS',
                    'Expected nodes missing or unexpected nodes running'),
    'snapshot':    ('SNAPSHOT DIFF',
                    'Changes detected since last saved baseline'),
    'latency':     ('LATENCY ANALYSIS',
                    'Message timing and jitter measurements'),
    'dds_domain':  ('DDS DOMAIN',
                    'DDS domain configuration and port status'),
    'lifecycle':   ('LIFECYCLE STATE',
                    'Node lifecycle state checks'),
}


# ── Human-readable output ─────────────────────────────────────────────────────

def print_human(findings: List[Finding],
                severity_threshold: int = 1,
                simulation_mode: bool = False,
                ignored: set = None,
                stream=None) -> None:

    if stream is None:
        stream = sys.stdout

    filtered = [f for f in findings if f.severity >= severity_threshold]

    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    sim_tag = _c(Fore.MAGENTA, ' [SIMULATION MODE]') if (HAS_COLOR and simulation_mode) else \
              (' [SIMULATION MODE]' if simulation_mode else '')

    # ── Banner ────────────────────────────────────────────────────────────────
    stream.write('\n')
    stream.write(_bold('┌' + '─' * 62 + '┐') + '\n')
    stream.write(_bold('│') +
                 _bold('  ros2 triage  -  Runtime Diagnostic Report') +
                 sim_tag +
                 _bold('  │') + '\n')
    stream.write(_bold('│') +
                 f'  {now}' +
                 (' ' * (60 - len(now))) +
                 _bold('│') + '\n')
    stream.write(_bold('└' + '─' * 62 + '┘') + '\n\n')

    # ── Context note ─────────────────────────────────────────────────────────
    # The tool only inspects the ROS 2 graph (topics, nodes, TF frames) that
    # is visible at the instant of the check. It does not observe the actual
    # physical robot state, and it cannot see remappings that are not active
    # yet. Late-starting nodes, namespace remaps, or intentionally unused
    # topics may therefore appear as findings even if the robot seems to work.
    stream.write(
        '  This report reflects the current ROS 2 graph only '
        '(publishers, subscribers, TF frames), not the robot\'s\n'
    )
    stream.write(
        '  physical state. Remappings, namespaces, or nodes that start late\n'
        '  can temporarily show up as warnings until the graph settles.\n\n'
    )

    # ── Empty state ───────────────────────────────────────────────────────────
    if not filtered:
        stream.write(
            '  ' + _ok() + '  ' +
            _c(Fore.GREEN, 'No issues found') +
            f' (severity ≥ {severity_threshold})\n'
        )
        if ignored:
            stream.write(
                '  ' + _c(Fore.CYAN, f'{len(ignored)} topic(s) ignored via --ignore\n')
            )
        _footer(findings, filtered, stream)
        return

    # ── Sections ──────────────────────────────────────────────────────────────
    groups = _group(filtered)

    for check_name, check_findings in groups.items():
        header, desc = SECTION.get(check_name, (check_name.upper(), ''))
        stream.write(_bold(f'  {header}') + '\n')
        if desc:
            stream.write(f'  {_c(Fore.WHITE, desc)}\n')
        stream.write('  ' + '─' * 60 + '\n\n')

        for f in check_findings:
            badge = _sev_badge(f.severity)
            stream.write(f'  {badge}  {_bold(f.topic)}\n')
            stream.write(f'         {f.message}\n')
            # Multi-line suggestions
            suggestion_lines = f.suggestion.split('\n')
            stream.write(
                f'         {_c(Fore.GREEN, suggestion_lines[0])}\n'
            )
            for line in suggestion_lines[1:]:
                stream.write(f'            {_c(Fore.GREEN, line)}\n')
            stream.write('\n')

    # ── Ignored note ─────────────────────────────────────────────────────────
    if ignored:
        stream.write(
            '  ' + _c(Fore.CYAN,
                       f'{len(ignored)} topic(s) ignored via --ignore: '
                       + ', '.join(sorted(ignored))) + '\n\n'
        )

    _footer(findings, filtered, stream)


def _footer(all_findings, filtered, stream) -> None:
    crits = sum(1 for f in filtered if f.severity == 3)
    warns = sum(1 for f in filtered if f.severity == 2)
    infos = sum(1 for f in filtered if f.severity == 1)

    stream.write(_bold('─' * 64) + '\n')
    stream.write('  Summary:  ')
    stream.write(_c(Fore.RED,    f'{crits} CRITICAL') + '   ')
    stream.write(_c(Fore.YELLOW, f'{warns} WARNING') + '   ')
    stream.write(_c(Fore.CYAN,   f'{infos} INFO') + '\n')

    if crits > 0:
        stream.write(
            '  ' +
            _c(Fore.RED, 'Exit code 1 - CI pipeline should fail on CRITICAL findings.') +
            '\n'
        )
    elif warns > 0:
        stream.write(
            '  ' + _c(Fore.YELLOW, 'Warnings found - review before deploying.') + '\n'
        )
    else:
        stream.write(
            '  ' + _c(Fore.GREEN, 'System looks healthy.') + '\n'
        )
    stream.write(_bold('─' * 64) + '\n\n')


# ── JSON / CI output ──────────────────────────────────────────────────────────

def print_json(findings: List[Finding],
               severity_threshold: int = 1,
               simulation_mode: bool = False,
               stream=None) -> None:
    if stream is None:
        stream = sys.stdout

    filtered = [f for f in findings if f.severity >= severity_threshold]
    groups = _group(filtered)

    crits = sum(1 for f in filtered if f.severity == 3)
    warns = sum(1 for f in filtered if f.severity == 2)
    infos = sum(1 for f in filtered if f.severity == 1)

    output = {
        'schema_version': '1.1',
        'timestamp': datetime.now().isoformat(),
        'simulation_mode': simulation_mode,
        'severity_threshold': severity_threshold,
        'total_findings': len(filtered),
        'summary': {
            'critical': crits,
            'warning':  warns,
            'info':     infos,
            'status':   'FAIL' if crits > 0 else ('WARN' if warns > 0 else 'PASS'),
        },
        # Context note: ros2-triage always reasons from the live ROS 2 graph
        # (topics, publishers, subscribers, TF frames) at the instant of the
        # check. It does not observe the physical robot state directly, and it
        # cannot account for nodes that have not started yet. This field
        # mirrors the human-readable banner note for CI/automation users.
        'note': (
            'This report reflects the current ROS 2 graph only '
            '(publishers, subscribers, TF frames), not the robot\'s physical '
            'state. Remappings, namespaces, or nodes that start late can '
            'temporarily appear as findings until the graph settles.'
        ),
        'checks': [
            {
                'name': check_name,
                'count': len(check_findings),
                'findings': [f.to_dict() for f in check_findings],
            }
            for check_name, check_findings in groups.items()
        ],
    }
    stream.write(json.dumps(output, indent=2))
    stream.write('\n')


# ── Rich output (preferred when available) ────────────────────────────────────

def print_rich(
    findings: List[Finding],
    severity_threshold: int = 1,
    simulation_mode: bool = False,
    ignored: set = None,
    system_metrics: dict = None,
    console: Optional['Console'] = None
) -> None:
    """
    Print findings using Rich library for enhanced terminal output.
    
    Falls back to print_human() if Rich is not available.
    """
    if not HAS_RICH:
        print_human(findings, severity_threshold, simulation_mode, ignored)
        return
    
    if console is None:
        console = Console()
    
    filtered = [f for f in findings if f.severity >= severity_threshold]
    ignored = ignored or set()
    
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # Header
    header_text = Text()
    header_text.append("ROS 2 Triager - Runtime Diagnostic Report\n", style="bold cyan")
    header_text.append(f"{now}", style="dim")
    if simulation_mode:
        header_text.append("  [SIMULATION MODE]", style="bold magenta")
    
    console.print(Panel(header_text, box=box.DOUBLE, border_style="cyan"))
    console.print()
    
    # Context note
    console.print(
        "  This report reflects the current ROS 2 graph only (publishers, "
        "subscribers, TF frames).",
        style="dim"
    )
    console.print(
        "  Remappings, namespaces, or late-starting nodes may appear as "
        "temporary findings.",
        style="dim"
    )
    console.print()
    
    # Empty state
    if not filtered:
        console.print(
            Panel(
                Text("No issues found", style="bold green"),
                title="STATUS",
                border_style="green"
            )
        )
        if ignored:
            console.print(f"  {len(ignored)} topic(s) ignored via --ignore", style="cyan")
        return
    
    # Group findings by check
    groups = _group(filtered)
    
    for check_name, check_findings in groups.items():
        header, desc = SECTION.get(check_name, (check_name.upper(), ''))
        
        # Create table for this section
        table = Table(
            title=header,
            box=box.ROUNDED,
            show_header=True,
            header_style="bold"
        )
        table.add_column("SEV", width=5, style="bold")
        table.add_column("TOPIC", style="cyan", ratio=1)
        table.add_column("MESSAGE", ratio=2)
        
        for f in check_findings:
            sev_style = {3: "bold red", 2: "bold yellow", 1: "bold cyan"}.get(
                f.severity, "white"
            )
            sev_label = SEVERITY_LABEL.get(f.severity, "????")
            
            table.add_row(
                Text(sev_label, style=sev_style),
                f.topic,
                f.message
            )
        
        console.print(table)
        console.print()
    
    # Summary
    crits = sum(1 for f in filtered if f.severity == 3)
    warns = sum(1 for f in filtered if f.severity == 2)
    infos = sum(1 for f in filtered if f.severity == 1)
    
    summary = Text()
    summary.append("Summary: ", style="bold")
    summary.append(f"{crits} CRITICAL", style="bold red")
    summary.append("  ")
    summary.append(f"{warns} WARNING", style="bold yellow")
    summary.append("  ")
    summary.append(f"{infos} INFO", style="bold cyan")
    
    if crits > 0:
        summary.append("\nExit code 1 - CI pipeline should fail on CRITICAL findings.", 
                      style="red")
    elif warns > 0:
        summary.append("\nWarnings found - review before deploying.", style="yellow")
    else:
        summary.append("\nSystem looks healthy.", style="green")
    
    border_color = "red" if crits > 0 else ("yellow" if warns > 0 else "green")
    console.print(Panel(summary, box=box.ROUNDED, border_style=border_color))
    
    if ignored:
        console.print(f"  {len(ignored)} topic(s) ignored: {', '.join(sorted(ignored))}", 
                     style="cyan")
    console.print()


# ── Auto-select best output method ────────────────────────────────────────────

def print_report(
    findings: List[Finding],
    severity_threshold: int = 1,
    simulation_mode: bool = False,
    ignored: set = None,
    use_rich: bool = True,
    stream=None
) -> None:
    """
    Print findings using the best available output method.
    
    Automatically selects Rich if available, otherwise falls back to colorama.
    
    Args:
        findings: List of diagnostic findings
        severity_threshold: Minimum severity to display
        simulation_mode: Whether in simulation mode
        ignored: Set of ignored topics
        use_rich: Whether to use Rich (if available)
        stream: Output stream (for colorama fallback)
    """
    if use_rich and HAS_RICH:
        print_rich(findings, severity_threshold, simulation_mode, ignored)
    else:
        print_human(findings, severity_threshold, simulation_mode, ignored, stream)
