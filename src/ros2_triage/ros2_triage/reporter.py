# Copyright 2024 darshan — Apache-2.0
"""
reporter.py — Formats and prints triage findings.

  print_human() — coloured terminal output via colorama
  print_json()  — structured JSON for CI pipelines
"""

import json
import sys
from datetime import datetime
from typing import List

from .checks.finding import Finding, SEVERITY_LABEL

try:
    import colorama
    from colorama import Fore, Style, Back
    colorama.init(autoreset=True)
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = False


# ── Color helpers ─────────────────────────────────────────────────────────────

def _c(color_code: str, text: str) -> str:
    return (color_code + text + Style.RESET_ALL) if HAS_COLOR else text


def _bold(text: str) -> str:
    return (Style.BRIGHT + text + Style.RESET_ALL) if HAS_COLOR else text


def _sev_color(severity: int) -> str:
    if not HAS_COLOR:
        return ''
    return {3: Fore.RED, 2: Fore.YELLOW, 1: Fore.CYAN}.get(severity, '')


def _ok() -> str:
    return _c(Fore.GREEN, '[OK]') if HAS_COLOR else '[OK]'


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
                    'Publisher ↔ Subscriber QoS incompatibilities (messages may be dropped)'),
    'tf':          ('TF TREE ISSUES',
                    'TF frame connectivity problems'),
    'hz':          ('RATE ANOMALIES',
                    'Topics publishing slower than expected (sensor/control latency risk)'),
    'nodes':       ('NODE STATUS',
                    'Expected nodes missing or unexpected nodes running'),
    'snapshot':    ('SNAPSHOT DIFF',
                    'Changes detected since last saved baseline'),
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
                 _bold('  ros2 triage  —  Runtime Diagnostic Report') +
                 sim_tag +
                 _bold('  │') + '\n')
    stream.write(_bold('│') +
                 f'  {now}' +
                 (' ' * (60 - len(now))) +
                 _bold('│') + '\n')
    stream.write(_bold('└' + '─' * 62 + '┘') + '\n\n')

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
            _c(Fore.RED, 'Exit code 1 — CI pipeline should fail on CRITICAL findings.') +
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
