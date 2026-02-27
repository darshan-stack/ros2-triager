# Copyright 2024 darshan — Apache-2.0
"""
command/triage.py — ROS 2 CLI command extension.

Registered via setup.py:
  ros2cli.command:
    triage = ros2_triage.command.triage:TriageCommand

New in v2:
  --check-hz           Measure topic publish rates
  --expected YAML      Detect missing/unexpected nodes
  --snapshot-save FILE Save current state as baseline
  --snapshot-diff FILE Diff current state vs baseline
  --watch              Live monitoring mode (refreshes every N seconds)
"""

import sys
import os
import time
from ros2cli.command import CommandExtension


class TriageCommand(CommandExtension):
    """
    ros2 triage — Runtime graph diagnostic tool.

    Detects: dead topics, QoS mismatches, TF issues,
             slow topic rates, missing nodes, and graph drift vs baseline.

    Use --json for CI output. Use --watch for live monitoring.
    """

    def add_arguments(self, parser, cli_name):
        parser.description = (
            'Analyse the live ROS 2 graph and report runtime problems.\n'
            'Smart filtering removes Gazebo/Rviz/Nav2 noise automatically.\n\n'
            'Examples:\n'
            '  ros2 triage                          # full check\n'
            '  ros2 triage --simulation             # with Gazebo running\n'
            '  ros2 triage --check-hz               # + rate anomaly check\n'
            '  ros2 triage --expected nodes.yaml    # + missing node check\n'
            '  ros2 triage --snapshot-save good.json  # save healthy baseline\n'
            '  ros2 triage --snapshot-diff good.json  # compare vs baseline\n'
            '  ros2 triage --watch                  # live refresh (Ctrl+C to exit)\n'
            '  ros2 triage --json                   # CI/CD JSON output\n'
        )

        # ── Standard check toggles ────────────────────────────────────────
        parser.add_argument('--dead-topics', dest='check_dead_topics',
                            action='store_true', default=True,
                            help='Check dead topics (default: on)')
        parser.add_argument('--no-dead-topics', dest='check_dead_topics',
                            action='store_false', help='Disable dead-topic check')

        parser.add_argument('--qos', dest='check_qos',
                            action='store_true', default=True,
                            help='Check QoS mismatches (default: on)')
        parser.add_argument('--no-qos', dest='check_qos',
                            action='store_false', help='Disable QoS check')

        parser.add_argument('--tf', dest='check_tf',
                            action='store_true', default=True,
                            help='Check TF tree (default: on)')
        parser.add_argument('--no-tf', dest='check_tf',
                            action='store_false', help='Disable TF check')

        # ── New checks ────────────────────────────────────────────────────
        parser.add_argument(
            '--check-hz', dest='check_hz',
            action='store_true', default=False,
            help=(
                'Measure topic publish rates and flag topics below expected Hz. '
                'Adds ~3s to check time (measurement window). '
                'Use --hz-window to adjust.'
            ),
        )
        parser.add_argument(
            '--hz-window',
            type=float, default=3.0,
            metavar='SECS',
            help='Measurement window for rate check in seconds (default: 3.0)',
        )
        parser.add_argument(
            '--expected', '-e',
            metavar='YAML_FILE',
            default=None,
            help=(
                'Path to expected_nodes.yaml — reports nodes declared there '
                'that are missing from the live graph (and vice versa).'
            ),
        )

        # ── Snapshot ──────────────────────────────────────────────────────
        snapshot_group = parser.add_mutually_exclusive_group()
        snapshot_group.add_argument(
            '--snapshot-save',
            metavar='FILE',
            default=None,
            help='Save current graph state to FILE as a JSON baseline.',
        )
        snapshot_group.add_argument(
            '--snapshot-diff',
            metavar='FILE',
            default=None,
            help='Compare current state against a previously saved snapshot FILE.',
        )

        # ── Smart filters ─────────────────────────────────────────────────
        parser.add_argument(
            '--simulation', '-sim',
            action='store_true', default=False,
            help='Simulation mode: suppress Gazebo, Rviz, visualisation topics.',
        )
        parser.add_argument(
            '--ignore', '-i',
            nargs='+', metavar='TOPIC', default=[],
            help='Topics or nodes to ignore completely.',
        )
        parser.add_argument(
            '--show-all',
            action='store_true', default=False,
            help='Show ALL topics including sim/viz/infra (disables smart filtering).',
        )

        # ── Watch mode ────────────────────────────────────────────────────
        parser.add_argument(
            '--watch', '-w',
            action='store_true', default=False,
            help=(
                'Live monitoring mode — refreshes every --watch-interval seconds. '
                'Clears the terminal and re-runs all checks. Press Ctrl+C to exit.'
            ),
        )
        parser.add_argument(
            '--watch-interval',
            type=float, default=5.0,
            metavar='SECS',
            help='Refresh interval for --watch mode in seconds (default: 5.0)',
        )

        # ── Output ────────────────────────────────────────────────────────
        parser.add_argument(
            '--json', action='store_true', default=False,
            help='Machine-readable JSON output (for CI pipelines)',
        )
        parser.add_argument(
            '--severity-threshold', '-s',
            type=int, choices=[1, 2, 3], default=1, metavar='LEVEL',
            help='Only show findings ≥ this level. 1=INFO 2=WARN 3=CRIT (default: 1)',
        )
        parser.add_argument(
            '--timeout',
            type=float, default=3.0,
            help='Seconds to wait for the ROS 2 graph to settle (default: 3.0)',
        )

    def main(self, *, args, **kwargs):
        """Entry point — single-shot or watch loop."""
        try:
            import rclpy  # noqa: F401
        except ImportError:
            print(
                'ERROR: rclpy not available. Source ROS 2 first:\n'
                '  source /opt/ros/humble/setup.bash',
                file=sys.stderr,
            )
            return 1

        if args.watch:
            return self._watch_loop(args)
        return self._run_once(args)

    # ── Single run ────────────────────────────────────────────────────────────

    def _run_once(self, args) -> int:
        from ros2_triage.checks.graph_utils import build_topic_graph
        from ros2_triage.checks.dead_topic import check_dead_topics
        from ros2_triage.checks.qos_check import check_qos
        from ros2_triage.checks.tf_check import check_tf
        from ros2_triage.checks.hz_check import check_hz
        from ros2_triage.checks.node_check import check_nodes
        from ros2_triage.checks.snapshot import save_snapshot, diff_snapshot
        from ros2_triage.reporter import print_human, print_json

        ignore_set = set(args.ignore) if args.ignore else set()
        skip_noisy = not args.show_all
        sim = args.simulation

        all_findings = []

        # ── Build graph ───────────────────────────────────────────────────
        graph, nodes = {}, []
        if args.check_dead_topics or args.check_qos or args.check_hz \
                or args.snapshot_save or args.snapshot_diff or args.expected:
            _status('Inspecting topic graph', args.json)
            try:
                graph, nodes = build_topic_graph(timeout_sec=args.timeout)
            except Exception as e:
                _warn(f'Could not build topic graph: {e}', args.json)

        # ── Snapshot save ─────────────────────────────────────────────────
        if args.snapshot_save:
            from ros2_triage.checks.snapshot import save_snapshot
            save_snapshot(graph, nodes, args.snapshot_save)
            return 0  # save-only mode

        # ── Dead topics ───────────────────────────────────────────────────
        if args.check_dead_topics and graph:
            _status('Checking dead topics', args.json)
            try:
                all_findings.extend(check_dead_topics(
                    graph,
                    skip_noisy=skip_noisy,
                    extra_ignore=ignore_set,
                    simulation_mode=sim,
                ))
            except Exception as e:
                _warn(f'Dead-topic check failed: {e}', args.json)

        # ── QoS ───────────────────────────────────────────────────────────
        if args.check_qos and graph:
            _status('Checking QoS compatibility', args.json)
            try:
                all_findings.extend(check_qos(graph, ignore_set=ignore_set))
            except Exception as e:
                _warn(f'QoS check failed: {e}', args.json)

        # ── TF ────────────────────────────────────────────────────────────
        if args.check_tf:
            _status('Checking TF tree', args.json)
            try:
                all_findings.extend(check_tf())
            except Exception as e:
                _warn(f'TF check failed: {e}', args.json)

        # ── Hz rate check (NEW) ───────────────────────────────────────────
        if args.check_hz and graph:
            _status(f'Measuring topic rates ({args.hz_window}s window)', args.json)
            try:
                all_findings.extend(check_hz(
                    graph,
                    window=args.hz_window,
                    ignore_set=ignore_set,
                ))
            except Exception as e:
                _warn(f'Hz check failed: {e}', args.json)

        # ── Node check (NEW) ─────────────────────────────────────────────
        if args.expected:
            _status('Checking expected nodes', args.json)
            try:
                all_findings.extend(check_nodes(
                    expected_yaml=args.expected,
                    running_nodes=nodes,
                    ignore_set=ignore_set,
                ))
            except Exception as e:
                _warn(f'Node check failed: {e}', args.json)

        # ── Snapshot diff (NEW) ───────────────────────────────────────────
        if args.snapshot_diff:
            _status(f'Diffing against snapshot: {args.snapshot_diff}', args.json)
            try:
                all_findings.extend(diff_snapshot(
                    graph, nodes, args.snapshot_diff, ignore_set=ignore_set
                ))
            except Exception as e:
                _warn(f'Snapshot diff failed: {e}', args.json)

        # ── Report ────────────────────────────────────────────────────────
        if args.json:
            print_json(all_findings,
                       severity_threshold=args.severity_threshold,
                       simulation_mode=sim)
        else:
            print_human(all_findings,
                        severity_threshold=args.severity_threshold,
                        simulation_mode=sim,
                        ignored=ignore_set)

        crits = [f for f in all_findings
                 if f.severity == 3 and f.severity >= args.severity_threshold]
        return 1 if crits else 0

    # ── Watch loop ────────────────────────────────────────────────────────────

    def _watch_loop(self, args):
        """Continuously re-run all checks and refresh the terminal."""
        import shutil

        try:
            from colorama import Fore, Style, init
            init(autoreset=True)
            has_color = True
        except ImportError:
            has_color = False

        interval = args.watch_interval
        iteration = 0

        print(f'\n  ros2 triage --watch  │  refreshing every {interval:.0f}s  │  Ctrl+C to exit\n')
        time.sleep(1)

        try:
            while True:
                iteration += 1
                # Clear terminal
                os.system('clear' if os.name == 'posix' else 'cls')

                # Watch header
                cols = shutil.get_terminal_size(fallback=(80, 24)).columns
                ts = time.strftime('%H:%M:%S')
                header = (
                    f'  🔬  ros2 triage --watch  │  '
                    f'refresh #{iteration}  │  {ts}  │  '
                    f'every {interval:.0f}s  │  Ctrl+C to exit'
                )
                if has_color:
                    print(Fore.CYAN + Style.BRIGHT + header + Style.RESET_ALL)
                else:
                    print(header)
                print()

                # Run all checks
                rc = self._run_once(args)

                # Show next refresh countdown
                if has_color:
                    print(Fore.BLUE + f'\n  ⏳  Next refresh in {interval:.0f}s…' + Style.RESET_ALL)
                else:
                    print(f'\n  Next refresh in {interval:.0f}s…')

                time.sleep(interval)

        except KeyboardInterrupt:
            print('\n\n  ros2 triage watch stopped.\n')
            return 0


# ── Private helpers ───────────────────────────────────────────────────────────

def _status(msg: str, json_mode: bool) -> None:
    if json_mode:
        return
    try:
        from colorama import Fore, Style, init
        init(autoreset=True)
        print(f'{Fore.BLUE}  ›{Style.RESET_ALL} {msg}…', file=sys.stderr)
    except ImportError:
        print(f'  › {msg}…', file=sys.stderr)


def _warn(msg: str, json_mode: bool) -> None:
    print(f'  ⚠  {msg}', file=sys.stderr)
