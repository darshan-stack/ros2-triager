# Copyright 2024 darshan - Apache-2.0
"""
command/triage.py - ROS 2 CLI command extension.

Registered via setup.py:
  ros2cli.command:
    triage = ros2_triage.command.triage:TriageCommand

New in v2:
  --check-hz           Measure topic publish rates
  --expected YAML      Detect missing/unexpected nodes
  --snapshot-save FILE Save current state as baseline
  --snapshot-diff FILE Diff current state vs baseline
  --watch              Live monitoring mode (refreshes every N seconds)
  tui                  Launch interactive Textual TUI dashboard
"""

import sys
import os
import time
from ros2cli.command import CommandExtension


class TriageCommand(CommandExtension):
    """
    ros2 triage - Runtime graph diagnostic tool.

    Detects: dead topics, QoS mismatches, TF issues,
             slow topic rates, missing nodes, and graph drift vs baseline.

    Use --json for CI output. Use --watch for live monitoring.
    Use 'tui' subcommand for interactive Textual TUI dashboard.
    """

    def add_arguments(self, parser, cli_name):
        parser.description = (
            'Analyse the live ROS 2 graph and report runtime problems.\n'
            'Smart filtering removes Gazebo/Rviz/Nav2 noise automatically.\n\n'
            'Subcommands:\n'
            '  tui                              # launch Textual TUI dashboard\n\n'
            'Examples:\n'
            '  ros2 triage                          # full check\n'
            '  ros2 triage tui                      # interactive TUI\n'
            '  ros2 triage tui --preflight          # headless preflight check\n'
            '  ros2 triage --simulation             # with Gazebo running\n'
            '  ros2 triage --check-hz               # + rate anomaly check\n'
            '  ros2 triage --expected nodes.yaml    # + missing node check\n'
            '  ros2 triage --snapshot-save good.json  # save healthy baseline\n'
            '  ros2 triage --snapshot-diff good.json  # compare vs baseline\n'
            '  ros2 triage --watch                  # live refresh (Ctrl+C to exit)\n'
            '  ros2 triage --json                   # CI/CD JSON output\n'
        )

        # ── TUI subcommand ────────────────────────────────────────────────
        subparsers = parser.add_subparsers(dest='subcommand')
        _add_tui_subcommand(subparsers)

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
                'Path to expected_nodes.yaml - reports nodes declared there '
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
                'Live monitoring mode - refreshes every --watch-interval seconds. '
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
            help='Only show findings >= this level. 1=INFO 2=WARN 3=CRIT (default: 1)',
        )
        parser.add_argument(
            '--timeout',
            type=float, default=3.0,
            help='Seconds to wait for the ROS 2 graph to settle (default: 3.0)',
        )

        # ── Advanced diagnostics ──────────────────────────────────────────
        parser.add_argument(
            '--check-latency', dest='check_latency',
            action='store_true', default=False,
            help='Measure message latency and jitter (T_arrival - T_header_stamp).',
        )
        parser.add_argument(
            '--latency-window',
            type=float, default=5.0,
            metavar='SECS',
            help='Measurement window for latency check in seconds (default: 5.0)',
        )
        parser.add_argument(
            '--check-dds', dest='check_dds',
            action='store_true', default=False,
            help='Probe DDS domain for port conflicts and configuration issues.',
        )
        parser.add_argument(
            '--correlate', dest='use_correlation',
            action='store_true', default=False,
            help='Enable correlation engine for root cause analysis.',
        )

        # ── Interactive mode ──────────────────────────────────────────────
        parser.add_argument(
            '--interactive', '-I',
            action='store_true', default=False,
            help='Launch interactive TUI with keyboard navigation.',
        )
        parser.add_argument(
            '--rich', dest='use_rich',
            action='store_true', default=True,
            help='Use Rich library for enhanced output (default: on)',
        )
        parser.add_argument(
            '--no-rich', dest='use_rich',
            action='store_false',
            help='Disable Rich output, use colorama fallback.',
        )

    def main(self, *, args, **kwargs):
        """Entry point - single-shot or watch loop."""
        try:
            import rclpy  # noqa: F401
        except ImportError:
            print(
                'ERROR: rclpy not available. Source ROS 2 first:\n'
                '  source /opt/ros/humble/setup.bash',
                file=sys.stderr,
            )
            return 1

        # Dispatch to TUI subcommand if requested
        if getattr(args, 'subcommand', None) == 'tui':
            return _run_tui(args)

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
        from ros2_triage.reporter import print_human, print_json, print_report

        ignore_set = set(args.ignore) if args.ignore else set()
        skip_noisy = not args.show_all
        sim = args.simulation

        all_findings = []
        hypotheses = []

        # ── Build graph ───────────────────────────────────────────────────
        graph, nodes = {}, []
        needs_graph = (
            args.check_dead_topics or args.check_qos or args.check_hz or
            args.check_latency or args.snapshot_save or args.snapshot_diff or
            args.expected
        )
        if needs_graph:
            _status('Inspecting topic graph', args.json)
            try:
                graph, nodes = build_topic_graph(timeout_sec=args.timeout)
            except Exception as e:
                _warn(f'Could not build topic graph: {e}', args.json)

        # ── Snapshot save ─────────────────────────────────────────────────
        if args.snapshot_save:
            save_snapshot(graph, nodes, args.snapshot_save)
            if not args.json:
                print(f'  Snapshot saved to: {args.snapshot_save}')
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

        # ── Hz rate check ─────────────────────────────────────────────────
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

        # ── Node check ────────────────────────────────────────────────────
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

        # ── Snapshot diff ─────────────────────────────────────────────────
        if args.snapshot_diff:
            _status(f'Diffing against snapshot: {args.snapshot_diff}', args.json)
            try:
                all_findings.extend(diff_snapshot(
                    graph, nodes, args.snapshot_diff, ignore_set=ignore_set
                ))
            except Exception as e:
                _warn(f'Snapshot diff failed: {e}', args.json)

        # ── Latency check (Advanced) ──────────────────────────────────────
        if args.check_latency and graph:
            _status(f'Measuring latency ({args.latency_window}s window)', args.json)
            try:
                from ros2_triage.checks.latency_engine import check_latency
                all_findings.extend(check_latency(
                    graph,
                    window_sec=args.latency_window,
                    ignore_set=ignore_set,
                ))
            except Exception as e:
                _warn(f'Latency check failed: {e}', args.json)

        # ── DDS domain check (Advanced) ───────────────────────────────────
        if args.check_dds:
            _status('Probing DDS domain', args.json)
            try:
                from ros2_triage.checks.dds_probe import check_dds_domain
                all_findings.extend(check_dds_domain(ignore_set=ignore_set))
            except Exception as e:
                _warn(f'DDS probe failed: {e}', args.json)

        # ── Correlation engine (Advanced) ─────────────────────────────────
        if args.use_correlation and all_findings:
            _status('Running correlation engine', args.json)
            try:
                from ros2_triage.correlation_engine import CorrelationEngine
                engine = CorrelationEngine()
                
                for finding in all_findings:
                    if finding.check == 'dead_topics' and 'UNPUBLISHED' in finding.message:
                        # Get subscriber nodes from finding
                        hyp = engine.analyze_unpublished_topic(
                            finding.topic,
                            finding.extra.get('subscribers', []),
                            graph
                        )
                        hypotheses.append(hyp)
                    elif finding.check == 'qos':
                        # Parse QoS mismatch info
                        hyp = engine.analyze_qos_mismatch(
                            finding.topic,
                            finding.extra.get('pub_node', 'unknown'),
                            finding.extra.get('sub_node', 'unknown'),
                            finding.extra.get('mismatch_type', 'unknown')
                        )
                        hypotheses.append(hyp)
            except Exception as e:
                _warn(f'Correlation engine failed: {e}', args.json)

        # ── Interactive mode ──────────────────────────────────────────────
        if args.interactive:
            try:
                from ros2_triage.interactive_tui import InteractiveTUI
                tui = InteractiveTUI(
                    refresh_callback=lambda: self._run_once(args)
                )
                tui.update_data(
                    findings=all_findings,
                    hypotheses=hypotheses,
                    system_metrics={}
                )
                tui.run(refresh_interval=args.watch_interval if hasattr(args, 'watch_interval') else 5.0)
                return 0
            except Exception as e:
                _warn(f'Interactive mode failed: {e}', args.json)
                # Fall through to normal output

        # ── Report ────────────────────────────────────────────────────────
        if args.json:
            print_json(all_findings,
                       severity_threshold=args.severity_threshold,
                       simulation_mode=sim)
        else:
            print_report(
                all_findings,
                severity_threshold=args.severity_threshold,
                simulation_mode=sim,
                ignored=ignore_set,
                use_rich=args.use_rich
            )

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
                    f'  ros2 triage --watch  │  '
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
                    print(Fore.BLUE + f'\n  [*] Next refresh in {interval:.0f}s...' + Style.RESET_ALL)
                else:
                    print(f'\n  Next refresh in {interval:.0f}s...')

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
    print(f'    {msg}', file=sys.stderr)


# ── TUI subcommand helpers ────────────────────────────────────────────────────

def _add_tui_subcommand(subparsers) -> None:
    """Register the 'tui' subparser under the main triage parser."""
    tui_parser = subparsers.add_parser(
        "tui", help="Launch interactive Textual TUI dashboard"
    )
    tui_parser.add_argument(
        "--config", "-c", default=None,
        help="Path to YAML config file (optional; zero-config if omitted)"
    )
    tui_parser.add_argument(
        "--domain", "-d", type=int, default=0,
        help="ROS_DOMAIN_ID to use (default: 0)"
    )
    tui_parser.add_argument(
        "--preflight", action="store_true",
        help="Run headless preflight check instead of launching TUI"
    )
    tui_parser.add_argument(
        "--preflight-timeout", type=int, default=30,
        dest="preflight_timeout",
        help="Preflight timeout in seconds (default: 30)"
    )


def _run_tui(args) -> int:
    """Launch the Textual TUI dashboard (or headless preflight check)."""
    import os
    import threading
    import rclpy

    os.environ["ROS_DOMAIN_ID"] = str(getattr(args, "domain", 0))
    rclpy.init()
    node = rclpy.create_node("_ros2_triager_node")

    from ros2_triage.state.state_bus import StateBus
    from ros2_triage.collectors.topic_collector import TopicCollector
    from ros2_triage.collectors.tf_collector import TFCollector
    from ros2_triage.collectors.node_collector import NodeCollector
    from ros2_triage.collectors.diagnostic_collector import DiagnosticCollector
    from ros2_triage.collectors.lifecycle_collector import LifecycleCollector
    from ros2_triage.collectors.odom_map_collector import OdomMapCollector
    from ros2_triage.config.config_manager import load_config
    from ros2_triage.tui.app import TriagerApp

    bus = StateBus()
    config = load_config(getattr(args, "config", None))

    # Start all collectors
    collectors = [
        TopicCollector(node, bus),
        TFCollector(node, bus),
        NodeCollector(node, bus),
        DiagnosticCollector(node, bus),
        LifecycleCollector(node, bus),
        OdomMapCollector(node, bus),
    ]
    for c in collectors:
        c.start()

    # Spin rclpy in background thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    if getattr(args, "preflight", False):
        _run_preflight(bus, config, getattr(args, "preflight_timeout", 30))
        # _run_preflight calls sys.exit(), so this is a safety return
        return 0

    try:
        app = TriagerApp(bus, config)
        app.run()
    finally:
        for c in collectors:
            c.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return 0


def _run_preflight(bus, config: dict, timeout_sec: int) -> None:
    """Run headless preflight check. Exits with 0=pass, 1=fail."""
    import time
    import sys
    from ros2_triage.engine.health_scorer import compute_health

    min_health = config.get("settings", {}).get("preflight_min_health", 80)
    deadline = time.monotonic() + timeout_sec
    print(f"Running preflight check (timeout: {timeout_sec}s, required: {min_health}/100)...")

    while time.monotonic() < deadline:
        score = compute_health(bus)
        if score.overall >= min_health:
            print(f"✔ PREFLIGHT PASSED — health score: {score.overall}/100")
            sys.exit(0)
        time.sleep(1.0)

    score = compute_health(bus)
    print(
        f"✖ PREFLIGHT FAILED — timeout after {timeout_sec}s "
        f"(final score: {score.overall}/100, required: {min_health}/100)"
    )
    sys.exit(1)
