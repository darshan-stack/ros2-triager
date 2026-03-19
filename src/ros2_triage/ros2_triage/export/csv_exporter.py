# export/csv_exporter.py
from __future__ import annotations
import csv
import io
import time
from ..state.state_bus import StateBus


def export_topics_csv(bus: StateBus) -> str:
    """Export topic stats to CSV string."""
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow([
        "timestamp", "topic", "msg_type", "status",
        "actual_hz", "expected_hz", "bandwidth_kbps",
        "publisher_count", "subscriber_count", "is_critical",
    ])
    ts = time.time()
    with bus._lock:
        for t in bus.topics.values():
            writer.writerow([
                ts, t.name, t.msg_type, t.status,
                f"{t.actual_hz:.3f}",
                f"{t.expected_hz:.3f}" if t.expected_hz else "",
                f"{t.bandwidth_kbps:.3f}",
                t.publisher_count, t.subscriber_count,
                t.is_critical,
            ])
    return output.getvalue()


def export_nodes_csv(bus: StateBus) -> str:
    """Export node stats to CSV string."""
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow([
        "timestamp", "node", "namespace", "status",
        "is_zombie", "pub_count", "sub_count", "service_count",
    ])
    ts = time.time()
    with bus._lock:
        for n in bus.nodes.values():
            writer.writerow([
                ts, n.name, n.namespace, n.status,
                n.is_zombie, len(n.pub_topics),
                len(n.sub_topics), len(n.services),
            ])
    return output.getvalue()


def export_alerts_csv(bus: StateBus) -> str:
    """Export alert log to CSV string."""
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["timestamp", "severity", "source", "message"])
    with bus._lock:
        for a in list(bus.alerts)[:200]:
            writer.writerow([a.timestamp, a.severity, a.source, a.message])
    return output.getvalue()


def write_csv_files(bus: StateBus, prefix: str = "/tmp/ros2_triage") -> list[str]:
    """Write all CSV exports to files. Returns list of file paths."""
    import datetime
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    paths = []
    for name, fn in [
        ("topics", export_topics_csv),
        ("nodes", export_nodes_csv),
        ("alerts", export_alerts_csv),
    ]:
        path = f"{prefix}_{name}_{ts}.csv"
        with open(path, "w") as f:
            f.write(fn(bus))
        paths.append(path)
    return paths
