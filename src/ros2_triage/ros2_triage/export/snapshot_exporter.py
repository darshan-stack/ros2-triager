# export/snapshot_exporter.py
from __future__ import annotations
import dataclasses
import time
from collections import deque
from ..state.state_bus import StateBus


def _safe_asdict(obj) -> dict:
    """Convert a dataclass to dict, handling deque fields."""
    result = {}
    for f in dataclasses.fields(obj):
        val = getattr(obj, f.name)
        if isinstance(val, deque):
            result[f.name] = list(val)
        elif dataclasses.is_dataclass(val):
            result[f.name] = _safe_asdict(val)
        elif isinstance(val, dict):
            result[f.name] = {
                k: _safe_asdict(v) if dataclasses.is_dataclass(v) else v
                for k, v in val.items()
            }
        elif isinstance(val, list):
            result[f.name] = [
                _safe_asdict(item) if dataclasses.is_dataclass(item) else item
                for item in val
            ]
        else:
            result[f.name] = val
    return result


def export_snapshot(bus: StateBus) -> dict:
    with bus._lock:
        return {
            "timestamp": time.time(),
            "uptime_sec": bus.uptime_seconds(),
            "health": _safe_asdict(bus.health),
            "topics": {k: _safe_asdict(v) for k, v in bus.topics.items()},
            "nodes": {k: _safe_asdict(v) for k, v in bus.nodes.items()},
            "tf": _safe_asdict(bus.tf),
            "diagnostics": _safe_asdict(bus.diag),
            "lifecycle": {k: _safe_asdict(v) for k, v in bus.lifecycle.items()},
            "odom": _safe_asdict(bus.odom),
            "alerts": [_safe_asdict(a) for a in list(bus.alerts)[:50]],
        }
