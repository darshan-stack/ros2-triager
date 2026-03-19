# state/state_bus.py
from __future__ import annotations
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TopicState:
    name: str
    msg_type: str
    actual_hz: float = 0.0
    expected_hz: float | None = None
    bandwidth_kbps: float = 0.0
    publisher_count: int = 0
    subscriber_count: int = 0
    last_msg_time: float = 0.0  # monotonic
    status: str = "UNKNOWN"  # OK | LOW_HZ | DEAD | NO_PUB | STALE
    hz_history: deque = field(default_factory=lambda: deque(maxlen=60))
    is_critical: bool = False


@dataclass
class NodeState:
    name: str
    namespace: str = "/"
    pub_topics: list[str] = field(default_factory=list)
    sub_topics: list[str] = field(default_factory=list)
    services: list[str] = field(default_factory=list)
    is_zombie: bool = False
    status: str = "OK"  # OK | ZOMBIE | MISSING


@dataclass
class TFState:
    frames: dict[str, str] = field(default_factory=dict)  # child → parent
    stale_frames: list[str] = field(default_factory=list)
    broken_chains: list[str] = field(default_factory=list)
    last_update: float = 0.0


@dataclass
class DiagItem:
    hardware_id: str
    name: str
    level: int  # 0=OK 1=WARN 2=ERROR 3=STALE
    message: str
    values: dict[str, str] = field(default_factory=dict)
    stamp: float = 0.0


@dataclass
class DiagState:
    items: dict[str, DiagItem] = field(default_factory=dict)
    error_count: int = 0
    warn_count: int = 0


@dataclass
class LifecycleState:
    name: str
    state_id: int = 0
    state_label: str = "UNKNOWN"
    last_transition: float = 0.0
    # 1=UNCONFIGURED 2=INACTIVE 3=ACTIVE 4=FINALIZED


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    linear_vel: float = 0.0
    angular_vel: float = 0.0
    last_update: float = 0.0


@dataclass
class Alert:
    timestamp: float
    severity: str  # ERROR | WARN | INFO
    source: str
    message: str


@dataclass
class HealthScore:
    topics_score: int = 100
    nodes_score: int = 100
    tf_score: int = 100
    diag_score: int = 100
    lifecycle_score: int = 100
    overall: int = 100

    @property
    def color(self) -> str:
        if self.overall >= 80:
            return "green"
        if self.overall >= 50:
            return "yellow"
        return "red"

    @property
    def label(self) -> str:
        if self.overall >= 80:
            return "HEALTHY"
        if self.overall >= 50:
            return "DEGRADED"
        return "CRITICAL"


class StateBus:
    """Thread-safe shared state store. All collectors write here; TUI reads here."""

    def __init__(self):
        self._lock = threading.RLock()
        self.topics: dict[str, TopicState] = {}
        self.nodes: dict[str, NodeState] = {}
        self.tf = TFState()
        self.diag = DiagState()
        self.lifecycle: dict[str, LifecycleState] = {}
        self.odom = OdomState()
        self.alerts: deque[Alert] = deque(maxlen=500)
        self.health = HealthScore()
        self._started_at: float = time.monotonic()

    def push_alert(self, severity: str, source: str, message: str) -> None:
        with self._lock:
            self.alerts.appendleft(Alert(
                timestamp=time.time(),
                severity=severity,
                source=source,
                message=message,
            ))

    def uptime_seconds(self) -> float:
        return time.monotonic() - self._started_at
