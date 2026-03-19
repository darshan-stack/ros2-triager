# engine/lifecycle_checker.py
from ..state.state_bus import LifecycleState

STATE_LABELS = {
    1: "UNCONFIGURED",
    2: "INACTIVE",
    3: "ACTIVE",
    4: "FINALIZED",
}


def check_lifecycle(lifecycle: dict[str, LifecycleState]) -> dict:
    """
    Check all lifecycle nodes for non-ACTIVE states.

    Returns:
      - unconfigured: list of node names in UNCONFIGURED state
      - inactive: list of node names in INACTIVE state
      - finalized: list of node names in FINALIZED state
      - active: list of node names in ACTIVE state
      - issues: list of (node_name, state_label) tuples for non-ACTIVE nodes
      - is_healthy: bool
    """
    unconfigured = []
    inactive = []
    finalized = []
    active = []

    for name, state in lifecycle.items():
        sid = state.state_id
        if sid == 1:
            unconfigured.append(name)
        elif sid == 2:
            inactive.append(name)
        elif sid == 4:
            finalized.append(name)
        else:
            active.append(name)

    issues = (
        [(n, "UNCONFIGURED") for n in unconfigured]
        + [(n, "INACTIVE") for n in inactive]
        + [(n, "FINALIZED") for n in finalized]
    )

    return {
        "unconfigured": unconfigured,
        "inactive": inactive,
        "finalized": finalized,
        "active": active,
        "issues": issues,
        "is_healthy": len(issues) == 0,
    }


def lifecycle_status_color(state_id: int) -> str:
    """Return a Textual color class for a lifecycle state."""
    if state_id == 3:  # ACTIVE
        return "success"
    if state_id == 2:  # INACTIVE
        return "warning"
    if state_id in (1, 4):  # UNCONFIGURED / FINALIZED
        return "error"
    return "muted"
