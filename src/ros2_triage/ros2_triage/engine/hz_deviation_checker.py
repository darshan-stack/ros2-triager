# engine/hz_deviation_checker.py
from ..state.state_bus import TopicState


def compute_deviation(topic: TopicState) -> dict:
    """
    Compute Hz deviation from expected for a topic.

    Returns:
      - deviation_pct: percentage deviation from expected (0 if no expected_hz)
      - status: 'OK' | 'WARN_HZ' | 'LOW_HZ' | 'DEAD'
      - message: human-readable description
    """
    if topic.expected_hz is None or topic.expected_hz <= 0:
        return {
            "deviation_pct": 0.0,
            "status": topic.status,
            "message": f"{topic.actual_hz:.1f} Hz (no expected rate configured)",
        }

    if topic.actual_hz <= 0:
        deviation_pct = 100.0
    else:
        deviation_pct = abs(topic.actual_hz - topic.expected_hz) / topic.expected_hz * 100.0

    if deviation_pct >= 50.0:
        status = "LOW_HZ"
    elif deviation_pct >= 20.0:
        status = "WARN_HZ"
    else:
        status = "OK"

    message = (
        f"{topic.actual_hz:.1f} Hz vs {topic.expected_hz:.1f} expected "
        f"({deviation_pct:.1f}% deviation)"
    )
    return {
        "deviation_pct": deviation_pct,
        "status": status,
        "message": message,
    }
