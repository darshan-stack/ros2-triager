# engine/tf_staleness_checker.py
import time
from ..state.state_bus import TFState


def check_tf_staleness(
    tf: TFState,
    stale_threshold_sec: float = 0.5,
) -> dict:
    """
    Check TF state for stale frames and broken chains.

    Returns a dict with:
      - stale_frames: list of frame IDs that haven't been updated recently
      - broken_chains: list of frame IDs in loops or with missing parents
      - age_sec: seconds since last TF update
      - is_healthy: bool
    """
    age_sec = time.monotonic() - tf.last_update if tf.last_update else 9999.0
    return {
        "stale_frames": tf.stale_frames,
        "broken_chains": tf.broken_chains,
        "frame_count": len(tf.frames),
        "age_sec": age_sec,
        "is_healthy": len(tf.stale_frames) == 0 and len(tf.broken_chains) == 0,
    }


def tf_root_frames(frames: dict[str, str]) -> list[str]:
    """Return all root TF frames (those with no parent or parent not in dict)."""
    return [f for f, p in frames.items() if not p or p not in frames]


def tf_chain(frames: dict[str, str], child: str) -> list[str]:
    """Return the chain from child up to root, detecting loops."""
    chain = []
    visited = set()
    cur = child
    while cur and cur not in visited:
        chain.append(cur)
        visited.add(cur)
        cur = frames.get(cur, "")
    return chain
