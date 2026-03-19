# engine/health_scorer.py
from ..state.state_bus import StateBus, HealthScore


def compute_health(bus: StateBus) -> HealthScore:
    with bus._lock:
        topics = list(bus.topics.values())
        nodes = list(bus.nodes.values())
        tf = bus.tf
        diag = bus.diag
        lifecycle = list(bus.lifecycle.values())

    # Topics score: -20 per DEAD critical, -10 per DEAD non-critical, -5 per LOW_HZ
    t_score = 100
    for t in topics:
        if t.status == "DEAD":
            t_score -= 20 if t.is_critical else 10
        elif t.status in ("LOW_HZ", "NO_PUB"):
            t_score -= 5

    # Nodes score: -30 per zombie
    n_score = 100 - sum(30 for n in nodes if n.is_zombie)

    # TF score: -15 per stale frame, -30 per broken chain
    tf_score = 100 - len(tf.stale_frames) * 15 - len(tf.broken_chains) * 30

    # Diagnostics score: -20 per ERROR, -5 per WARN
    d_score = 100 - diag.error_count * 20 - diag.warn_count * 5

    # Lifecycle score: -20 per UNCONFIGURED, -10 per INACTIVE
    lc_score = 100
    for lc in lifecycle:
        if lc.state_id == 1:
            lc_score -= 20
        elif lc.state_id == 2:
            lc_score -= 10

    def clamp(v: int) -> int:
        return max(0, min(100, v))

    scores = [
        clamp(t_score),
        clamp(n_score),
        clamp(tf_score),
        clamp(d_score),
        clamp(lc_score),
    ]
    # Weighted average: topics 30%, nodes 20%, tf 20%, diag 20%, lifecycle 10%
    weights = [0.30, 0.20, 0.20, 0.20, 0.10]
    overall = int(sum(s * w for s, w in zip(scores, weights)))

    return HealthScore(
        topics_score=clamp(t_score),
        nodes_score=clamp(n_score),
        tf_score=clamp(tf_score),
        diag_score=clamp(d_score),
        lifecycle_score=clamp(lc_score),
        overall=overall,
    )
