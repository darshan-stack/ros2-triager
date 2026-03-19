# engine/dead_topic_detector.py
# DEAD TOPIC RULE — must satisfy this exact logic (not just hz==0):
#
# DEAD   = publisher_count == 0 AND age > threshold
# NO_PUB = publisher_count == 0 (regardless of age)
# LOW_HZ = hz > 0 AND |hz - expected| / expected > 0.5
# OK     = everything else
#
# Rationale: latched topics (/map, /tf_static) publish once, then hz=0 forever.
# Checking publisher_count avoids false-positives on these topics.
import time
from ..state.state_bus import TopicState


def classify_topic(topic: TopicState, dead_threshold_sec: float = 5.0) -> str:
    age = time.monotonic() - topic.last_msg_time if topic.last_msg_time else 9999.0
    if topic.publisher_count == 0:
        if age > dead_threshold_sec:
            return "DEAD"
        return "NO_PUB"
    if topic.actual_hz == 0.0 and age > dead_threshold_sec:
        return "DEAD"
    if topic.expected_hz and topic.actual_hz > 0:
        deviation = abs(topic.actual_hz - topic.expected_hz) / topic.expected_hz
        if deviation > 0.5:
            return "LOW_HZ"
        if deviation > 0.2:
            return "WARN_HZ"
    return "OK"
