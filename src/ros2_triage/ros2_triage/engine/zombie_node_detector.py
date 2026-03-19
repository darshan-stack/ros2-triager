# engine/zombie_node_detector.py
from ..state.state_bus import NodeState, TopicState


def is_zombie(node: NodeState, topics: dict[str, TopicState]) -> bool:
    """
    Node is a zombie if:
    - It has at least one published topic
    - ALL of its published topics are DEAD or NO_PUB
    """
    if not node.pub_topics:
        return False
    return all(
        topics.get(t, TopicState(name=t, msg_type="")).status in ("DEAD", "NO_PUB")
        for t in node.pub_topics
    )
