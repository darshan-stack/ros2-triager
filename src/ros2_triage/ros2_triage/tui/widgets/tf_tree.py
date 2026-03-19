# tui/widgets/tf_tree.py
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Tree
from ...state.state_bus import StateBus


class TFTreeWidget(Widget):
    def __init__(self, bus: StateBus):
        super().__init__()
        self.bus = bus

    def compose(self) -> ComposeResult:
        yield Tree("TF Frames", id="tf-tree")

    def refresh_data(self) -> None:
        try:
            tree = self.query_one("#tf-tree", Tree)
        except Exception:
            return
        tree.clear()

        with self.bus._lock:
            frames = dict(self.bus.tf.frames)
            stale = set(self.bus.tf.stale_frames)
            broken = set(self.bus.tf.broken_chains)

        # Build tree: find root frames (no parent or parent not in frames)
        all_frames = set(frames.keys())
        roots = [f for f, p in frames.items() if not p or p not in frames]

        def add_children(node, parent_name: str) -> None:
            for child, parent in frames.items():
                if parent == parent_name:
                    label = child
                    if child in broken:
                        label = f"[red]⚠ {child} (BROKEN)[/red]"
                    elif child in stale:
                        label = f"[yellow]◌ {child} (STALE)[/yellow]"
                    child_node = node.add(label)
                    add_children(child_node, child)

        for root in sorted(roots):
            label = root
            if root in broken:
                label = f"[red]⚠ {root} (BROKEN)[/red]"
            elif root in stale:
                label = f"[yellow]◌ {root} (STALE)[/yellow]"
            root_node = tree.root.add(label, expand=True)
            add_children(root_node, root)

        # Show frame count
        tree.root.label = f"TF Frames ({len(frames)} total)"
