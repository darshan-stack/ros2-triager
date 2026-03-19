# tui/widgets/sparkline.py
from textual.widget import Widget

BRAILLE = " ⣀⣄⣤⣦⣶⣷⣿"


def hz_sparkline(history: list, width: int = 8) -> str:
    """Render a braille sparkline from a list of Hz values."""
    if not history:
        return " " * width
    mx = max(history) or 1
    samples = list(history)[-width:]
    return "".join(
        BRAILLE[min(int(v / mx * (len(BRAILLE) - 1)), len(BRAILLE) - 1)]
        for v in samples
    )


class SparklineWidget(Widget):
    """
    Simple inline sparkline widget displaying a list of float values
    using braille dot characters.
    """

    def __init__(self, data: list[float] | None = None, label: str = ""):
        super().__init__()
        self._data: list[float] = data or []
        self._label = label

    def update(self, data: list[float]) -> None:
        self._data = data
        self.refresh()

    def render(self) -> str:
        spark = hz_sparkline(self._data, width=20)
        if self._label:
            return f"{self._label}: {spark}"
        return spark
