# engine/diagnostics_parser.py
from ..state.state_bus import DiagState, DiagItem

LEVEL_LABELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def parse_diagnostics(diag: DiagState) -> dict:
    """
    Parse DiagState into a structured summary.

    Returns:
      - errors: list of DiagItem with level >= 2
      - warnings: list of DiagItem with level == 1
      - ok: list of DiagItem with level == 0
      - stale: list of DiagItem with level == 3
      - summary: human-readable one-liner
    """
    errors = []
    warnings = []
    ok_items = []
    stale = []

    for item in diag.items.values():
        if item.level >= 3:
            stale.append(item)
        elif item.level == 2:
            errors.append(item)
        elif item.level == 1:
            warnings.append(item)
        else:
            ok_items.append(item)

    total = len(diag.items)
    summary = (
        f"{len(errors)} ERROR, {len(warnings)} WARN, {len(ok_items)} OK"
        + (f", {len(stale)} STALE" if stale else "")
        + f" ({total} total)"
    )

    return {
        "errors": errors,
        "warnings": warnings,
        "ok": ok_items,
        "stale": stale,
        "summary": summary,
        "is_healthy": len(errors) == 0 and len(stale) == 0,
    }


def format_diag_row(item: DiagItem) -> tuple[str, str, str, str]:
    """Format a DiagItem into (level_str, hardware_id, name, message) for display."""
    return (
        LEVEL_LABELS.get(item.level, "???"),
        item.hardware_id or "—",
        item.name,
        item.message,
    )
