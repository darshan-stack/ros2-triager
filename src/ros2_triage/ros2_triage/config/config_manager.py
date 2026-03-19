# config/config_manager.py
import yaml
import os
from .schema import TriagerConfig


def load_config(path: str | None = None) -> dict:
    """Load YAML config. If no path, return defaults (zero-config mode)."""
    if path and os.path.exists(path):
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg = TriagerConfig(**raw)
    else:
        cfg = TriagerConfig()
    return cfg.model_dump()
