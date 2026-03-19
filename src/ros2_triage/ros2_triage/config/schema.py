# config/schema.py
from pydantic import BaseModel, Field


class TopicConfig(BaseModel):
    name: str
    topic: str
    type: str | None = None
    expected_hz: float | None = None
    critical: bool = False
    dead_threshold_sec: float = 5.0


class TFFrameCheck(BaseModel):
    parent: str
    child: str
    description: str = ""


class Settings(BaseModel):
    refresh_interval_ms: int = 1000
    stale_data_timeout_sec: float = 5.0
    dead_topic_threshold_sec: float = 5.0
    hz_deviation_warn_pct: float = 20.0
    hz_deviation_dead_pct: float = 50.0
    preflight_min_health: int = 80


class TriagerConfig(BaseModel):
    settings: Settings = Field(default_factory=Settings)
    monitored_topics: list[TopicConfig] = Field(default_factory=list)
    tf_frames: list[TFFrameCheck] = Field(default_factory=list)
    lifecycle_nodes: list[str] = Field(default_factory=list)
    alert_webhook_url: str | None = None
    robot_name: str = "robot"
