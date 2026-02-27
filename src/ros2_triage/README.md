# ros2 triage 🔬

A **ROS 2 CLI plugin** for runtime graph diagnostics. Think of it as `ros2doctor` for your running robot — it finds dead topics, QoS mismatches, and TF tree issues while your system is live.

```
ros2 triage
```

> **ros2doctor** answers: *"Is your ROS 2 system installed correctly?"*
> **ros2 triage** answers: *"Is your robot behaving correctly right now?"*

---

## Features

| Check | What it finds | Flag |
|-------|--------------|------|
| 🔴 Dead Topics | Topics with publishers but no subscribers (or vice versa) | `--dead-topics` / `--no-dead-topics` |
| ⚠️ QoS Mismatches | Reliability/durability incompatibilities that drop messages | `--qos` / `--no-qos` |
| 🔗 TF Tree | Missing frames, broken transform chains | `--tf` / `--no-tf` |

All findings are **severity-ranked** (1=INFO, 2=WARN, 3=CRIT) and include **actionable suggestions**.

---

## Quickstart

### Prerequisites

```bash
# ROS 2 Humble (or compatible distro)
source /opt/ros/humble/setup.bash
```

### Build

```bash
cd ~/ros2_triger
colcon build --symlink-install --packages-select ros2_triage
source install/local_setup.bash
```

### Run

```bash
# Full check (all enabled by default)
ros2 triage

# JSON output for CI
ros2 triage --json

# Only critical findings
ros2 triage --severity-threshold 3

# Skip QoS check
ros2 triage --no-qos

# Help
ros2 triage --help
```

---

## Example Output

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ros2 triage — Runtime Diagnostic Report
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  🔴  DEAD TOPICS
  Topics with missing publishers or subscribers
  ────────────────────────────────────────────────────────────
  [CRIT]  /cmd_vel
         1 subscriber(s) [nav2_node] but 0 publishers — topic is UNPUBLISHED.
         💡 Check if the node that should publish this topic is running:
            `ros2 node list`. Verify launch files include the publisher node.

  ⚠️   QoS MISMATCHES
  Publisher ↔ Subscriber QoS incompatibilities
  ────────────────────────────────────────────────────────────
  [CRIT]  /sensor_data
         Reliability mismatch: publisher [sensor_node]=RELIABLE,
         subscriber [processor]=BEST_EFFORT. Messages will be DROPPED.
         💡 Change processor subscription QoS to RELIABLE.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Summary: 2 CRITICAL  0 WARNING  0 INFO
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### JSON output (`--json`)

```json
{
  "schema_version": "1.0",
  "total_findings": 2,
  "summary": {"critical": 2, "warning": 0, "info": 0},
  "checks": [
    {
      "name": "dead_topics",
      "findings": [
        {
          "check": "dead_topics",
          "topic": "/cmd_vel",
          "severity": 3,
          "message": "1 subscriber(s) [nav2_node] but 0 publishers — topic is UNPUBLISHED.",
          "suggestion": "Check if the node that should publish this topic is running..."
        }
      ]
    }
  ]
}
```

---

## Demo Scenarios

### Dead Topic Demo

```bash
# Terminal 1: Launch nodes with intentional dead topics
ros2 launch ros2_triage dead_topic_demo.launch.py

# Terminal 2: Run triage
ros2 triage --no-qos --no-tf
```

Expected: `/cmd_vel` flagged as UNPUBLISHED (severity 3), `/sensor_data` flagged as UNSUBSCRIBED.

### QoS Mismatch Demo

```bash
# See demo/qos_mismatch_demo.launch.py for one-liner commands
# Terminal 1: RELIABLE publisher on /qos_demo
# Terminal 2: BEST_EFFORT subscriber on /qos_demo
# Terminal 3:
ros2 triage --no-dead-topics --no-tf
```

---

## CI Integration

Use `--json` and exit code to fail the pipeline on critical findings:

```yaml
# .github/workflows/ros2_check.yml
- name: Run ros2 triage
  run: |
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash
    ros2 triage --json --severity-threshold 3 > triage_report.json
  # Exits with code 1 if any severity-3 findings exist
```

---

## Architecture

```
ros2_triage/
├── command/
│   └── triage.py          # TriageCommand — main CLI entry point
├── checks/
│   ├── finding.py         # Finding dataclass + severity constants
│   ├── graph_utils.py     # rclpy topic graph snapshot
│   ├── dead_topic.py      # Dead publisher/subscriber detection
│   ├── qos_check.py       # QoS reliability/durability mismatch
│   └── tf_check.py        # TF tree frame connectivity
├── reporter.py            # Human (colorama) + JSON output
demo/
├── dead_topic_demo.launch.py
└── qos_mismatch_demo.launch.py
test/
├── test_dead_topic.py
├── test_qos_check.py
└── test_finding.py
```

---

## Running Tests

```bash
cd ~/ros2_triger/src/ros2_triage
source /opt/ros/humble/setup.bash
python3 -m pytest test/ -v
```

---

## License

Apache 2.0
