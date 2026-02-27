# ROS 2 Triager 🔬: Advanced Runtime Diagnostics for Robotic Systems

**ROS 2 Triager** is a powerful command-line interface (CLI) plugin designed for **real-time runtime graph diagnostics** in ROS 2 environments. It goes beyond static configuration checks to actively monitor your running robot, identifying critical issues like dead topics, QoS mismatches, TF tree problems, and more, all while providing actionable suggestions for resolution.

> **`ros2doctor`** answers: *"Is your ROS 2 system installed correctly?"*
> **`ros2 triage`** answers: *"Is your robot behaving correctly right now?"*

--- 

## ✨ Key Differentiators & Novelty

ROS 2 Triager stands out by focusing on **dynamic runtime analysis**, offering capabilities crucial for maintaining the health and performance of complex robotic deployments. Its novel features provide unparalleled insight and automation potential:

*   **Live Monitoring (`--watch`):** Continuously observe your robot's health in real-time, with automatic refreshes and clear terminal output, allowing for immediate detection and response to emerging issues.
*   **Snapshot-based Differencing (`--snapshot-save`, `--snapshot-diff`):** Establish a "golden" baseline of a healthy system's graph state and then automatically compare current states against it. This enables proactive detection of regressions, unexpected changes, and graph drift over time.
*   **Expected Node Checking (`--expected YAML`):** Define the anticipated set of running nodes in a simple YAML file. ROS 2 Triager will then report any missing or unexpected nodes, ensuring all critical components are operational.
*   **Actionable Suggestions:** Every detected finding is accompanied by clear, concise, and context-aware suggestions, empowering users to quickly diagnose and resolve problems.
*   **CI/CD Integration (`--json`):** Generate machine-readable JSON output and leverage exit codes for severity levels, facilitating seamless integration into automated testing and continuous integration/continuous deployment pipelines.

--- 

## 🚀 Features

ROS 2 Triager provides a comprehensive suite of checks to ensure the robustness of your robotic applications:

| Check | What it finds | Flag |
|-------|---------------|------|
| 🔴 **Dead Topics** | Topics with publishers but no subscribers (or vice versa), indicating communication breakdowns. | `--dead-topics` / `--no-dead-topics` |
| ⚠️ **QoS Mismatches** | Incompatible Quality of Service settings (e.g., reliability, durability) between publishers and subscribers, leading to message loss. | `--qos` / `--no-qos` |
| 🔗 **TF Tree Issues** | Missing frames, broken transform chains, or inconsistencies in the robot's coordinate transformation tree. | `--tf` / `--no-tf` |
| ⏱️ **Hz Rate Check** | Anomalies in topic publishing rates, flagging topics that are slower than expected. | `--check-hz` |
| 🤖 **Expected Nodes** | Deviations from a predefined list of expected running nodes, identifying missing or rogue processes. | `--expected YAML_FILE` |
| 🔄 **Graph Drift** | Changes in the ROS 2 graph structure compared to a saved baseline snapshot. | `--snapshot-diff FILE` |

All findings are **severity-ranked** (1=INFO, 2=WARN, 3=CRIT) and include **actionable suggestions**.

--- 

## 📐 Architecture

The modular architecture of ROS 2 Triager ensures efficient and extensible diagnostic capabilities. It operates by leveraging a temporary `rclpy` node to non-intrusively inspect the live ROS 2 graph.

![ROS 2 Triager Architecture](architecture_diagram.png)

**Core Components:**

*   **ROS 2 CLI Integration:** Seamlessly integrates as a `ros2` subcommand (`ros2 triage`).
*   **TriageCommand:** The primary entry point, handling argument parsing and orchestrating the diagnostic process.
*   **rclpy Node (Inspector):** A transient ROS 2 node responsible for gathering real-time information about topics, nodes, QoS settings, and the TF tree.
*   **Checks Orchestration:** Manages the execution of various diagnostic modules, including both foundational and advanced checks.
*   **Findings:** Standardized data structures encapsulating detected issues, their severity, and suggested resolutions.
*   **Reporter:** Formats findings for human-readable console output (with color-coding) or machine-readable JSON for automated systems.

--- 

## ⚙️ Workflow

ROS 2 Triager's workflow is designed for both interactive debugging and automated system health monitoring.

![ROS 2 Triager Workflow](workflow_diagram.png)

**Diagnostic Flow:**

1.  **Command Execution:** A user or automated system invokes `ros2 triage`.
2.  **Initialization & Argument Parsing:** The tool initializes the ROS 2 context and processes command-line arguments to determine the desired checks and output format.
3.  **Execution Mode Selection:** Depending on the `--watch` flag, it either performs a single diagnostic run or enters a continuous monitoring loop.
4.  **Graph Introspection:** The Inspector Node builds a real-time snapshot of the ROS 2 graph.
5.  **Check Execution:** All enabled diagnostic checks are performed against the current graph state.
6.  **Finding Collection:** Results from each check are aggregated into a comprehensive list of findings.
7.  **Reporting:** Findings are presented in the specified format (human-readable or JSON).
8.  **Exit Status:** The tool exits with a status code reflecting the highest severity finding, enabling CI/CD pipeline integration.

--- 

## ⚡ Quickstart

### Prerequisites

```bash
# ROS 2 Humble (or compatible distro, e.g., Jazzy, Rolling)
source /opt/ros/humble/setup.bash
```

### Build

```bash
cd ~/ros2-triager
colcon build --symlink-install --packages-select ros2_triage
source install/local_setup.bash
```

### Run

```bash
# Full check (all default checks enabled)
ros2 triage

# JSON output for CI/CD pipelines
ros2 triage --json

# Only critical findings (severity 3)
ros2 triage --severity-threshold 3

# Skip QoS check
ros2 triage --no-qos

# Enable Hz rate check (measures for 3 seconds by default)
ros2 triage --check-hz

# Check against an expected_nodes.yaml file
ros2 triage --expected path/to/expected_nodes.yaml

# Save a snapshot of the current healthy graph state
ros2 triage --snapshot-save healthy_baseline.json

# Diff current state against a saved snapshot
ros2 triage --snapshot-diff healthy_baseline.json

# Live monitoring mode (refreshes every 5 seconds)
ros2 triage --watch

# Help
ros2 triage --help
```

--- 

## 📊 Example Output

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

## 🤖 Demo Scenarios

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

## ⚙️ CI Integration

Leverage `--json` output and exit codes to automate checks in your CI/CD pipelines:

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

## 🧑‍💻 Development & Testing

### Project Structure

```
ros2_triage/
├── command/
│   └── triage.py          # TriageCommand — main CLI entry point
├── checks/
│   ├── finding.py         # Finding dataclass + severity constants
│   ├── graph_utils.py     # rclpy topic graph snapshot
│   ├── dead_topic.py      # Dead publisher/subscriber detection
│   ├── qos_check.py       # QoS reliability/durability mismatch
│   ├── tf_check.py        # TF tree frame connectivity
│   ├── hz_check.py        # Topic rate anomaly check (NEW)
│   └── node_check.py      # Missing/unexpected node check (NEW)
├── reporter.py            # Human (colorama) + JSON output
demo/
├── dead_topic_demo.launch.py
└── qos_mismatch_demo.launch.py
test/
├── test_dead_topic.py
├── test_qos_check.py
└── test_finding.py
```

### Running Tests

```bash
cd ~/ros2-triager/src/ros2_triage
source /opt/ros/humble/setup.bash
python3 -m pytest test/ -v
```

--- 

## 📄 License

Apache 2.0
