#!/usr/bin/env bash
# ros2_triage_env.sh
# Source this file to make `ros2 triage` available in your shell.
# Usage:  source ~/ros2_triger/ros2_triage_env.sh

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS 2 Humble base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS 2 Humble sourced"
else
    echo "❌ /opt/ros/humble/setup.bash not found — is ROS 2 installed?"
    return 1
fi

# 2. Source the ros2_triage workspace overlay
if [ -f "$WORKSPACE_DIR/install/local_setup.bash" ]; then
    source "$WORKSPACE_DIR/install/local_setup.bash"
    echo "✅ ros2_triage workspace sourced ($WORKSPACE_DIR)"
else
    echo "❌ install/local_setup.bash not found — did you run: colcon build?"
    echo "   cd $WORKSPACE_DIR && colcon build --symlink-install --packages-select ros2_triage"
    return 1
fi

echo "🔬 ros2 triage is ready! Try: ros2 triage --help"
