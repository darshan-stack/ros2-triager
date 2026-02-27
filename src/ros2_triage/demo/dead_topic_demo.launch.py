"""
dead_topic_demo.launch.py
────────────────────────────────────────────────────────────
Demo scenario: A subscriber node listens to /cmd_vel,
but nothing publishes to it.  This will trigger a severity-3
UNPUBLISHED finding in ros2 triage.

Run:
  ros2 launch ros2_triage dead_topic_demo.launch.py

Then in another terminal:
  ros2 triage
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # This node subscribes to /cmd_vel but nobody publishes
    dead_subscriber = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='cmd_vel_listener',
        remappings=[('chatter', 'cmd_vel')],
        output='screen',
    )

    # This node publishes to /sensor_data which nobody subscribes to
    dead_publisher = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='sensor_data_publisher',
        remappings=[('chatter', 'sensor_data')],
        output='screen',
    )

    return LaunchDescription([
        dead_subscriber,
        dead_publisher,
    ])
