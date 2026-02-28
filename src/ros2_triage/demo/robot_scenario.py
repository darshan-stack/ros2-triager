#!/usr/bin/env python3
"""
robot_scenario.py — Simulated robot with INTENTIONAL problems.

Mimics a mobile robot stack where:
  1. /cmd_vel  → nav2 subscribes but motor_driver is NOT running   [UNPUBLISHED]
  2. /scan     → lidar publishes but slam_toolbox is NOT running   [UNSUBSCRIBED]
  3. /battery  → sensor publishes RELIABLE, monitor subscribes BEST_EFFORT [QoS MISMATCH]
  4. /imu/data → imu driver publishes but no one listens           [UNSUBSCRIBED]

Run this in Terminal 1, then `ros2 triage` in Terminal 2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu


# ── Node 1: Navigation Controller ─────────────────────────────────────────────
# Simulates nav2 / move_base: subscribes to /cmd_vel (waiting for motor driver)
# Also subscribes to /map (no one publishes map because map_server isn't running)
class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.get_logger().info('Navigation controller started')

        # Problem 1: subscribes /cmd_vel — motor_driver node is missing
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            lambda msg: self.get_logger().info(f'Received cmd_vel'),
            10)

        # Problem (extra): subscribes /map — nobody publishes
        self.map_sub = self.create_subscription(
            String, '/map_data',
            lambda msg: None,
            10)

        self.get_logger().warn('Waiting for /cmd_vel publisher (motor_driver missing?)')


# ── Node 2: LiDAR Driver ──────────────────────────────────────────────────────
# Simulates a lidar publishing /scan — but slam_toolbox isn't running to consume it
class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_driver')
        self.get_logger().info('LiDAR driver started — publishing /scan at 10 Hz')

        # Problem 2: publishes /scan but no subscriber (slam_toolbox not running)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.range_min = 0.1
        msg.range_max = 30.0
        msg.ranges = [1.5] * 314
        self.scan_pub.publish(msg)


# ── Node 3: IMU Driver ────────────────────────────────────────────────────────
# IMU publishes data but nobody is subscribed (ekf_node not running)
class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.get_logger().info('IMU driver started — publishing /imu/data at 100 Hz')

        # Problem 4: publishes /imu/data but no subscriber
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        self.imu_pub.publish(msg)


# ── Node 4: Battery Sensor (RELIABLE publisher) ───────────────────────────────
class BatterySensor(Node):
    def __init__(self):
        super().__init__('battery_sensor')
        self.get_logger().info('Battery sensor started — publishing RELIABLE /battery_state')

        # Problem 3a: RELIABLE publisher
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.bat_pub = self.create_publisher(Float32, '/battery_state', reliable_qos)
        self.timer = self.create_timer(1.0, self.publish_battery)

    def publish_battery(self):
        msg = Float32()
        msg.data = 87.5  # 87.5% battery
        self.bat_pub.publish(msg)


# ── Node 5: Battery Monitor (BEST_EFFORT subscriber) ─────────────────────────
class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.get_logger().info('Battery monitor started — subscribing BEST_EFFORT /battery_state')

        # Problem 3b: BEST_EFFORT subscriber (will NOT connect to RELIABLE pub — messages dropped)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.bat_sub = self.create_subscription(
            Float32, '/battery_state',
            lambda msg: self.get_logger().warn(f'Battery: {msg.data}%'),
            best_effort_qos)

        self.get_logger().error(
            'QoS MISMATCH: publisher is RELIABLE, I am BEST_EFFORT — messages will be dropped!')


# ── Spin all nodes in parallel ─────────────────────────────────────────────────

def main():
    print('\n' + '='*60)
    print('  Robot Scenario - Simulated Failure Demo')
    print('='*60)
    print('''
Intentional problems created:
  [CRIT] /cmd_vel       \u2192 nav2 waiting, motor_driver MISSING
  [CRIT] /map_data      \u2192 nav2 waiting, map_server MISSING
  [WARN] /scan          \u2192 lidar publishing, slam_toolbox NOT running
  [WARN] /imu/data      \u2192 imu publishing, ekf_node NOT running
  [CRIT] /battery_state \u2192 QoS MISMATCH (RELIABLE pub \u2194 BEST_EFFORT sub)

Now open a NEW terminal and run:
  source /opt/ros/humble/setup.bash
  source ~/ros2_triger/install/local_setup.bash
  ros2 triage

Or for CI JSON output:
  ros2 triage --json

Press Ctrl+C to stop.
''')
    print('='*60 + '\n')

    rclpy.init()

    # Use MultiThreadedExecutor — the correct way to run multiple nodes in one process
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=6)

    nodes = [
        NavigationController(),
        LidarDriver(),
        ImuDriver(),
        BatterySensor(),
        BatteryMonitor(),
    ]
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n\nShutting down demo nodes...')
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
