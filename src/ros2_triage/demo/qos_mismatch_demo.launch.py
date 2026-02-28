"""
qos_mismatch_demo.launch.py
────────────────────────────────────────────────────────────
Demo scenario: A RELIABLE publisher and a BEST_EFFORT subscriber
on the same topic, intentionally creating a QoS incompatibility.

Since launch actions can't directly set QoS, we use minimal Python
nodes defined inline via launch_ros composition.

A simpler alternative is to run two separate nodes from the terminal:
  Terminal 1 (RELIABLE publisher):
    python3 -c "
    import rclpy; from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    n = Node('reliable_pub')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    p = n.create_publisher(String, '/qos_demo', qos)
    import time
    while rclpy.ok():
        p.publish(String(data='hello')); time.sleep(0.5)
        rclpy.spin_once(n, timeout_sec=0)
    "

  Terminal 2 (BEST_EFFORT subscriber):
    python3 -c "
    import rclpy; from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    n = Node('best_effort_sub')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    n.create_subscription(String, '/qos_demo', lambda m: print(m.data), qos)
    rclpy.spin(n)
    "

  Terminal 3 (triage):
    ros2 triage --no-dead-topics

─────────────────────────────────────────────────────────────────
Expected output:
  [CRIT]  /qos_demo
    Reliability mismatch: publisher=RELIABLE, subscriber=BEST_EFFORT
    Change subscriber QoS to RELIABLE or publisher to BEST_EFFORT.
"""
