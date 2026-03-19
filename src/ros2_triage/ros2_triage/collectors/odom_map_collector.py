# collectors/odom_map_collector.py
from __future__ import annotations
import time
from rclpy.node import Node
from ..state.state_bus import StateBus, OdomState
from .base_collector import BaseCollector
import rclpy.qos as qos

try:
    from nav_msgs.msg import Odometry, OccupancyGrid
    _HAS_NAV = True
except ImportError:
    _HAS_NAV = False


class OdomMapCollector(BaseCollector):
    """Subscribes to /odom and /map for TUI visualization."""

    def __init__(self, node: Node, bus: StateBus):
        super().__init__(node, bus, interval_sec=99999)  # callback-driven
        self._map_data: list | None = None
        self._map_info: dict | None = None
        self._odom_sub = None
        self._map_sub = None

    def start(self) -> None:
        if not _HAS_NAV:
            self._node.get_logger().warn(
                "nav_msgs not available — odom/map collection disabled"
            )
            return
        self._odom_sub = self._node.create_subscription(
            Odometry, "/odom", self._odom_cb, qos.qos_profile_sensor_data
        )
        try:
            self._map_sub = self._node.create_subscription(
                OccupancyGrid, "/map", self._map_cb,
                rclpy.qos.QoSProfile(
                    depth=1,
                    durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                )
            )
        except Exception:
            pass

    def collect(self) -> None:
        pass  # callback-driven

    def _odom_cb(self, msg: "Odometry") -> None:
        with self._bus._lock:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            t = msg.twist.twist
            self._bus.odom = OdomState(
                x=p.x, y=p.y, z=p.z,
                qx=o.x, qy=o.y, qz=o.z, qw=o.w,
                linear_vel=(t.linear.x**2 + t.linear.y**2)**0.5,
                angular_vel=t.angular.z,
                last_update=time.monotonic(),
            )

    def _map_cb(self, msg: "OccupancyGrid") -> None:
        # Store map metadata for TUI visualization
        self._map_data = list(msg.data)
        self._map_info = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin_x": msg.info.origin.position.x,
            "origin_y": msg.info.origin.position.y,
        }

    @property
    def map_data(self) -> list | None:
        return self._map_data

    @property
    def map_info(self) -> dict | None:
        return self._map_info
