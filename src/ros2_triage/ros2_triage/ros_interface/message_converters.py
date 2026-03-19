# ros_interface/message_converters.py
from __future__ import annotations
from typing import Any


def msg_to_dict(msg: Any) -> dict:
    """
    Convert a ROS message instance to a plain Python dict.
    Handles nested messages, arrays, and primitive types.
    """
    if hasattr(msg, "__slots__") and hasattr(msg, "get_fields_and_field_types"):
        result = {}
        for field_name in msg.__slots__:
            # __slots__ entries start with underscore for some msg types
            key = field_name.lstrip("_")
            val = getattr(msg, key, getattr(msg, field_name, None))
            result[key] = msg_to_dict(val)
        return result
    elif isinstance(msg, (list, tuple)):
        return [msg_to_dict(item) for item in msg]
    elif isinstance(msg, (int, float, str, bool, bytes)) or msg is None:
        return msg  # type: ignore[return-value]
    else:
        return str(msg)


def header_stamp_to_sec(stamp: Any) -> float:
    """Convert a ROS Header stamp (with sec + nanosec) to float seconds."""
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except AttributeError:
        return 0.0


def pose_to_dict(pose: Any) -> dict:
    """Convert a geometry_msgs/Pose to a simple dict."""
    try:
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }
    except AttributeError:
        return {}


def twist_to_dict(twist: Any) -> dict:
    """Convert a geometry_msgs/Twist to a simple dict."""
    try:
        return {
            "linear_x": twist.linear.x,
            "linear_y": twist.linear.y,
            "linear_z": twist.linear.z,
            "angular_x": twist.angular.x,
            "angular_y": twist.angular.y,
            "angular_z": twist.angular.z,
        }
    except AttributeError:
        return {}
