"""Core utility functions: angle math, quaternion conversions, pose types.

Conventions follow ROS REP-103: +x=forward, +y=left, +yaw=CCW (left turn).
"""

import math
from collections import namedtuple


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    """Extract yaw (rotation around Z) from a quaternion.

    Accepts any object with .x .y .z .w attributes
    (ROS quaternion message or geometry_msgs Quaternion).
    """
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ── Pose types ───────────────────────────────────────────────────

TagPose = namedtuple('TagPose', ['dist', 'lat', 'yaw', 'normal', 'stamp_ns'])
"""Tag pose in robot base_link frame (REP-103).

dist     : float — forward distance (m), positive = tag ahead
lat      : float — lateral offset (m), positive = tag to the left
yaw      : float — bearing to the tag (rad) = atan2(lat, dist). ≈0 when the
                   robot is pointed straight at the tag.
normal   : float — direction of the tag's OUTWARD normal expressed as an angle
                   in the base_link ground plane (rad). This is the direction
                   the robot must eventually face (reversed) to dock squarely.
                   0 ⇒ tag faces straight back at the robot along -x.
stamp_ns : int   — detection timestamp (nanoseconds, from sensor_msgs/Header)
"""
