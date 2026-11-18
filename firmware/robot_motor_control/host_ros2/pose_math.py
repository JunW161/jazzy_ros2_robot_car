"""Pure-python, ROS-independent differential-drive goal geometry.
Coordinate convention: x forward, y left, positive yaw counterclockwise.
"""
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6 or not all(map(math.isfinite, (qx,qy,qz,qw))):
        raise ValueError('Quaternion is invalid')
    qx,qy,qz,qw = (i/norm for i in (qx,qy,qz,qw))
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))


def goal_errors(x, y, yaw, gx, gy, gyaw):
    dx, dy = gx-x, gy-y
    return (math.hypot(dx,dy),wrap(math.atan2(dy,dx)-yaw),wrap(gyaw-yaw))


def forward_arc_command(distance, heading_error, track=0.195, max_v=0.12):
    """Forward only. Prevent either wheel needing reverse rotation."""
    v = clamp(0.9*distance, 0.10, max_v)
    raw_w = clamp(1.7*heading_error, -0.7, 0.7)
    w_limit = 1.80 * v / track  # < 2*v/track => wheel refs non-negative
    return v, clamp(raw_w, -w_limit, w_limit)
