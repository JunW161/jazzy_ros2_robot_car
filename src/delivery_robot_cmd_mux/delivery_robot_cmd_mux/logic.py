from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Velocity:
    linear_x: float = 0.0
    angular_z: float = 0.0


def clamp_velocity(value: Velocity, max_linear: float, max_angular: float) -> Velocity:
    values = (value.linear_x, value.angular_z, max_linear, max_angular)
    if not all(math.isfinite(v) for v in values):
        return Velocity()
    if max_linear <= 0 or max_angular <= 0:
        return Velocity()
    return Velocity(
        max(-max_linear, min(max_linear, float(value.linear_x))),
        max(-max_angular, min(max_angular, float(value.angular_z))),
    )


def select_velocity(mode: str, manual: Velocity, nav: Velocity,
                    manual_fresh: bool, nav_fresh: bool) -> Velocity:
    mode = mode.upper()
    if mode == 'MANUAL' and manual_fresh:
        return manual
    if mode == 'NAVIGATION' and nav_fresh:
        return nav
    return Velocity()
