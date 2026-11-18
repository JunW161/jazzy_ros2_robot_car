"""Wheel-RPM interface for the geometry in the user's robot_config.h.
This computes setpoints, not a guarantee that measured RPM equals the request.
The ESP32 remains responsible for PID, reverse interlocks and its watchdog.
"""
import math

RADIUS_M = 0.044
TRACK_M = 0.195
MAX_RPM = 40.0
PROFILE_VERSION = 21


def profile_updates():
    return {
        'control_profile_version': PROFILE_VERSION,
        'wheel_radius_m': RADIUS_M,
        'wheel_track_m': TRACK_M,
        'manual_max_wheel_rpm': MAX_RPM,
        'manual_straight_rpm': 40.0,
        'manual_turn_rpm': 20.0,
        'max_linear': 2 * math.pi * RADIUS_M * MAX_RPM / 60,
        'max_angular': 4 * math.pi * RADIUS_M * MAX_RPM / (60 * TRACK_M),
    }


def wheel_rpms(v, w, config):
    r = config.get('wheel_radius_m', RADIUS_M)
    track = config.get('wheel_track_m', TRACK_M)
    factor = 60.0 / (2.0 * math.pi * r)
    return (v - w * track / 2) * factor, (v + w * track / 2) * factor


def clamp_twist(v, w, config):
    """Keep independent limits AND both wheel limits, including combined requests."""
    v = max(-config['max_linear'], min(config['max_linear'], v))
    w = max(-config['max_angular'], min(config['max_angular'], w))
    left, right = wheel_rpms(v, w, config)
    peak = max(abs(left), abs(right))
    limit = min(MAX_RPM, float(config.get('manual_max_wheel_rpm', MAX_RPM)))
    if peak > limit:
        scale = limit / peak
        v *= scale
        w *= scale
    return v, w


def limits_info(config):
    return {
        'linear': config['max_linear'], 'angular': config['max_angular'],
        'radius_m': config['wheel_radius_m'], 'track_m': config['wheel_track_m'],
        'max_wheel_rpm': config['manual_max_wheel_rpm'],
        'straight_rpm': config['manual_straight_rpm'],
        'turn_rpm': config['manual_turn_rpm'],
        'choices_rpm': [20, 30, 40],
    }
