#!/usr/bin/env python3
"""Regression checks for V8.3.1 user-facing command and telemetry contract."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
main=(root/'main/main.cpp').read_text()
config=(root/'main/robot_config.h').read_text()
drive=(root/'main/robot_drive.cpp').read_text()
bench=(root/'host_ros2/bench_wheel_test_v8.py').read_text()
safe=(root/'host_ros2/safe_cmd_vel.py').read_text()
checks={
 'pid six floats': 'msg->data.size==6' in main,
 'pid best-effort output V10': 'rclc_publisher_init_best_effort(&pubPid' in main,
 'wheel best-effort output V10': 'rclc_publisher_init_best_effort(&pubWheel' in main,
 'best effort diagnostics': 'V10 debug telemetry uses BEST_EFFORT' in main,
 'PID request implemented': 'robot_drive_pid_request' in drive,
 'PID status implemented': 'robot_drive_pid_status' in drive,
 'PID status contains 8': 'pidStatus.data.size=8' in main,
 'wheel debug contains 10': 'wheelMsg.data.size=10' in main,
 'preserve user PID sample KdR': 'a.kd<=1' in drive,
 'encoder launch timeout preserved': 'fault("START_ENCODER_TIMEOUT")' in drive,
 'less aggressive launch': 'LAUNCH_MAX_MS=750' in config and 'LAUNCH_MOVING_RPM=6.0f' in config,
 'VL6180X no motion dependency': 'NearStopGate' not in drive and 'forward_limit(' not in drive,
 'yaw controller retained': 'yawCtl.observe(' in drive and 'yawCtl.update(' in drive,
 'bench requires raise confirmation': 'wheels-raised-confirmed' in bench,
 'bench waits for DDS': 'self.arm.get_subscription_count()' in bench,
 'bench checks firmware ack': 'self.safety[9]' in bench,
 'regular motion waits for DDS': 'self.publisher.get_subscription_count()' in safe,
 'regular motion monitors PWM': 'measured_RPM=' in safe and 'PWM=' in safe,
 'normal motion sends stop': 'node.zero_for(1.6)' in safe,
}
for text,okay in checks.items():
 assert okay, f'FAIL: {text}'
for pin in ['I2C_SDA_GPIO=15','I2C_SCL_GPIO=16','IMU_INT_GPIO=17',
            'TOF_XSHUT_GPIO=47','TOF_INT_GPIO=21',
            'L_RPWM=4','L_LPWM=5','L_EN=6','L_ENC_A=10','L_ENC_B=11',
            'R_RPWM=40','R_LPWM=41','R_EN=42','R_ENC_A=1','R_ENC_B=2',
            'TRACK_M=0.195f','RADIUS_M=0.044f']:
 assert pin in config,pin
print(f'PASS V8.3.1 contract ({len(checks)} checks + fixed GPIO/geometry)')
