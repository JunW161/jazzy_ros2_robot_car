#!/usr/bin/env python3
"""Capture one FF+PID wheel RPM trial from /robot/wheel_debug (ROS 2 Jazzy).

Usage (ROS-sourced terminal): python3 record_wheel_test.py [output.csv]
Run one constant /cmd_vel trial while recording. Ctrl+C to save summary.
"""
import csv
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

COLUMNS = ('timestamp_s','target_rpm_L','target_rpm_R','actual_rpm_L',
           'actual_rpm_R','pwm_L','pwm_R','distance_L_m','distance_R_m',
           'heading_deg','state')

class Recorder(Node):
    def __init__(self):
        super().__init__('wheel_test_csv_recorder')
        self.rows = []
        self.create_subscription(Float32MultiArray, '/robot/wheel_debug', self.recv,
            QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('Capturing /robot/wheel_debug. Run ONE constant speed test; Ctrl+C to finish.')

    def recv(self, msg):
        if len(msg.data) != 10:
            self.get_logger().warning('wheel_debug expected exactly 10 floats')
            return
        if not all(math.isfinite(x) for x in msg.data):
            return
        self.rows.append((time.time(),) + tuple(float(x) for x in msg.data))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else time.strftime('wheel_trial_%Y%m%d_%H%M%S.csv')
    path = Path(name).expanduser()
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rows = node.rows
        node.destroy_node()
        rclpy.shutdown()
        path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(COLUMNS)
            w.writerows(rows)
        print(f'\nSaved {len(rows)} samples to: {path}')
        # Only analyze actively controlled samples of a single trial; exclude
        # the first two seconds after entering ACTIVE (ramp/transient).
        samples = [r for r in rows if int(round(r[10]))==3 and abs(r[1])>1 and abs(r[2])>1]
        if len(samples)<10:
            print('Not enough ACTIVE data for assessment; keep cmd_vel running longer.')
            return
        start = samples[0][0]
        steady = [r for r in samples if r[0]>=start+2.0]
        if len(steady)<10:
            steady = samples
            print('Warning: less than 2 s steady-state data; transient affects results.')
        def avg(i): return statistics.mean(r[i] for r in steady)
        def mae(t,m): return statistics.mean(abs(r[t]-r[m]) for r in steady)
        def rmse(t,m): return math.sqrt(statistics.mean((r[t]-r[m])**2 for r in steady))
        print('--- FF + PID assessment (ACTIVE data) ---')
        print(f'Steady samples: {len(steady)} | duration: {steady[-1][0]-steady[0][0]:.2f}s')
        print(f'L: target mean {avg(1):.2f} RPM | measured mean {avg(3):.2f} RPM | MAE {mae(1,3):.2f} | RMSE {rmse(1,3):.2f}')
        print(f'R: target mean {avg(2):.2f} RPM | measured mean {avg(4):.2f} RPM | MAE {mae(2,4):.2f} | RMSE {rmse(2,4):.2f}')
        print(f'Left-right actual RPM mean difference: {avg(3)-avg(4):.2f} RPM')
        print('Note: equal wheel RPM does NOT guarantee a straight floor trajectory when wheel slip is present.')

if __name__ == '__main__':
    main()
