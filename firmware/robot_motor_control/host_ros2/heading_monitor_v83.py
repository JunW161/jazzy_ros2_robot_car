#!/usr/bin/env python3
"""Read-only MPU yaw observer monitor for V8.3. No /cmd_vel published.

ROS2 usage: /usr/bin/python3 host_ros2/heading_monitor_v83.py --duration 30
"""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

KEYS=("yaw_deg", "target_deg", "error_deg", "gyro_z_radps", "cmd_w_radps",
      "effective_w_radps", "correction_radps", "mode", "imu_fresh", "imu_age_ms")
MODES={0: "IDLE", 1: "HEADING_HOLD", 2: "RATE_AND_HEADING", 3: "IMU_FALLBACK"}

class Monitor(Node):
    def __init__(self):
        super().__init__("delivery_heading_monitor_v83")
        self.last=None
        self.rx=0.0
        self.create_subscription(Float32MultiArray,"/robot/heading_status",self.got,qos_profile_sensor_data)
    def got(self,msg):
        if len(msg.data)<len(KEYS):
            self.get_logger().warning("Unexpected /robot/heading_status length: %d" % len(msg.data))
            return
        self.last={k:float(v) for k,v in zip(KEYS,msg.data)}
        self.rx=time.monotonic()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration",type=float,default=30.0)
    args=p.parse_args()
    if not 1<=args.duration<=600:p.error("--duration must be 1..600 seconds")
    rclpy.init()
    n=Monitor()
    start=time.monotonic();next_print=start
    try:
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(n,timeout_sec=0.05)
            now=time.monotonic()
            if now<next_print:continue
            if n.last is None or now-n.rx>1.0:
                print("/robot/heading_status NO_FRESH_DATA: check ESP32/Agent and publisher count")
            else:
                d=n.last
                print("yaw={yaw_deg:+.2f}deg target={target_deg:+.2f}deg error={error_deg:+.2f}deg "
                      "gyro={gyro_z_radps:+.4f}cmd={cmd_w_radps:+.3f}eff={effective_w_radps:+.3f} "
                      "corr={correction_radps:+.4f} mode={mode} fresh={imu_fresh:.0f} imu_age={imu_age_ms:.1f}ms".format(
                          **{**d,"mode":MODES.get(round(d["mode"]),"UNKNOWN")}) )
            next_print=now+0.5
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
