#!/usr/bin/env python3
"""Write synchronized-ish ROS 2 heading PID + wheel telemetry to CSV, no motor commands.
For quantitative timing, compare per-sample host reception; ROS Float32MultiArray
has no header/timestamp, so this does NOT provide sensor-synchronized timing.
"""
import argparse,csv,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
H=['yaw_deg','target_deg','error_deg','gyro_z_radps','cmd_w_radps',
   'w_effective_radps','correction_radps','mode','imu_fresh','imu_age_ms',
   'pid_enabled','pid_active','kp','ki','kd','p_radps','i_radps','d_radps',
   'saturated','threshold_deg','gain_status','gain_seq',
   'imu_initialized','imu_cal_progress','imu_errors','imu_seq',
   'gyro_raw_z_counts','gyro_bias_z_counts','encoder_yaw_rate_radps',
   'heading_guard_reason','steering_authority','yaw_delta_rpm_RminusL',
   'imu_whoami','imu_z_sign','heading_slew_limited',
   'imu_cal_reject_reason','imu_accel_norm_mps2','imu_cal_restart_count']
W=['target_rpm_l','target_rpm_r','measured_rpm_l','measured_rpm_r',
   'pwm_l','pwm_r','distance_l_m','distance_r_m','encoder_yaw_deg','motor_state']
class Logger(Node):
    def __init__(self,writer):
        super().__init__('heading_pid_csv_logger')
        self.writer=writer;self.wheel=None;self.wt=0.;self.count=0
        self.create_subscription(Float32MultiArray,'/robot/wheel_debug',self.on_wheel,qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray,'/robot/heading_status',self.on_heading,qos_profile_sensor_data)
    def on_wheel(self,m):
        if len(m.data)==len(W): self.wheel=list(m.data);self.wt=time.monotonic()
    def on_heading(self,m):
        if len(m.data)<22:return
        now=time.monotonic()
        age=now-self.wt if self.wheel is not None else -1
        heading=list(m.data[:len(H)])+['']*max(0,len(H)-len(m.data))
        self.writer.writerow([time.time(),now,age]+heading+
                             (self.wheel if self.wheel is not None else ['']*len(W)))
        self.count+=1

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--duration',type=float,default=30.0)
    ap.add_argument('--csv',type=Path,required=True)
    args=ap.parse_args()
    if not 1<=args.duration<=600:ap.error('duration 1..600 s')
    args.csv=args.csv.expanduser()
    args.csv.parent.mkdir(parents=True,exist_ok=True)
    if args.csv.exists():ap.error('CSV exists; choose a new filename to avoid overwriting')
    with args.csv.open('x',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['host_wall_s','host_monotonic_s','wheel_sample_age_s']+H+W)
        rclpy.init();node=Logger(writer);start=time.monotonic()
        try:
            while time.monotonic()-start<args.duration:
                rclpy.spin_once(node,timeout_sec=0.05)
        except KeyboardInterrupt:pass
        finally:
            print('Saved',node.count,'heading messages to',args.csv,flush=True)
            node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
