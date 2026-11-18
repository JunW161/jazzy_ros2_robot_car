#!/usr/bin/env python3
"""Read-only robot diagnostic. Never sends motor commands.
Requires ROS 2 Jazzy and a single micro-ROS Agent.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry

REASONS = {
    0:'NONE', 1:'WAIT_CMD', 2:'CMD_EXPIRED', 3:'CMD_INVALID',
    4:'IMU_UNAVAILABLE', 5:'FRONT_UNKNOWN_NO_LIDAR', 6:'FRONT_NEAR_STOP',
    7:'MOTOR_FAULT', 8:'CMD_LIMIT', 9:'OUTPUT_DISABLED',
    10:'BENCH_EXPIRED', 11:'START_ENCODER_TIMEOUT',
    12:'ENCODER_STALL', 13:'CONTROL_OVERRUN',
}
STATES = ['DISARMED','IDLE','LAUNCH','ACTIVE','FAULT','DECEL']

class Diagnostic(Node):
    def __init__(self):
        super().__init__('robot_v8_diagnostic')
        self.msgs = {}
        self.last = {}
        for typ, topic in [(Float32MultiArray,'/robot/safety_status'),
                           (Float32MultiArray,'/robot/wheel_debug'),
                           (Float32MultiArray,'/robot/link_debug'),
                           (Imu,'/imu/data_raw'),(Range,'/front_range'),
                           (Odometry,'/odom')]:
            self.create_subscription(typ,topic,lambda m,t=topic:self.recv(t,m),qos_profile_sensor_data)
    def recv(self,topic,msg):
        self.msgs[topic]=msg
        self.last[topic]=time.monotonic()
    def summary(self):
        now=time.monotonic()
        for topic in ('/robot/safety_status','/robot/wheel_debug','/robot/link_debug','/imu/data_raw','/front_range','/odom'):
            age=now-self.last.get(topic,now)
            m=self.msgs.get(topic)
            if m is None:
                print(f'{topic}: NO_DATA (publisher absent, wrong QoS, or not yet publishing)')
                continue
            if topic=='/robot/safety_status':
                a=list(m.data)
                if len(a)<12:
                    print(f'{topic}: {len(a)} fields; EXPECT V8 12 (old firmware/build?)')
                else:
                    reason=int(a[8]);state=int(a[5]);
                    name=STATES[state] if 0<=state<len(STATES) else f'UNKNOWN({state})'
                    print(f'{topic}: state={name} reason={REASONS.get(reason,reason)} '
                          f'front_m={a[1]:.3f} speed_cap_mps={a[2]:.3f} '
                          f'gyroZ={a[4]:+.4f} yaw_corr={a[3]:+.4f} '
                          f'bench_arm={int(a[9])} bench_left_ms={a[10]:.0f} '
                          f'cmd_fresh={int(a[11])} age={age:.2f}s')
            elif topic=='/robot/wheel_debug':
                a=list(m.data)
                if len(a)>=10:
                    print(f'{topic}: target_rpm={a[0]:+.1f}/{a[1]:+.1f} '
                          f'measured={a[2]:+.1f}/{a[3]:+.1f} '
                          f'pwm={a[4]:+.0f}/{a[5]:+.0f} state={int(a[9])} age={age:.2f}s')
            elif topic=='/robot/link_debug':
                a=list(m.data)
                if len(a)>=6:
                    print(f'{topic}: cmd_age_ms={a[0]:.0f} watchdog_stops={a[2]:.0f} '
                          f'bad_cmds={a[3]:.0f} control_overruns={a[4]:.0f} age={age:.2f}s')
            elif topic=='/imu/data_raw':
                print(f'{topic}: gyro_z={m.angular_velocity.z:+.5f} rad/s age={age:.2f}s')
            elif topic=='/front_range':
                print(f'{topic}: range={m.range:.3f}m age={age:.2f}s')
            elif topic=='/odom':
                print(f'{topic}: linear={m.twist.twist.linear.x:+.3f} m/s '
                      f'angular={m.twist.twist.angular.z:+.3f} rad/s age={age:.2f}s')

def main():
    pa=argparse.ArgumentParser()
    pa.add_argument('--duration',type=float,default=5.0)
    a=pa.parse_args()
    if not 0.5<=a.duration<=120:pa.error('0.5 <= duration <= 120 seconds')
    rclpy.init();node=Diagnostic()
    try:
        end=time.monotonic()+a.duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.1)
        node.summary()
    finally:
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
