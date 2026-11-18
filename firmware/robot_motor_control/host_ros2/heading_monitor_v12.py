#!/usr/bin/env python3
"""Read-only combined heading PID and wheel debug (does NOT command motors).
Run: /usr/bin/python3 host_ros2/heading_pid_monitor.py --duration 30
"""
import argparse,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

class Monitor(Node):
    def __init__(self):
        super().__init__('delivery_heading_pid_monitor')
        self.h=None;self.w=None;self.ht=0;self.wt=0
        self.create_subscription(Float32MultiArray,'/robot/heading_status',self.heading,qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray,'/robot/wheel_debug',self.wheels,qos_profile_sensor_data)
    def heading(self,msg):
        if len(msg.data)>=22:
            self.h=list(msg.data);self.ht=time.monotonic()
    def wheels(self,msg):
        if len(msg.data)>=10:
            self.w=list(msg.data);self.wt=time.monotonic()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--duration',type=float,default=30)
    args=ap.parse_args()
    if not 1<=args.duration<=600:ap.error('duration 1..600 seconds')
    rclpy.init();n=Monitor();t0=time.monotonic();nextprint=t0
    try:
        while time.monotonic()-t0 < args.duration:
            rclpy.spin_once(n,timeout_sec=0.03)
            now=time.monotonic()
            if now<nextprint:continue
            if n.h is None or now-n.ht>1:
                print('HEADING: NO DATA (check REP103 heading PID firmware and best_effort QoS)')
            else:
                h=n.h
                print(f'heading enabled={int(h[10])} active={int(h[11])} mode={int(h[7])} '
                      f'yaw={h[0]:+.2f}deg target={h[1]:+.2f}deg error={h[2]:+.2f}deg '
                      f'gyro={h[3]:+.4f} cmd={h[4]:+.3f} w_eff={h[5]:+.3f}')
                if len(h)>=38:
                    print(f'IMU initialized={int(h[22])} WHO=0x{int(h[32]):02X} '
                          f'calibration={int(h[23])}/150 seq={int(h[25])} '
                          f'errors={int(h[24])} age={h[9]:.1f}ms fresh={int(h[8])} '
                          f'rawZ={h[26]:+.1f} biasZ={h[27]:+.1f} '
                          f'accel_norm={h[36]:.2f} cal_reject={int(h[35])} '
                          f'cal_restarts={int(h[37])}')
                    print(f'feedback encoder_w={h[28]:+.4f}rad/s '
                          f'guard={int(h[29])} steer_owner={int(h[30])} '
                          f'delta_RminusL={h[31]:+.2f}RPM slew_limited={int(h[34])}')
                    if not h[22] or h[23]<150:
                        print('IMU NOT READY: keep robot still; check WHO/I2C if progress stays 0')
                    elif not h[8]:
                        print('IMU STALE: check sensor timestamp/seq/I2C, do NOT tune Kp')
                    if len(h)>=42:
                        print(f'V12 enable received={int(h[38])} applied={int(h[39])} '
                              f'guard_resets={int(h[40])} apply_status={int(h[41])} '
                              f'trigger={h[19]:.2f}deg')
                    if h[29]:
                        print('HEADING GUARD BLOCKED: stop robot, verify yaw sign/IMU, then OFF at IDLE -> ON')
                print(f'PID kp={h[12]:.3f} ki={h[13]:.3f} kd={h[14]:.3f} '
                      f'P={h[15]:+.4f} I={h[16]:+.4f} D={h[17]:+.4f} '
                      f'corr={h[6]:+.4f} saturated={int(h[18])} '
                      f'gain_status={int(h[20])} gain_seq={int(h[21])}')
            if n.w is None or now-n.wt>1:
                print('WHEELS: NO DATA')
            else:
                w=n.w
                print(f'wheels targetL/R={w[0]:+.1f}/{w[1]:+.1f} RPM '
                      f'measuredL/R={w[2]:+.1f}/{w[3]:+.1f} RPM '
                      f'PWM_L/R={w[4]:+.0f}/{w[5]:+.0f} motor_state={int(w[9])}')
            print('---',flush=True)
            nextprint=now+0.25
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
