#!/usr/bin/env python3
"""Robot_Delivery_V10 FF/Scale set/read/save. Firmware enforces IDLE itself."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray

MAPNAMES=['FWD_L','FWD_R','REV_L','REV_R']
RPM=[20,30,40,50,60,70,80,90,100]
class Client(Node):
    def __init__(self):
        super().__init__('ff_tune_v10_client')
        self.status=None
        self.wheel=None
        self.ffpoint=self.create_publisher(Float32MultiArray,'/robot/ff_set',10)
        self.ffscale=self.create_publisher(Float32MultiArray,'/robot/ff_scale_set',10)
        self.ffsave=self.create_publisher(Bool,'/robot/ff_save',10)
        self.ffreset=self.create_publisher(Bool,'/robot/ff_reset',10)
        self.create_subscription(Float32MultiArray,'/robot/ff_status',self.set_status,qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray,'/robot/wheel_debug',self.set_wheel,qos_profile_sensor_data)
    def set_status(self,msg):
        if len(msg.data)>=43:self.status=list(msg.data)
    def set_wheel(self,msg):
        if len(msg.data)>=10:self.wheel=list(msg.data)
def spin_until(node,cond,seconds=6):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=0.10)
        if cond():return True
    return False
def show(data):
    print(f'FF result={data[0]:.0f} seq={data[1]:.0f} NVS_saved={bool(data[2])}')
    for m in range(4):
        print(MAPNAMES[m],'scale=',f'{data[3+m]:.3f}', 'RPM→PWM:', ', '.join(f'{rpm}:{pwm:.1f}' for rpm,pwm in zip(RPM,data[7+m*9:16+m*9])))
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    sub=ap.add_subparsers(dest='command',required=True)
    sub.add_parser('show')
    p=sub.add_parser('point');p.add_argument('wheel',choices=['L','R']);p.add_argument('direction',choices=['F','B']);p.add_argument('rpm',type=float);p.add_argument('pwm',type=float)
    p=sub.add_parser('scale');p.add_argument('wheel',choices=['L','R']);p.add_argument('direction',choices=['F','B']);p.add_argument('value',type=float)
    sub.add_parser('save');sub.add_parser('reset')
    a=ap.parse_args()
    rclpy.init();n=Client()
    try:
        if not spin_until(n,lambda:n.status is not None):
            raise SystemExit('Không nhận /robot/ff_status — kiểm tra Agent/firmware V10 và QoS best_effort')
        if a.command=='show':show(n.status);return
        if not spin_until(n,lambda:n.wheel is not None):
            raise SystemExit('Không nhận /robot/wheel_debug; không gửi yêu cầu hiệu chỉnh')
        w=n.wheel
        if w[9]!=1 or any(abs(w[i])>=2 for i in (2,3)) or any(abs(w[i])>0 for i in (4,5)):
            raise SystemExit('Yêu cầu IDLE, RPM <2 và PWM =0. Dừng robot trước khi chỉnh')
        before=n.status[1]
        msg=None
        if a.command=='point':
            msg=Float32MultiArray(data=[0.0 if a.wheel=='L' else 1.0,1.0 if a.direction=='F' else -1.0,a.rpm,a.pwm]);publisher=n.ffpoint
        elif a.command=='scale':
            msg=Float32MultiArray(data=[0.0 if a.wheel=='L' else 1.0,1.0 if a.direction=='F' else -1.0,a.value]);publisher=n.ffscale
        else:
            msg=Bool(data=True);publisher=n.ffsave if a.command=='save' else n.ffreset
        if publisher.get_subscription_count()==0:
            raise SystemExit('Chưa có FF subscriber trên ESP32 V10')
        publisher.publish(msg)
        if not spin_until(n,lambda:n.status is not None and n.status[1]>before,5):
            raise SystemExit('Không có phản hồi FF ACK mới, đừng giả định đã áp dụng')
        show(n.status)
        if n.status[0] <0:raise SystemExit(f'Firmware từ chối yêu cầu: status {n.status[0]:.0f}')
    finally:
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
