#!/usr/bin/env python3
import time, statistics
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

class Check(Node):
    def __init__(self):
        super().__init__('check_official_odom_v14')
        self.raw=[]; self.filt=[]
        self.create_subscription(Odometry, '/odom', lambda m:self.raw.append(self.age(m)), 20)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m:self.filt.append(self.age(m)), 50)
        self.buf=Buffer(); self.listener=TransformListener(self.buf,self)
    def age(self,m):
        return self.get_clock().now().nanoseconds/1e9 - (m.header.stamp.sec + m.header.stamp.nanosec*1e-9)

def main():
    rclpy.init(); n=Check(); end=time.monotonic()+8
    while time.monotonic()<end: rclpy.spin_once(n, timeout_sec=.1)
    ok=True
    for name,v,minn in [('raw /odom',n.raw,20),('filtered',n.filt,50)]:
        if not v: print(f'{name}: NO DATA'); ok=False
        else: print(f'{name}: {len(v)} samples age avg={statistics.mean(v)*1000:+.1f}ms min={min(v)*1000:+.1f} max={max(v)*1000:+.1f}')
    try:
        t=n.buf.lookup_transform('odom','base_link',rclpy.time.Time())
        print('TF odom -> base_link: OK')
    except Exception as e:
        print('TF odom -> base_link: FAIL:',e); ok=False
    print('OFFICIAL ODOM CHECK:', 'OK' if ok else 'FAIL')
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
