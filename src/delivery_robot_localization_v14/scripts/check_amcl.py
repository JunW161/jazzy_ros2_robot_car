#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

class Check(Node):
    def __init__(self):
        super().__init__('check_amcl_v14')
        self.map=0; self.scan=0; self.pose=0
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',self.map+1),10)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',self.scan+1),10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'pose',self.pose+1),10)
        self.buf=Buffer(); self.listener=TransformListener(self.buf,self)

def main():
    rclpy.init(); n=Check(); end=time.monotonic()+10
    while time.monotonic()<end: rclpy.spin_once(n, timeout_sec=.1)
    print(f'/map={n.map} /scan={n.scan} /amcl_pose={n.pose}')
    ok=n.map>0 and n.scan>0
    for target,source in [('odom','base_link'),('map','odom')]:
        try:
            n.buf.lookup_transform(target,source,rclpy.time.Time())
            print(f'TF {target} -> {source}: OK')
        except Exception as e:
            print(f'TF {target} -> {source}: FAIL: {e}'); ok=False
    if n.pose==0:
        print('AMCL pose not received yet: set initial pose in RViz or run set_initial_pose.py.'); ok=False
    print('AMCL CHECK:', 'OK' if ok else 'NOT READY')
    n.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
