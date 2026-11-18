#!/usr/bin/env python3
"""Read-only local twist reference monitor, no motor command issued."""
import argparse,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

class Monitor(Node):
    def __init__(self):
        super().__init__('rep103_local_trajectory_monitor')
        self.last=None
        self.create_subscription(Float32MultiArray,'/robot/trajectory_status',self.handle,qos_profile_sensor_data)
    def handle(self,msg):
        d=msg.data
        if len(d)!=14:
            self.get_logger().warn(f'Expected 14 floats, got {len(d)}')
            return
        self.last=time.monotonic()
        print(f'ref_xy=({d[0]:+.3f},{d[1]:+.3f})m '
              f'est_xy=({d[2]:+.3f},{d[3]:+.3f})m '
              f'cte={d[4]:+.3f}m along={d[5]:+.3f}m '
              f'kappa={d[6]:+.3f}/m R={d[7]:+.3f}m '
              f'yaw={d[8]:+.2f}deg tgt={d[9]:+.2f}deg err={d[10]:+.2f}deg '
              f'path_corr={d[11]:+.4f}rad/s total_corr={d[12]:+.4f}rad/s '
              f'mode={int(d[13])}',flush=True)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--duration',type=float,default=30)
    args=ap.parse_args()
    rclpy.init();node=Monitor();end=time.monotonic()+args.duration
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.25)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
