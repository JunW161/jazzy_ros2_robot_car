#!/usr/bin/env python3
"""Measure V14 covariance growth while the robot remains stationary."""
import argparse, statistics, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Monitor(Node):
    def __init__(self):
        super().__init__('delivery_fusion_v14_stationary_covariance')
        self.samples=[]
        self.create_subscription(Odometry,'/odometry/filtered_v14',self.cb,50)
    def cb(self,m):
        self.samples.append((time.monotonic(), m.pose.covariance[7], m.pose.covariance[35],
                             m.twist.covariance[7], m.twist.covariance[35],
                             m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))

def slope_per_min(samples, idx):
    if len(samples)<2: return float('nan')
    dt=samples[-1][0]-samples[0][0]
    return (samples[-1][idx]-samples[0][idx])/dt*60.0 if dt>0 else float('nan')

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--duration',type=float,default=60.0)
    args=ap.parse_args(); rclpy.init(); n=Monitor()
    print(f'Keep robot physically stationary for {args.duration:.0f}s. Measuring /odometry/filtered_v14...')
    end=time.monotonic()+args.duration
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=0.1)
    s=n.samples
    if not s:
        print('NO DATA: start ekf_compare.launch.py first.'); n.destroy_node(); rclpy.shutdown(); return
    def rng(i): return (s[0][i],s[-1][i],min(x[i] for x in s),max(x[i] for x in s))
    py=rng(1); pyaw=rng(2); tvy=rng(3); twz=rng(4)
    print(f'samples={len(s)}')
    print(f'pose cov y   start={py[0]:.6g} end={py[1]:.6g} range=[{py[2]:.6g},{py[3]:.6g}] slope={slope_per_min(s,1):+.6g}/min')
    print(f'pose cov yaw start={pyaw[0]:.6g} end={pyaw[1]:.6g} range=[{pyaw[2]:.6g},{pyaw[3]:.6g}] slope={slope_per_min(s,2):+.6g}/min')
    print(f'twist cov vy  start={tvy[0]:.6g} end={tvy[1]:.6g} slope={slope_per_min(s,3):+.6g}/min')
    print(f'twist cov wz  start={twz[0]:.6g} end={twz[1]:.6g} slope={slope_per_min(s,4):+.6g}/min')
    vx=[abs(x[5]) for x in s]; vy=[abs(x[6]) for x in s]; wz=[abs(x[7]) for x in s]
    print(f'max |state velocity| during test: vx={max(vx):.6f} vy={max(vy):.6f} wz={max(wz):.6f}')
    if max(vx)>0.03 or max(vy)>0.01 or max(wz)>0.05:
        print('REVIEW: robot/filter was not stationary enough for a covariance-at-rest test.')
    else:
        print('STATIONARY SAMPLE QUALITY: OK')
    print('Interpretation: V14 aims to reduce covariance growth at rest, not force covariance to zero.')
    n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
