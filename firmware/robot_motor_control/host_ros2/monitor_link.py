#!/usr/bin/env python3
"""Logs /robot/link_debug: [age_ms, max_cmd_gap_ms, watchdog_stops,
invalid_cmds, control_overruns, motor_state]. Best-effort matching ESP32.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
class LinkMonitor(Node):
    def __init__(self):
        super().__init__('delivery_link_monitor')
        self.create_subscription(Float32MultiArray,'/robot/link_debug',self.on_msg,
          QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.old_stops=None
        self.get_logger().info('Listening on /robot/link_debug (best effort).')
    def on_msg(self,m):
        if len(m.data)!=6:return
        age,gap,stops,bad,overruns,state=m.data
        if self.old_stops is not None and stops>self.old_stops:
            self.get_logger().warning('WATCHDOG STOPS INCREASED: %g -> %g'%(self.old_stops,stops))
        self.old_stops=stops
        self.get_logger().info('cmd_age=%5.0fms max_cmd_gap=%5.0fms stop=%3.0f invalid=%3.0f overrun=%3.0f state=%.0f'%(age,gap,stops,bad,overruns,state))
def main():
    rclpy.init();node=LinkMonitor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
