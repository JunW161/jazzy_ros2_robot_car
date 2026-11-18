from __future__ import annotations
import argparse
import math
import rclpy
from rclpy.node import Node
from delivery_robot_interfaces.srv import StartDelivery


def fill(pose, node, x, y, yaw_deg):
    pose.header.frame_id = 'map'; pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x; pose.pose.position.y = y
    yaw = math.radians(yaw_deg); pose.pose.orientation.z = math.sin(yaw / 2); pose.pose.orientation.w = math.cos(yaw / 2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pickup-name', default='Pickup'); p.add_argument('--pickup-x', type=float, required=True); p.add_argument('--pickup-y', type=float, required=True); p.add_argument('--pickup-yaw', type=float, default=0.0)
    p.add_argument('--dropoff-name', default='Dropoff'); p.add_argument('--dropoff-x', type=float, required=True); p.add_argument('--dropoff-y', type=float, required=True); p.add_argument('--dropoff-yaw', type=float, default=0.0)
    p.add_argument('--auto-confirm-pickup', action='store_true'); p.add_argument('--auto-confirm-dropoff', action='store_true')
    args = p.parse_args(); rclpy.init(); node = Node('delivery_robot_mission_cli')
    client = node.create_client(StartDelivery, '/delivery_robot/mission/start')
    if not client.wait_for_service(timeout_sec=5.0): raise SystemExit('Mission service unavailable')
    req = StartDelivery.Request(); req.pickup_name = args.pickup_name; req.dropoff_name = args.dropoff_name
    fill(req.pickup_pose, node, args.pickup_x, args.pickup_y, args.pickup_yaw)
    fill(req.dropoff_pose, node, args.dropoff_x, args.dropoff_y, args.dropoff_yaw)
    req.require_pickup_confirmation = not args.auto_confirm_pickup
    req.require_dropoff_confirmation = not args.auto_confirm_dropoff
    future = client.call_async(req); rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done(): raise SystemExit('Mission service timeout')
    resp = future.result(); print(f'accepted={resp.accepted} mission_id={resp.mission_id} message={resp.message}')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
