from __future__ import annotations
import argparse
import math
import sqlite3
from pathlib import Path
import rclpy
from rclpy.node import Node
from delivery_robot_interfaces.srv import StartDelivery


def main():
    p = argparse.ArgumentParser(description='Start mission using pickup/dropoff stored by the existing Web database.')
    p.add_argument('--db', default='~/.local/share/delivery_robot_web/app.sqlite3')
    p.add_argument('--map-name', required=True); p.add_argument('--pickup', required=True); p.add_argument('--dropoff', required=True)
    p.add_argument('--auto-confirm-pickup', action='store_true'); p.add_argument('--auto-confirm-dropoff', action='store_true')
    args = p.parse_args(); db = sqlite3.connect(str(Path(args.db).expanduser()))
    row = db.execute('SELECT id FROM maps WHERE name=? ORDER BY created DESC LIMIT 1', (args.map_name,)).fetchone()
    if not row: raise SystemExit('Map name not found in Web database')
    def point(name):
        r = db.execute('SELECT name,x,y,yaw FROM points WHERE map_id=? AND name=? LIMIT 1', (row[0], name)).fetchone()
        if not r: raise SystemExit(f'Point not found: {name}')
        return r
    pickup = point(args.pickup); dropoff = point(args.dropoff)
    rclpy.init(); node = Node('mission_from_web_points'); client = node.create_client(StartDelivery, '/delivery_robot/mission/start')
    if not client.wait_for_service(timeout_sec=5.0): raise SystemExit('Mission service unavailable')
    req = StartDelivery.Request(); req.pickup_name = pickup[0]; req.dropoff_name = dropoff[0]
    for pose, data in ((req.pickup_pose, pickup), (req.dropoff_pose, dropoff)):
        pose.header.frame_id = 'map'; pose.header.stamp = node.get_clock().now().to_msg(); pose.pose.position.x = data[1]; pose.pose.position.y = data[2]
        pose.pose.orientation.z = math.sin(data[3]/2); pose.pose.orientation.w = math.cos(data[3]/2)
    req.require_pickup_confirmation = not args.auto_confirm_pickup; req.require_dropoff_confirmation = not args.auto_confirm_dropoff
    future = client.call_async(req); rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    resp = future.result(); print(f'accepted={resp.accepted} mission_id={resp.mission_id} message={resp.message}')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
