import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from delivery_robot_interfaces.msg import MissionState


def main():
    rclpy.init(); node = Node('delivery_robot_mission_status')
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    def cb(m):
        print(f'{m.state_name}: {m.message} | {m.pickup_name} -> {m.dropoff_name} | remaining={m.distance_remaining:.2f}m error={m.nav_error_code}')
    node.create_subscription(MissionState, '/delivery_robot/mission/state', cb, qos)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
