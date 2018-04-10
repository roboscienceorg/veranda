import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import sys

def lidar_callback(msg):
    print('Lidar Message: [%s]' % msg)

def main():
    args = sys.argv

    if len(args) != 2:
        print("Usage: lidar_listener channel")
        return

    channel = str(args[1])
    print("Listening on channel \'" + channel + "\'")

    rclpy.init(args=args)

    node = Node("lidar_snooper")
    node.create_subscription(LaserScan, channel, lidar_callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()