import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import sys

def joystick_callback(msg):
    print('Joystick Message: [%s]' % msg)

def main():
    args = sys.argv

    if len(args) != 2:
        print("Usage: joystick_listener {topic}")
        return

    channel = str(args[1])
    print("Listening on topic \'" + channel + "\'")

    rclpy.init(args=args)

    node = Node("joystick_snooper")
    node.create_subscription(Joy, channel, joystick_callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()