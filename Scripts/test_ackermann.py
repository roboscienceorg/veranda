import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

import math

import sys

R = 0.75
L = 1.5

def publishWheelSteer(pub, phi):
    msg = Float32()

    msg.data = phi
    pub.publish(msg)

def main():
    args = sys.argv

    if len(args) != 2:
        print("Usage: test_ackermann {robot}")
        return

    channel = args[1] + "/joystick"

    rclpy.init(args=args)

    node = Node("test_ackermann")
    pubsteer = node.create_publisher(Float32, args[1] + "/steer")

    phi = 0
    def cb():
        nonlocal phi

        phi = phi + math.pi/50
        
        steerTo = phi % math.pi - math.pi/2.0
        publishWheelSteer(pubsteer, steerTo)
        print(steerTo)

    timer = node.create_timer(0.5, cb)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()