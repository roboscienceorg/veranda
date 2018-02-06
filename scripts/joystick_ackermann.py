import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

import math

import sys

R = 0.75
L = 1.5
SPEED = 5

# Publishes a set of wheel velocities
# in the format required by the STDR
def publishWheelVelocity(publeft, pubright, phi1, phi2):
    msg = Float32()

    msg.data = phi1
    publeft.publish(msg)

    msg.data = phi2
    pubright.publish(msg)

def publishWheelSteer(pub, phi):
    msg = Float32()

    msg.data = phi
    pub.publish(msg)

def main():
    args = sys.argv

    if len(args) != 2:
        print("Usage: joystick_differential {robot}")
        print("Joystick should be {robot}/joystick")
        print("Wheels will be {robot}/left_wheel and {robot}/right_wheel")
        return

    channel = args[1] + "/joystick"

    rclpy.init(args=args)

    node = Node("joystick_differential")
    publeft = node.create_publisher(Float32, args[1] + "/left_wheel")
    pubright = node.create_publisher(Float32, args[1] + "/right_wheel")
    pubsteer = node.create_publisher(Float32, args[1] + "/steer")

    def joystick_callback(msg):
        if len(msg.axes) < 2:
            print("Not enough joystick axes")
            return
        
        speed = msg.axes[1]
        steer = msg.axes[0]

        phi1 = phi2 = 1/R*speed*SPEED
        phi3 = math.pi/2.0 * steer

        #publishWheelVelocity(publeft, pubright, phi1, phi2)
        publishWheelSteer(pubsteer, phi3)

    node.create_subscription(Joy, channel, joystick_callback)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()