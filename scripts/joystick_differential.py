import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

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

def main():
    args = sys.argv

    if len(args) != 2 and len(args) != 3:
        print("Usage: joystick_differential {channelin} [channelout]")
        print("Joystick should be {channelin}/joystick")
        print("Wheels will be {channelout}/left_wheel and {channelout}/right_wheel")
        print("If no {channelout} given, {channelin} will be used for both")
        return

    channelin = channelout = args[1]
    if len(args) == 3:
        channelout = args[2]

    channelin = channelin + "/joystick"

    rclpy.init(args=args)

    node = Node("joystick_differential")
    publeft = node.create_publisher(Float32, channelout + "/left_wheel")
    pubright = node.create_publisher(Float32, channelout + "/right_wheel")

    def joystick_callback(msg):
        if len(msg.axes) < 2:
            print("Not enough joystick axes")
            return
        
        speed = msg.axes[1]
        steer = msg.axes[0]
        print(speed, steer)

        phi1 = 1/R * (speed * SPEED + L * steer)
        phi2 = 1/R * (speed * SPEED - L * steer)

        publishWheelVelocity(publeft, pubright, phi1, phi2)

    node.create_subscription(Joy, channel, joystick_callback)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()