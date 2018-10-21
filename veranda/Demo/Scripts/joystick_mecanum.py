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
def publishWheelVelocity(publishers, speeds):
    if len(publishers) != len(speeds):
        print("Different number of publishers and speeds")
        return

    for i in range(len(publishers)):
        msg = Float32()
        msg.data = speeds[i]
        publishers[i].publish(msg)

def main():
    args = sys.argv

    if len(args) != 2 and len(args) != 3:
        print("Usage: joystick_differential {topic-in} [topic-out]")
        print("Joystick should be {topic-in}/joystick")
        print("Wheels will be {topic-out}/front_left_wheel, {topic-out}/front_right_wheel, {topic-out}/back_left_wheel, and {topic-out}/back_right_wheel")
        print("If no {topic-out} given, {topic-in} will be used for both")
        return

    channelin = channelout = args[1]
    if len(args) == 3:
        channelout = args[2]

    channelin = channelin + "/joystick"

    rclpy.init(args=args)

    node = Node("joystick_differential")
    pubfrontleft = node.create_publisher(Float32, channelout + "/front_left_wheel")
    pubfrontright = node.create_publisher(Float32, channelout + "/front_right_wheel")
    pubbackleft = node.create_publisher(Float32, channelout + "/back_left_wheel")
    pubbackright = node.create_publisher(Float32, channelout + "/back_right_wheel")
    channels = (pubfrontleft, pubfrontright, pubbackleft, pubbackright)

    def joystick_callback(msg):
        if len(msg.axes) < 2:
            print("Not enough joystick axes")
            return
        
        y_ax = msg.axes[1]
        x_ax = msg.axes[0]
        z_ax = msg.axes[2]
        print(x_ax, y_ax, z_ax)

        phifl = SPEED * (y_ax + x_ax + z_ax)
        phifr = SPEED * (y_ax - x_ax - z_ax)
        phibl = SPEED * (y_ax - x_ax + z_ax)
        phibr = SPEED * (y_ax + x_ax - z_ax)
        
        publishWheelVelocity(channels, (phifl, phifr, phibl, phibr))

    node.create_subscription(Joy, channelin, joystick_callback)
    publishWheelVelocity(channels, (1., 1., 1., 1.))

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()