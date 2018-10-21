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
        print("Wheels will be {topic-out}/back_wheel, {topic-out}/right_wheel, {topic-out}/left_wheel")
        print("If no {topic-out} given, {topic-in} will be used for both")
        return

    channelin = channelout = args[1]
    if len(args) == 3:
        channelout = args[2]

    channelin = channelin + "/joystick"

    rclpy.init(args=args)

    node = Node("joystick_differential")
    pubback = node.create_publisher(Float32, channelout + "/back_wheel")
    publeft = node.create_publisher(Float32, channelout + "/left_wheel")
    pubright = node.create_publisher(Float32, channelout + "/right_wheel")
    channels = (pubback, pubright, publeft)

    def joystick_callback(msg):
        if len(msg.axes) < 2:
            print("Not enough joystick axes")
            return
        
        y_ax = msg.axes[1]
        x_ax = msg.axes[0]
        z_ax = msg.axes[2]
        print(x_ax, y_ax, z_ax)

        phil = SPEED * (y_ax + max(0, x_ax) - z_ax)
        phir = SPEED * (y_ax + -min(0, x_ax) + z_ax)
        phib = SPEED * (x_ax + z_ax)
        
        publishWheelVelocity(channels, (phib, phir, phil))

    node.create_subscription(Joy, channelin, joystick_callback)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()