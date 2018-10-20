import rclpy
from rclpy.node import Node

from veranda.SimTimer import SimTimer

from geometry_msgs.msg import Pose2D

import math
import numpy as np

# Publishes target velocities for the robot to travel at
def publishTargetVelocity(pub, dx, dy, dt):
    msg = Pose2D()

    msg.x = dx
    msg.y = dy
    msg.theta = dt

    pub.publish(msg)


def main():
    rclpy.init()
    node = Node("talker")

    pub = node.create_publisher(Pose2D, 'robot0/target_velocity')

    simTime = SimTimer(True, "veranda/timestamp", node)
    
    # Factor to scale down speed by
    speedScale = 10

    # Tick time at 10 hz
    dt = 0.1

    def cb():
        # Calculate wheel velocities for current time
        x, y = speedScale*math.sin(simTime.global_time()), speedScale*2*math.cos(2*simTime.global_time())
        t = 0.

        print(x, y)

        # Publish velocities
        publishTargetVelocity(pub, x, y, t)

    simTime.create_timer(dt, cb)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
