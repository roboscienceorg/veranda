import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import math
import numpy as np
from scipy.misc import derivative

# Robot parameters
R = 0.75
L = 1.5

# Location Functions to form a figure 8 of the necessary size
def x_t(t, scale=1.0):
    return 12*math.sin(t/scale)

def y_t(t, scale=1.0):
    return 12*math.sin(t/scale)*math.cos(t/scale)

# Differential drive inverse kinematics
def DD_IK(x_t, y_t, t, scale):
    # Calculate xdot and xdotdot at current time
    x_dot_t = derivative(x_t, t, dx=0.001, order=7, args=(scale,))
    x_dotdot_t = derivative(x_t, t, n=2, dx=0.001, order=7, args=(scale,))

    # Calculate ydot and ydot dot at current time
    y_dot_t = derivative(y_t, t, dx=0.001, order=7, args=(scale,))
    y_dotdot_t = derivative(y_t, t, n=2, dx=0.001, order=7, args=(scale,))

    #Calculate phi1, phi2
    v = math.sqrt(x_dot_t * x_dot_t + y_dot_t * y_dot_t)
    k = (x_dot_t * y_dotdot_t - y_dot_t * x_dotdot_t)/(v*v*v)

    phi1 = v/R*(k*L+1)
    phi2 = v/R*(-k*L + 1)

    return (phi1, phi2)


# Publishes a set of wheel velocities
# in the format required by the STDR
def publishWheelVelocity(publeft, pubright, phi1, phi2):
    msg = Float32()

    msg.data = phi1
    publeft.publish(msg)

    msg.data = phi2
    pubright.publish(msg)


def main():
    rclpy.init()
    node = Node("talker")

    publeft = node.create_publisher(Float32, 'robot0/left_wheel')
    pubright = node.create_publisher(Float32, 'robot0/right_wheel')
    
    # Factor to scale down speed by
    speedScale = 2

    # Start time at pi because that's the function lines up
    # with the robot starting location I've set
    t = math.pi

    # Start time at multiple of pi because the location
    # function will divide the scale back out
    t *= speedScale

    # Tick time at 10 hz
    dt = 0.1

    def cb():
        nonlocal t

        # Calculate wheel velocities for current time
        phi1, phi2 = DD_IK(x_t, y_t, t, speedScale)

        print(phi1, phi2)

        # Increment time
        t += dt

        # Publish velocities
        publishWheelVelocity(publeft, pubright, phi1, phi2)

    timer = node.create_timer(dt, cb)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()