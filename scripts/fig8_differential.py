import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import math
import numpy as np
from scipy.misc import derivative

# Robot parameters
R = 0.2
L = 1

# Location Functions to form a figure 8 of the necessary size
def x_t(t, scale=1.0):
    return 10*math.sin(t/scale)

def y_t(t, scale=1.0):
    return 10*math.sin(t/scale)*math.cos(t/scale)

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
def publishWheelVelocity(pub, phi1, phi2):
    layout = MultiArrayLayout()
    layout.dim.insert(0, [MultiArrayDimension()] )

    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 2
    data.layout.dim[0].stride = 1
    data.data = [phi1, phi2]
    pub.publish(data)


if __name__ == '__main__':
        pub = rospy.Publisher('robot0/wheel_velocities', Float64MultiArray, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        
        # Factor to scale down speed by
        speedScale = 2

        # Start time at pi because that's the function lines up
        # with the robot starting location I've set
        time = math.pi

        # Start time at multiple of pi because the location
        # function will divide the scale back out
        time *= speedScale

        # Tick time at 10 hz
        dt = 0.1
        rate = rospy.Rate(1/dt)

        while not rospy.is_shutdown():
            # Calculate wheel velocities for current time
            phi1, phi2 = DD_IK(x_t, y_t, time, speedScale)

            print(phi1, phi2)

            # Increment time
            time += dt

            # Publish velocities
            publishWheelVelocity(pub, phi1, phi2)
            rate.sleep()
