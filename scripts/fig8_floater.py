import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import math

# Robot parameters
R = 0.2
L = 0.1

# Location Functions to form a figure 8 of the necessary size
def x_t(t, scale=1.0):
    return 100*math.sin(t/scale)

def y_t(t, scale=1.0):
    return 100*math.cos(t*2/scale)

# Publishes a set of wheel velocities
# in the format required by the STDR
def publishWheelVelocity(pub, xdot, ydot, thetadot):
    layout = MultiArrayLayout()
    layout.dim.insert(0, [MultiArrayDimension()] )

    data = Float64MultiArray(data=[])
    data.layout = MultiArrayLayout()
    data.layout.dim = [MultiArrayDimension()]
    data.layout.dim[0].label = "Parameters"
    data.layout.dim[0].size = 3
    data.layout.dim[0].stride = 1
    data.data = [xdot, ydot, thetadot]
    pub.publish(data)


if __name__ == '__main__':
        pub = rospy.Publisher('robot0/world_velocity', Float64MultiArray, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        
        # Factor to scale down speed by
        speedScale = 1

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
            xdot = x_t(time)
            ydot = y_t(time)

            print(xdot, ydot)

            # Increment time
            time += dt

            # Publish velocities
            publishWheelVelocity(pub, xdot, ydot, 0)
            rate.sleep()
