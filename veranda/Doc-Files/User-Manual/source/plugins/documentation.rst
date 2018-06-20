Plugins Included with Veranda
=============================

One of the main advantages to the design of this project is that it can be 
extended through WorldObjectComponent Plugins and FileHandler Plugins. Unfortunately, 
because it is a ROS package, the project (and any such plugins) must be built with 
Ament and CMake. A number of plugins have been included with the project to provide 
basic functionalities, and those plugins can be used as examples of how other plugins 
should be set up; however, not all plugins will follow the same patterns, so this 
section will describe the existing plugins as well as some of the requirements to 
build and use a plugin. Each plugin documented here will have a table listing the properties they
expose, and some will also list ROS channels they can communicate on.

Shape Plugins
-------------

Shape plugins create a solid mass of a specific shape.

Rectangle Plugin
^^^^^^^^^^^^^^^^

The Rectangle Plugin creates a rectangular shape. The width and height
can be specified. 

:ref:`Properties Table <tab-rectangle_properties>`

.. _tab-rectangle_properties:

==================   ==============   ============================
Property Name        Data Type	      Description
==================   ==============   ============================
Width                Double           Width of rectangle (meters)	
Heigh		         Double           Height of rectangle (meters)
==================   ==============   ============================

Circle Plugin
^^^^^^^^^^^^^

The Circle Plugin creates a circle shape. The radius
can be specified.

:ref:`Properties Table <tab-circle_properties>`

.. _tab-circle_properties:

==================   ==============   ============================
Property Name        Data Type	      Description
==================   ==============   ============================
Radius               Double           Radius of circle (meters)
==================   ==============   ============================

Polygon Plugin
^^^^^^^^^^^^^^

The Polygon Plugin can be used to create a solid mass in any arbitrary shape, which could 
have holes in it. It is not intended to be added to objects manually, but should be used 
by other plugins to create specifically shaped masses. In the future, it may be 
possible to add a polygon and modify its points, but at the time of this writing, it is 
not possible. Finally, the components created by this plugin are anchored and will not 
move in the physics engine. 

:ref:`Properties Table <tab-polygon_properties>`

.. _tab-polygon_properties:

======================   ==========================   ==================================================
Property Name            Data Type	                  Description
======================   ==========================   ==================================================
polygon_count            Integer                      Read only: Number of triangle produced in triangulations
outer_shape              List of Points               The points that make up the outer loop of the shape
inner_shape              List of Lists of Points      The loops of points making up the holes in the outer shape
straightness             Integer > 0                  Threshold for cross product in the simplification algorithm
scale/horiz              Double                       Horizantal scaling factor
scale/vert               Double                       Vertical scaling factor
======================   ==========================   ==================================================

Design Details 
""""""""""""""

    #. The Box2D polygon type does not work properly with concave polygon and has a limitation on how many points can be in a polygon. To prevent either of these limitations from being a problem, the polygon is triangulated and all of the triangles are loaded as separate shapes. The triangulation algorithm is provided by the OpenGL GLU Tesselator.
    #. Polygons are simplified according to the straightness property setting. This simplification algorithm is called with the straightness value as its cross product threshold.
    #. Triangulation is performed each time the straightness property or one of the scaling properties changes.


Drive Plugins
-------------

Driving plugins produce force to propel a world object. Perhaps they would be better 
named 'propulsion plugins', but they usually emulate wheel-like hardware, so we're 
calling them driving plugins. Box2D does not actually simulate rolling of wheels in 
a top-down environment, so it is all done virtually. Given the size of a wheel and its 
angular velocity, we can calculate its linear velocity, and apply that velocity. 
Similarly, given its linear velocity, we can calculate the portions of that velocity 
in invalid directions and negate it (for example, to prevent a wheel from slipping or 
sliding).

It's important to note that, at least in the built-in plugins, the forces produced are 
proportional to the mass of the wheels. This means that if the body of a vehicle-like 
object is much more massive than the wheels, the wheels will have little effect on the 
object. This can be compensated for by allowing the user to tune the density of wheels 
so that their mass can be tweaked and made closer to the mass of what they are moving.

Fixed Wheel Plugin
^^^^^^^^^^^^^^^^^^

The fixed wheel plugin emulates a single wheel which cannot rotate to face a 
different direction. The wheel may be driven, which means its target angular 
velocity can be set and it will produce force in an attempt to travel the 
correct linear velocity. The wheel will always produce forces to negate any 
movement parallel to its axle (or where the axle would be if one existed). 

:ref:`Properties Table <tab-fixedwheel_properties>`

:ref:`ROS Channels <tab-fixedwheel_channels>`

.. _tab-fixedwheel_properties:

======================   ==============   ========================================================
Property Name            Data Type	      Description
======================   ==============   ========================================================
channels/input\_speed    String           The ROS topic to listen on for target angular velocity
wheel\_radius            Double           Radius of the wheel in meters
wheel\_width             Double           Width of the wheel in meters
is_driven                Bool             Whether or not the wheel should produce driving force based on the input speed
density                  Double           Density of the wheel. Can be tuned to give more or less mass of the object better
======================   ==============   ========================================================


.. _tab-fixedwheel_channels:

=========================   =============================   =========   ========================================================
ROS Topic                   Message Type                    In/Out      Description
=========================   =============================   =========   ========================================================
channels/[input_channel]    std_msgs::msg::Float32          In          Target angular velocity (rad/s) to simulate. This information is ignored if [is_drive] is false.
=========================   =============================   =========   ========================================================

Ackermann Steering Plugin
^^^^^^^^^^^^^^^^^^^^^^^^^

The Ackermann steering plugin produces two linked wheels which can be steered 
together and will follow the Ackermann constraint. The wheels cannot be driven, 
they only produce forces to negate horizontal sliding, which can be used to 
steer a vehicle-like object.

:ref:`Properties Table <tab-ackermann_properties>`

:ref:`ROS Channels <tab-ackermann_channels>`

.. _tab-ackermann_properties:

======================   ==============   ========================================================
Property Name            Data Type	      Description
======================   ==============   ========================================================
channels/input\_angle    String           The ROS topic to listen on for target steering angle
wheel_radius             Double           Radius of the wheel in meters
wheel_width              Double           Width of the wheel in meters
    axle_lengh               Double           Distance between the two wheels (meters)
vehicle length           Double           Distance from the front azle of the object to the back axle to be used in the Ackermann constraint (meters)
density                  Double           Density of wheels. Can be tuned to give wheel more or less mass so it affects the object attached better
steer_angle              Bool             Read Only: the current angle being steered to
======================   ==============   ========================================================


.. _tab-ackermann_channels:

=========================   =============================   =========   ========================================================
ROS Topic                   Message Type                    In/Out      Description
=========================   =============================   =========   ========================================================
channels/input_angle        std_msgs::msg::Float32          In          Angle to steer towards in radians. 
=========================   =============================   =========   ========================================================

Sensor Plugins
--------------

Sensors travel with a world object and collect information about the environment. 
This information is usually published to the control code through ROS messages so 
that the control code can react to what is happening. Sensors plugins should produce 
the same messages as their hardware counterparts so that any control code listening to 
the messages does not behave differently when the messages are produced by hardware.

GPS Receiver
^^^^^^^^^^^^
The GPS Sensor is used to return the current position of an object in the world. It publishes a ROS Pose2d message, which contains
X, Y, and Theta values. The sensor can be tuned to simulate a broken GPS which drops values, has noise, or drifts over time. The noise and drift values follow normal distributions, and the chance to drop values is uniform. When a value is dropped, NaN is published instead of the current coordinate or angle. Theta is published in radians. The properties exposed to set these 
distributions as well as specify the ROS topic can be found in table \ref{tab:gps_props}.
	
:ref:`Properties Table <tab-gps_properties>`

:ref:`ROS Channels <tab-gps_channels>`

.. _tab-gps_properties:

======================   ==============   ========================================================
Property Name            Data Type	      Description
======================   ==============   ========================================================
channels/output\_pose    String           Topic to publish Pose2D messages on\\ \hline
publish\_rate            double           Rate of publishing (hz)\\ \hline
probabilities/x          double           Probability of publishing x coordinate [0.0, 1.0]\\ \hline
probabilities/y          double           Probability of publishing y coordinate [0.0, 1.0]\\ \hline
probabilities/theta      double           Probability of publishing theta [0.0, 1.0]\\ \hline
drift/x/sigma            double           Variance of drift in x direction per second\\ \hline
drift/x/mu               double           Mean of drift in x direction per second\\ \hline
drift/y/sigma            double           Variance of drift in y direction per second\\ \hline
drift/y/mu               double           Mean of drift in y direction per second\\ \hline
drift/theta/sigma        double           Variance of drift in theta direction per second\\ \hline
drift/theta/mu           double           Mean of drift in theta direction per second\\ \hline
noise/x/sigma            double           Variance of noise in x direction\\ \hline
noise/x/mu               double           Mean of noise in x direction\\ \hline
noise/y/sigma            double           Variance of noise in y direction\\ \hline
noise/y/mu               double           Mean of noise in y direction\\ \hline
noise/theta/sigma        double           Variance of noise in theta\\ \hline
noise/theta/mu           double           Mean of noise in theta
======================   ==============   ========================================================


.. _tab-gps_channels:

=========================   =============================   =========   ========================================================
ROS Topic                   Message Type                    In/Out      Description
=========================   =============================   =========   ========================================================
channels/output\_channel    geometry\_msgs::msg::Pose2D     Out         The 2D Pose message containing x, y, and theta
=========================   =============================   =========   ========================================================

Design Details
""""""""""""""

    #. Drift accumulates over time even on ticks when the value is not published
    #. NaN is produced using :code:`std::numeric_limits<double>::quiet_NaN()`

Touch Sensor Ring
^^^^^^^^^^^^^^^^^

The touch sensor ring plugin mimics a ring of touch sensors with a specific 
radius. It is intended to be used on circular robots, so that its radius can 
be set to the same as the robot. The sensor ring can be set to sense a specific 
section of the circle, and the number of sensors used can be specified. Whenever 
the state of one of the buttons changes, a ROS message is published. Extra 
circles are drawn on the world visualization to show which touch sensors are 
triggered. 

:ref:`Properties Table <tab-touchring_properties>`

:ref:`ROS Channels <tab-touchring_channels>`

.. _tab-touchring_properties:

======================   ==============   ========================================================
Property Name            Data Type	      Description
======================   ==============   ========================================================
channels/ouput_channel   String           ROS topic to output sensor messages
angle_start              Double (0-360)   Angle that the sensed section starts at
angle_end                Double (0-360)   Angle that the sensed section ends at
ring_radius              Double           Radius of the touch sensor ring
sensor_count             Integer          Number of sensor spaced evenly in the slice of the circle defined by angle_start and angle_end
======================   ==============   ========================================================


.. _tab-touchring_channels:

========================   ==================================   =========   ========================================================
ROS Topic                  Message Type                         In/Out      Description
========================   ==================================   =========   ========================================================
channels/output_channel    sensor_msgs::msg::ByteMultiArray     Out         1D vector with one element for each touch sensor on the ring. Untriggered buttons are set of 0, trigggers ones are non-0.
========================   ==================================   =========   ========================================================

Design Details 
""""""""""""""

    #. All the required circle shapes for the touch sensors are generated at the start. They are added to and removed from the model when they become active or inactive.
    #. The touch sensor circle is a solid physics shape which can collide. This generates collision points from Box2D that can be used to know what is sensed.
    #. The body with the sensor ring fixture is held in place relative to the robot by a Box2D Weld Joint. The ring has a very low density (so as not to affect how the robot drives) so, due to the implementation of Weld Joints,  it is possible for the ring to behave in strange ways if it is larger than the robot it surrounds. Specifically, if there is a collsion and the robot continues to drive, the robot can be seen moving around within the touch sensor ring; it does not stay anchored in the center as one might expect.

Lidar Sensor
^^^^^^^^^^^^

The lidar sensor behaves as a regular hardware lidar would. It scans an area in 
front of (or around) itself at a high rate and reports the distances to the 
objects scanned. The lidar provided by this plugin can be customized to scan any 
size range from 0 to 360 degrees, with the center of the range directly in front 
of the lidar. The number of scan rays and the radius (maximum range) can also be 
specified, along with the rate of scanning. Each time a scan is performed, the 
image representing the lidar and its scan will be updated, and a ROS message 
will be published. 

:ref:`Properties Table <tab-lidar_properties>`

:ref:`ROS Channels <tab-lidar_channels>`

.. _tab-lidar_properties:

======================   ==============   ========================================================
Property Name            Data Type	      Description
======================   ==============   ========================================================
channels/ouput_channel   String           ROS topic to output sensor messages
scan_range               Double (0-360)   Total angle range to sense, in degrees	
scan radius              Double           Max distance away from the sensor to detect
scan_points              Integer >= 2     Number of beams spaced evently in the range. Including the first and last beam.
scan_rate                Double           Rate (hz) of scans and messages
======================   ==============   ========================================================


.. _tab-lidar_channels:

========================   ==================================   =========   ========================================================
ROS Topic                  Message Type                         In/Out      Description
========================   ==================================   =========   ========================================================
channels/output_channel    sensor_msgs::msg::LaserScan          Out         The standard ROS LaserScan message with parameters filled according to the property settings of the sensor.
========================   ==================================   =========   ========================================================

Design Details 
""""""""""""""

    #. The Box2D shapes used to represent the lidar beams are created any time a Property changes which affects them. During runtime, their lengths are modified to represent what is seen.
    #. Given a range of x degrees, the lidar will report beams from the angle of -x/2 to x/2, relative to the lidar body.	
    #. Any beams which do not detect an obstacle will report a distance of infinity, as defined by the implementation of :code:`std::numeric_limits<float32>::infinity()`.

Image Loading Plugin
--------------------

One of the functions of the original STDR was that it could load an image, and then the dark pixels of the image would be treated as obstacles. This was 
simple for the STDR to do because of the way it did collision detection - on a per-pixel basis. Collisions in this project are done in the Box2D engine, 
so in order to provide a similar functionality, a plugin is included which converts an image to a set of obstacles which can be loaded into the physics engine. 
The plugin can handle the common image formats .png, .jpg, and .bmp. All images are converted to black and white before processing. A threshold is chosen by the 
user, and all pixels with an intensity above that are changed to 255 and all pixels with intensity at or below it are set to 0. When a file is loaded, the user is 
presented with a prompt in which they can specify the following:

    * Width and height of the image (meters)
    * Pixels per meter in both the horizontal and vertical directions
    * Straightness value for the shapes read
    * Greyscale threshold

If you load up an image and there are pieces missing or shaped incorrectly, try loading it again with different settings and you might have better luck.

Design Details 
""""""""""""""

    #. The triangulation and simplification necessary to load polygons into the physics engine is not done in the plugin, but in the polygon component type. Similarly, scaling is also done in the polygon component type.
	
JSON Plugin
-----------

One general-purpose file handler plugin is provided as part of the project. It is able format WorldObjects and WorldObjectComponents as JSON objects which can be 
stored in files. The JSON formatter can be used to save single World Objects and entire Simulations
