Simulating A World
==================

The simulator mode is the core feature of Veranda. Without it, the software has little purpose. When you are in simulation mode, you can import images to make obstacles, load robots and place them in the world, create virtual joysticks to generate ROS messages, and more! This section will describe in detail how to do all of those things using the buttons placed around the window in simulation mode.

The Simulator - Left Panel
--------------------------

So you want to simulate some robots? First thing you need to do is make sure you're in the simulation mode. Find the big gear button in the upper left corner of the application window and press it. When it is grey and disabled, you know you are in simulation mode.

Below the simulation mode gear, there are a bunch of different buttons you can hit. In left-to-right, top-to-bottom order, here's what they do (you can follow along on the :ref:`picture of the buttons <fig-simulator_options>`):

    * Game controller button: This button opens a new virtual joystick. You can open as many as you want! See the section about :ref:`sec-virtual_joysticks` for more information about these
    * Camera: Takes a snapshot of the simulation and saves it as a picture. These images are saved in the root of your Ament workspace
    * Runner: Sets the speed of the simulation. The simulator supports 1x, 2x, 3x, and 0.5x speeds. Press this button to cycle through the speeds, in that order. If your control code uses timers, you may want to use the :ref:`SimTimer <sec-simTimer>` so that your control code runs at the same speed as the simulation
    * Play/Pause: Press this to start or stop the simulation. You can only edit the simulation while it is stopped.
    * Plus: This will clear your simulation, leaving you with a clean slate to start over
    * Save: Lets you save your simulation in a file so you can make backups or keep working later
    * Load: Lets you load saved simulations, or import other files to make new simulations. The types of files that can be loaded are determined by the file handling plugins the application finds. Veranda includes a plugin that lets you load black and white images as maps of obstacles.
    * Orange Foot: Quicksaves your simulation in a temporary file
    * Blue Foot: Loads the last quicksave that you made by pressing the orange foot

.. _fig-simulator_options:

.. figure:: /images/simulator/options.png
    :figwidth: 90%
    :width: 50%
    :align: center

    The simulator gear button and all the option buttons that come with it.

The next two things you will see on the left panel of the simulator are the :ref:`objects list <fig-simulator_objects>` and the :ref:`properties window <fig-simulator_properties>`. The objects list shows you the names of everything you've added to the simulation, and lets you select one to make it active so you can move it or change its properties. The Properties window shows the named properties of the currently selected object. Some properties cannot be changed; they are read-only data that will be calculated during simulation. You might notice that some properties that you could set in the designer cannot be set in the simulation; these properties are read-only when the object is not in the designer. The properties shown during a simulation will contain all of the properties of all of the components in the object. If you need more room in the properties window to view the text displayed, you can click between the header values and drag left or right to resize the columns.

.. _fig-simulator_objects:

.. figure:: /images/simulator/objects.png
    :figwidth: 90%
    :width: 50%
    :align: center

    An example of the simulator objects list. You can see I had a bunch of objects that were added from importing an image, and a couple of robots that I added to the simulation.

.. _fig-simulator_properties:

.. figure:: /images/simulator/properties.png
    :figwidth: 90%
    :width: 50%
    :align: center

    Some properties of a robot in simulation. You can see that the properties list contains properties from the Left and Right wheels, as well as the location of the robot itself.

.. _sec-virtual_joysticks:

Virtual Joysticks
^^^^^^^^^^^^^^^^^

Virtual joysticks can be created to help you test your robots out before you're ready to handle sensor input and have it drive autonomously. You can create as many virtual joysticks as you want, and each one can be set to publish to a different ROS topic. The virtual joysticks send the standard ROS Joy message, so if you switch to a real joystick, it should behave similarly. To set the ROS topic that the virtual joystick publishes to, type the topic name into the text box at the top of the joystick window. You can control the joystick by clicking on it and dragging it, or by pressing the mapped keys. By default, the keys are 'w', 'a', 's', 'd', but you can changes this for each joystick in with the keymapping menu. Each virtual joystick simulates a 3-axis joystick, with the large circle being the X and Y axes, and the slider bar at the bottom being a Z axis.

.. figure:: /images/simulator/joystick.png
    :figwidth: 90%
    :width: 50%
    :align: center

    The virtual joystick window. If you close the main application, all joysticks will also be closed.

.. _sec-simTimer:

Using The SimTimer Object
^^^^^^^^^^^^^^^^^^^^^^^^^

The simulator publishes a timestamp message containing 
both the delta time for the most recent tick and the total time passed in the 
simulation. This message can be used to make control code more accurate when it relies 
on dead reckoning and when the simulator is progressing at non-realtime. To make using this message easier, the python SimTimer class is included 
with the project. The SimTimer has two simple methods:

    #. double global_time() - Returns total number of seconds 
    #. void create_timer(delta, callback) - Starts calling [callback] every [delta] seconds

Depending on its initialization parameters, the SimTimer will either listen 
to the simulator messages for its timestamp, in which case the global time 
will be the total time reported in those messages, or use a ROS node to create 
a repeating timer, in which case the global time will be seconds since epoch.

.. code-block:: python

	from veranda.SimTimer import SimTimer

SimTimer is constructed with three parameters

- useSimulatedTimed: bool - Whether or not to use timestamps from the simulation. If False, the OS clock will be used through a ROS node
- simulatedTimeChannel: string - ROS topic to listen on for timestamp messages. Until further notice, this should always be "veranda/timestamp"
- rosNode: rclpy.Node - The ROS node to use for listening to messages and registering timers

When listening to the timestamp messages, the SimTimer keeps a priority queue of all 
callbacks. They are ordered by their next target activation times. Each time a timestamp
is recieved, it is compared to the front of the queue, and until that first item has a 
higher activation time than the current time, items are dequeued, called, and queued 
again with their next activation time. Activation time is calculated by adding the 
callback delta to the time that the callback should have been called in a perfect 
simulation, which may differ slightly from the time that it was actually called.

.. code-block:: python
    :caption: A Python example using the SimTimer to trigger a callback. For further example, see the figure-8 Differential-Drive demo code. 

	# Import ROS2 libs
	import rclpy
	
	from rclpy.node import Node
	
	# Import custom timer
	from veranda.SimTimer import SimTimer
	
	# Start a ROS2 Node
	rclpy.init()
	
	node = Node("myNode")

	# Create a custom timer
	# Parameter 1: Boolean    - True if the timer should listen to timestamps from the simulator; False if it should use the OS clock
	# Parameter 2: String     - ROS2 topic of the timestamp message. Always "veranda/timestamp" for now.
	# Parameter 3: Rclpy.Node - The ROS2 Node to use to subscribe to messages
	simTime = SimTimer(True, "veranda/timestamp", node)
	
	# Define a callback that prints the amount of time passed
	def cb():
	    print(str(simTime.global_time()) + " seconds have passed")

	# Create a timer to trigger the callback every 1 seconds
	simTime.create_timer(1, cb)

	# Start the node
	rclpy.spin(node)

	# Cleanup on exit
	node.destroy_node()

	rclpy.shutdown()

The Simulator - Right Panel
---------------------------

On the right panel of the Simulator, you will find the :ref:`object toolbox <fig-simulator_toolbox>`. The toolbox shows you all of the objects available that you can put in the simulation. When you create objects, you can assign them a name and a group; that name and group will be used to place the object in the toolbox when you load the object. The buttons at the top of the toolbox can be used to add the selected object to the simulation and delete the selected object in the simulation.

.. _fig-simulator_toolbox:

.. figure:: /images/simulator/toolbox.png
    :figwidth: 90%
    :width: 50%
    :align: center

    Example of the simulation toolbox with some robots available

The Simulator - Central View
----------------------------

The main panel of the simulator is the center component of the window. It shows all the objects you've added to the simulation, lets you reposition them when the simulation isn't running, and can be used to pan and zoom around the simulation. On the left and right of the center panel are buttons you can use to minimize the left and right panels to give you more room to design. If you want to adjust your view, you can use the 'q' and 'e' keys to zoom in and out, and 'w', 'a', 's', and 'd' to pan around the designer.

.. figure:: /images/simulator/mainview.png
    :figwidth: 90%
    :width: 80%
    :align: center

    The main view of the simulator mode. In this picture, I had loaded an image as obstacles, and then put a couple of robots in the environment to sense the obstacles.