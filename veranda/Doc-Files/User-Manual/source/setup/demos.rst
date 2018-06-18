.. _sec-demos:

Running the Demos
=================

The Veranda project comes with a couple of demo robots and control scripts pre-made. The robots are specifically configured to work with the control scripts; you should be able to load the demo robots right into the simulation, start the script, press play on the simulation, and watch them go. Robots and Scripts can be found in the ``Demo`` folder of the Veranda repository.

The Demo Robots
-------------------------

There are three robots included with the project:
    * Differential-w-GPS

        This is the classic turtle robot. It is a circular body with two wheels, which can be controlled as a differential drive system. The robot has a GPS attached to it, and will publish its absolute location in the simulation.

        Input Topics

        ==================  ========  ==========================================================
        Topic               Datatype  Purpose
        ==================  ========  ==========================================================
        robot0/left_wheel   Float32   Sets the velocity of the left wheel in radians per second
        robot0/right_wheel  Float32   Sets the velocity of the right wheel in radians per second
        ==================  ========  ==========================================================

        Output Topics

        =============  ========  ========================================================
        Topic          Datatype  Purpose
        =============  ========  ========================================================
        robot0/pose2d  Pose2D    Reports the current X, Y, Theta positioning of the robot
        =============  ========  ========================================================

    * Differential-w-Lidar-Touch

        This robot is the classic turtle robot with a twist; attached to the robot are a 360-degree Lidar sensor and a Bump sensor which will detect contact with any part of the robot body.

        Input Topics

        ==================  ========  ==========================================================
        Topic               Datatype  Purpose
        ==================  ========  ==========================================================
        robot0/left_wheel   Float32   Sets the velocity of the left wheel in radians per second
        robot0/right_wheel  Float32   Sets the velocity of the right wheel in radians per second
        ==================  ========  ==========================================================

        Output Topics

        ==============  ==============  ============================================================
        Topic           Datatype        Purpose
        ==============  ==============  ============================================================
        robot0/laser    LaserScan       Reports what is seen by the robot's Lidar sensor
        robot0/touches  ByteMultiArray  Reports the state of all the buttons spaced around the robot
        ==============  ==============  ============================================================
        
    * Ackermann-w-Lidar

        This robot is like a car! It has a rectangular base with 4 wheels; the back wheels are fixed and produce thrust, the front wheels cannot produce thrust, but are used to steer. The front wheels turn following the Ackermann constraint.
        This robot also has a lidar on it, but it only covers a 90-degree area in front of the robot.

        Input Topics

        ==================  ========  ===============================================================
        Topic               Datatype  Purpose
        ==================  ========  ===============================================================
        robot1/left_wheel   Float32   Sets the velocity of the left rear wheel in radians per second
        robot1/right_wheel  Float32   Sets the velocity of the right rear wheel in radians per second
        robot1/steer        Float32   Sets the angle in radians for the robot to steer towards
        ==================  ========  ===============================================================

        Output Topics

        ==============  ==============  ============================================================
        Topic           Datatype        Purpose
        ==============  ==============  ============================================================
        robot1/laser    LaserScan       Reports what is seen by the robot's Lidar sensor
        ==============  ==============  ============================================================

The Demo Scripts
-------------------------

The Demo/Scripts folder contains 6 different python scripts which can be run with the default robots. All of them can be run with the command ``python3 [scriptname]``, but some of them require the command line arguments described below. Remember to :ref:`source <sec-sourcing>` ROS2 and the Veranda workspace before running these scripts!

    * ``fig8_differential.py``

        Usage: ``python3 fig8_differential.py``

        Publishes left and right wheel velocities which will drive a robot in a figure-8 shape. It uses the topics ``robot0/left_wheel`` and ``robot0/right_wheel``. As you might expect, it can be used to control either of the demo differential drive robots. If multiple of them are in the simulation, they will all be controlled.

    * ``joystick_differential.py``

        Usage: ``python3 joystick_differential.py {input-topic} [output-topic]``

        Listens for messages from a 2-axis joystick and publishes left/right wheel commands on the topics ``[output-topic]/left_wheel`` and ``[output-topic]/right_wheel`` to drive a differential robot. The topic listened to for the joystick is ``[input-topic]/joystick``. If no output topic is given, it will be the same as the input topic. (For example, if the ``[input-topic]`` were 'banana', then the topics used would ``banana/joystick``, ``banana/left_wheel``, and ``banana/right_wheel``.

    * ``joystick_ackermann.py``

        Usage: ``python3 joystick_ackermann.py {input-topic} [output-topic]``

        Similarly to ``joystick_differential.py``, this script listens for messages from a 2-axis joystick; but it uses them to produce messages on three topics, in order to drive the Ackermann-w-lidar demo robot. The rules for determining the topic names are the same as for the differential joystick script; but there is an extra topic: ``[output-topic]/steer`` to control the steering wheels.

    * ``joystick_listener.py``

        Usage: ``python3 joystick_listener.py {topic}``

        Listens to the specified topic for joystick messages and writes the joystick information to stdout. It is similar to using the ``ros2 topic echo`` command, and is a good example of how to listen for joystick messages in a script.

    * ``lidar_listener.py``

        Usage: ``python3 lidar_listener.py {topic}``

        Listens to the specified topic for LaserScan messages and writes the lidar information to stdout. It is similar to using the ``ros2 topic echo`` command, and is a good example of how to listen for lidar messages in a script.

    * ``linux_joy_reader.py``

        Usage: ``python3 linux_joy_reader.py [device]``

        This script is only usable on Linux systems! It uses the plug-n-play joystick drivers available in the OS to listen for input from hardware joysticks. It has only been tested with Playstation controllers plugged into a USB port.

        If the script is run with no arguments, it will look through the available devices and print a list of all of the ones which are joysticks (they start with 'js'). When the script is run with one of these devices as its argument, it will listen to the input from that device and publish joystick messages to the topic ``[device]/joystick``.

Enough Talking, Lets Do Some Demos!
-----------------------------------

Here we've outlined a number of demos that you can run right after installing Veranda. Each one will require that you have multiple command line terminals open, and will number them 1-n. The first time you encounter a terminal number, you should ``cd`` into the Veranda workspace and :ref:`source <sec-sourcing>` both ROS2 and the Veranda workspace immediately before continuing the demo. All commands are given in terms of Linux, so you may need to make adjustments!

Demo 1: Driving GPS Turtle in a Figure-8
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Differential-w-GPS robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/fig8_differential.py``
    * Bask in the glory of your achievement

Demo 2: Driving Lidar Turtle with the Virtual Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Differential-w-Lidar-Touch robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Create a Virtual Joystick
    * Set the joystick topic to ``demo/joystick``
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_differential.py demo robot0``
    * Use the virtual joystick to drive the robot
    * Take over the world


Demo 3: Driving Ackermann Bot with the Virtual Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Terminal 1: ``ros2 run veranda veranda``
    * Load the Ackermann-w-Lidar robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Create a Virtual Joystick
    * Set the joystick topic to ``demo/joystick``
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_ackermann.py demo robot1``
    * Use the virtual joystick to drive the robot
    * Steal the moon

Bonus Demo! Driving Ackermann Bot with a Real, Live Joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    * Acquire a USB joystick controller
    * Plug that joystick into your Linux machine
    * Terminal 1: ``python3 ./src/veranda/veranda/Demo/Scripts/linux_joy_reader.py``
    * Terminal 1: ``python3 ./src/veranda/veranda/Demo/Scripts/linux_joy_reader.py [insert device here]``
    * Try devices until one works, and the terminal prints stuff when you move the joystick
    * Terminal 2: ``python3 ./src/veranda/veranda/Demo/Scripts/joystick_ackermann.py [device] robot1``
    * Terminal 3: ``ros2 run veranda veranda``
    * Load the Ackermann-w-Lidar robot into the toolbox
    * Add that robot to the simulation
    * Start the simulation
    * Use the joystick to drive the robot
    * Step 3: Profit