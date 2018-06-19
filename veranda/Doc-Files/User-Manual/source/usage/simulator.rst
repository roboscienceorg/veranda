Simulating A World
==================

The simulator mode is the core feature of Veranda. Without it, the software has little purpose. When you are in simulation mode, you can import images to make obstacles, load robots and place them in the world, create virtual joysticks to generate ROS messages, and more! This section will describe in detail how to do all of those things using the buttons placed around the window in simulation mode.

The Simulator - Left Panel
--------------------------

So you want to simulate some robots? First thing you need to do is make sure you're in the simulation mode. Find the big gear button in the upper left corner of the application window and press it. When it is grey and disabled, you know you are in simulation mode.

Below the simulation mode gear, there are a bunch of different buttons you can hit. In left-to-right, top-to-bottom order, here's what they do (you can follow along on the :ref:`picture of the buttons` <fig-simulator_options>`):

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

.. _sec-virtual_joysticks:

Virtual Joysticks
^^^^^^^^^^^^^^^^^

Virtual joysticks can be created to help you test your robots out before you're ready to handle sensor input and have it drive autonomously. You can create as many virtual joysticks as you want, and each one can be set to publish to a different ROS topic. The virtual joysticks send the standard ROS Joy message, so if you switch to a real joystick, it should behave similarly. To set the ROS topic that the virtual joystick publishes to, type the topic name into the text box at the top of the joystick window. You can control the joystick by clicking on it and dragging it, or by pressing the mapped keys. By default, the keys are 'w', 'a', 's', 'd', but you can changes this for each joystick in with the keymapping menu. Each virtual joystick simulates a 3-axis joystick, with the large circle being the X and Y axes, and the slider bar at the bottom being a Z axis.

.. figure:: /images/simulator/joystick.png
    :figwidth: 90%
    :width: 50%
    :align: center

    The virtual joystick window. If you close the main application, all joysticks will also be closed.

.. _sec-simTimer

Using The SimTimer Object
^^^^^^^^^^^^^^^^^^^^^^^^^

The Simulator - Right Panel
---------------------------

The Simulator - Central View
----------------------------