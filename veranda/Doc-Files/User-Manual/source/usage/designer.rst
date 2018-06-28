Designing Robots!
=================

Before you can simulate robots in Veranda, you need to tell the simulator what your robot looks like. For you to do this, Veranda includes a designer mode, which allows you to drag and drop the components added by your plugins. The collection of 1 or more components that you put in the designer view is considered to be an 'object', and can be saved and loaded again or added to a simulation. All of the pieces of an object will move together during simulation, even if they don't appear to actually be touching each other, and they will never collide with each other, even if they overlap. Some pieces, usually the sensors, will not even collide with walls or other objects in your simulation! The behaviors of components are determined by the plugins which add them to your toolbox.

(Hint! We're using the word 'Robot' to describe the things you're designing, but you can design things that are just obstacles and can't move too!)

The Designer - Left Panel
--------------------------

The first thing we need to do is switch to designer mode. This can be done by selecting the pencil button in the upper-left corner of the application. You'll know you are in designer mode when the button becomes grey and disabled; you can't enter designer mode when you are already in it!

Once you are in designer mode, a couple of options specific to designing will become available to you directly below the designer mode pencil. These buttons are described in the figure with the designer mode options :ref:`below <fig-designer_options>`

.. _fig-designer_options:

.. figure:: /images/designer/options.png
    :figwidth: 90%
    :width: 50%
    :align: center
    
    The Designer mode button and Designer-specific option buttons. Select the 'Plus' button to clear the designer and start a new robot. Choose the save button to save your robot in a file. Press the folder button to load a robot that was saved in a file previously.

The next two things you will see on the left panel of the designer are the :ref:`components list <fig-designer_components>` and the :ref:`properties window <fig-designer_properties>`. The components list shows you the names of all the components you've added to the designer, and lets you select one to make it active so you can move it or change its properties. The Properties window shows the named properties of the currently selected component. Some properties cannot be changed; they are read-only data that will be calculated during simulation.

.. _fig-designer_components:

.. figure:: /images/designer/objects.png
    :figwidth: 90%
    :width: 50%
    :align: center

    The Designer components list. You can see that I had 4 wheels, a lidar, and a rectangle body on my robot. (It was the Ackermann demo robot)

.. _fig-designer_properties:

.. figure:: /images/designer/properties.png
    :figwidth: 90%
    :width: 50%
    :align: center

    The Properties window for the currently selected component. Here you can fine-tune its location, set ROS topic names, or modify other exposed variables that affect the component.

The Designer - Right Panel
--------------------------

On the right panel of the Designer, you will find the :ref:`component toolbox <fig-designer_toolbox>`. The toolbox shows you all of the components available to you which have been added by plugins. Plugins can specify a grouping for the plugin they add, and the toolbox will make these groups available to you as different tabs. The Veranda project comes with a couple of wheels, static shapes, and sensors. The buttons at the top of the toolbox can be used to add the selected component in the toolbox, delete the selected component in the designer, and export the designed robot directly to the simulator without having to save it in a file.

.. _fig-designer_toolbox:

.. figure:: /images/designer/toolbox.png
    :figwidth: 90%
    :width: 40%
    :align: center

    The Designer mode toolbox, which shows the components you can add to your robot.

The Designer - Central View
---------------------------

The main panel of the Designer is right in the middle! It shows the robot you are currently designing, and lets you drag the pieces around to move and rotate them. On the left and right of the center panel are buttons you can use to minimize the left and right panels to give you more room to design. If you want to adjust your view, you can use the 'q' and 'e' keys to zoom in and out, and 'w', 'a', 's', and 'd' to pan around the designer.

.. _fig-designer_mainview:

.. figure:: /images/designer/mainview.png
    :figwidth: 90%
    :width: 80%
    :align: center

    The main central view of the designer mode. In this picture, I had the lidar selected, and could move it using the green arrows in the right bottom corner.