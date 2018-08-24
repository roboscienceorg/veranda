Setting up Veranda
==================

Getting set up to use Veranda is, at the moment, somewhat of an in-depth endeavor. It requires first installing ROS2, and then downloading the source for Verand and building it. A one-stop install is coming soon to Operating Systems near you!

Installing ROS2
---------------

Veranda was developed using the very first version of ROS2: Ardent Apalone. Instructions for downloading and installing that can be found on their Github at `this address`_.

.. _this address: https://github.com/ros2/ros2/wiki/Installation

You are welcome to install either by building the source or downloading the pre-built binaries for your system. Regardless of if you install from source or binaries, you will need to follow the instructions on the "building from source" page for installing Qt in order to get Qt 5.10 on your system.

Building Veranda
----------------

Building the Veranda project is fairly simple once ROS2 is installed. First, you must make yourself a workspace, in which all of your work will be stored. This workspace is simply a directory with a specific structure.

NOTE: A couple of the steps below state that you should 'source the ROS environment' or 'source the project environment.' Doing this is described in the ROS2 documentation, but we have provided some short examples at the :ref:`bottom of the page <sec-sourcing>`. (Just in case you didn't see the online examples)

    #. Make a workspace directory
    #. Within that directory, make a directory called ``src``
    #. ``cd`` into the ``src`` directory, and execute ``git clone https://github.com/roboscienceorg/veranda.git``

Once your workspace is set up and the project has been cloned into the ``src`` directory, we can build the project

    #. ``cd`` into the root workspace directory
    #. Source the ROS2 environment
    #. Execute the command ``ament build``
    
Now the project should be built! Running the project now is easy

    #. ``cd`` into the root workspace directory
    #. Source the ROS2 environment
    #. Source the workspace environment
    #. Execute the command ``ros2 run veranda veranda``

At this point, the application should open, and you can start simulating robots to your heart's content. To make sure your system is set up correctly, we've provided a couple of :ref:`demos <sec-demos>` for you.

.. _sec-sourcing:

What is 'Sourcing your environment'?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing the ROS2 environment or your workspace environment sets up system variables that are specific to your terminal instance. These variables can contain all sorts of information. In ROS2, it contains all of the paths to the header files and binaries that you can link to. When you source your workspace, it stores the location of your built project files so that they can be run by ROS2.

Sourcing is different on different systems. In Linux, the command to run is ``source``; on OSX, it's merely `.`, and on Windows, it is `call`. We will provide examples here for Linux if you installed the pre-built binaries, and if you are on a different OS, you will need to hunt down the specific filenames to source for your system. If you built ROS from source, then you will need to source the workspace in which you built ROS instead. (Hint: Sourcing a workspace will use the same file on all three; it's just the ROS environment that might be different)

Sourcing the ROS2 environment: ``source /opt/ros/ardent/local_setup.bash``
Sourcing a ROS2 workspace: ``source [path/to/workspace]/install/local_setup.bash``

You may notice that in both instances, there are files ``local_setup.bash`` and ``setup.bash``. Sourcing the second one will source all other files that were sourced before it was created. So, after you build Veranda, you can source ``setup.bash`` in the workspace it was built in to source both the ROS environment and the workspace environment! (NOTE: On Windows [and possibly OSX], this will NOT source the install scripts for OpenSplice, or whatever RMW implementation you download. That should always be done after sourcing the workspace)
