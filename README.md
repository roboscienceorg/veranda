# Veranda: A 2-Dimensional Mobile Robotics Simulation Environment

This repository contains the code for a 2D Robotics simulation tool. The tool is written to interface with other applications through the ROS2 communication layer, and is usable on all operating systems that can have Qt and ROS2 installed.

## Project Structure
For anyone who is interested in building this project, the target version of ROS 2 is Ardent Apalone. If the project developers choose to support other versions of ROS, side branches will be created with the changes necessary to build/run Veranda under other versions of ROS.

### Current Support
 * ROS Ardent
   * Supported on the 'master' branch
   * Built using the ROS build tool Ament
 * ROS Bouncy
   * Supported on the 'bouncy' branch
   * Built using the ROS build tool Colcon; Ament is not supported
   * May require modifying CMAKE_PREFIX_PATH so that Qt5Config.cmake can be found

## Project Documentation
Detailed instructions for building, running, and developing this project are found in the User Manual and Design Document which can be built from this repository. Building these documents requires thee command line tools to be installed
 * pdflatex
 * doxygen
 * sphinx

To build the documentation, clone the repository and run the make_documentation script in the veranda subfolder. This will generate a new Documentation folder with the following
 * Online Doxygen Reference
 * Online Sphinx User Manual
 * PDF Doxygen Reference
 * PDF Sphinx User Manual
 * Design Documentation

The design documentation contains the other two PDF documents within itself, as well as details of the internal design of the simulator and a record of its development.
