###########
## SETUP ##
###########
cmake_minimum_required(VERSION 2.8.3)
project(sdsmt_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sdsmt_simulator_box2d
)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( ${CMAKE_BINARY_DIR} )

## TODO If possible: Figure out how to mark Qt as a dependency
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sdsmt_simulator_box2d
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS

    include/sdsmt_simulator/drivetrain_if.h
    include/sdsmt_simulator/sensor_if.h
    include/sdsmt_simulator/robotcomponent_if.h
    include/sdsmt_simulator/property.h
    include/sdsmt_simulator/model.h
    include/sdsmt_simulator/world_object_component_if.h

    include/interfaces/simulator_physics_if.h
    include/interfaces/simulator_ui_if.h
    include/interfaces/simulator_visual_if.h
    include/interfaces/old_world_object_if.h
    include/interfaces/new_world_object_if.h

    include/basic_physics.h
    include/basic_viewer.h
    include/simulator_core.h
    include/robot.h
    include/map.h
    include/world_object.h

    include/ui/mainwindow.h
    )

## .ui qt widgets form files
set(UI_FILES
    ui/mainwindow.ui)

## Any other source files
set(CPP_SRCS
    src/basic_maploader.cpp
    src/basic_physics.cpp
    src/basic_robotloader.cpp
    src/basic_viewer.cpp
    src/simulator_core.cpp
    src/robot.cpp
    src/world_object.cpp

    src/map.cpp
    src/ui/mainwindow.cpp
    src/main.cpp
    )

set(RCC_FILES ui/resources.qrc)
qt5_add_resources(RCC_SRCS ${RCC_FILES})

###########
## Build ##
###########

## Locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Wrap .ui files as cpp files
qt5_wrap_ui(UI_SRCS ${UI_FILES})

## Build the executable
add_executable(sdsmt_simulator ${UI_SRCS} ${CPP_SRCS} ${MOC_SRCS} ${RCC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator Core Gui Widgets)

## Link any other libraries
target_link_libraries(sdsmt_simulator
  ${catkin_LIBRARIES}
)

#############
## INSTALL ##
#############
install(TARGETS sdsmt_simulator
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
