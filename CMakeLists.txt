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
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS

    include/interfaces/drivetrain_if.h
    include/interfaces/robot_interfaces.h
    include/interfaces/sensor_if.h
    include/interfaces/simulator_physics_if.h
    include/interfaces/simulator_ui_if.h
    include/interfaces/simulator_visual_if.h
    include/interfaces/screen_model_if.h

    include/basic_physics.h
    include/basic_ui.h
    include/basic_viewer.h
    include/sdsmt_simulator.h
    include/robot.h

    include/ui/emptysimwindow.h
    include/ui/mainwindow.h)

## .ui qt widgets form files
set(UI_FILES
    ui/emptysimwindow.ui
    ui/mainwindow.ui)

## Any other source files
set(CPP_SRCS
    src/main.cpp
    src/basic_maploader.cpp
    src/basic_physics.cpp
    src/basic_robotloader.cpp
    src/basic_ui.cpp
    src/basic_viewer.cpp
    src/sdsmt_simulator.cpp
    src/robot.cpp

    src/ui/emptysimwindow.cpp
    src/ui/mainwindow.cpp
    )

set(RCC_FILES ui/resources.qrc)
qt5_add_resources(RCC_SRCS ${RCC_FILES})

## Box2D sources
set(BOX2D_PATH Box2D/Box2D/Box2D)
set(BOX2D_SRCS

    ${BOX2D_PATH}/Common/b2BlockAllocator.cpp
    ${BOX2D_PATH}/Common/b2Draw.cpp
    ${BOX2D_PATH}/Common/b2Math.cpp
    ${BOX2D_PATH}/Common/b2Settings.cpp
    ${BOX2D_PATH}/Common/b2StackAllocator.cpp
    ${BOX2D_PATH}/Common/b2Timer.cpp

    ${BOX2D_PATH}/Collision/b2BroadPhase.cpp
    ${BOX2D_PATH}/Collision/b2CollideCircle.cpp
    ${BOX2D_PATH}/Collision/b2CollidePolygon.cpp
    ${BOX2D_PATH}/Collision/b2CollideEdge.cpp
    ${BOX2D_PATH}/Collision/b2Collision.cpp
    ${BOX2D_PATH}/Collision/b2Distance.cpp
    ${BOX2D_PATH}/Collision/b2DynamicTree.cpp
    ${BOX2D_PATH}/Collision/b2TimeOfImpact.cpp

    ${BOX2D_PATH}/Dynamics/b2Body.cpp
    ${BOX2D_PATH}/Dynamics/b2ContactManager.cpp
    ${BOX2D_PATH}/Dynamics/b2Fixture.cpp
    ${BOX2D_PATH}/Dynamics/b2Island.cpp
    ${BOX2D_PATH}/Dynamics/b2WorldCallbacks.cpp
    ${BOX2D_PATH}/Dynamics/b2World.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2DistanceJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2Joint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2PrismaticJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2RopeJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2FrictionJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2MotorJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2PulleyJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2WeldJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2GearJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2MouseJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2RevoluteJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2WheelJoint.cpp

    ${BOX2D_PATH}/Rope/b2Rope.cpp
)

###########
## Build ##
###########

## Locations of header files
include_directories(
  include
  Box2D/Box2D
  ${catkin_INCLUDE_DIRS}
)

## Build Box2d
add_library(box2d STATIC ${BOX2D_SRCS})

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
  box2d
  ${catkin_LIBRARIES}
)
