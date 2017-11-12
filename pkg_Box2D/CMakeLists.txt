###########
## SETUP ##
###########
cmake_minimum_required(VERSION 2.8.3)
project(sdsmt_simulator_box2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

## TODO If possible: Figure out how to mark Qt as a dependency
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sdsmt_simulator_box2d
)

###################
## FILE LISTINGS ##
###################

set(BOX2D_PATH include/Box2D)
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
    
    ${BOX2D_PATH}/Collision/Shapes/b2ChainShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2CircleShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2EdgeShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2PolygonShape.cpp
                        
    ${BOX2D_PATH}/Dynamics/b2Body.cpp
    ${BOX2D_PATH}/Dynamics/b2ContactManager.cpp
    ${BOX2D_PATH}/Dynamics/b2Fixture.cpp
    ${BOX2D_PATH}/Dynamics/b2Island.cpp
    ${BOX2D_PATH}/Dynamics/b2WorldCallbacks.cpp
    ${BOX2D_PATH}/Dynamics/b2World.cpp

    ${BOX2D_PATH}/Dynamics/Contacts/b2ChainAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2ChainAndPolygonContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2CircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2Contact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2ContactSolver.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2EdgeAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2PolygonAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2PolygonContact.cpp
    
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
)

## Build Box2d
add_library(sdsmt_simulator_box2d STATIC ${BOX2D_SRCS})
