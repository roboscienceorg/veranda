###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_box2d)

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)

# Remove -g from general builds
string(REPLACE -g "" CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS)

# Always use -Wall
set(CMAKE_CXX_FLAGS "-Wall")

# Add -g to debug builds
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# Use O3 for release builds
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-DBOX2D_DLL)
find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(sdsmt_simulator_box2d)

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
add_library(sdsmt_simulator_box2d SHARED ${BOX2D_SRCS})

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY include/
  DESTINATION include
)

## Install library

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(INSTALLDIR lib)
if(WIN32)
    set(INSTALLDIR bin)
endif()

install(
  TARGETS sdsmt_simulator_box2d
  DESTINATION ${INSTALLDIR}
)

ament_package()
