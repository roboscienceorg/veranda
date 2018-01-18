cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_default_robot_loader_plugin)

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(sdsmt_simulator REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

include_directories( ${CMAKE_BINARY_DIR} )
include_directories( include )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    std_msgs
    sdsmt_simulator
    sdsmt_simulator_box2d
)

# Need to include this early so that moc is happy
include_directories(${sdsmt_simulator_INCLUDE_DIRS})

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/default_robot_loader.h
    include/default_robot_loader_plugin.h)

## Any other source files
set(CPP_SRCS
    src/default_robot_loader.cpp
    src/default_robot_loader_plugin.cpp)

###########
## Build ##
###########

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build the executable
add_library(default_robot_loader_plugin SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(default_robot_loader_plugin Core)

## Pull in stuff exported from other packages
ament_target_dependencies(default_robot_loader_plugin
    "sdsmt_simulator"
    "sdsmt_simulator_box2d"
)

## Install lib into same directory as simulator executable
install(
  TARGETS default_robot_loader_plugin
  DESTINATION bin
)

ament_package()

