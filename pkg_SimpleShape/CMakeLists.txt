cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_simple_shape_plugin)

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sdsmt_simulator_api REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

include_directories( include )
include_directories( ${CMAKE_BINARY_DIR} )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    sdsmt_simulator_api
    sdsmt_simulator_box2d
)

# Need to include this early so that moc is happy
include_directories(${sdsmt_simulator_api_INCLUDE_DIRS})
include_directories(${sdsmt_simulator_box2d_INCLUDE_DIRS})

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/simple_shape.h
    include/simple_shape_plugin.h)

## Any other source files
set(CPP_SRCS
    src/simple_shape.cpp
    src/simple_shape_plugin.cpp)

###########
## Build ##
###########
# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build the executable
add_library(simple_shape_plugin SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(simple_shape_plugin Core)

## Pull in stuff exported from other packages
ament_target_dependencies(simple_shape_plugin
    "rclcpp"
    "sdsmt_simulator_box2d"
    "sdsmt_simulator_api"
)

## Install lib into same directory as simulator executable
install(
  TARGETS simple_shape_plugin
  DESTINATION lib
)

ament_package()

