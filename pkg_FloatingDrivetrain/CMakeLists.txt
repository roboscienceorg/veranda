cmake_minimum_required(VERSION 2.8.3)
project(sdsmt_simulator_floating_drivetrain_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
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

## TODO If possible: Figure out how to mark Qt as a dependency
catkin_package(
    CATKIN_DEPENDS sdsmt_simulator sdsmt_simulator_box2d
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/floating_drivetrain.h
    include/floating_drivetrain_plugin.h)

## Any other source files
set(CPP_SRCS
    src/floating_drivetrain.cpp
    src/floating_drivetrain_plugin.cpp)

###########
## Build ##
###########

## Locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${sdsmt_simulator_INCLUDE_DIRS}
  ${sdsmt_simulator_box2d_INCLUDE_DIRS}
)

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build the executable
add_library(floating_drivetrain_plugin SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(floating_drivetrain_plugin Core)

## Link any other libraries
target_link_libraries(floating_drivetrain_plugin
  ${catkin_LIBRARIES}
  ${sdsmt_simulator_box2d_LIBRARIES}
)

#############
## INSTALL ##
#############
install(TARGETS floating_drivetrain_plugin
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
