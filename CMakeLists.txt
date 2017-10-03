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
set(MOC_HDRS )

## .ui qt widgets form files
set(UI_FILES )

## Any other source files
set(CPP_SRCS src/main.cpp)

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
add_executable(sdsmt_simulator ${CPP_SRCS} ${MOC_SRCS} ${UI_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator Core Gui Widgets)

## Link any other libraries
target_link_libraries(sdsmt_simulator
  ${catkin_LIBRARIES}
)
