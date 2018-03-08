###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_api)

## Compile as C++11
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)
find_package(rclcpp REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core Gui
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( include ${CMAKE_BINARY_DIR} )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(
    sdsmt_simulator_box2d
    rclcpp
)

ament_export_include_directories( include )
ament_export_libraries(sdsmt_simulator_api)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/sdsmt_simulator/world_object.h
    include/sdsmt_simulator/model.h
    include/sdsmt_simulator/property.h
    include/sdsmt_simulator/world_object_component.h
)

set(CPP_HDRS
    include/sdsmt_simulator/world_object_component_plugin.h
    include/sdsmt_simulator/world_object_file_handler_plugin.h
    include/sdsmt_simulator/world_object_loader_if.h
    include/sdsmt_simulator/world_object_saver_if.h
)

set(CPP_SRCS
    src/world_object.cpp
    src/world_object_component.cpp
)

###########
## Build ##
###########

# Need to include this early to make the MOC happy
include_directories("${sdsmt_simulator_box2d_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build Library
add_definitions(-DSDSMT_SIMULATOR_DLL)
add_library(sdsmt_simulator_api SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to library
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator_api Core Gui)

## Pull in stuff exported from other packages
ament_target_dependencies(sdsmt_simulator_api
    "sdsmt_simulator_box2d"
    "rclcpp"
)

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY include/sdsmt_simulator
  DESTINATION include
)

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(INSTALLDIR lib)
if(WIN32)
    set(INSTALLDIR bin)
endif()

install(
  TARGETS sdsmt_simulator_api
  DESTINATION ${INSTALLDIR}
)

ament_package()

