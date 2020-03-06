cmake_minimum_required(VERSION 3.5)
project(veranda_qt_frontend_default_robot_file_handler)

## Compile as C++14 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_qt_frontend_plugin_api REQUIRED)
find_package(rclcpp REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core Gui
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
    veranda_core_api
    veranda_box2d
    veranda_qt_frontend_plugin_api
)

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
# Need to include this early so that moc is happy
include_directories("${veranda_box2d_INCLUDE_DIRS}")
include_directories("${veranda_core_api_INCLUDE_DIRS}")
include_directories("${veranda_qt_frontend_plugin_api_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build the executable
add_library(default_robot_loader_plugin SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(default_robot_loader_plugin Core Gui)

## Pull in stuff exported from other packages
ament_target_dependencies(default_robot_loader_plugin
    "veranda_core_api"
    "veranda_box2d"
    "rclcpp"
    "veranda_qt_frontend_plugin_api"
)

ament_index_get_resource(PLUGIN_PATH "veranda_plugin_path" "veranda_qt_frontend")

## Install lib into same directory as simulator executable
install(
  TARGETS default_robot_loader_plugin
  DESTINATION ${PLUGIN_PATH}
)

ament_package()

