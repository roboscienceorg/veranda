cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_image_loader_plugin)

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(WIN32)
  add_definitions(-DWINDOWS)
endif()

# Remove -g from general builds
string(REPLACE -g "" CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS)

# Always use -Wall
set(CMAKE_CXX_FLAGS "-Wall")

# Add -g to debug builds
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# Use O3 for release builds
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(ament_cmake REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)
find_package(sdsmt_simulator_api REQUIRED)
find_package(rclcpp REQUIRED)

find_package(OpenGL REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

include_directories( ${CMAKE_BINARY_DIR} )
include_directories( include )

include_directories( ${OPENGL_INCLUDE_DIRS} )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    std_msgs
    sdsmt_simulator_api
    sdsmt_simulator_box2d
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/image_loader.h
    include/image_loader_plugin.h
    include/polygonscomponent.h
    include/optiondialog.h)

## Any other source files
set(CPP_SRCS
    src/image_loader.cpp
    src/image_loader_plugin.cpp
    src/polygonscomponent.cpp
    src/imageparser.cpp)

###########
## Build ##
###########
# Need to include this early so that moc is happy
include_directories("${sdsmt_simulator_box2d_INCLUDE_DIRS}")
include_directories("${sdsmt_simulator_api_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build the executable
add_library(image_loader_plugin SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(image_loader_plugin Core Gui Widgets)

## Pull in stuff exported from other packages
ament_target_dependencies(image_loader_plugin
    "sdsmt_simulator_api"
    "sdsmt_simulator_box2d"
    "rclcpp"
)

target_link_libraries(image_loader_plugin ${OPENGL_LIBRARIES} )

## Install lib into same directory as simulator executable
install(
  TARGETS image_loader_plugin
  DESTINATION lib
)

ament_package()

