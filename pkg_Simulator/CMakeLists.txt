###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator)

set(CMAKE_VERBOSE_MAKEFILE ON)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Gui
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( include ${CMAKE_BINARY_DIR} )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    std_msgs
    sdsmt_simulator_box2d
)

ament_export_include_directories( include )
ament_export_libraries(sdsmt_simulator_objs)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS

    include/sdsmt_simulator/drivetrain_if.h
    include/sdsmt_simulator/sensor_if.h
    include/sdsmt_simulator/robotcomponent_if.h

    include/interfaces/simulator_physics_if.h
    include/interfaces/simulator_ui_if.h
    include/interfaces/simulator_visual_if.h
    include/interfaces/old_world_object_if.h

    include/basic_physics.h
    include/basic_viewer.h
    include/simulator_core.h
    include/robot.h
    include/map.h

    include/ui/mainwindow.h
    )

set(LIB_MOC_HDRS
    include/sdsmt_simulator/world_object.h
    include/sdsmt_simulator/model.h
    include/sdsmt_simulator/property.h
    include/sdsmt_simulator/world_object_component_if.h
    )

## .ui qt widgets form files
set(UI_FILES
    ui/mainwindow.ui)

## Any other source files
set(CPP_SRCS
    src/basic_physics.cpp
    src/basic_viewer.cpp
    src/simulator_core.cpp
    src/robot.cpp

    src/map.cpp
    src/ui/mainwindow.cpp
    src/main.cpp
    )

set(LIB_SRCS
    src/world_object.cpp
)

set(RCC_FILES ui/resources.qrc)
qt5_add_resources(RCC_SRCS ${RCC_FILES})

###########
## Build ##
###########
include_directories("${sdsmt_simulator_box2d_INCLUDE_DIRS}")

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(LIB_MOC_SRCS ${LIB_MOC_HDRS})
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Wrap .ui files as cpp files
qt5_wrap_ui(UI_SRCS ${UI_FILES})

## Add fPIC for library exports
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Build Library
add_library(sdsmt_simulator_objs STATIC ${LIB_SRCS} ${LIB_MOC_SRCS})

## Add qt module dependencies to library
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator_objs Core)

## Pull in stuff exported from other packages
ament_target_dependencies(sdsmt_simulator_objs
    "rclcpp"
)

## Remove fPIC for executable
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

## Build the executable
add_executable(sdsmt_simulator ${UI_SRCS} ${CPP_SRCS} ${MOC_SRCS} ${RCC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator Core Gui Widgets)

## Pull in stuff exported from other packages
ament_target_dependencies(sdsmt_simulator
    "rclcpp"
    "std_msgs"
)

set(WHOLE_LIBS -Wl,--whole-archive ${sdsmt_simulator_box2d_LIBRARIES} sdsmt_simulator_objs -Wl,--no-whole-archive)
target_link_libraries(sdsmt_simulator "${WHOLE_LIBS}")

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY include/sdsmt_simulator
  DESTINATION include
)

## Install executable and lib
install(
  TARGETS sdsmt_simulator
  DESTINATION lib/sdsmt_simulator
)

install(
  TARGETS sdsmt_simulator_objs
  DESTINATION bin
)

ament_package()

