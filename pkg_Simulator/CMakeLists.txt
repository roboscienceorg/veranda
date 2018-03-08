###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sdsmt_simulator_box2d REQUIRED)
find_package(sdsmt_simulator_api REQUIRED)

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
    sensor_msgs
    sdsmt_simulator_box2d
    sdsmt_simulator_api
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/interfaces/simulator_physics_if.h
    include/interfaces/simulator_ui_if.h
    include/interfaces/simulator_visual_if.h
    include/interfaces/world_object_wrappers.h

    include/basic_physics.h
    include/basic_viewer.h
    include/simulator_core.h

    include/ui/mainwindow.h
    include/ui/joystickprototype.h
    include/ui/settingspopup.h
    )

## .ui qt widgets form files
set(UI_FILES
    ui/mainwindow.ui
    ui/joystickprototype.ui
    ui/settingspopup.ui)

## Any other source files
set(CPP_SRCS
    src/basic_physics.cpp
    src/basic_viewer.cpp
    src/simulator_core.cpp

    src/ui/mainwindow.cpp
    src/ui/joystickprototype.cpp
    src/ui/settingspopup.cpp
    src/main.cpp
    )

set(RCC_FILES ui/resources.qrc)
qt5_add_resources(RCC_SRCS ${RCC_FILES})

###########
## Build ##
###########

# Need to include this early to make the MOC happy
include_directories("${sdsmt_simulator_box2d_INCLUDE_DIRS}")
include_directories("${sdsmt_simulator_api_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Wrap .ui files as cpp files
qt5_wrap_ui(UI_SRCS ${UI_FILES})

add_executable(sdsmt_simulator ${UI_SRCS} ${CPP_SRCS} ${MOC_SRCS} ${RCC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(sdsmt_simulator Core Gui Widgets)

## Pull in stuff exported from other packages
ament_target_dependencies(sdsmt_simulator
    "rclcpp"
    "std_msgs"
    "sensor_msgs"
    "sdsmt_simulator_box2d"
    "sdsmt_simulator_api"
)

#############
## Install ##
#############

## Install executable
install(
  TARGETS sdsmt_simulator
  DESTINATION lib/sdsmt_simulator
)

## On windows, we need to run windeployqt
## To set up plugins and dlls because Windows
if(WIN32)
    install(CODE 
        "
        MESSAGE(\"Running WindDeployQt.\")

        execute_process (
            COMMAND cmd /c echo ${CMAKE_INSTALL_PREFIX}
        )

        execute_process (
            COMMAND cmd /c \"%QT5_DIR%/bin/windeployqt sdsmt_simulator.exe --no-compiler-runtime  --no-translations\"
            WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/sdsmt_simulator
        )
        "
    )
endif()

ament_package()

