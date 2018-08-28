###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(veranda)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(veranda_api REQUIRED)

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
    veranda_box2d
    veranda_api
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
    include/simulator_core.h

    include/ui/mainwindow.h
    include/ui/joystickprototype.h
    include/ui/mode_controller.h
    include/ui/settingspopup.h
    include/ui/qgraphicssimulationviewer.h
    include/ui/customgraphicsview.h
    )

## .ui qt widgets form files
set(UI_FILES
    ui/mainwindow.ui
    ui/joystickprototype.ui
    ui/settingspopup.ui
    ui/qgraphicssimulationviewer.ui)

## Any other source files
set(CPP_SRCS
    src/basic_physics.cpp
    src/simulator_core.cpp

    src/ui/mainwindow.cpp
    src/ui/joystickprototype.cpp
    src/ui/mode_controller.cpp
    src/ui/settingspopup.cpp
    src/ui/designer_widget.cpp
    src/ui/qgraphicssimulationviewer.cpp

    src/main.cpp
    )

set(RCC_FILES ui/resources.qrc)
qt5_add_resources(RCC_SRCS ${RCC_FILES})

###########
## Build ##
###########

# Need to include this early to make the MOC happy
include_directories("${veranda_box2d_INCLUDE_DIRS}")
include_directories("${veranda_api_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Wrap .ui files as cpp files
qt5_wrap_ui(UI_SRCS ${UI_FILES})

add_executable(veranda ${UI_SRCS} ${CPP_SRCS} ${MOC_SRCS} ${RCC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(veranda Core Gui Widgets)

## Pull in stuff exported from other packages
ament_target_dependencies(veranda
    "rclcpp"
    "std_msgs"
    "sensor_msgs"
    "veranda_box2d"
    "veranda_api"
)

#############
## Install ##
#############

## Install executable
install(
  TARGETS veranda
  DESTINATION lib/veranda
)

## Install python timer object
ament_python_install_package(veranda)

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
            COMMAND cmd /c \"%QT5_DIR%/bin/windeployqt veranda.exe --no-compiler-runtime  --no-translations\"
            WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/veranda
        )
        "
    )
endif()

ament_package()

