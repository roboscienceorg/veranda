cmake_minimum_required(VERSION 3.5)
project(veranda_wheel_plugins)

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(veranda_api REQUIRED)
find_package(veranda_box2d REQUIRED)

## Find and configure QT
find_package(Qt5 REQUIRED COMPONENTS
  Core Gui
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
    veranda_api
    veranda_box2d
    std_msgs
)

# Need to include this early so that moc is happy
include_directories(${veranda_api_INCLUDE_DIRS})
include_directories(${veranda_box2d_INCLUDE_DIRS})

###########################################
## Make a plugin from source and headers ##
###########################################
function(make_plugin plugin_name plugin_moc_hdrs plugin_srcs)

    ## Run MOC on files with Qt Keywords and Macros
    qt5_wrap_cpp(plugin_moc_srcs ${plugin_moc_hdrs})

    ## Build the plugin lib
    add_library(${plugin_name} SHARED ${plugin_srcs} ${plugin_moc_srcs})

    ## Add qt module dependencies to executable
    ## this will find all the headers and link all the libraries
    qt5_use_modules(${plugin_name} Core Gui)

    ## Pull in stuff exported from other packages
    ament_target_dependencies(${plugin_name}
        "rclcpp"
        "veranda_box2d"
        "veranda_api"
        "std_msgs")

    ## Install lib into same directory as simulator executable
    install(
      TARGETS ${plugin_name}
      DESTINATION lib
    )
endfunction()

###########
## Build ##
###########
# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

set(headers include/fixed_wheel.h include/fixed_wheel_plugin.h include/encoder.h)
set(sources src/fixed_wheel.cpp src/fixed_wheel_plugin.cpp src/encoder.cpp)
make_plugin(fixed_wheel_plugin "${headers}" "${sources}")

set(headers include/ackermann_steer.h include/ackermann_steer_plugin.h)
set(sources src/ackermann_steer.cpp src/ackermann_steer_plugin.cpp)
make_plugin(ackermann_steer_plugin "${headers}" "${sources}")

ament_package()

