###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_qt_plugin_api)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

project(${PROJECT_NAME})

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(veranda_core_api REQUIRED)

ament_export_dependencies(
    veranda_core_api
)

ament_export_include_directories(${HEADER_INSTALL_DIR})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_qt_plugins)

###########
## Build ##
###########

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY ${HEADERS_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

ament_package()

