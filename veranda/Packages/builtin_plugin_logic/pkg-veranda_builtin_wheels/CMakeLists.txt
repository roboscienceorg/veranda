cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_builtin_wheels)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(LIB_INSTALL_DIR lib)
if(WIN32)
    set(LIB_INSTALL_DIR bin)
endif()

project(${PROJECT_NAME})

## Compile as C++14 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_box2d REQUIRED)

## Find and configure QT
set(QT_COMPONENTS Core Gui)
find_package(Qt5 REQUIRED COMPONENTS
  ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( include include/veranda_wheels )
include_directories( ${CMAKE_BINARY_DIR} )

ament_export_dependencies(
    rclcpp
    veranda_core_api
    veranda_box2d
    std_msgs
    geometry_msgs
)

ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_wheels)

set(MOC_HDRS
        ${HEADERS_DIR}/ackermann_steer.h
        ${HEADERS_DIR}/encoder.h
        ${HEADERS_DIR}/fixed_wheel.h
        ${HEADERS_DIR}/omni_drive.h)

## Any other source files
set(CPP_SRCS
    src/ackermann_steer.cpp
    src/encoder.cpp
    src/fixed_wheel.cpp
    src/omni_drive.cpp)

###########
## Build ##
###########

# Need to include this early so that moc is happy
include_directories(${veranda_core_api_INCLUDE_DIRS})
include_directories(${veranda_box2d_INCLUDE_DIRS})

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build Library
add_definitions(-DVERANDA_WHEELS_BUILD_DLL)
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

## Pull in stuff exported from other packages
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "std_msgs"
    "geometry_msgs"
    "veranda_box2d"
    "veranda_core_api"
)

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY ${HEADERS_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${LIB_INSTALL_DIR}
)

ament_package()

