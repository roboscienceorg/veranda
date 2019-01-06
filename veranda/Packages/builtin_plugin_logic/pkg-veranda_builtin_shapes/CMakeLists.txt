cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_builtin_shapes)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(LIB_INSTALL_DIR lib)
if(WIN32)
    set(LIB_INSTALL_DIR bin)
endif()

project(${PROJECT_NAME})

## Compile as C++11 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else()
  add_definitions(-DWINDOWS)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_box2d REQUIRED)

## Find and configure QT
set(QT_COMPONENTS Core Gui)
find_package(Qt5 REQUIRED COMPONENTS
  ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( include include/veranda_shapes )
include_directories( ${CMAKE_BINARY_DIR} )

ament_export_dependencies(
    rclcpp
    veranda_core_api
    veranda_box2d
)

ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_shapes)

set(MOC_HDRS
        ${HEADERS_DIR}/circle.h
        ${HEADERS_DIR}/polygon.h
        ${HEADERS_DIR}/rectangle.h)

## Any other source files
set(CPP_SRCS
    src/circle.cpp
    src/polygon.cpp
    src/rectangle.cpp
    src/triangulator.cpp)

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
add_definitions(-Dveranda_DLL)
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} )

## Add qt module dependencies to executable
## this will find all the headers and link all the libraries
qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

## Pull in stuff exported from other packages
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
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

