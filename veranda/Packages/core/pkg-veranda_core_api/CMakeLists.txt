###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_core_api)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(LIB_INSTALL_DIR lib)
if(WIN32)
    set(LIB_INSTALL_DIR bin)
endif()

project(${PROJECT_NAME})

## Compile as C++11
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(rclcpp REQUIRED)

## Find and configure QT
set(QT_COMPONENTS Core Gui Widgets)
find_package(Qt5 REQUIRED COMPONENTS
    ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( include ${CMAKE_BINARY_DIR} )

ament_export_dependencies(
    veranda_box2d
    rclcpp
)

ament_export_include_directories( ${HEADER_INSTALL_DIR} )
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_core)

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    ${HEADERS_DIR}/world_object.h
    ${HEADERS_DIR}/model.h
    ${HEADERS_DIR}/property.h
    ${HEADERS_DIR}/world_object_component.h
    ${HEADERS_DIR}/filter.h
    ${HEADERS_DIR}/interfaces/simulator_ui_if.h
    ${HEADERS_DIR}/interfaces/simulator_physics_if.h
    ${HEADERS_DIR}/interfaces/world_object_wrappers.h
)

set(CPP_HDRS
    ${HEADERS_DIR}/world_object_component_plugin.h
    ${HEADERS_DIR}/world_object_file_handler_plugin.h
    ${HEADERS_DIR}/object_loader_if.h
    ${HEADERS_DIR}/object_saver_if.h
)

set(CPP_SRCS
    src/world_object.cpp
    src/world_object_component.cpp
    src/filter.cpp
)

###########
## Build ##
###########

# Need to include this early to make the MOC happy
include_directories("${veranda_box2d_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build Library
add_definitions(-Dveranda_DLL)
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})

## Add qt module dependencies to library
## this will find all the headers and link all the libraries
qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

## Pull in stuff exported from other packages
ament_target_dependencies(${PROJECT_NAME}
    "veranda_box2d"
    "rclcpp"
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

#############
## Tests ##
#############

if(BUILD_TESTING)
    find_package(veranda_catch2 REQUIRED)

    ament_export_dependencies(
        veranda_catch2
    )

    set(CATCH_CMAKE "${veranda_catch2_INCLUDE_DIRS}/catch2/ament_cmake_add_catch_test.cmake")
    include(${CATCH_CMAKE})

    ament_add_catch_test(
        "model_datatype_tests"
        CPP_SOURCES tests/test_model.cpp
        QT_HEADERS ${HEADERS_DIR}/model.h
        QT_LIBS Core Gui
        ROS_LIBS "veranda_box2d"
        )

    ament_add_catch_test(
        "property_datatype_tests"
        CPP_SOURCES tests/test_property.cpp
        QT_HEADERS ${HEADERS_DIR}/property.h
        QT_LIBS Core Gui
        )
endif()

ament_package()

