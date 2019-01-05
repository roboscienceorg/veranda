cmake_minimum_required(VERSION 3.5)
project(veranda_builtin_shapes)

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
    veranda_core_api
    veranda_box2d
)

# Need to include this early so that moc is happy
include_directories(${veranda_core_api_INCLUDE_DIRS})
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
        "veranda_core_api")

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

set(headers include/circle.h include/circle_plugin.h)
set(sources src/circle.cpp src/circle_plugin.cpp)
make_plugin(circle_plugin "${headers}" "${sources}")

set(headers include/rectangle.h include/rectangle_plugin.h)
set(sources src/rectangle.cpp src/rectangle_plugin.cpp)
make_plugin(rectangle_plugin "${headers}" "${sources}")

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS} )

set(headers include/polygon.h include/polygon_plugin.h)
set(sources src/polygon.cpp src/polygon_plugin.cpp src/triangulator.cpp)
make_plugin(polygon_plugin "${headers}" "${sources}")

target_link_libraries(polygon_plugin ${OPENGL_LIBRARIES} )

ament_package()

