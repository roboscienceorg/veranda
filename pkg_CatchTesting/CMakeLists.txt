###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)
project(sdsmt_simulator_catch_tests)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

###########
## Build ##
###########
## Locations of header files
include_directories(
  include
)

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
