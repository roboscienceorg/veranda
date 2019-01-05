###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_catch2)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

project(${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

ament_export_include_directories(${HEADER_INSTALL_DIR})

#############
## Install ##
#############
set(CATCH2_DIR include/catch2)

## Install include dir
install(
  DIRECTORY ${CATCH2_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

ament_package()
