cmake_minimum_required(VERSION 2.8.3)
project(radar)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

include(FindProtobuf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  visualization_msgs
  geometry_msgs 
  message_generation
  tf2_ros
  tf2_geometry_msgs
  custom_msgs
  common
  yaml-cpp
  REQUIRED
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES radar
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs custom_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  src/
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
add_executable(radar_processor_node src/radar_processor_node.cc)
add_dependencies(radar_processor_node  ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_processor_node ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS radar_processor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


