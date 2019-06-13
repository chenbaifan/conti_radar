cmake_minimum_required(VERSION 2.8.3)
project(radar_driver)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

include(FindProtobuf)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pb_msgs
  polyx_nodea
  common
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  # Radar config and state
  Conti_radar_state.msg      
  Conti_radar_config.msg     
  Conti_filter_config.msg     
  # For objs output mode.msg 
  Conti_obj_status.msg        
  Conti_obj_general.msg     
  Conti_obj_quality.msg     
  Conti_obj_extended.msg    
  Conti_obj_warning.msg     
  # For cluster output mode 
  Conti_cluster_status.msg  
  Conti_cluster_general.msg 
  Conti_cluster_quality.msg 
  # Motion Input
  Conti_motion_rx.msg    
  Radar_Target.msg
  Radar_State_Cfg.msg
  RadarTrack.msg
  RadarTrackArray.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp message_runtime std_msgs message_generation
  INCLUDE_DIRS include
  LIBRARIES kvaser_interface_radar
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
)

###################
## CAN Interface ##
###################
# Kvaser can interface
add_library(kvaser_interface_radar
  src/can_interface/kvaser_interface.cc
  src/can_interface/utils.cc
)
target_link_libraries(kvaser_interface_radar
  canlib
  ${catkin_LIBRARIES}
)


########################
## Radar Controller ##
########################

add_executable(radar_controller
  src/radar/radar_node.cc
  src/radar/radar_controller.cc
  src/radar/conti_ars408/conti_ars408_controller.cc
  src/radar/conti_ars408/conti_ars408.cc
)

target_link_libraries(radar_controller kvaser_interface_radar ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
add_dependencies(radar_controller ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS radar_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS kvaser_interface_radar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
