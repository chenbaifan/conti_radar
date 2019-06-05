cmake_minimum_required(VERSION 2.8.3)
project(radar_driver)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

include(FindProtobuf)

add_compile_options(-std=c++14)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(NVIDIA_DRIVE 1)
  add_definitions(-DNVIDIA_DRIVE)
endif()


if (NVIDIA_DRIVE)
  find_package(CUDA REQUIRED)
  find_package(Threads REQUIRED)
  find_package(Driveworks REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS} ${Driveworks_INCLUDE_DIR})

  list(APPEND Driveworks_LIBRARIES ${CUDA_LIBRARIES} ${CUDA_cublas_LIBRARY})

  if(LINUX)
    list(APPEND Driveworks_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} dl rt)
  elseif(VIBRANTE)
    list(APPEND Driveworks_LIBRARIES ${vibrante_LIBRARIES} ${vibrante_Xlibs_LIBRARIES} nvmedia ${EGL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  endif()
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
  # RadarRaw
  ContiRaw.msg 
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
if (NOT NVIDIA_DRIVE)
  catkin_package(
    CATKIN_DEPENDS roscpp message_runtime std_msgs message_generation
    INCLUDE_DIRS include
    LIBRARIES kvaser_interface
  )
else()
  catkin_package(
    CATKIN_DEPENDS roscpp message_runtime std_msgs message_generation
    INCLUDE_DIRS include
    LIBRARIES mttcan_interface
  )
endif()


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

if (NOT NVIDIA_DRIVE)
  # Kvaser can interface
  add_library(kvaser_interface
    src/can_interface/kvaser_interface.cc
    src/can_interface/utils.cc
  )
  target_link_libraries(kvaser_interface
    canlib
    ${catkin_LIBRARIES}
  )
else()
  # Mtt can interface
  add_library(mttcan_interface
    src/can_interface/mttcan_interface.cc
    src/can_interface/utils.cc
  )
  target_link_libraries(mttcan_interface
    ${Driveworks_LIBRARIES}
    ${catkin_LIBRARIES}
  )  
endif()

########################
## Radar Controller ##
########################

add_executable(radar_controller
  src/radar/radar_node.cc
  src/radar/radar_controller.cc
  src/radar/conti_ars408/conti_ars408_controller.cc
  src/radar/conti_ars408/conti_ars408.cc
)

if (NOT NVIDIA_DRIVE)
  target_link_libraries(radar_controller kvaser_interface ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
else()
  target_link_libraries(radar_controller mttcan_interface ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
endif()
add_dependencies(radar_controller ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#py_install_executable_script(scripts teleop_keyboard)
#py_install_executable_script(scripts teleop_keyboard_aux)

install(TARGETS radar_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (NOT NVIDIA_DRIVE)
  install(TARGETS kvaser_interface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS mttcan_interface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
