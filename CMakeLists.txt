cmake_minimum_required(VERSION 2.8.3)
project(dronebridge_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  sensor_msgs
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  mavros_msgs 
  geometry_msgs 
  mav_msgs 
  sensor_msgs 
  message_runtime 
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(avoidance
#   src/${PROJECT_NAME}/avoidance.cpp
# )
set(PX4_REALSENSE_BRIDGE_CPP_FILES   "src/nodes/px4_realsense_bridge.cpp"
                              "src/nodes/px4_realsense_bridge_node.cpp"
)

add_library(px4_realsense_bridge  "${PX4_REALSENSE_BRIDGE_CPP_FILES}")

add_dependencies(px4_realsense_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(px4_realsense_bridge_node src/nodes/px4_realsense_bridge_node.cpp)

add_executable(vision_mavros_node src/nodes/vision_to_mavros.cpp)

#====================================================

target_link_libraries(px4_realsense_bridge_node
  px4_realsense_bridge
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})
  
 target_link_libraries(vision_mavros_node
  ${catkin_LIBRARIES}
)
  
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



