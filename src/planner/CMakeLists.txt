cmake_minimum_required(VERSION 3.0.2)  # Updated minimum version
project(waypoint_global_planner)

# Enable C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure  # Moved to main components list
  geometry_msgs
  nav_core
  nav_msgs
  navfn               # Added for fallback planner
  pluginlib
  roscpp
  sensor_msgs         # Added for laser support
  tf2_ros            # Use tf2 instead of tf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

# Generate dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/WaypointPlanner.cfg
)

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    waypoint_global_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    tf2_ros
    visualization_msgs
)

# Library target
add_library(waypoint_global_planner 
  src/waypoint_global_planner.cpp
)

# Dependencies must come before target_link_libraries
add_dependencies(waypoint_global_planner 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg  # For dynamic reconfigure
)

target_link_libraries(waypoint_global_planner
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS waypoint_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  FILES plugin.xml  # Fixed from bgp_plugin.xml to plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
