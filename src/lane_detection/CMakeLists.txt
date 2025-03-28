cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy cv_bridge sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/lane_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

