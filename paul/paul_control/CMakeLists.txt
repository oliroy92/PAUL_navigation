cmake_minimum_required(VERSION 2.8.3)
project(paul_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  Elevation.msg
)

add_service_files(
  FILES
  ElevationPosition.srv
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)