cmake_minimum_required(VERSION 2.8.3)
project(paul_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
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