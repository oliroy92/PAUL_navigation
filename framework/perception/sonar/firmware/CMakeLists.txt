cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# more details available at: https://github.com/queezythegreat/arduino-cmake
generate_arduino_firmware(sonar
  SRCS sonar.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/ttyACM0
)