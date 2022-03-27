cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_library(NewPing
  SRCS NewPing/NewPing.cpp
  HDRS NewPing/NewPing.h
  BOARD mega2560
)

# more details available at: https://github.com/queezythegreat/arduino-cmake
generate_arduino_firmware(sonar
  SRCS firmware.cpp ${ROS_LIB_DIR}/time.cpp
  LIBS NewPing
  BOARD mega2560
  PORT /dev/ttyACM0
)