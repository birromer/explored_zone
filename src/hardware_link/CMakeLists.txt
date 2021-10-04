cmake_minimum_required(VERSION 2.8.3)
project(hardware_link)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_link src/arduino.cpp)
add_executable(gnss_inertial_link src/gnss_inertial.cpp)
target_link_libraries(arduino_link ${catkin_LIBRARIES})
target_link_libraries(gnss_inertial_link ${catkin_LIBRARIES})


