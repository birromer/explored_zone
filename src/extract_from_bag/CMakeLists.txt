cmake_minimum_required(VERSION 2.8.3)
project(extract)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PROJ)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs  
  tf
  sensor_msgs
  std_msgs
  message_generation
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_generation message_runtime geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(extract src/extract.cpp)
target_link_libraries(extract ${catkin_LIBRARIES} ${PROJ_LIBRARIES})


