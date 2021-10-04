cmake_minimum_required(VERSION 2.8.3)
project(wayfinder_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs  
  std_msgs
  serial
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_executable(wayfinder src/wayfinder.cpp src/driver.cpp)
target_link_libraries(wayfinder ${catkin_LIBRARIES})

