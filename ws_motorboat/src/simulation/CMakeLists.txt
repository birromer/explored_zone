cmake_minimum_required(VERSION 2.8.3)
project(simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs  
  tf
  sensor_msgs
  std_msgs
)

find_package(PROJ)


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

add_executable(simulation src/simulation.cpp src/vibes.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES} ${PROJ_LIBRARIES})



