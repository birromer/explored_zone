cmake_minimum_required(VERSION 2.8.3)
project(remote_control)

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

find_package(PROJ)

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

add_executable(robotControlLink src/robotControlLink.cpp src/protocol.cpp)
target_link_libraries(robotControlLink ${catkin_LIBRARIES})

add_executable(fieldCtrl src/fieldCtrl.cpp)
target_link_libraries(fieldCtrl ${catkin_LIBRARIES} ${PROJ_LIBRARIES})

add_executable(customMessage src/customMessage.cpp)
target_link_libraries(customMessage ${catkin_LIBRARIES})

