cmake_minimum_required(VERSION 3.0.2)
project(robot_control_kat)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Qt5 COMPONENTS Core Gui Widgets Quick Network QuickWidgets Positioning)

find_package(Qt5QuickCompiler)

find_package(PROJ QUIET)
if(PROJ VERSION_LESS "4.0")
        find_package(PROJ4 REQUIRED)
        set(PROJ_LIBRARIES ${PROJ4_LIBRARIES})
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tp3
  CATKIN_DEPENDS
    geometry_msgs
    message_generation
    message_runtime
    roscpp
    rospy
    std_msgs
    tf
    tf2
    visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_executable(robotcontrol main.cpp robotcontrol.cpp protocol.cpp serial/serial.cc serial/impl/unix.cc serial/impl/win.cc serial/impl/list_ports/list_ports_linux.cc serial/impl/list_ports/list_ports_win.cc)

qt5_use_modules(robotcontrol Core Gui Widgets Quick Network QuickWidgets Positioning)

target_link_libraries(robotcontrol ${PROJ_LIBRARIES} ${catkin_LIBRARIES})

if(WIN32)
        target_link_libraries(robotcontrol Setupapi) # For serial.
endif()
