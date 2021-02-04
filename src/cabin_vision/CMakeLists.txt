cmake_minimum_required(VERSION 3.0.2)
project(cabin_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  darknet_ros_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cabin_vision
#  CATKIN_DEPENDS roscpp rosmsgs rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_deviation src/object_deviation.cpp)

target_link_libraries(object_deviation
   ${catkin_LIBRARIES}
   yaml-cpp
)

add_dependencies(object_deviation darknet_ros_msgs_gencpp)

