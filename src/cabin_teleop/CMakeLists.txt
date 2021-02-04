cmake_minimum_required(VERSION 2.8.3)
project(cabin_teleop)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cabin_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(t4_controller src/t4_controller.cpp)
target_link_libraries(t4_controller
   ${catkin_LIBRARIES}
)
add_dependencies(t4_controller cabin_msgs_gencpp)

add_executable(logicool_controller src/logicool_controller.cpp)
target_link_libraries(logicool_controller
   ${catkin_LIBRARIES}
)
add_dependencies(logicool_controller cabin_msgs_gencpp)

