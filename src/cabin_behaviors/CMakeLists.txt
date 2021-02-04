cmake_minimum_required(VERSION 3.0.2)
project(cabin_behaviors)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cabin_msgs
  darknet_ros_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/pid_tracking.cfg
)

catkin_package(
  CATKIN_DEPENDS
     dynamic_reconfigure
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(pid_tracking src/pid_tracking.cpp)

target_link_libraries(pid_tracking
   ${catkin_LIBRARIES}
)
add_dependencies(pid_tracking ${catkin_EXPORTED_TARGETS})
add_dependencies(pid_tracking cabin_behaviors_gencfg)
add_dependencies(pid_tracking cabin_msgs_gencpp)
add_dependencies(pid_tracking darknet_ros_msgs_gencpp)
