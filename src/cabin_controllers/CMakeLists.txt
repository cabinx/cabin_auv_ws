cmake_minimum_required(VERSION 2.8.3)
project(cabin_controllers)
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cabin_msgs
  serial
  geometry_msgs
  dynamic_reconfigure
)

find_package(Ceres REQUIRED)
#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#include_directories($ {EIGEN3_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/VehicleProperties.cfg
)

catkin_package(
  INCLUDE_DIRS
     include
     ${EIGEN3_INCLUDE_DIR}
     ${CERES_INCLUDE_DIRS}
  CATKIN_DEPENDS
     dynamic_reconfigure
)

include_directories(include ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(serial_to_mcu src/serial_to_mcu.cpp)
target_link_libraries(serial_to_mcu
   ${catkin_LIBRARIES}
)
add_dependencies(serial_to_mcu cabin_msgs_gencpp)

add_executable(pwm_controller src/pwm_controller.cpp)
target_link_libraries(pwm_controller
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_dependencies(pwm_controller cabin_msgs_gencpp)

add_executable(thruster_controller src/thruster_controller.cpp)
target_link_libraries(thruster_controller ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
add_dependencies(thruster_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(thruster_controller cabin_controllers_gencfg)
add_dependencies(thruster_controller cabin_msgs_gencpp)

