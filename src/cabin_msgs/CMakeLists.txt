cmake_minimum_required(VERSION 2.8.3)
project(cabin_msgs)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   Thrust.msg
   ThrustStamped.msg
   Pwm.msg
   PwmStamped.msg
   SwitchState.msg
   ResetControls.msg
   Depth.msg
   DepthCommand.msg
   LinearCommand.msg
   AttitudeCommand.msg
   Imu.msg
   ControlStatus.msg
   NetLoad.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
