cmake_minimum_required(VERSION 3.0.2)
project(robot_diagnostic)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  IndividualState.msg
  SystemStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include/robot_diagnostic
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater roscpp rospy std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_diagnostic 
  src/main_monitor.cpp
)

target_link_libraries(robot_diagnostic
  ${catkin_LIBRARIES}
)
