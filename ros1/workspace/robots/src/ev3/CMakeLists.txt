cmake_minimum_required(VERSION 2.8.3)
project(ev3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
)

add_message_files(DIRECTORY msg)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(CATKIN_DEPENDS
    ros_tcp_endpoint
    message_runtime)

catkin_install_python(PROGRAMS
  scripts/server_endpoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy std_msgs genmsg message_generation
)

catkin_package()
include_directories(
  include 
  ${catkin_INCLUDE_DIRS})
add_executable(ev3ctrl src/ev3ctrl/ev3ctrl.cpp)
target_link_libraries(ev3ctrl ${catkin_LIBRARIES})