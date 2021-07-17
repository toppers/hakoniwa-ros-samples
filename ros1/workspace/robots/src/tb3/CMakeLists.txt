cmake_minimum_required(VERSION 2.8.3)
project(tb3)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  ros_tcp_endpoint
  message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
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
add_executable(tb3ctrl src/tb3ctrl/tb3ctrl.cpp)
target_link_libraries(tb3ctrl ${catkin_LIBRARIES})