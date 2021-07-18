cmake_minimum_required(VERSION 2.8.3)
project(ev3)

catkin_package(CATKIN_DEPENDS
	ev3_msgs
    message_runtime)

find_package(catkin REQUIRED COMPONENTS 
  roscpp ev3_msgs std_msgs genmsg message_generation
)

catkin_package(CATKIN_DEPENDS
    message_runtime)

catkin_package()
include_directories(
  include 
  ${catkin_INCLUDE_DIRS})
add_executable(ev3_node src/ev3ctrl.cpp)
target_link_libraries(ev3_node ${catkin_LIBRARIES})
