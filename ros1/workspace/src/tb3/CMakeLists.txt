cmake_minimum_required(VERSION 2.8.3)
project(tb3)


catkin_package(CATKIN_DEPENDS
    message_runtime)

find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy std_msgs genmsg message_generation
)

catkin_package()
include_directories(
  include 
  ${catkin_INCLUDE_DIRS})
add_executable(tb3_node src/tb3ctrl.cpp)
target_link_libraries(tb3_node ${catkin_LIBRARIES})
