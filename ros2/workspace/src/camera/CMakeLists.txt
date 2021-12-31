cmake_minimum_required(VERSION 3.5)
project(camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rule_msgs REQUIRED)
#find_package(OpenCV REQUIRED)
#message(STATUS ${OpenCV_INCLUDE_DIRS})
#message(STATUS ${OpenCV_LIBS})

add_executable(camera_node 
	src/camera.cpp
	src/camera_image.cpp
	#src/camera_movie.cpp
	src/camera_mqtt.cpp
)
target_include_directories(camera_node
	PRIVATE	${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(camera_node 
	rclcpp 
	std_msgs 
	geometry_msgs 
	sensor_msgs 
  rule_msgs
)
target_link_libraries(camera_node
	#${OpenCV_LIBRARIES}
  mosquitto
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
