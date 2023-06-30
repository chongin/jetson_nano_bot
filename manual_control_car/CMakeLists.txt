cmake_minimum_required(VERSION 3.0.2)
project(manual_control_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)
include_directories(${catkin_INCLUDE_DIRS})
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(manual_control_node src/manual_control.cpp)
add_dependencies(manual_control_node ${manual_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manual_control_node ${catkin_LIBRARIES} ${SDL2_LIBRARIES})
