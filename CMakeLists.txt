cmake_minimum_required(VERSION 3.0.2)
project(simple_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  tf2_ros
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES simple_planner
  CATKIN_DEPENDS roscpp std_msgs nav_msgs tf
  #  DEPENDS system_lib
  )
  
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(planner_node
  src/planner_node.cpp
  src/map.cpp
)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)


