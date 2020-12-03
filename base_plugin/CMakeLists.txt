cmake_minimum_required(VERSION 3.0.2)
project(base_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES base_plugin
#  CATKIN_DEPENDS pluginlib roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  user_plugin_node
  src/user_plugin_node.cpp
  src/user_plugin.cpp
)
target_link_libraries(user_plugin_node ${catkin_LIBRARIES})