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

add_executable(
  plugin_loader_node
  src/plugin_loader_node.cpp
  src/plugin_loader.cpp
)
target_link_libraries(plugin_loader_node ${catkin_LIBRARIES})

## Unit Test
# http://wiki.ros.org/gtest
catkin_add_gtest(
  plugin_test
  test/plugin_test.cpp
)
if(TARGET plugin_test)
  target_link_libraries(
    plugin_test
    ${catkin_LIBRARIES}
  )
endif()