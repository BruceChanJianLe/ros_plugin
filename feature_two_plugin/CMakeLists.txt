cmake_minimum_required(VERSION 3.0.2)
project(feature_two_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  base_plugin
  pluginlib
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES feature_two_plugin
#  CATKIN_DEPENDS base_plugin pluginlib roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(feature_two src/feature_two_plugin.cpp)

add_dependencies(feature_two ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_two ${catkin_LIBRARIES} )
install(TARGETS feature_two
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)