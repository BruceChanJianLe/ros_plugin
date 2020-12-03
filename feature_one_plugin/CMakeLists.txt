cmake_minimum_required(VERSION 3.0.2)
project(feature_one_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  base_plugin
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES feature_one_plugin
#  CATKIN_DEPENDS pluginlib roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(feature_one src/feature_one_plugin.cpp)

add_dependencies(feature_one ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_one ${catkin_LIBRARIES} )
install(TARGETS feature_one
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
