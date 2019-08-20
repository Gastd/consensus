cmake_minimum_required(VERSION 2.8.3)
project(consensus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES consensus
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
