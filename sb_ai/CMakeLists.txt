cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dstarlite src/sb_dstarlite.cpp)
target_link_libraries(dstarlite ${catkin_LIBRARIES})
