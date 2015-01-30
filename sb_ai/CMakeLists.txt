cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dstarlite src/sb_dstarlite.cpp)
target_link_libraries(dstarlite ${catkin_LIBRARIES})

add_executable(dijkstra src/sb_dijkstra.cpp)
target_link_libraries(dijkstra ${catkin_LIBRARIES})
