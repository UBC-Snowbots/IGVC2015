cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)


add_message_files(
  FILES
  AISimMap.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dstarlite src/sb_dstarlite.cpp)
target_link_libraries(dstarlite ${catkin_LIBRARIES})
