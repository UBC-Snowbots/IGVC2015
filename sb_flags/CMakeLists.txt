cmake_minimum_required(VERSION 2.8.3)
project(sb_flags)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sb_msgs
  std_msgs
  genmsg
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sb_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sb_flags_node src/flag_detector.cpp)

 target_link_libraries(sb_flags_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )
