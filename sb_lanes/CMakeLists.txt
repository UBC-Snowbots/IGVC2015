cmake_minimum_required(VERSION 2.8.3)
project(sb_lanes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sb_msgs
)
find_package(OpenCV REQUIRED)
add_message_files(FILES Num.msg)

generate_messages(
   DEPENDENCIES
   std_msgs sb_msgs 
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(LaneDetector src/LaneDetector.cpp)
target_link_libraries(LaneDetector ${catkin_LIBRARIES} ${OpenCV_LIBS})


