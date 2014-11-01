cmake_minimum_required(VERSION 2.8.3)
project(sb_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Num.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(Line_Detector src/Line_Detector.cpp)

target_link_libraries( Line_Detector ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(Line_Detector sb_vision_generate_messages_cpp)


