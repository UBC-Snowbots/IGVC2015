cmake_minimum_required(VERSION 2.8.3)
project(sb_vision)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sb_msgs
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Num.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  sb_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime roscpp rospy std_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(Line_Detector src/Line_Detector.cpp)
add_executable(LaneDetector src/LaneDetector.cpp)
add_executable(test_code src/test_code.cpp)
add_executable(Jan7thTest src/Jan7thTest.cpp)

target_link_libraries( Line_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( LaneDetector  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( test_code ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( Jan7thTest ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(Line_Detector sb_vision_generate_messages_cpp)


