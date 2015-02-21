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
 ${PROJECT_BINARY_DIR}
)

add_executable(Line_Detector src/Line_Detector.cpp)
add_executable(LaneDetector src/LaneDetector.cpp)
add_executable(LineFilter src/LineFilter.cpp)
add_executable(test_code src/test_code.cpp)
add_executable(Jan7thTest src/Jan7thTest.cpp)
add_executable( test1 src/test1.cpp)
add_executable(image_stitching src/image_stitching.cpp)
add_executable(jan11 src/jan11.cpp)
add_executable(CameraCal src/Perspective_Transform/Camera_Cal/sample_camera_calibration.cpp)
add_executable( ipm src/Perspective_Transform/ipm_code/main.cpp 
			src/Perspective_Transform/ipm_code/IPM.cpp 
			src/Perspective_Transform/ipm_code/IPM.h)

target_link_libraries( Line_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( LaneDetector  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( LineFilter    ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( test_code ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( Jan7thTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(  test1 ${catkin_LIBRARIES})
target_link_libraries( image_stitching ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( jan11 ${catkin_LIBRARIES})
target_link_libraries( CameraCal ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( ipm ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(Line_Detector sb_vision_generate_messages_cpp)


