cmake_minimum_required(VERSION 2.8.3)
project(sb_vision)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sb_msgs
  cv_bridge
  image_transport
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
add_executable(image_stitching_simon src/image_stitching_simon.cpp)
add_executable(stitch_sub src/stitch_sub.cpp)
add_executable(stitch_pub src/stitch_pub.cpp)
add_executable(image_stitching src/image_stitching.cpp)
add_executable(CameraCal src/Perspective_Transform/Camera_Cal/sample_camera_calibration.cpp)
add_executable(ObjectDetect src/ObjectDetect.cpp)
add_executable( ipm src/Perspective_Transform/ipm_code/main.cpp 
			src/Perspective_Transform/ipm_code/IPM.cpp 
			src/Perspective_Transform/ipm_code/IPM.h)
add_executable( Pylon_Detector src/Pylon_Detector/Pylon_Detector.cpp )

target_link_libraries( Line_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( LaneDetector  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( LineFilter    ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( image_stitching ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( image_stitching_simon ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( stitch_sub ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( stitch_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries( CameraCal ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( ObjectDetect ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( ipm ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( Pylon_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(LaneDetector sb_msgs_generate_messages_cpp)

add_dependencies(Line_Detector sb_vision_generate_messages_cpp)


