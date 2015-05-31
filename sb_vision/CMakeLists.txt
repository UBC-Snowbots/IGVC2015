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
add_subdirectory(src)

add_executable(Line_Detector src/Line_Detector.cpp)
target_link_libraries( Line_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(LaneDetector src/LaneDetector.cpp)
target_link_libraries( LaneDetector  ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(LineFilter src/LineFilter.cpp)
target_link_libraries( LineFilter    ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(image_stitching_simon src/image_stitching_simon.cpp)
target_link_libraries( image_stitching_simon ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(image_stitching src/image_stitching.cpp)
target_link_libraries( image_stitching ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(CameraCal src/Perspective_Transform/Camera_Cal/sample_camera_calibration.cpp)
target_link_libraries( CameraCal ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ObjectDetect src/ObjectDetect.cpp)
target_link_libraries( ObjectDetect ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable( ipm src/Perspective_Transform/ipm_code/main.cpp 
			src/Perspective_Transform/ipm_code/IPM.cpp 
			src/Perspective_Transform/ipm_code/IPM.h)
target_link_libraries( ipm ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable( Pylon_Detector src/Pylon_Detector/Pylon_Detector.cpp )
target_link_libraries( Pylon_Detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable( vision src/vision.cpp )
target_link_libraries( vision ${catkin_LIBRARIES} ${OpenCV_LIBS} filter)

#add_executable( filter src/filter.cpp )
#target_link_libraries( filter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(LaneDetector sb_msgs_generate_messages_cpp)

add_dependencies(Line_Detector sb_vision_generate_messages_cpp)




