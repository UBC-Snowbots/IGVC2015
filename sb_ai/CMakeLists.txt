cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sb_msgs
  cv_bridge
  image_transport
  nav_msgs
)
add_service_files(
	FILES
	Gps_Service.srv
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(global_map src/Controllers/TestGlobal.cpp)
target_link_libraries(global_map ai_utilities ${catkin_LIBRARIES})
add_dependencies(global_map sb_msgs_generate_messages_cpp)





