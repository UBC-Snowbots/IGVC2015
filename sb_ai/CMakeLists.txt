cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
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





