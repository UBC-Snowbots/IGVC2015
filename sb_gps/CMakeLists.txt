cmake_minimum_required(VERSION 2.8.3)
project(sb_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
  message_generation
)

add_service_files(
	FILES
	GotoWaypoint.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sb_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs message_runtime
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(sb_gps src/gps.cpp)
add_dependencies(sb_gps sb_msgs_generate_messages_cpp)
## add_executable(createAngle src/angle.c)

target_link_libraries(sb_gps ${catkin_LIBRARIES})
#target_link_libraries(createAngle ${catkin_LIBRARIES})

## To add package paths:
## source ~/catkin_ws/devel/setup.bash
