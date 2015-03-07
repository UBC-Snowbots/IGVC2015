cmake_minimum_required(VERSION 2.8.3)
project(sb_waypoint_manager)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sb_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs sb_gps message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sb_waypoint_manager src/main.cpp src/WaypointManager.cpp)
add_dependencies(sb_waypoint_manager sb_gps_generate_messages_cpp)
add_dependencies(sb_waypoint_manager sb_msgs_generate_messages_cpp)

target_link_libraries(sb_waypoint_manager ${catkin_LIBRARIES})


## To add package paths:
## source ~/catkin_ws/devel/setup.bash
