cmake_minimum_required(VERSION 2.8.3)
project(sb_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
  message_generation
)

catkin_python_setup()

generate_messages(
	DEPENDENCIES
	std_msgs
	sb_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/gps_receiver.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## To add package paths:
## source ~/catkin_ws/devel/setup.bash
