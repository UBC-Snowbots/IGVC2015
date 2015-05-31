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

add_executable(simple_mapping src/AI_Mapping/simple_mapping.cpp)
target_link_libraries(simple_mapping ${catkin_LIBRARIES})
#add_dependencies(simple_mapping ${PROJECT_NAME}_gencpp)
add_dependencies(simple_mapping sb_msgs_generate_messages_cpp sb_msgs_generate_messages_cpp)

add_executable(simple_ai src/AI_Mapping/simple_ai.cpp)
target_link_libraries(simple_ai ${catkin_LIBRARIES})
# catkin_make should work without the -j1 flag with this
add_dependencies(simple_ai ${PROJECT_NAME}_gencpp sb_msgs_generate_messages_cpp)




