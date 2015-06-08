cmake_minimum_required(VERSION 2.8.3)
project(sb_mapping)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sb_msgs
)

add_subdirectory(src/Mapping)

generate_messages(
  DEPENDENCIES
  std_msgs 
  sb_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp rospy std_msgs sb_msgs 
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(sb_mapping src/main.cpp)
target_link_libraries(sb_mapping ${catkin_LIBRARIES} mapping)
add_dependencies(sb_mapping sb_msgs_generate_messages_cpp)

#add_executable(image_to_grid_test_publisher src/image_to_grid_test_publisher.cpp)
#target_link_libraries(image_to_grid_test_publisher ${catkin_LIBRARIES})

#add_executable(image_to_grid_test src/image_to_grid.hpp src/image_to_grid_test.cpp src/image_to_grid.cpp)
#target_link_libraries(image_to_grid_test ${catkin_LIBRARIES})


