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

generate_messages(
  DEPENDENCIES
  std_msgs 
  sb_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp rospy std_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(hector_lidar_mapping src/HectorMapping/hector_lidar_mapping.cpp)
target_link_libraries( hector_lidar_mapping ${catkin_LIBRARIES})

add_executable(image_to_grid_test_publisher src/image_to_grid_test_publisher.cpp)
target_link_libraries(image_to_grid_test_publisher ${catkin_LIBRARIES})

add_executable(image_to_grid_test src/image_to_grid.hpp src/image_to_grid_test.cpp src/image_to_grid.cpp)
target_link_libraries(image_to_grid_test ${catkin_LIBRARIES})


