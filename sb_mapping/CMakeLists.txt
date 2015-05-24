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

add_executable(hector_lidar_mapping src/hector_lidar_mapping.cpp)
target_link_libraries( hector_lidar_mapping ${catkin_LIBRARIES})

add_executable(local_lidar_map src/HectorMappingHack/LocalLidarMap.cpp)
target_link_libraries( local_lidar_map ${catkin_LIBRARIES})
