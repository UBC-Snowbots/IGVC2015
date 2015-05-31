cmake_minimum_required(VERSION 2.8.3)
project(sb_lidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
  message_runtime
  #gazebo_ros
)

#find_package(gazebo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE})

catkin_package(   
	CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs message_runtime #gazebo_ros  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(imagine_lidar src/imagineday_lidar.cpp)
target_link_libraries(imagine_lidar ${catkin_LIBRARIES})
add_dependencies(imagine_lidar sb_msgs_generate_messages_cpp)

add_executable(lidar_v1 src/lidar_v1.cpp)
target_link_libraries(lidar_v1 ${catkin_LIBRARIES})
add_dependencies(lidar_v1 sb_msgs_generate_messages_cpp)

add_executable(local_lidar_map src/LocalLidarMap/LocalLidarMap.cpp)
target_link_libraries(local_lidar_map ${catkin_LIBRARIES})

#add_executable(demo_lidar src/demo_code.cpp)
#target_link_libraries(demo_lidar ${catkin_LIBRARIES})
#add_executable(lidar_force_nav src/lidar_force_nav.cpp)
#target_link_libraries(lidar_force_nav
#   ${catkin_LIBRARIES}
#)


