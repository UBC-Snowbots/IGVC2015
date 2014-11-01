cmake_minimum_required(VERSION 2.8.3)
project(sb_commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(sb_commander_node src/sb_commander_node.cpp)
add_executable(imagine_commander src/sb_imagineday_commander.cpp)

 target_link_libraries(sb_commander_node
   ${catkin_LIBRARIES}
 )
target_link_libraries(imagine_commander ${catkin_LIBRARIES})
