cmake_minimum_required(VERSION 2.8.3)
project(sb_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_library(sb_serial_communication
   src/SerialCommunication.cpp
 )

 add_library(sb_driver_node_headers src/arduino_driver.cpp)
 add_executable(sb_driver_node src/arduino_driver.cpp)
 add_dependencies(sb_driver_node sb_msgs_generate_messages_cpp)

 target_link_libraries(
  sb_driver_node 
  sb_serial_communication
  sb_driver_node_headers
  ${catkin_LIBRARIES}
 )


