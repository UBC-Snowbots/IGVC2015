cmake_minimum_required(VERSION 2.8.3)
project(sb_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dstarlite src/sb_dstarlite.cpp)
target_link_libraries(dstarlite ${catkin_LIBRARIES})
add_dependencies(dstarlite sb_msgs_generate_messages_cpp)

add_executable(dijkstra src/sb_dijkstra.cpp)
target_link_libraries(dijkstra ${catkin_LIBRARIES})
add_dependencies(dijkstra sb_msgs_generate_messages_cpp)

add_executable(sim_map src/map_sim.cpp)
target_link_libraries(sim_map ${catkin_LIBRARIES})
add_dependencies(sim_map sb_msgs_generate_messages_cpp)

add_executable(sim_gps src/gps_sim.cpp)
target_link_libraries(sim_gps ${catkin_LIBRARIES})
add_dependencies(sim_gps sb_msgs_generate_messages_cpp)

add_executable(test_output src/test_output.cpp)
target_link_libraries(test_output ${catkin_LIBRARIES})

add_executable(simple_mapping src/AI_Mapping/simple_mapping.cpp)
target_link_libraries(simple_mapping ${catkin_LIBRARIES})
add_dependencies(simple_mapping ${PROJECT_NAME}_gencpp)

add_executable(simple_ai src/AI_Mapping/simple_ai.cpp)
target_link_libraries(simple_ai ${catkin_LIBRARIES})
# catkin_make should work without the -j1 flag with this
add_dependencies(simple_ai ${PROJECT_NAME}_gencpp)

# add_executable(simple_mapping_justin src/AI_Mapping/simple_mapping_justin.cpp)
# target_link_libraries(simple_mapping_justin ${catkin_LIBRARIES})
# add_dependencies(simple_mapping_justin ${PROJECT_NAME}_gencpp)

# add_executable(simple_mapping_justin_test src/AI_Mapping/simple_mapping_justin_test.cpp)
# target_link_libraries(simple_mapping_justin_test ${catkin_LIBRARIES})
# add_dependencies(simple_mapping_justin_test ${PROJECT_NAME}_gencpp)

add_subdirectory(src)

