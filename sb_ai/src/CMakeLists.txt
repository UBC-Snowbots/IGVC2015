add_subdirectory(AI)

set(MAIN_HEADERS
	ControllerBase.hpp
	DijkstraController.hpp
	sb_ai.h)

set(MAIN_SOURCE
	DijkstraController.cpp
	main.cpp)

set(MODE_LIBRARIES
	ai_dijkstra)

add_executable(sb_ai ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_ai sb_msgs_generate_messages_cpp)
target_link_libraries(sb_ai ${catkin_LIBRARIES} ${MODE_LIBRARIES})

