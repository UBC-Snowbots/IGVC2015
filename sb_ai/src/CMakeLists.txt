add_subdirectory(Controllers/AI)

set(MAIN_HEADERS
	Controllers/ControllerBase.hpp
	Controllers/DijkstraController.hpp
	Controllers/sb_ai.h)

set(MAIN_SOURCE
	Controllers/DijkstraController.cpp
	main.cpp)

set(MODE_LIBRARIES
	ai_dijkstra)

add_executable(sb_ai ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_ai sb_msgs_generate_messages_cpp)
add_library(dijkstra_rec Controllers/AI/dijkstra/dijkstra_recursive.cpp)
target_link_libraries(sb_ai ${catkin_LIBRARIES} ${MODE_LIBRARIES} dijkstra_rec)

