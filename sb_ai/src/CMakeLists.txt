add_subdirectory(AI)

find_package(OpenCV REQUIRED)

set(MAIN_HEADERS
	ControllerBase.hpp
	DijkstraController.hpp
	VisionController.hpp
  LidarController.hpp
	sb_ai.h)

set(MAIN_SOURCE
	DijkstraController.cpp
	VisionController.cpp
  LidarController.cpp
	main.cpp)

set(MODE_LIBRARIES
	ai_dijkstra
	${OpenCV_LIBS})

set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_ai ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_ai sb_msgs_generate_messages_cpp)
target_link_libraries(sb_ai ${catkin_LIBRARIES} ${MODE_LIBRARIES})

