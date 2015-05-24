add_subdirectory(Controllers/AI)

find_package(OpenCV REQUIRED)

set(MAIN_HEADERS
<<<<<<< HEAD
	Controllers/ControllerBase.hpp
	Controllers/DijkstraController.hpp
#	ControllerBase.hpp
#	DijkstraController.hpp
	VisionController.hpp
	LidarController.hpp
	Controllers/sb_ai.h
#	sb_ai.h
	)

set(MAIN_SOURCE
	Controllers/DijkstraController.cpp
#	DijkstraController.cpp
	VisionController.cpp
<<<<<<< HEAD
  LidarController.cpp
	sb_gps.cpp
=======
	LidarController.cpp
>>>>>>> 096c6d06cb28d11c3ffe97fe3f6802d78e3aab71
	main.cpp)

set(MODE_LIBRARIES
	ai_dijkstra
	${OpenCV_LIBS})

set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_ai ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_ai sb_msgs_generate_messages_cpp)
add_library(dijkstra_rec Controllers/AI/dijkstra/dijkstra_recursive.cpp)
target_link_libraries(sb_ai ${catkin_LIBRARIES} ${MODE_LIBRARIES} dijkstra_rec)

