add_subdirectory(Controllers/AI)

find_package(OpenCV REQUIRED)

set(MAIN_HEADERS
	Controllers/ControllerBase.hpp
	Controllers/DijkstraController.hpp
	Controllers/ControllerBase.hpp
	Controllers/VisionController.hpp
	Controllers/LidarController.hpp
	Controllers/GpsController.hpp
	Controllers/DSLiteController.hpp
	#Controllers/DSLiteBController.hpp
	Controllers/sb_ai.h
)

set(MAIN_SOURCE
	Controllers/DijkstraController.cpp
	Controllers/VisionController.cpp
	Controllers/LidarController.cpp
	Controllers/GpsController.cpp
	Controllers/DSLiteController.cpp
	#Controllers/DSLiteBController.cpp
	main.cpp
)

set(MODE_LIBRARIES
	ai_dijkstra
	ai_utilities
	ai_dstarliteA
	#ai_dstarliteB
	${OpenCV_LIBS})

set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_ai ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_ai sb_msgs_generate_messages_cpp)
add_dependencies(sb_ai sb_gps_generate_messages_cpp)
target_link_libraries(sb_ai ${catkin_LIBRARIES} ${MODE_LIBRARIES})

add_executable(global_map Controllers/TestGlobal.cpp)
target_link_libraries(global_map ai_utilities ${catkin_LIBRARIES})
add_dependencies(global_map sb_msgs_generate_messages_cpp)


