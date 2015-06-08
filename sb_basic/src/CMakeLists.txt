

find_package(OpenCV REQUIRED)

set(MAIN_HEADERS
	Controllers/VisionController.hpp
	Controllers/LidarController.hpp
	Controllers/GpsController.hpp
	Controllers/sb_ai.h
)

set(MAIN_SOURCE
	Controllers/VisionController.cpp
	Controllers/LidarController.cpp
	Controllers/GpsController.cpp
	main.cpp
)

set(MODE_LIBRARIES
	${OpenCV_LIBS})

set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_basic ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_basic sb_msgs_generate_messages_cpp)
target_link_libraries(sb_basic ${catkin_LIBRARIES} ${MODE_LIBRARIES})




