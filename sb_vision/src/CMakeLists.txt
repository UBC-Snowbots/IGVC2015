find_package(OpenCV REQUIRED)
add_library(filter filter.h filter.cpp)
set(MAIN_HEADERS
	filter.h
)

set(MAIN_SOURCE
	vision.cpp
)


set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_vision ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_vision sb_msgs_generate_messages_cpp)
target_link_libraries(sb_vision ${catkin_LIBRARIES} ${OpenCV_LIBS} filter)

