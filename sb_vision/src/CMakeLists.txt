find_package(OpenCV REQUIRED)

add_library(filter filter.h filter.cpp)

#add_library(camera Camera.h Camera.cpp)

add_library(ipm IPM.h IPM.cpp)

#When adding class files, add them here for any other to include it
set(MAIN_HEADERS
	filter.h
	IPM.h
	Camera.h
)

set(MAIN_SOURCE
	vision.cpp
	Camera.cpp
)


set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_vision ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_vision sb_msgs_generate_messages_cpp)
target_link_libraries(sb_vision ${catkin_LIBRARIES} ${OpenCV_LIBS} filter ipm)

