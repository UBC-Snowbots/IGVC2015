find_package(OpenCV REQUIRED)

add_library(filter filter.h filter.cpp)

add_library(image_stitcher image_stitcher.h image_stitcher.cpp)

add_library(ipm IPM.h IPM.cpp)

set(MAIN_HEADERS
	filter.h
	IPM.h
)

set(MAIN_SOURCE
	vision.cpp
)


set(MODE_INCLUDE_DIRS
	${OpenCV_INCLUDE_DIRS})

include_directories( ${MODE_INCLUDE_DIRS} )

add_executable(sb_vision ${MAIN_HEADERS} ${MAIN_SOURCE})
add_dependencies(sb_vision sb_msgs_generate_messages_cpp)
target_link_libraries(sb_vision ${catkin_LIBRARIES} ${OpenCV_LIBS} filter ipm)

