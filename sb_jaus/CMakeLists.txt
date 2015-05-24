
cmake_minimum_required(VERSION 2.8)

project(sb_jaus)

#ADD_DEFINITIONS( -Wall )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_CUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_CUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

include_directories(JAUS++/src)

set(JAUS_LIBS
	JAUS++/lib/libCxUtils.so
	JAUS++/lib/libJAUSCore.so
	JAUS++/lib/libJAUSMobility.so
	JAUS++/lib/libJAUSEnvironment.so
)

set(BUILD_JAUS_ONLY yes)
set(ACTIVE_PROJECT_ROOT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/JAUS++)
add_subdirectory(JAUS++)

add_executable(jaus_server src/server.cpp)

add_executable(jaus_judge src/judge.cpp)

#find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(jaus_server JAUSCore JAUSEnvironment JAUSMobility CxUtils)
target_link_libraries(jaus_judge JAUSCore JAUSEnvironment JAUSMobility CxUtils)

