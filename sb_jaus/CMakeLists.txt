
cmake_minimum_required(VERSION 2.8)

project(sb_jaus)

ADD_DEFINITIONS(
        -std=c++11
        -Wall
)

add_executable(jaus_node src/main.cpp)

include_directories(OpenJAUS/include)

add_executable(jaus_node src/main.cpp)

find_library(OpenJAUS NAMES openjaus PATHS OpenJAUS/lib REQUIRED)

if(${OpenJAUS} EQUAL "NOTFOUND")
        message(SEND_ERROR "JAUS not found")
endif(${OpenJAUS} EQUAL "NOTFOUND")

target_link_libraries(jaus_node ${OpenJAUS})

