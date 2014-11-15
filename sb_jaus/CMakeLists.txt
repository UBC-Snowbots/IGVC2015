
cmake_minimum_required(VERSION 2.8)

project(sb_jaus)

ADD_DEFINITIONS(
        -std=c++11
        -Wall
)

include_directories(OpenJAUS/include)


add_executable(jaus_node src/main.cpp src/JAUSComponent.cpp)

find_library(OpenJAUS NAMES openjaus PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Core NAMES openjaus-core PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Mobility NAMES openjaus-mobility PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Environment NAMES openjaus-environment PATHS OpenJAUS/lib REQUIRED)
find_library(Json NAMES json PATHS OpenJAUS/lib REQUIRED)

if(${OpenJAUS} EQUAL "NOTFOUND")
        message(SEND_ERROR "JAUS not found")
endif(${OpenJAUS} EQUAL "NOTFOUND")

target_link_libraries(jaus_node ${OpenJAUS})
target_link_libraries(jaus_node ${OpenJAUS_Core})
target_link_libraries(jaus_node ${OpenJAUS_Mobility})
target_link_libraries(jaus_node ${OpenJAUS_Environment})
target_link_libraries(jaus_node ${Json})

