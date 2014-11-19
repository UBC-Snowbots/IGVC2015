
cmake_minimum_required(VERSION 2.8)

project(sb_jaus)

ADD_DEFINITIONS(
        -std=c++11
        -Wall
)

include_directories(OpenJAUS/include)


add_executable(jaus_test_client JAUSJudgeClient.cpp)

find_library(OpenJAUS NAMES openjaus PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Core NAMES openjaus-core PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Mobility NAMES openjaus-mobility PATHS OpenJAUS/lib REQUIRED)
find_library(OpenJAUS_Environment NAMES openjaus-environment PATHS OpenJAUS/lib REQUIRED)
find_library(Json NAMES json PATHS OpenJAUS/lib REQUIRED)

if(${OpenJAUS} EQUAL "NOTFOUND")
        message(SEND_ERROR "JAUS not found")
endif(${OpenJAUS} EQUAL "NOTFOUND")

target_link_libraries(jaus_test_client ${OpenJAUS})
target_link_libraries(jaus_test_client ${OpenJAUS_Core})
target_link_libraries(jaus_test_client ${OpenJAUS_Mobility})
target_link_libraries(jaus_test_client ${OpenJAUS_Environment})
target_link_libraries(jaus_test_client ${Json})

