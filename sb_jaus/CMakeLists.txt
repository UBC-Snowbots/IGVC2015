
cmake_minimum_required(VERSION 2.8)

project(sb_jaus)

ADD_DEFINITIONS(
        -std=c++11
        -Wall
)

include_directories(JAUS++/src)

add_executable(jaus_server src/server.cpp)
add_executable(jaus_judge src/judge.cpp)

find_library(jaus_pp_core NAMES JAUSCore PATHS JAUS++/lib REQUIRED)
find_library(jaus_pp_environment NAMES JAUSEnvironment PATHS JAUS++/lib REQUIRED)
find_library(jaus_pp_mobility NAMES JAUSMobility PATHS JAUS++/lib REQUIRED)
find_library(jaus_pp_cxutils NAMES CxUtils PATHS JAUS++/lib REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(jaus_server ${jaus_pp_core} ${jaus_pp_environment} ${jaus_pp_mobility} ${jaus_pp_cxutils} ${Boost_LIBRARIES})
target_link_libraries(jaus_judge ${jaus_pp_core} ${jaus_pp_environment} ${jaus_pp_mobility} ${jaus_pp_cxutils} ${Boost_LIBRARIES})

