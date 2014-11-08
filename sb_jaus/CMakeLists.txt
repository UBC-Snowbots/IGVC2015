
cmake_minimum_required(VERSION 2.8)

project(sn_jaus)

ADD_DEFINITIONS(
        -std=c++11
        -Wall
)

include_directories(JAUS++/src)

add_executable(jaus_node src/main.cpp)

#find_library(Boost 1.53 REQUIRED
#        COMPONENTS system)
find_library(Boost NAMES boost_system REQUIRED)
find_library(JAUS NAMES JAUSCore CxUtils PATHS JAUS++/lib REQUIRED)

if(${Boost} EQUAL "NOTFOUND")
        message(SEND_ERROR "Boost not found")
endif(${Boost} EQUAL "NOTFOUND")
if(${JAUS} EQUAL "NOTFOUND")
        message(SEND_ERROR "JAUS not found")
endif(${JAUS} EQUAL "NOTFOUND")

target_link_libraries(jaus_node ${Boost})
target_link_libraries(jaus_node ${JAUS})

