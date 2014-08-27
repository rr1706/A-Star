project(astar)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(astar_demo main.cpp astar.cpp)
add_library(astar astar.cpp)
SET_TARGET_PROPERTIES(astar_demo PROPERTIES OUTPUT_NAME astar)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # for gcc >= 4.7
