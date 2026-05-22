cmake_minimum_required(VERSION 3.10)
project(basicWalk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(unitree_sdk2 REQUIRED)

add_executable(test basicWalk.cpp)
target_link_libraries(test PRIVATE unitree_sdk2)