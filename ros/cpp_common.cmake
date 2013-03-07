cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_BASE_PATH roscpp_core/cpp_common)
project(cpp_common)

include_directories(
	${CMAKE_BASE_PATH}/include/
)

add_library(cpp_common STATIC
	${CMAKE_BASE_PATH}/src/debug.cpp
)