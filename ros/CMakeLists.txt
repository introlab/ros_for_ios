cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake)

project(ros_for_ios)

include(cpp_common.cmake)

message("${CMAKE_CXX_LINK_FLAGS}")