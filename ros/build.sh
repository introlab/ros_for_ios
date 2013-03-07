#!/bin/sh -x

ROS_BRANCH=groovy-devel

#git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros.git

#git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
#git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git

./cmake_gen.sh roscpp_core cpp_common
./cmake_gen.sh roscpp_core roscpp_serialization boost
./cmake_gen.sh roscpp_core roscpp_traits boost
./cmake_gen.sh roscpp_core rostime boost

./cmake_gen.sh ros_comm/utilities xmlrpcpp 
./cmake_gen.sh ros_comm/clients roscpp boost
./cmake_gen.sh ros_comm/tools rosconsole boost log4cxx

./cmake_gen.sh ros/core roslib boost

cat > ./CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_TOOLCHAIN_FILE 
	${CMAKE_CURRENT_SOURCE_DIR}/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake)

#set (CMAKE_SYSTEM_FRAMEWORK_PATH ${CMAKE_SYSTEM_FRAMEWORK_PATH}
#	"/Users/Ronan/Documents/Xcode_workspace/frameworks")
	
project(ros_for_ios)

include(cpp_common.cmake)
include(roscpp_serialization.cmake)
include(roscpp_traits.cmake)
include(rostime.cmake)

include(xmlrpcpp.cmake)
include(roscpp.cmake)
include(rosconsole.cmake)

include(roslib.cmake)
EOF


