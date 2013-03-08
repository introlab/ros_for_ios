#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
ROS_BRANCH=groovy-devel

#===============================================================================
echo "Cloning git repositories ..."

#git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros.git

#git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
#git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git

#===============================================================================
echo "Generating make submodules ..."

$SRCDIR/cmake_gen.sh roscpp_core cpp_common
$SRCDIR/cmake_gen.sh roscpp_core roscpp_serialization boost
$SRCDIR/cmake_gen.sh roscpp_core roscpp_traits boost
$SRCDIR/cmake_gen.sh roscpp_core rostime boost

$SRCDIR/cmake_gen.sh ros_comm/utilities xmlrpcpp 
$SRCDIR/cmake_gen.sh ros_comm/clients roscpp boost log4cxx
$SRCDIR/cmake_gen.sh ros_comm/tools rosconsole boost log4cxx

$SRCDIR/cmake_gen.sh ros/core roslib boost

#===============================================================================
echo "Patching ..."

#patch $SRCDIR/ros/core/roslib/src/package.cpp patches/package.patch

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > ./CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_TOOLCHAIN_FILE 
	\${CMAKE_CURRENT_SOURCE_DIR}/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake)

#set (CMAKE_SYSTEM_FRAMEWORK_PATH ${CMAKE_SYSTEM_FRAMEWORK_PATH}
#	/Users/Ronan/Documents/Xcode_workspace/frameworks)
	
project(ros_for_ios)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

# cpp_common

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
if(HAVE_EXECINFO_H)
  add_definitions(-DHAVE_EXECINFO_H=1)
endif(HAVE_EXECINFO_H)
# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include<cxxabi.h>\nintmain(intargc,char**argv){}" HAVE_CXXABI_H)
if(HAVE_CXXABI_H)
  add_definitions(-DHAVE_CXXABI_H=1)
endif()
CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)
if(HAVE_GLIBC_BACKTRACE)
  add_definitions(-DHAVE_GLIBC_BACKTRACE)
endif(HAVE_GLIBC_BACKTRACE)

include(cpp_common.cmake)

# roscpp_serialization
include(roscpp_serialization.cmake)

# roscpp_traits
include(roscpp_traits.cmake)

# rostime
include(rostime.cmake)

# xmlrpcpp
include(xmlrpcpp.cmake)

# roscpp

set(roscpp_VERSION 1.0.41)
# split version in parts and pass to extra file
string(REPLACE "." ";" roscpp_VERSION_LIST "\${roscpp_VERSION}")
list(LENGTH roscpp_VERSION_LIST _count)
if(NOT _count EQUAL 3)
  message(FATAL_ERROR 
	"roscpp version '\${roscpp_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
endif()
list(GET roscpp_VERSION_LIST 0 roscpp_VERSION_MAJOR)
list(GET roscpp_VERSION_LIST 1 roscpp_VERSION_MINOR)
list(GET roscpp_VERSION_LIST 2 roscpp_VERSION_PATCH)

configure_file(\${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/clients/roscpp/include/ros/common.h.in
	\${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/clients/roscpp/include/ros/common.h)

# Output test results to config.h
configure_file(\${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/clients/roscpp/src/libros/config.h.in
	\${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/clients/roscpp/src/libros/config.h)

include(roscpp.cmake)

# rosconsole
include(rosconsole.cmake)

# roslib
include(roslib.cmake)
EOF
