#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
ROS_BRANCH=groovy-devel

#===============================================================================
echo "Cloning git repositories ..."

#git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros.git

#git clone -b $ROS_BRANCH https://github.com/ros/gencpp.git
#git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
#git clone -b $ROS_BRANCH https://github.com/ros/std_msgs.git
#git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git

#===============================================================================
echo "Generating cmake submodules ..."

#$SRCDIR/cmake_gen.sh roscpp_core cpp_common
#$SRCDIR/cmake_gen.sh roscpp_core roscpp_serialization boost
#$SRCDIR/cmake_gen.sh roscpp_core roscpp_traits boost
#$SRCDIR/cmake_gen.sh roscpp_core rostime boost

#$SRCDIR/cmake_gen.sh ros_comm/utilities xmlrpcpp 
#$SRCDIR/cmake_gen.sh ros_comm/clients roscpp boost log4cxx
#$SRCDIR/cmake_gen.sh ros_comm/tools rosconsole boost log4cxx

#$SRCDIR/cmake_gen.sh ros/core roslib boost

#===============================================================================
echo "Patching ..."

#patch $SRCDIR/ros/core/roslib/src/package.cpp patches/package.patch

#===============================================================================
echo "Setuping genmsg and gencpp ..."

# www.alcyone.com/pyos/empy/ :
# A powerful and robust templating system for Python.
#

# genmsg

# gencpp


#===============================================================================
echo "Generating ROS messages ..."

# From : ros.org/rosdoclite/groovy/api/genmsg/html/developer.html
# Python script gen_cpp.py
# /path/to/Some.msg
# The flagless argument is the path to the input .msg file.
# -I NAMESPACE:/some/path
# find messages in NAMESPACE in directory /some/path
# -p THIS_NAMESPACE
# put generated message into namespace THIS_NAMESPACE
# -o /output/dir
# Generate code into directory /output/dir
# -e /path/to/templates
# Find empy templates in this directory

[ ! -d $SRCDIR/rosgraph_msgs ] && mkdir $SRCDIR/rosgraph_msgs
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/rosgraph_msgs/msg/Clock.msg -p rosgraph_msgs -o $SRCDIR/rosgraph_msgs -e $SRCDIR/gencpp/scripts/
	
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/rosgraph_msgs/msg/Log.msg -Istd_msgs:$SRCDIR/std_msgs/msg/ -p rosgraph_msgs -o $SRCDIR/rosgraph_msgs -e $SRCDIR/gencpp/scripts/
	
[ ! -d $SRCDIR/std_srvs ] && mkdir $SRCDIR/std_srvs
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/std_srvs/srv/Empty.srv -p std_srvs -o $SRCDIR/std_srvs -e $SRCDIR/gencpp/scripts/

[ ! -d $SRCDIR/roscpp ] && mkdir $SRCDIR/roscpp
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/clients/roscpp/msg/Logger.msg -Istd_msgs:$SRCDIR/std_msgs/msg/ -p roscpp -o $SRCDIR/rosgraph_msgs -e $SRCDIR/gencpp/scripts/
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/clients/roscpp/srv/Empty.srv -p roscpp -o $SRCDIR/roscpp -e $SRCDIR/gencpp/scripts/
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/clients/roscpp/srv/GetLoggers.srv -Iroscpp:$SRCDIR/ros_comm/clients/roscpp/msg/ -p roscpp -o $SRCDIR/roscpp -e $SRCDIR/gencpp/scripts/
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/clients/roscpp/srv/SetLoggerLevel.srv -Istd_msgs:$SRCDIR/std_msgs/msg/ -p roscpp -o $SRCDIR/roscpp -e $SRCDIR/gencpp/scripts/

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > ./CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_TOOLCHAIN_FILE 
	\${CMAKE_CURRENT_SOURCE_DIR}/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake)

#set (CMAKE_SYSTEM_FRAMEWORK_PATH \${CMAKE_SYSTEM_FRAMEWORK_PATH}
#	/Users/Ronan/Documents/Xcode_workspace/frameworks)
	
project(ros_for_ios)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

include_directories(\${CMAKE_CURRENT_SOURCE_DIR})

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
