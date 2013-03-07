#!/bin/sh -x

ROS_BRANCH=groovy-devel

#git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
#git clone -b $ROS_BRANCH https://github.com/ros/ros.git

#git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
#git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git

./cmake_gen.sh roscpp_core cpp_common
./cmake_gen.sh roscpp_core roscpp_serialization
./cmake_gen.sh roscpp_core roscpp_traits
./cmake_gen.sh roscpp_core rostime boost

./cmake_gen.sh ros_comm/utilities xmlrpcpp 
./cmake_gen.sh ros_comm/client roscpp
./cmake_gen.sh ros_comm/tools rosconsole

./cmake_gen.sh ros_comm/tools rosconsole



