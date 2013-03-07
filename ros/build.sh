#!/bin/sh -x

ROS_BRANCH=groovy-devel

git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
git clone -b $ROS_BRANCH https://github.com/ros/ros.git

git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git
