#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`

#===============================================================================
#BOOST

git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git boostonios

(cd $SRCDIR/boostonios; boost.sh);

#===============================================================================
#LOG4CXX

(cd $SRCDIR/log4cxx; build.sh);

#===============================================================================
#ROS

mkdir $SRCDIR/ros/frameworks
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/

(cd $SRCDIR/ros; build.sh);

#===============================================================================