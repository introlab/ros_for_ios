#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`

#===============================================================================
#BOOST

[ -d $SRCDIR/boostonios ] && rm -rf $SRCDIR/boostonios
git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git $SRCDIR/boostonios

cd $SRCDIR/boostonios
sh boost.sh

#===============================================================================
#LOG4CXX

cd $SRCDIR/log4cxx
sh build.sh

#===============================================================================
#ROS

[ -d $SRCDIR/ros/frameworks ] && rm -rf $SRCDIR/ros/frameworks
mkdir $SRCDIR/ros/frameworks
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/

cd $SRCDIR/ros
sh build.sh

#===============================================================================