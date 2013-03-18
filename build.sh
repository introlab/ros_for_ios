#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
LOGFILE=$SRCDIR/log.txt

echo "It will take some minutes. Grab a glass of your favorite drink ..."

#===============================================================================
#BOOST

if [ -d $SRCDIR/boostonios ]
    then
        (cd $SRCDIR/boostonios; git pull  >& $LOGFILE)
else
    git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git $SRCDIR/boostonios  >& $LOGFILE
fi

echo "Building Boost"
(cd $SRCDIR/boostonios; sh boost.sh  >& $LOGFILE)


#===============================================================================
#LOG4CXX

echo "Building Log4cxx"
(cd $SRCDIR/log4cxx; sh build.sh >>& $LOGFILE)

#===============================================================================
#ROS

echo "Building ROS"
[ -d $SRCDIR/ros/frameworks ] && rm -rf $SRCDIR/ros/frameworks
mkdir $SRCDIR/ros/frameworks
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/

(cd $SRCDIR/ros; sh build.sh  >>& $LOGFILE);

#===============================================================================

echo "Finished"

