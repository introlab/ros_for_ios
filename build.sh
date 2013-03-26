#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`

ROS_BRANCH=groovy-devel
LOGFILE=$SRCDIR/log.txt

echo "It will take some minutes. Grab a glass of your favorite drink ..."

#===============================================================================
#BOOST

if [ -d $SRCDIR/boostonios ]
    then
        (cd $SRCDIR/boostonios; git pull &> $LOGFILE)
else
    git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git $SRCDIR/boostonios  &>> $LOGFILE
fi

echo "Building Boost"
(cd $SRCDIR/boostonios; sh boost.sh  &>> $LOGFILE)

#===============================================================================
#LOG4CXX

echo "Building Log4cxx"
(cd $SRCDIR/log4cxx; sh build.sh &>> $LOGFILE)

#===============================================================================
#ROS std_msgs

echo "Building ROS std_msgs"

if [ -d $SRCDIR/ros_msgs/std_msgs ]
    then
    (cd $SRCDIR/ros_msgs/std_msgs; git pull &>> $LOGFILE)
else
    git clone -b $ROS_BRANCH https://github.com/ros/std_msgs.git &>> $LOGFILE
fi

(cd $SRCDIR/ros_msgs; sh messages_gen.sh $SRCDIR/ros_msgs/std_msgs &>> $LOGFILE);

#===============================================================================
#ROS common_msgs

echo "Downloading ROS common_msgs"

if [ -d common_msgs ]
    then
        (cd $SRCDIR/common_msgs; git pull &>> $LOGFILE)
else
    git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git &>> $LOGFILE
fi

#===============================================================================
#ROS

echo "Building ROS"
[ -d $SRCDIR/ros/frameworks ] && rm -rf $SRCDIR/ros/frameworks
mkdir $SRCDIR/ros/frameworks &>> $LOGFILE
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/ &>> $LOGFILE
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/ &>> $LOGFILE
mv $SRCDIR/ros_msgs/std_msgs.framework $SRCDIR/ros/frameworks/ &>> $LOGFILE

(cd $SRCDIR/ros; sh build.sh  &>> $LOGFILE);
mv $SRCDIR/ros/ros.framework $SRCDIR/ros/frameworks/ &>> $LOGFILE
#===============================================================================

echo "Cleaning ..."
(cd $SRCDIR/ros; sh build.sh  &>> $LOGFILE);
(cd $SRCDIR/ros; sh build.sh  &>> $LOGFILE);
echo "Finished"

