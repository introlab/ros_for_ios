#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`

ROS_BRANCH=groovy-devel
LOGFILE=$SRCDIR/log.txt

echo "It will take some minutes. Grab a glass of your favorite drink ..."

#===============================================================================
#CMake Toolchain

echo "Copying the cmake ios toolchain"

(
[ ! -d $SRCDIR/log4cxx/ios_cmake ] && cp -r $SRCDIR/ios_cmake $SRCDIR/log4cxx/
[ ! -d $SRCDIR/ros_msgs/ios_cmake ] && cp -r $SRCDIR/ios_cmake $SRCDIR/ros/ros_msgs/
[ ! -d $SRCDIR/ros/ios_cmake ] && cp -r $SRCDIR/ios_cmake $SRCDIR/ros/
) > $LOGFILE 2>&1

#===============================================================================
#BOOST

echo "Building Boost"

(
if [ -d $SRCDIR/boostonios ]
    then
        (cd $SRCDIR/boostonios; git pull)
else
    git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git $SRCDIR/boostonios
fi

(cd $SRCDIR/boostonios; sh boost.sh)
) >> $LOGFILE 2>&1

#===============================================================================
#LOG4CXX

echo "Building Log4cxx"

(
(cd $SRCDIR/log4cxx; sh build.sh )
) >> $LOGFILE 2>&1

#===============================================================================
#ROS std_msgs

echo "Building ROS std_msgs"

(
if [ -d $SRCDIR/ros/ros_msgs/std_msgs ]
    then
    (cd $SRCDIR/ros/ros_msgs/std_msgs; git pull >> $LOGFILE)
else
    (cd $SRCDIR/ros/ros_msgs; git clone -b $ROS_BRANCH https://github.com/ros/std_msgs.git)
fi

(cd $SRCDIR/ros/ros_msgs; sh messages_gen.sh $SRCDIR/ros/ros_msgs/std_msgs);
) >> $LOGFILE 2>&1

#===============================================================================
#ROS common_msgs

echo "Building ROS common_msgs"

(
if [ -d $SRCDIR/ros/ros_msgs/common_msgs ]
    then
        (cd $SRCDIR/ros/ros_msgs/common_msgs; git pull)
else
    (cd $SRCDIR/ros/ros_msgs; git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git)
fi

(cd $SRCDIR/ros/ros_msgs; sh messages_gen.sh $SRCDIR/ros/ros_msgs/common_msgs/geometry_msgs $SRCDIR/ros/ros_msgs/std_msgs);

(cd $SRCDIR/ros/ros_msgs; sh messages_gen.sh $SRCDIR/ros/ros_msgs/common_msgs/nav_msgs $SRCDIR/ros/ros_msgs/std_msgs $SRCDIR/ros/ros_msgs/common_msgs/geometry_msgs);

(cd $SRCDIR/ros/ros_msgs; sh messages_gen.sh $SRCDIR/ros/ros_msgs/common_msgs/sensor_msgs $SRCDIR/ros/ros_msgs/std_msgs $SRCDIR/ros/ros_msgs/common_msgs/geometry_msgs);

) >> $LOGFILE 2>&1

#===============================================================================
#ROS

echo "Building ROS core"

(
[ -d $SRCDIR/ros/frameworks ] && rm -rf $SRCDIR/ros/frameworks
mkdir $SRCDIR/ros/frameworks
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/ros_msgs/std_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/ros_msgs/geometry_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/ros_msgs/nav_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/ros_msgs/sensor_msgs.framework $SRCDIR/ros/frameworks/

(cd $SRCDIR/ros; sh build.sh);
mv $SRCDIR/ros/ros.framework $SRCDIR/ros/frameworks/
) >> $LOGFILE 2>&1

#===============================================================================

echo "Cleaning ..."

(
(cd $SRCDIR/log4cxx; rm CMakeLists.txt; rm *.tar.gz);
(cd $SRCDIR/ros/ros_msgs; rm *.cpp; rm CMakeLists.txt);
(cd $SRCDIR/ros; rm *.cmake; rm CMakeLists.txt);
) >> $LOGFILE 2>&1

echo "Finished !"
