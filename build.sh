#!/bin/sh -x

#===============================================================================
#Functions

error_exit()
{
    echo "$1";
    exit 1;
}

#===============================================================================

SRCDIR=`pwd`

ROS_BRANCH=groovy-devel
LOGFILE=$SRCDIR/log.txt

echo "It will take few minutes. Grab a glass of your favorite drink ..."

#===============================================================================
#CMake Toolchain

echo "Copying the cmake ios toolchain"

(
[ ! -d $SRCDIR/log4cxx/ios_cmake ] && cp -r $SRCDIR/ios_cmake $SRCDIR/log4cxx/
[ ! -d $SRCDIR/ros/ios_cmake ] && cp -r $SRCDIR/ios_cmake $SRCDIR/ros/
) > $LOGFILE 2>&1

#===============================================================================
#BOOST

echo "Building Boost :"

(
if [ -d $SRCDIR/boostonios ]
    then
        (cd $SRCDIR/boostonios; git pull)
else
    git clone git://gitorious.org/~galbraithjoseph/boostoniphone/galbraithjosephs-boostoniphone.git $SRCDIR/boostonios
fi
) >> $LOGFILE 2>&1

if (cd $SRCDIR/boostonios; sh boost.sh) >> $LOGFILE 2>&1;
    then
        echo "Ok"
else
    error_exit "Error ! Aborting."
fi

#===============================================================================
#LOG4CXX

echo "Building Log4cxx :"

if (cd $SRCDIR/log4cxx; sh build.sh) >> $LOGFILE 2>&1;
    then
        echo "Ok"
else
    error_exit "Error ! Aborting."
fi

#===============================================================================
#ROS std_msgs

echo "Building ROS std_msgs :"

(
if [ -d $SRCDIR/ros/std_msgs ]
    then
    (cd $SRCDIR/ros/std_msgs; git pull >> $LOGFILE)
else
    (cd $SRCDIR/ros; git clone -b $ROS_BRANCH https://github.com/ros/std_msgs.git)
fi
) >> $LOGFILE 2>&1

if (cd $SRCDIR/ros; sh messages_gen.sh -f $SRCDIR/ros/std_msgs) >> $LOGFILE 2>&1;
    then
        echo "Ok"
else
    error_exit "Error ! Aborting."
fi


#===============================================================================
#ROS common_msgs

echo "Building ROS common_msgs :"

(
if [ -d $SRCDIR/ros/common_msgs ]
    then
        (cd $SRCDIR/ros/common_msgs; git pull)
else
    (cd $SRCDIR/ros; git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git)
fi
) >> $LOGFILE 2>&1

if (cd $SRCDIR/ros; sh messages_gen.sh -f $SRCDIR/ros/common_msgs/geometry_msgs $SRCDIR/ros/std_msgs) >> $LOGFILE 2>&1;
    then
        echo "- geometry_msgs Ok"
else
    error_exit "Error ! Aborting."
fi

if (cd $SRCDIR/ros; sh messages_gen.sh -f $SRCDIR/ros/common_msgs/nav_msgs $SRCDIR/ros/std_msgs $SRCDIR/ros/common_msgs/geometry_msgs) >> $LOGFILE 2>&1;
    then
        echo "- nav_msgs Ok"
else
    error_exit "Error ! Aborting."
fi

if (cd $SRCDIR/ros; sh messages_gen.sh -f $SRCDIR/ros/common_msgs/sensor_msgs $SRCDIR/ros/std_msgs $SRCDIR/ros/common_msgs/geometry_msgs) >> $LOGFILE 2>&1;
    then
        echo "- sensor_msgs Ok"
else
    error_exit "Error ! Aborting."
fi

#===============================================================================
#ROS

echo "Building ROS core :"

(
[ -d $SRCDIR/ros/frameworks ] && rm -rf $SRCDIR/ros/frameworks
mkdir $SRCDIR/ros/frameworks
mv $SRCDIR/boostonios/ios/framework/boost.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/log4cxx/log4cxx.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/std_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/geometry_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/nav_msgs.framework $SRCDIR/ros/frameworks/
mv $SRCDIR/ros/sensor_msgs.framework $SRCDIR/ros/frameworks/

(cd $SRCDIR/ros; sh build_ros.sh);
mv $SRCDIR/ros/ros.framework $SRCDIR/ros/frameworks/
) >> $LOGFILE 2>&1
echo "Ok"

#===============================================================================
#ROS image_transport

echo "Building image_transport (user password required) :"

if (cd $SRCDIR/ros; sh build_image_transport.sh) >> $LOGFILE 2>&1;
    then
        echo "Ok"
else
    error_exit "Error ! Aborting."
fi

#===============================================================================

echo "Cleaning"

(
mv -f $SRCDIR/ros/frameworks $SRCDIR/frameworks
git clean -df $SRCDIR/ros
git clean -df $SRCDIR/log4cxx
mv -f $SRCDIR/frameworks $SRCDIR/ros/frameworks
) >> $LOGFILE 2>&1

echo "Finished !"

#===============================================================================
