#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build

ROS_BRANCH=groovy-devel

#===============================================================================
echo "Cloning git repositories ..."


if [ -d ffmpegc ]
    then
        (cd $SRCDIR/ffmpegc; git pull)
    else
        git clone -b master https://github.com/lvjian700/ffmpegc.git
fi

if [ -d introlab-ros-pkg ]
    then
        (cd $SRCDIR/introlab-ros-pkg; git pull)
    else
        git clone -b master https://github.com/introlab/introlab-ros-pkg.git
fi

if [ -d image_common ]
    then
        (cd $SRCDIR/image_common; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/introlab/image_common.git
fi

IT_SRC_DIR=$SRCDIR/image_common/image_transport/src
IT_INC_DIR=$SRCDIR/image_common/image_transport/include

X264_IT_SRC_DIR=$SRCDIR/introlab-ros-pkg/x264_image_transport/src
X264_IT_INC_DIR=$SRCDIR/introlab-ros-pkg/x264_image_transport/include

rm -f $IT_SRC_DIR/manifest.cpp
rm -f $IT_SRC_DIR/list_transports.cpp
rm -f $IT_SRC_DIR/republish.cpp

cp -f $X264_IT_SRC_DIR/x264_publisher.cpp $IT_SRC_DIR
cp -f $X264_IT_SRC_DIR/x264_subscriber.cpp $IT_SRC_DIR

ln -s $X264_IT_INC_DIR/x264_image_transport $IT_INC_DIR/x264_image_transport

#===============================================================================
echo "Building the ffmpeg frameworks ..."

(cd ffmpegc; sh install-ffmpeg.sh);

for f in $SRCDIR/ffmpegc/ffmpeg*/build/*.a
    do
        LIB_NAME=`basename $f`
        LIB_NAME=${LIB_NAME%.a}
        sh framework_gen.sh $LIB_NAME $SRCDIR/ffmpegc/ffmpeg*/build/armv7 $SRCDIR/ffmpegc/ffmpeg*/build/i386 $SRCDIR/ffmpegc/ffmpeg*/$LIB_NAME
        rm -rf $SRCDIR/frameworks/$LIB_NAME.framework
        mv -f $SRCDIR/$LIB_NAME.framework $SRCDIR/frameworks/
done

sh framework_gen.sh libx264 $SRCDIR/ffmpegc/ffmpeg*/x264/build/armv7/lib $SRCDIR/ffmpegc/ffmpeg*/x264/build/i386/lib $SRCDIR/ffmpegc/ffmpeg*/x264/build/armv7/include
rm -rf $SRCDIR/frameworks/libx264.framework
mv -f $SRCDIR/libx264.framework $SRCDIR/frameworks/

#===============================================================================
echo "Generating ROS packages ..."

PACKAGES=("ros_comm/utilities/message_filters"
        "introlab-ros-pkg/x264_image_transport"
        "image_common/image_transport")

sh $SRCDIR/build_ros_package.sh $SRCDIR/${PACKAGES[0]} boost ros
mv -f *.framework $SRCDIR/frameworks/
sh $SRCDIR/messages_gen.sh -f $SRCDIR/${PACKAGES[1]} $SRCDIR/std_msgs
mv -f $SRCDIR/*.framework $SRCDIR/frameworks/
sh $SRCDIR/build_ros_package.sh $SRCDIR/${PACKAGES[2]} boost ros message_filters libavcodec libavformat libswscale libavutil sensor_msgs x264_image_transport
mv -f *.framework $SRCDIR/frameworks/

echo "Done !"
