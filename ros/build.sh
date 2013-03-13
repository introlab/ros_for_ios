#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build

ROS_BRANCH=groovy-devel

#===============================================================================
echo "Cloning git repositories ..."

[[ -d roscpp_core ]] && rm -rf roscpp_core
git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
[[ -d ros_comm ]] && rm -rf ros_comm
git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
[[ -d ros ]] && rm -rf ros
git clone -b $ROS_BRANCH https://github.com/ros/ros.git
[[ -d genmsg ]] && rm -rf genmsg
git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
[[ -d gencpp ]] && rm -rf gencpp
git clone -b $ROS_BRANCH https://github.com/ros/gencpp.git
[[ -d std_msgs ]] && rm -rf std_msgs
git clone -b $ROS_BRANCH https://github.com/ros/std_msgs.git
[[ -d common_msgs ]] && rm -rf common_msgs
git clone -b $ROS_BRANCH https://github.com/ros/common_msgs.git
[[ -d empy.tar.gz ]] && rm -rf empy.tar.gz
curl http://www.alcyone.com/software/empy/empy-latest.tar.gz -o ./empy.tar.gz

#===============================================================================
echo "Generating cmake submodules ..."

PACKAGES=("roscpp_core/cpp_common"
        "roscpp_core/roscpp_serialization"
        "roscpp_core/roscpp_traits"
        "roscpp_core/rostime"
        "ros_comm/utilities/xmlrpcpp"
        "ros_comm/clients/roscpp"
        "ros_comm/tools/rosconsole"
        "ros/core/roslib")

sh $SRCDIR/cmake_gen.sh ${PACKAGES[0]}
sh $SRCDIR/cmake_gen.sh ${PACKAGES[1]} boost
sh $SRCDIR/cmake_gen.sh ${PACKAGES[2]} boost
sh $SRCDIR/cmake_gen.sh ${PACKAGES[3]} boost
sh $SRCDIR/cmake_gen.sh ${PACKAGES[4]}
sh $SRCDIR/cmake_gen.sh ${PACKAGES[5]} boost log4cxx
sh $SRCDIR/cmake_gen.sh ${PACKAGES[6]} boost log4cxx
sh $SRCDIR/cmake_gen.sh ${PACKAGES[7]} boost

#===============================================================================
echo "Patching ..."

patch -N $SRCDIR/ros/core/roslib/src/package.cpp $SRCDIR/patches/package.patch
patch -N $SRCDIR/roscpp_core/roscpp_traits/include/ros/message_forward.h $SRCDIR/patches/message_forward.patch
patch -N $SRCDIR/ros_comm/utilities/xmlrpcpp/include/base64.h $SRCDIR/patches/base64.patch

#===============================================================================
echo "Setuping genmsg and gencpp ..."

# www.alcyone.com/pyos/empy/ :
# A powerful and robust templating system for Python.

[[ -d empy-3.3 ]] && rm -rf empy-3.3
tar xvf empy.tar.gz

# empy
(cd empy-3.3; python setup.py install --prefix=$SRCDIR/empy/);
# genmsg
patch -N $SRCDIR/genmsg/setup.py $SRCDIR/patches/setup_genmsg.patch
(cd genmsg; python setup.py install --prefix=$SRCDIR/genmsg/);
# gencpp
patch -N $SRCDIR/gencpp/setup.py $SRCDIR/patches/setup_gencpp.patch
(cd gencpp; python setup.py install --prefix=$SRCDIR/gencpp/);

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

echo "- rosgraph_msgs -"

[ ! -d $SRCDIR/rosgraph_msgs ] && mkdir $SRCDIR/rosgraph_msgs

python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/rosgraph_msgs/msg/Clock.msg -p rosgraph_msgs -o $SRCDIR/rosgraph_msgs -e $SRCDIR/gencpp/scripts/	
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/rosgraph_msgs/msg/Log.msg -Istd_msgs:$SRCDIR/std_msgs/msg/ -p rosgraph_msgs -o $SRCDIR/rosgraph_msgs -e $SRCDIR/gencpp/scripts/

echo "- std_srvs -"

[ ! -d $SRCDIR/std_srvs ] && mkdir $SRCDIR/std_srvs
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/messages/std_srvs/srv/Empty.srv -p std_srvs -o $SRCDIR/std_srvs -e $SRCDIR/gencpp/scripts/

echo "- roscpp -"

[ ! -d $SRCDIR/roscpp ] && mkdir $SRCDIR/roscpp
python $SRCDIR/gencpp/scripts/gen_cpp.py $SRCDIR/ros_comm/clients/roscpp/msg/Logger.msg -Istd_msgs:$SRCDIR/std_msgs/msg/ -p roscpp -o $SRCDIR/roscpp -e $SRCDIR/gencpp/scripts/

FILES=$SRCDIR/ros_comm/clients/roscpp/srv/*

for f in $FILES
    do
        python $SRCDIR/gencpp/scripts/gen_cpp.py $f -Iroscpp:$SRCDIR/ros_comm/clients/roscpp/msg/ -Istd_msgs:$SRCDIR/std_msgs/msg/ -p roscpp -o $SRCDIR/roscpp -e $SRCDIR/gencpp/scripts/
done

echo "- std_msgs -"

FILES=$SRCDIR/std_msgs/msg/*

for f in $FILES
    do
        python $SRCDIR/gencpp/scripts/gen_cpp.py $f -Istd_msgs:$SRCDIR/std_msgs/msg/ -p std_msgs -o $SRCDIR/std_msgs -e $SRCDIR/gencpp/scripts/
done

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

#set (CMAKE_SYSTEM_FRAMEWORK_PATH \${CMAKE_SYSTEM_FRAMEWORK_PATH} $SRCDIR/../)

project(ros_for_ios)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

# for the ros messages
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

EOF

for package in ${PACKAGES[@]}
    do
        PACKAGE_NAME=`basename $package`
        cat >> CMakeLists.txt <<EOF
include($PACKAGE_NAME.cmake)

EOF
done

#===============================================================================
echo "Building ..."

[[ -d $OS_BUILDDIR ]] && rm -rf $OS_BUILDDIR
[[ -d $SIMULATOR_BUILDDIR ]] && rm -rf $SIMULATOR_BUILDDIR

mkdir $OS_BUILDDIR
mkdir $SIMULATOR_BUILDDIR

cd $OS_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneOS_Xcode.cmake -DCMAKE_INSTALL_PREFIX=ros_iPhoenOS -GXcode ..

xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD

cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake -DCMAKE_INSTALL_PREFIX=ros_iPhoenSimulator -GXcode ..

xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD

#===============================================================================
echo "Scrunch all libs together in one lib per platform ..."

mkdir -p $OS_BUILDDIR/armv7/obj
mkdir -p $SIMULATOR_BUILDDIR/i386/obj

echo "Splitting all existing fat binaries..."

for f in $OS_BUILDDIR/Release-iphoneos/*.a
    do
        lipo "$f" -thin armv7 -o $OS_BUILDDIR/armv7/$(basename "$f")
done

cp $SIMULATOR_BUILDDIR/Release-iphonesimulator/*.a $SIMULATOR_BUILDDIR/i386/

echo "Decomposing each architecture's .a files"

for f in $OS_BUILDDIR/armv7/*.a
    do
        (cd $OS_BUILDDIR/armv7/obj; ar -x $f);
done

for f in $SIMULATOR_BUILDDIR/i386/*.a
    do
        (cd $SIMULATOR_BUILDDIR/i386/obj; ar -x $f);
done

echo "Linking each architecture into an uberlib (libros.a) ..."

(cd $OS_BUILDDIR/armv7/; ar crus libros.a obj/*.o; )
(cd $SIMULATOR_BUILDDIR/i386/; ar crus libros.a obj/*.o; )

#===============================================================================
cd $SRCDIR

VERSION_TYPE=Alpha
FRAMEWORK_NAME=ros
FRAMEWORK_VERSION=A

FRAMEWORK_CURRENT_VERSION=1.0
FRAMEWORK_COMPATIBILITY_VERSION=1.0

FRAMEWORK_BUNDLE=$SRCDIR/../$FRAMEWORK_NAME.framework
echo "Framework: Building $FRAMEWORK_BUNDLE ..."

[[ -d $FRAMEWORK_BUNDLE ]] && rm -rf $FRAMEWORK_BUNDLE

echo "Framework: Setting up directories..."
mkdir -p $FRAMEWORK_BUNDLE
mkdir -p $FRAMEWORK_BUNDLE/Versions
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Resources
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Headers
mkdir -p $FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/Documentation

echo "Framework: Creating symlinks..."
ln -s $FRAMEWORK_VERSION               $FRAMEWORK_BUNDLE/Versions/Current
ln -s Versions/Current/Headers         $FRAMEWORK_BUNDLE/Headers
ln -s Versions/Current/Resources       $FRAMEWORK_BUNDLE/Resources
ln -s Versions/Current/Documentation   $FRAMEWORK_BUNDLE/Documentation
ln -s Versions/Current/$FRAMEWORK_NAME $FRAMEWORK_BUNDLE/$FRAMEWORK_NAME

FRAMEWORK_INSTALL_NAME=$FRAMEWORK_BUNDLE/Versions/$FRAMEWORK_VERSION/$FRAMEWORK_NAME

echo "Lipoing library into $FRAMEWORK_INSTALL_NAME..."
lipo -create $OS_BUILDDIR/armv7/libros.a $SIMULATOR_BUILDDIR/i386/libros.a -o $FRAMEWORK_INSTALL_NAME

echo "Framework: Copying includes..."

# main packages
for package in ${PACKAGES[@]}
    do
        find $SRCDIR/$package/include -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers \;
done

# for the ros messages
find $SRCDIR/std_srvs -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers \;
find $SRCDIR/rosgraph_msgs -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers \;
find $SRCDIR/roscpp -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers \;

#TODO : speciic framework
find $SRCDIR/std_msgs -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers \;

echo "Framework: Creating plist..."

cat > $FRAMEWORK_BUNDLE/Resources/Info.plist <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
<key>CFBundleDevelopmentRegion</key>
<string>English</string>
<key>CFBundleExecutable</key>
<string>${FRAMEWORK_NAME}</string>
<key>CFBundleIdentifier</key>
<string>org.boost</string>
<key>CFBundleInfoDictionaryVersion</key>
<string>6.0</string>
<key>CFBundlePackageType</key>
<string>FMWK</string>
<key>CFBundleSignature</key>
<string>????</string>
<key>CFBundleVersion</key>
<string>${FRAMEWORK_CURRENT_VERSION}</string>
</dict>
</plist>
EOF

echo "Done !"
