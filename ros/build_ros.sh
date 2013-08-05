#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build

ROS_BRANCH=groovy-devel

#===============================================================================
echo "Cloning git repositories ..."

if [ -d roscpp_core ]
    then
        (cd $SRCDIR/roscpp_core; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/ros/roscpp_core.git
fi

if [ -d ros_comm ]
    then
        (cd $SRCDIR/ros_comm; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/ros/ros_comm.git
fi

if [ -d ros ]
    then
        (cd $SRCDIR/ros; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/ros/ros.git
fi

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

sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[0]}
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[1]} boost
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[2]} boost
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[3]} boost
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[4]}
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[5]} boost log4cxx rosgraph_msgs std_msgs
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[6]} boost log4cxx
sh $SRCDIR/cmake_gen.sh $SRCDIR/${PACKAGES[7]} boost

#===============================================================================
echo "Patching ..."

patch -N $SRCDIR/ros/core/roslib/src/package.cpp $SRCDIR/patches/package.patch
patch -N $SRCDIR/roscpp_core/roscpp_traits/include/ros/message_forward.h $SRCDIR/patches/message_forward.patch
patch -N $SRCDIR/ros_comm/utilities/xmlrpcpp/include/base64.h $SRCDIR/patches/base64.patch
patch -N $SRCDIR/ros_comm/clients/roscpp/include/ros/node_handle.h $SRCDIR/patches/node_handle.patch

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

sh messages_gen.sh -f $SRCDIR/ros_comm/messages/rosgraph_msgs $SRCDIR/std_msgs
mv $SRCDIR/rosgraph_msgs.framework $SRCDIR/frameworks/

echo "- std_srvs -"

sh messages_gen.sh -d $SRCDIR/ros_comm/messages/std_srvs $SRCDIR/std_msgs

echo "- roscpp -"

sh messages_gen.sh -d $SRCDIR/ros_comm/clients/roscpp

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 2.8.0)

set (CMAKE_FRAMEWORK_PATH \${CMAKE_SYSTEM_FRAMEWORK_PATH} $SRCDIR/frameworks)

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

[ -d $OS_BUILDDIR ] && rm -rf $OS_BUILDDIR
[ -d $SIMULATOR_BUILDDIR ] && rm -rf $SIMULATOR_BUILDDIR

mkdir $OS_BUILDDIR
mkdir $SIMULATOR_BUILDDIR

cd $OS_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneOS_Xcode.cmake -DCMAKE_INSTALL_PREFIX=ros_iPhoenOS -GXcode ..

if (! xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD)
    then
        exit 1
fi

cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake -DCMAKE_INSTALL_PREFIX=ros_iPhoenSimulator -GXcode ..

if (! xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD)
    then
        exit 1
fi

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

FRAMEWORK_BUNDLE=$SRCDIR/$FRAMEWORK_NAME.framework
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
lipo -create $OS_BUILDDIR/armv7/lib$FRAMEWORK_NAME.a $SIMULATOR_BUILDDIR/i386/lib$FRAMEWORK_NAME.a -o $FRAMEWORK_INSTALL_NAME

echo "Framework: Copying includes..."

# main packages
for package in ${PACKAGES[@]}
    do
        cp -r $SRCDIR/$package/include/* $FRAMEWORK_BUNDLE/Headers
done
	
mv $FRAMEWORK_BUNDLE/Headers/ros/*.h $FRAMEWORK_BUNDLE/Headers/
rm -r $FRAMEWORK_BUNDLE/Headers/ros/

# ros core messages
mkdir $FRAMEWORK_BUNDLE/Headers/std_srvs
find $SRCDIR/std_srvs -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers/std_srvs \;
mkdir $FRAMEWORK_BUNDLE/Headers/roscpp
find $SRCDIR/roscpp -name \*.h -exec cp {} $FRAMEWORK_BUNDLE/Headers/roscpp \;

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
<string>ros.org</string>
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
