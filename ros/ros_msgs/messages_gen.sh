#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build

ROS_BRANCH=groovy-devel

#===============================================================================
if [ -d genmsg ]
    then
        (cd $SRCDIR/genmsg; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/ros/genmsg.git
fi

if [ -d gencpp ]
    then
        (cd $SRCDIR/gencpp; git pull)
else
    git clone -b $ROS_BRANCH https://github.com/ros/gencpp.git
fi

#===============================================================================
echo "Setuping genmsg and gencpp ..."

# empy
# www.alcyone.com/pyos/empy/
# A powerful and robust templating system for Python.
if ! $(python -c "import em")
	then
		[ -d empy.tar.gz ] && rm -rf empy.tar.gz
		curl http://www.alcyone.com/software/empy/empy-latest.tar.gz -o ./empy.tar.gz
		tar xvf empy.tar.gz
		(cd empy*; python setup.py install --prefix=$SRCDIR/empy/);
fi

# genmsg
if ! $(python -c "import genmsg")
	then
		patch -N $SRCDIR/genmsg/setup.py $SRCDIR/patches/setup_genmsg.patch
		(cd genmsg; python setup.py install --prefix=$SRCDIR/genmsg/);
fi

# gencpp
if ! $(python -c "import gencpp")
	then
		patch -N $SRCDIR/gencpp/setup.py $SRCDIR/patches/setup_gencpp.patch
		(cd gencpp; python setup.py install --prefix=$SRCDIR/gencpp/);
fi

#===============================================================================

PACKAGE_NAME=`basename $1`

for ARG in $*
    do
        DEPENDENCIES=${DEPENDENCIES}" -I$PACKAGE_NAME:$ARG/msg/"
done

#===============================================================================
echo "Generating $PACKAGE_NAME messages ..."

FILES=$1/msg/*.msg

for f in $FILES
    do
        python $SRCDIR/gencpp/scripts/gen_cpp.py $f $DEPENDENCIES -p $PACKAGE_NAME -o $1 -e $SRCDIR/gencpp/scripts/
done

#===============================================================================
echo "Generating fake C++ file ..."

cat > $PACKAGE_NAME.cpp <<EOF
void foo()
{

}

EOF

#===============================================================================
echo "Generating CMakeLists.txt ..."

cat > CMakeLists.txt <<EOF

cmake_minimum_required(VERSION 2.8.0)

project($PACKAGE_NAME)

add_library($PACKAGE_NAME STATIC $PACKAGE_NAME.cpp)

EOF

#===============================================================================
echo "Building ..."

[ -d $OS_BUILDDIR ] && rm -rf $OS_BUILDDIR
[ -d $SIMULATOR_BUILDDIR ] && rm -rf $SIMULATOR_BUILDDIR

mkdir $OS_BUILDDIR
mkdir $SIMULATOR_BUILDDIR

cd $OS_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneOS_Xcode.cmake -DCMAKE_INSTALL_PREFIX=$LOG4CXX_iPhoenOS -GXcode ..

xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD

cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake -DCMAKE_INSTALL_PREFIX=$LOG4CXX_iPhoenSimulator -GXcode ..

xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD

#===============================================================================
cd $SRCDIR

VERSION_TYPE=Alpha
FRAMEWORK_NAME=$PACKAGE_NAME
FRAMEWORK_VERSION=A

FRAMEWORK_CURRENT_VERSION=1.0
FRAMEWORK_COMPATIBILITY_VERSION=1.0

FRAMEWORK_BUNDLE=$SRCDIR/$FRAMEWORK_NAME.framework
echo "Framework: Building $FRAMEWORK_BUNDLE ..."

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
lipo -create $OS_BUILDDIR/Release-iphoneos/lib$PACKAGE_NAME.a $SIMULATOR_BUILDDIR/Release-iphonesimulator/lib$PACKAGE_NAME.a -o $FRAMEWORK_INSTALL_NAME

echo "Framework: Copying includes..."

cp -r $1/*.h $FRAMEWORK_BUNDLE/Headers/

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
