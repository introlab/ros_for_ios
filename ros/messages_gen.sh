#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`
OS_BUILDDIR=$SRCDIR/iPhoneOS_build
SIMULATOR_BUILDDIR=$SRCDIR/iPhoneSimulator_build

ROS_BRANCH=groovy-devel

generate_framework=false

if [ $1 == "-f" ]
    then
        generate_framework=true
fi

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
		(cd empy*; python setup.py install --prefix=$SRCDIR/empy*/);
        export PYTHONPATH=$SRCDIR/empy*/lib/python2.7/site-packages/:$PYTHONPATH
fi

# genmsg
if ! $(python -c "import genmsg")
	then
		patch -N $SRCDIR/genmsg/setup.py $SRCDIR/patches/setup_genmsg.patch
		(cd genmsg; python setup.py install --prefix=$SRCDIR/genmsg/);
        export PYTHONPATH=$SRCDIR/genmsg/lib/python2.7/site-packages/:$PYTHONPATH
fi

# gencpp
if ! $(python -c "import gencpp")
	then
		patch -N $SRCDIR/gencpp/setup.py $SRCDIR/patches/setup_gencpp.patch
		(cd gencpp; python setup.py install --prefix=$SRCDIR/gencpp/);
        export PYTHONPATH=$SRCDIR/gencpp/lib/python2.7/site-packages/:$PYTHONPATH
fi

#===============================================================================

PACKAGE_NAME=`basename $2`

for ARG in $*
    do
        if [ $ARG != $1 ]
            then
                DEPENDENCY_NAME=`basename $ARG`
                DEPENDENCIES=${DEPENDENCIES}" -I $DEPENDENCY_NAME:$ARG/msg/"
        fi
done

#===============================================================================
echo "Generating $PACKAGE_NAME messages ..."

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

if [ -d $2/msg ]
    then

    FILES=$2/msg/*.msg

    for f in $FILES
        do
            python $SRCDIR/gencpp/scripts/gen_cpp.py $f $DEPENDENCIES -p $PACKAGE_NAME -o $2 -e $SRCDIR/gencpp/scripts/
    done
fi

if [ -d $2/srv ]
    then
        FILES=$2/srv/*.srv

        for f in $FILES
            do
                python $SRCDIR/gencpp/scripts/gen_cpp.py $f $DEPENDENCIES -p $PACKAGE_NAME -o $2 -e $SRCDIR/gencpp/scripts/
        done
fi

#===============================================================================
if [ $generate_framework == true ]; then

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

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneOS_Xcode.cmake -DCMAKE_INSTALL_PREFIX=$LOG4CXX_iPhoneOS -GXcode ..

xcodebuild -sdk iphoneos -configuration Release -target ALL_BUILD

cd $SIMULATOR_BUILDDIR

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$SRCDIR/ios_cmake/Toolchains/Toolchain-iPhoneSimulator_Xcode.cmake -DCMAKE_INSTALL_PREFIX=$LOG4CXX_iPhoneSimulator -GXcode ..

xcodebuild -sdk iphonesimulator -configuration Release -target ALL_BUILD

#===============================================================================
cd $SRCDIR
FRAMEWORK_NAME=$PACKAGE_NAME

mv $2/include/$PACKAGE_NAME/*.h $2
sh framework_gen.sh $FRAMEWORK_NAME $OS_BUILDDIR/Release-iphoneos $SIMULATOR_BUILDDIR/Release-iphonesimulator $2

else
mkdir -p $SRCDIR/$PACKAGE_NAME
mv -f $2/*.h $SRCDIR/$PACKAGE_NAME
fi

echo "Done !"
