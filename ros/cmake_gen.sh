#!/bin/sh -x

PACKAGE_NAME=`basename $1`

[[ -f $PACKAGE_NAME.cmake ]] && rm -f $PACKAGE_NAME.cmake

SOURCE_FILES=$(find ./$1/src -name \*.cpp)

if [[ $? -ne 0 ]]
    then
        SOURCES_FILES_SIZE=0
else
    SOURCES_FILES_SIZE=${#SOURCE_FILES[@]}
fi

cat > $PACKAGE_NAME.cmake <<EOF
cmake_minimum_required(VERSION 2.8.0)

include_directories(
./$1/include
)

EOF

if [ $SOURCES_FILES_SIZE -ne 0 ]
    then
        cat >> $PACKAGE_NAME.cmake <<EOF
add_library($PACKAGE_NAME STATIC
$SOURCE_FILES
)

EOF

        for ARG in $*
            do
                if [ $ARG != $1 ]
                    then
                        cat >> $PACKAGE_NAME.cmake <<EOF

#find_library(FRAMEWORK_$ARG
#    NAMES $ARG
#    PATHS \${CMAKE_SYSTEM_FRAMEWORK_PATH}
#    PATH_SUFFIXES Frameworks
#    NO_DEFAULT_PATH)

#    if(\${FRAMEWORK_$ARG} STREQUAL FRAMEWORK_$ARG-NOTFOUND)
#        message(ERROR ": Framework $ARG not found")
#    else()
#        target_link_libraries($2 "\${FRAMEWORK_$ARG}/$ARG")
#        message(STATUS "Framework $ARG found at \${FRAMEWORK_$ARG}")
#    endif()

#set(CMAKE_EXE_LINKER_FLAGS \${CMAKE_EXE_LINKER_FLAGS} "-framework $ARG")

EOF
                fi
        done
fi