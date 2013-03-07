#!/bin/sh -x

[[ -f ./$2.cmake ]] && rm -f ./$2.cmake

cat > ./$2.cmake <<EOF
cmake_minimum_required(VERSION 2.8.0)

include_directories(
./$1/$2/include
)

add_library($2 STATIC
$(find ./$1/$2 -name \*.cpp)
)

EOF

if [ $# -gt 2 ]
    then
        echo "Dependencies:"
        for ARG in $*
            do
                if [ $ARG != $1 -a $ARG != $2 ]
                    then
                        echo $ARG
                        cat >> ./$2.cmake <<EOF
find_library(FRAMEWORK_$ARG
    NAMES $ARG
    PATHS ${CMAKE_SYSTEM_FRAMEWORK_PATH}
    PATH_SUFFIXES Frameworks
    NO_DEFAULT_PATH)
EOF
                fi
        done
fi