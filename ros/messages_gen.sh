#!/bin/sh -x

#===============================================================================

SRCDIR=`pwd`

DEPENDENCIES="-Istd_msgs:$SRCDIR/std_msgs/msg/"

for ARG in $*
    do
        DEPENDENCIES=${DEPENDENCIES}" -I$ARG:$SRCDIR/common_msgs/$ARG/msg/"
done

#===============================================================================
echo "Generating $1 messages ..."

FILES=$SRCDIR/common_msgs/$1/msg/*.msg

for f in $FILES
    do
        python $SRCDIR/gencpp/scripts/gen_cpp.py $f $DEPENDENCIES -p $1 -o $SRCDIR/$1 -e $SRCDIR/gencpp/scripts/
done