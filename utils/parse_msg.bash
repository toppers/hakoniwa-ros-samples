#!/bin/bash

if [ $# -ne 2 ]
then
    echo "Usage: $0 <install_dir> <pkg/msg>"
    exit 1
fi

ROS_INSTALL_DIR=${1}
PKG_MSG=${2}
PKG_NAME=`echo ${PKG_MSG} | awk -F/ '{print $1}'`
MSG_NAME=`echo ${PKG_MSG} | awk -F/ '{print $2}'`

if [ -f ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${MSG_NAME} ]
then
    :
else
    exit 1
fi

cat ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${MSG_NAME} | grep -v "^#" | grep -v "^$" | awk '{print $1}' | sort |uniq > tmp

for i in `cat tmp`
do
    COUNT=`echo $i | awk -F/ '{print NF}'`
    if [ $COUNT -eq 2 ]
    then
        echo $i
    else
        if [ -f ${ROS_INSTALL_DIR}/${PKG_NAME}/msg/${i}.msg ]
        then
            echo ${PKG_NAME}/$i
        fi
    fi
done

rm -f tmp

exit 0