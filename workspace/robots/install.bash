#!/bin/bash


if [ $# -ne 1 ]
then
	echo "Usage: $0 <setting folder>" 
	exit 1
fi

if [ -d ${1} ]
then
	:
else
	echo "ERROR: Can not found folder: ${1}"
	exit 1
fi

SETTING_FOLDER=${1}
PKG_NAME=`echo ${SETTING_FOLDER} | sed -e 's/\// /g' | awk '{print $NF}'`

ROS_JSON_DIR=../../ros_json/
UNITY_PRJ_DIR=../unity/ros-sample
UNITY_DST_DIR=${UNITY_PRJ_DIR}/Assets/Scripts/Hakoniwa/PluggableAsset/Communication

rm -rf input
rm -rf output

mkdir input
mkdir output

if [ -f ${SETTING_FOLDER}/ros_msgs.txt ]
then
	for i in `cat ${SETTING_FOLDER}/ros_msgs.txt`
	do
		cp ${ROS_JSON_DIR}/${i} input/
	done
fi

HAS_MSG="true"
if [ -f ${SETTING_FOLDER}/my_msgs.txt ]
then
	for i in `cat ${SETTING_FOLDER}/my_msgs.txt`
	do
		cp ${ROS_JSON_DIR}/${i} input/
	done
else
	HAS_MSG="false"
fi

cp ../../template/*.tpl input/

bash ./generate.bash ./input ./output ${PKG_NAME} ${HAS_MSG}

UPPER_PKG_NAME=`echo ${PKG_NAME} | tr '[:lower:]' '[:upper:]'`

UNITY_SRC_FILE1=./output/RosTopicIo.cs
UNITY_SRC_FILE2=./output/RosTopicPduReaderConverter.cs
UNITY_SRC_FILE3=./output/RosTopicPduWriterConverter.cs
UNITY_SRC_FILE4=./output/RosTopicPduCommTypedData.cs
UNITY_DST_DIR1=${UNITY_DST_DIR}/Method/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR2=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR3=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR4=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/

cp ${UNITY_SRC_FILE1} ${UNITY_DST_DIR1}
cp ${UNITY_SRC_FILE2} ${UNITY_DST_DIR2}
cp ${UNITY_SRC_FILE3} ${UNITY_DST_DIR3}
cp ${UNITY_SRC_FILE4} ${UNITY_DST_DIR4}

#rm -rf input
#rm -rf output
