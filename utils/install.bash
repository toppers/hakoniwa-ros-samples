#!/bin/bash


if [ $# -ne 3 ]
then
	echo "Usage: $0 {ros1|ros2} <unity project root> <setting folder>" 
	exit 1
fi

ROS_VERSION=${1}
UNITY_PRJ_DIR=${2}
SETTING_FOLDER=${3}
ROS_JSON_DIR=${ROS_VERSION}/ros_json

if [ -d ${ROS_JSON_DIR} ]
then
	:
else
	echo "ERROR: Can not found folder: ${ROS_JSON_DIR}"
	exit 1
fi

if [ -d ${UNITY_PRJ_DIR} ]
then
	:
else
	echo "ERROR: Can not found folder: ${UNITY_PRJ_DIR}"
	exit 1
fi

if [ -d ${SETTING_FOLDER} ]
then
	:
else
	echo "ERROR: Can not found folder: ${SETTING_FOLDER}"
	exit 1
fi

PKG_NAME=`echo ${SETTING_FOLDER} | sed -e 's/\// /g' | awk '{print $NF}'`
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

cp ${ROS_VERSION}/template/*.tpl input/

bash ./utils/generate.bash ./input ./output ${PKG_NAME} ${HAS_MSG}

UPPER_PKG_NAME=`echo ${PKG_NAME} | tr '[:lower:]' '[:upper:]'`

UNITY_SRC_FILE1=./output/RosTopicIo.cs
UNITY_SRC_FILE2=./output/RosTopicPduReaderConverter.cs
UNITY_SRC_FILE3=./output/RosTopicPduWriterConverter.cs
UNITY_SRC_FILE4=./output/RosTopicPduCommTypedData.cs
UNITY_DST_DIR1=${UNITY_DST_DIR}/Method/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR2=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR3=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/
UNITY_DST_DIR4=${UNITY_DST_DIR}/Pdu/ROS/${UPPER_PKG_NAME}/


function copy()
{
	if [ -d $2 ]
	then
		:
	else
		mkdir -p $2
	fi
	cp -rp $1 $2
}

copy ${UNITY_SRC_FILE1} ${UNITY_DST_DIR1}
copy ${UNITY_SRC_FILE2} ${UNITY_DST_DIR2}
copy ${UNITY_SRC_FILE3} ${UNITY_DST_DIR3}
copy ${UNITY_SRC_FILE4} ${UNITY_DST_DIR4}

#rm -rf input
#rm -rf output
