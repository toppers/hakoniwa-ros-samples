#!/bin/bash

export CORE_IPADDR=127.0.0.1
export ROS_IPADDR=`ifconfig eth0 | grep netmask | awk '{print $2}'`

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

echo "###Phase1: Parsing ros_msgs from ${SETTING_FOLDER}/RosTopics.json"
bash utils/create_all_ros_msgs.bash ${SETTING_FOLDER}/${ROS_VERSION}_search_file_path.txt ${SETTING_FOLDER}/RosTopics.json | tee ${SETTING_FOLDER}/ros_msgs.txt
echo "###Phase1: Succless"

echo "###Phase2: Creating ros_json from ${SETTING_FOLDER}/ros_msgs.txt"
bash utils/convert_rosmsg2json.bash settings/ev3_tb3/ros2_search_file_path.txt settings/ev3_tb3/ros_msgs.txt ${ROS_JSON_DIR}
echo "###Phase2: Succless"

echo "###Phase3: Creating core_config"
bash utils/core_config/all.bash ${ROS_VERSION} ${UNITY_PRJ_DIR} ${SETTING_FOLDER}
echo "###Phase3: Succless"

echo "###Phase4: Creating Unity Scripts"

if [ -f ${SETTING_FOLDER}/ros_msgs.txt ]
then
	for i in `cat ${SETTING_FOLDER}/ros_msgs.txt`
	do
		cp ${ROS_JSON_DIR}/${i}.json input/
	done
	cat ${SETTING_FOLDER}/ros_msgs.txt  | awk -F/ '{print $1}' | sort | uniq | awk -F_msgs '{print $1}' > input/msg_pkg.txt
fi

cp ${ROS_VERSION}/template/*.tpl input/

bash ./utils/generate.bash ./input ./output ${PKG_NAME}

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

echo "###Phase4: Succless"

