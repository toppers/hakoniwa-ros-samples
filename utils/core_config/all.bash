#!/bin/bash

if [ -z ${CORE_IPADDR} ]
then
	echo "ERROR: CORE_IPADDR is not set"
	exit
fi
if [ -z ${ROS_IPADDR} ]
then
	echo "ERROR: ROS_IPADDR is not set"
	exit
fi

if [ $# -ne 3 ]
then
	echo "Usage: $0 {ros1|ros2} <unity project root> <setting folder>" 
	exit 1
fi

ROS_VERSION=${1}
UNITY_PRJ_DIR=${2}
SETTING_FOLDER=${3}

PKG_NAME_LOWER=`echo ${SETTING_FOLDER} | awk -F/ '{print $2}'`
PKG_NAME=`echo ${PKG_NAME_LOWER} | tr '[a-z]' '[A-Z]'`
PDU_CONFIG_INDIR="../../ros_json"

ROS_TOPIC_FILE=${SETTING_FOLDER}/RosTopics.json
ROS_TOPIC_FILE_PATH=../../../${SETTING_FOLDER}/RosTopics.json
OUT_DIR=${UNITY_PRJ_DIR}
ROS_MSG_LIST=${SETTING_FOLDER}/ros_msgs.txt

CUSTOM_FILE_PATH=${SETTING_FOLDER}/custom.json

echo "####Creating core_config"
python utils/core_config/create_core_config.py			${ROS_TOPIC_FILE_PATH} 	${CORE_IPADDR}  ${OUT_DIR}
echo "####Creating ros_topic_method"
python utils/core_config/create_ros_topic_method.py		${PKG_NAME}			${OUT_DIR}
echo "####Creating inside_assets"
python utils/core_config/create_inside_assets.py  		${ROS_TOPIC_FILE} 	${OUT_DIR}

echo "####Creating pdu_readers"
python utils/core_config/create_pdu_rw.py  				${ROS_TOPIC_FILE} 	${OUT_DIR} ${PKG_NAME} r ${CUSTOM_FILE_PATH}
echo "####Creating pdu_writers"
python utils/core_config/create_pdu_rw.py  				${ROS_TOPIC_FILE} 	${OUT_DIR} ${PKG_NAME} w ${CUSTOM_FILE_PATH}

echo "####Creating reader_channels"
python utils/core_config/create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} r ${CUSTOM_FILE_PATH}
echo "####Creating writer_channels"
python utils/core_config/create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} w ${CUSTOM_FILE_PATH}
echo "####Creating pdu_channel_connectors"
python utils/core_config/create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} p ${CUSTOM_FILE_PATH}

echo "####Creating unity_ros_params"
python utils/core_config/create_unity_ros_params.py 	${ROS_IPADDR}		${OUT_DIR}

echo "####Creating pdu_config"
python utils/core_config/create_pdu_config.py			${PDU_CONFIG_INDIR}	${ROS_MSG_LIST}	${OUT_DIR} ${CUSTOM_FILE_PATH}

if [ -f ${CUSTOM_FILE_PATH} ]
then
	echo "####Creating outside_assets"
	python utils/core_config/create_outside_assets.py  		${CUSTOM_FILE_PATH} 	${OUT_DIR}

	echo "####Creating udp_methods"
	python utils/core_config/create_udp_methods.py  		${CUSTOM_FILE_PATH} 	${CORE_IPADDR}	${OUT_DIR}
fi
