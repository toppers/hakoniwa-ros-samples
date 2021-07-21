#!/bin/bash

CORE_IPADDR=127.0.0.1
ROS_IPADDR=172.21.68.89
ROS_TOPIC_FILE=../../settings/ev3_tb3/RosTopics.json
ROS_TOPIC_FILE_PATH=../../../settings/ev3_tb3/RosTopics.json
OUT_DIR=../../ros2/unity/tb3_ros2/
PKG_NAME=EV3_TB3
PDU_CONFIG_INDIR=./ros_json
ROS_MSG_LIST=../all_msgs.txt

python create_core_config.py		${ROS_TOPIC_FILE_PATH} 	${CORE_IPADDR}  ${OUT_DIR}
python create_ros_topic_method.py	${PKG_NAME}			${OUT_DIR}
python create_inside_assets.py  	${ROS_TOPIC_FILE} 	${OUT_DIR}
python create_pdu_rw.py  			${ROS_TOPIC_FILE} 	${OUT_DIR} ${PKG_NAME} r
python create_pdu_rw.py  			${ROS_TOPIC_FILE} 	${OUT_DIR} ${PKG_NAME} w
python create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} r
python create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} w
python create_connector_rw.py  		${ROS_TOPIC_FILE} 	${OUT_DIR} p
python create_unity_ros_params.py 	${ROS_IPADDR}		${OUT_DIR}
python create_pdu_config.py			${PDU_CONFIG_INDIR}	${ROS_MSG_LIST}	${OUT_DIR}
