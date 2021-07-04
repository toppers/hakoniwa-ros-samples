#!/bin/bash


if [ -f in.txt ]
then
	:
else
	echo "Usage: $0 in.txt" 
	exit 1
fi

ROS_JSON_DIR=../../ros_json/
UNITY_PRJ_DIR=../../../hakoniwa-single_robot/unity/assets/ros-sample
UNITY_DST_DIR=${UNITY_PRJ_DIR}/Assets/Scripts/Hakoniwa/PluggableAsset/Communication

rm -rf input
rm -rf output

mkdir input
mkdir output

for i in `cat in.txt`
do
	cp ${ROS_JSON_DIR}/${i} input/
done

cp ../../template/*.tpl input/

bash ./generate.bash ./input ./output


UNITY_SRC_FILE1=./output/RosTopicIo.cs
UNITY_SRC_FILE2=./output/RosTopicPduReaderConverter.cs
UNITY_SRC_FILE3=./output/RosTopicPduWriterConverter.cs
UNITY_DST_DIR1=${UNITY_DST_DIR}/Method/ROS/
UNITY_DST_DIR2=${UNITY_DST_DIR}/Pdu/ROS/
UNITY_DST_DIR3=${UNITY_DST_DIR}/Pdu/ROS/

cp ${UNITY_SRC_FILE1} ${UNITY_DST_DIR1}
cp ${UNITY_SRC_FILE2} ${UNITY_DST_DIR2}
cp ${UNITY_SRC_FILE3} ${UNITY_DST_DIR3}

rm -rf input
rm -rf output
