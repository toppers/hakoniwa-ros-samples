#!/bin/bash

if [ $# -ne 2 ]
then
	echo "Usage: $0 <in> <out>"
	exit 1
fi

IN_DIR=${1}
OUT_DIR=${2}

for i in `ls ${IN_DIR}/*.msg`
do
	MSG_NAME=`echo $i | awk -F\. '{print $1}'`
	echo $MSG_NAME
	python utils/rosmsg2json.py ${i}
done

for i in `ls ${IN_DIR}/*.json`
do
	mv ${i} ${OUT_DIR}/
done

OUT_NAME=RosTopicIo
#python utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduReaderConverter
#python utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduWriterConverter
#python utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

