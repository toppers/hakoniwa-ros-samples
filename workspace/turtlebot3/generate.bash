#!/bin/bash

if [ $# -ne 2 ]
then
	echo "Usage: $0 <in> <out>"
	exit 1
fi
IN_DIR=${1}
OUT_DIR=${2}

OUT_NAME=RosTopicIo
python ../../utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduReaderConverter
python ../../utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduWriterConverter
python ../../utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} > ${OUT_DIR}/${OUT_NAME}.cs

