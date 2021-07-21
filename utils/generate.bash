#!/bin/bash

if [ $# -ne 3 ]
then
	echo "Usage: $0 <in> <out> <pkg_name>"
	exit 1
fi
IN_DIR=${1}
OUT_DIR=${2}
PKG_NAME=${3}

OUT_NAME=RosTopicIo
echo "###Creating ${OUT_DIR}/${OUT_NAME}.cs"
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduCommTypedData
echo "###Creating ${OUT_DIR}/${OUT_NAME}.cs"
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduReaderConverter
echo "###Creating ${OUT_DIR}/${OUT_NAME}.cs"
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduWriterConverter
echo "###Creating ${OUT_DIR}/${OUT_NAME}.cs"
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} > ${OUT_DIR}/${OUT_NAME}.cs

