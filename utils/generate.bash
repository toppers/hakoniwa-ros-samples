#!/bin/bash

if [ $# -ne 4 ]
then
	echo "Usage: $0 <in> <out> <pkg_name> <has_msg>"
	exit 1
fi
IN_DIR=${1}
OUT_DIR=${2}
PKG_NAME=${3}
HAS_MSG=${4}

OUT_NAME=RosTopicIo
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} ${HAS_MSG} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduCommTypedData
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} ${HAS_MSG} > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduReaderConverter
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} ${HAS_MSG}  > ${OUT_DIR}/${OUT_NAME}.cs

OUT_NAME=RosTopicPduWriterConverter
python ./utils/generate.py ${OUT_NAME} ${IN_DIR} ${OUT_DIR} ${PKG_NAME} ${HAS_MSG}  > ${OUT_DIR}/${OUT_NAME}.cs

