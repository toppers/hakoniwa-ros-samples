#!/bin/bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <rosmsg>"
	exit 1
fi

ROSMSG_FILE=${1}

LINE_NUM=`wc -l ${ROSMSG_FILE} | awk '{print $1}'`

CNT=1
echo "{"
echo "    \"fields\": ["
while read line 
do 
	name=`echo $line | awk '{print $2}'`
	type=`echo $line | awk '{print $1}'`
	echo "        {"
	echo "            \"name\": \"${name}\","
	echo "            \"type\": \"${type}\""
	if [ ${CNT} -eq ${LINE_NUM} ]
	then
		echo "        }"
	else
		echo "        },"
	fi
	CNT=`expr ${CNT} \+ 1`
done < ${ROSMSG_FILE}
echo "    ]"
echo "}"
