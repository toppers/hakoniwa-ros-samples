#!/bin/bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <ros_msg_list_file>"
	exit 1
fi

ROS_MSG_LIST_FILE=${1}

rm -f _msgs.txt
for i in `cat ${ROS_MSG_LIST_FILE}`
do
	bash parse_msg_recursive.bash search_file_path.txt ${i} >> _msgs.txt
done
cat _msgs.txt | sort | uniq 
rm -f _msgs.txt

