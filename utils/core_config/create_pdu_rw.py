#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re

if len(sys.argv) != 5:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <out_dir> <pkg_name> {r|w}"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
pkg_name=sys.argv[3]
rw_type=sys.argv[4]

if rw_type == 'r':
	out_filename='pdu_readers_config.json'
else:
	out_filename='pdu_writers_config.json'

file = open(in_file)
ros_topics = json.load(file)

def get_robolist():
	tmp_list = list()
	for e in ros_topics['fields']:
		tmp_list.append(e['robot_name'])
	return list(set(tmp_list))

def get_entry(type, name):
	tmp_list = list()
	for e in ros_topics['fields']:
		if e[type] == name:
			tmp_list.append( e )
	return tmp_list

robo_list = get_robolist()

container = list()
for e in ros_topics['fields']:
	name = e['robot_name'] + '_' + e['topic_type_name'] + 'Pdu'
	topic_message_name = e['topic_message_name']
	pdu_config_name = e['topic_type_name'] + 'Pdu'
	is_add = False
	if rw_type == 'r':
		class_name ='Hakoniwa.PluggableAsset.Communication.Pdu.ROS.RosTopicPduWriter'
		conv_class_name = 'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.' + pkg_name + '.RosTopicPduWriterConverter'
		is_add = (e['sub'] == True)
	else:
		class_name ='Hakoniwa.PluggableAsset.Communication.Pdu.ROS.RosTopicPduReader'
		conv_class_name = 'Hakoniwa.PluggableAsset.Communication.Pdu.ROS.' + pkg_name + '.RosTopicPduReaderConverter'
		is_add = (e['sub'] == False)

	if is_add:
		entry = dict()
		entry['name'] = name
		entry['class_name'] = class_name
		entry['conv_class_name'] = conv_class_name
		entry['topic_message_name'] = topic_message_name
		entry['pdu_config_name'] = pdu_config_name
		container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

