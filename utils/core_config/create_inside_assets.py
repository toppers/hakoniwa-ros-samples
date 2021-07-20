#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re

if len(sys.argv) != 3:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
out_filename='inside_assets.json'

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
for robo in robo_list:
	e_list = get_entry('robot_name', robo)
	entry = dict()
	entry['name'] = robo
	entry['pdu_writer_names'] = list()
	entry['pdu_reader_names'] = list()
	for e_list_entry in e_list:
		if e_list_entry['sub']:
			entry['pdu_reader_names'].append(robo + '_' + e_list_entry['topic_type_name'] + 'Pdu')
		else:
			entry['pdu_writer_names'].append(robo + '_' + e_list_entry['topic_type_name'] + 'Pdu')
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

