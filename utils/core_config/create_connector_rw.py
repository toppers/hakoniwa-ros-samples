#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re

import hakoniwa_utils

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <out_dir> {r|w|p}"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
rw_type=sys.argv[3]

if rw_type == 'r':
	head_name = 'reader'
	out_filename='reader_connectors.json'
elif rw_type == 'w':
	head_name = 'writer'
	out_filename='writer_connectors.json'
else:
	out_filename='pdu_channel_connectors.json'

file = open(in_file)
ros_topics = json.load(file)

w_index=0
r_index=0
index=0
container = list()
for e in ros_topics['fields']:
	is_add = False
	if (e['sub'] == True):
		index = r_index
		r_index = r_index + 1
		is_add = (rw_type == 'r')
	if (e['sub'] == False):
		index = w_index
		w_index = w_index + 1
		is_add = (rw_type == 'w')
	if rw_type == 'p':
		is_add = True
		if (e['sub'] == True):
			head_name = 'reader'
		else:
			head_name = 'writer'

	if is_add:
		entry = dict()
		if rw_type != 'p':
			entry['name'] = head_name + '_connector' + str(index)
			entry['pdu_name'] = hakoniwa_utils.get_pdu_name(e)
			entry['method_name'] = 'ros_topic_io'
		else:
			entry[head_name+'_connector_name'] = head_name + '_connector' + str(index)
		container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

