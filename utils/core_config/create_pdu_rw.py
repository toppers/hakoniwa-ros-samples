#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re

import hakoniwa_utils

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


container = list()
for e in ros_topics['fields']:
	is_add = False
	if rw_type == 'r':
		is_add = (e['sub'] == True)
	else:
		is_add = (e['sub'] == False)

	if is_add:
		entry = dict()
		entry['name'] = hakoniwa_utils.get_pdu_name(e)
		entry['class_name'] = hakoniwa_utils.get_pdu_class_name(rw_type)
		entry['conv_class_name'] = hakoniwa_utils.get_pdu_conv_class_name(rw_type, pkg_name)
		entry['topic_message_name'] = hakoniwa_utils.get_pdu_topic_name(e)
		entry['pdu_config_name'] = hakoniwa_utils.get_pdu_type_name(e)
		container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

