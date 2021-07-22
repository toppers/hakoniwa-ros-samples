#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <in_dir> <ros_msg_list> <out_dir>"
	sys.exit()

in_dir=sys.argv[1]
in_file=sys.argv[2]
out_dir=sys.argv[3]
out_filename='pdu_configs.json'

container = list()
for pkg_msg in open(in_file, 'r'):
    entry = OrderedDict()
    entry['pdu_type_name'] = pkg_msg.strip()
    entry['pdu_data_field_path'] = in_dir + '/' + pkg_msg.strip() + '.json'
    container.append(entry)
    entry = OrderedDict()
    entry['pdu_type_name'] = pkg_msg.strip().split('/')[1]
    entry['pdu_data_field_path'] = in_dir + '/' + pkg_msg.strip() + '.json'
    container.append(entry)
    

entry = OrderedDict()
entry['pdu_type_name'] = 'time'
entry['pdu_data_field_path'] = in_dir + '/builtin_interfaces/Time.json'
container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)


