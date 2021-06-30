#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys

if len(sys.argv) != 2:
    print('Usage: ' + sys.argv[0] + ' <ROS Message>')
    sys.exit()

in_file = sys.argv[1]
out_file = in_file.replace('.msg', '.json')

out_data = dict()
out_data['fields'] = list()

text = open(in_file, 'r')
for line in text:
  tmp = line.strip()
  if (len(tmp) > 0) and  (tmp[0:1] != '#'):
  	array=tmp.split()
	entry = { 'name': array[1], 'type': array[0] }
	out_data['fields'].append(entry)

#print(json.dumps(out_data, ensure_ascii=False, indent=2))

with open(out_file, mode='wt') as file:
  json.dump(out_data, file, ensure_ascii=False, indent=2)

text.close()
