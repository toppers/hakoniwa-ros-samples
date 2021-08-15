#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <custom> <core_ipaddr> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
core_ipaddr=sys.argv[2]
out_dir=sys.argv[3]

file = open(in_file)
custom = json.load(file)


for robo in custom['proxies']:
	entry = OrderedDict()
	entry['asset_name'] = robo['robot_name'] + 'Proxy'
	entry['target_exec_dir'] = robo['proxy_sync']['target_exec_dir']
	entry['target_bin_path'] = robo['proxy_sync']['target_bin_path']
	entry['target_options'] = robo['proxy_sync']['target_options']

	#tx
	entry['sync_tx_ipaddr'] = core_ipaddr
	entry['sync_tx_portno'] = robo['proxy_sync']['rx_udp_portno']

	#rx
	if robo['proxy_sync']['ipaddr'] != None:
		entry['sync_rx_ipaddr'] = robo['proxy_sync']['ipaddr']
	else:
		entry['sync_rx_ipaddr'] = '127.0.0.1'
	entry['sync_rx_portno'] = robo['proxy_sync']['tx_udp_portno']
	entry['sync_interval_msec'] = robo['proxy_sync']['interval_time_msec']

	out_filename= robo['robot_name'] + '_proxy_param.json'
	with open(out_dir + '/' + out_filename, mode='wt') as out_file:
	  json.dump(entry, out_file, ensure_ascii=False, indent=2)

