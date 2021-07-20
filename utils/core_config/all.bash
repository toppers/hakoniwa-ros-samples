#!/bin/bash

python create_inside_assets.py  ../../settings/ev3_tb3/RosTopics.json out/
python create_pdu_rw.py  ../../settings/ev3_tb3/RosTopics.json out/ EV3_TB3 r
python create_pdu_rw.py  ../../settings/ev3_tb3/RosTopics.json out/ EV3_TB3 w
python create_connector_rw.py  ../../settings/ev3_tb3/RosTopics.json out/ r
python create_connector_rw.py  ../../settings/ev3_tb3/RosTopics.json out/ w
python create_connector_rw.py  ../../settings/ev3_tb3/RosTopics.json out/ p
