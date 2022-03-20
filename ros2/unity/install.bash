#!/bin/bash

#!/bin/bash

CURR=`pwd`

cd tb3/Assets
if [ -d Model ]
then
	:
	echo "Model is already installed."
else
	wget https://github.com/toppers/hakoniwa-ros-samples/releases/download/v1.3.0/Model.tar.gz
	tar xzvf Model.tar.gz
	rm -f Model.tar.gz
fi

if [ -d Plugin ]
then
	echo "Plugin is already installed."
else
	wget https://github.com/toppers/hakoniwa-ros-samples/releases/download/v1.3.0/Plugin.tar.gz
	tar xzvf Plugin.tar.gz
	rm -f Plugin.tar.gz
fi

if [ -d Resources/Hakoniwa/Robots ]
then
	echo "Resources is already installed."
else
	mkdir -p Resources/Hakoniwa/Robots
	cp -p Prefabs/Robots/TB3RoboModel.prefab*  Resources/Hakoniwa/Robots/
fi


