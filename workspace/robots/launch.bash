  
#!/bin/bash

source devel/setup.bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <pkgname>"
	exit 1
fi

roslaunch ${1} robo.launch
