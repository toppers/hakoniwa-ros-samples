#!/bin/bash

#!/bin/bash

which unzip > /dev/null
if [ $? -eq 1 ]
then
	sudo apt install unzip
fi

BINARY_NAME=WindowsBinary.zip


wget https://github.com/toppers/hakoniwa-ros-samples/releases/download/v1.0.0/${BINARY_NAME}

mv ${BINARY_NAME} unity/
cd unity
unzip ${BINARY_NAME}
chmod +x WindowsBinary/ros-sample.exe
rm -f ${BINARY_NAME}
