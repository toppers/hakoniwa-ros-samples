FROM ros:foxy as hakoniwa-ros2-builder

ARG DEBIAN_FRONTEND=noninteractive

ENV CMAKE_INSTALL_DIR /local
ENV PATH $PATH:/grpc/cmake/build
ENV PATH $PATH:/grpc/cmake/build/third_party/protobuf
ENV PATH $PATH:$CMAKE_INSTALL_DIR/bin

RUN apt-get update && apt-get install -y \
	git	\
	build-essential	\
	wget	\
	gcc	\
	g++	\
	ruby	\
	vim	\
	libssl-dev \
	make	\
	autoconf \
	automake \
	pkg-config \
	curl \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN	wget -q -O cmake-linux.sh https://github.com/Kitware/CMake/releases/download/v3.17.0/cmake-3.17.0-Linux-x86_64.sh && \
	mkdir -p $CMAKE_INSTALL_DIR && \
	sh cmake-linux.sh --skip-license --prefix=$CMAKE_INSTALL_DIR && \
	rm cmake-linux.sh

RUN mkdir /root/workspace
WORKDIR /root/workspace

#install python
RUN apt update &&  apt install -y python && \
	curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && \
	python get-pip.py && \
	pip install jinja2

#install mqtt
RUN apt-get install -y software-properties-common 
RUN add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
RUN apt-get install -y libmosquitto-dev

WORKDIR /root/workspace/hakoniwa-ros-samples/ros2/workspace
