FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get upgrade -y
RUN apt-get autoremove -y
RUN apt-get install -y build-essential

# Text editors
RUN apt-get install -y \
	gedit \
	vim \ 
	wget \
	tmux \
	sed

#cmake
RUN apt-get install -y cmake

# suppress GTK warnings about accessibility because there's no dbus
# (WARNING **: Couldn't connect to accessibility bus: Failed to connect to socket /tmp/dbus-dw0fOAy4vj: Connection refused)
ENV NO_AT_BRIDGE 1
ENV HOME /kaist
WORKDIR /kaist

#------------------------------	#
#     INSTALL OPENCV 4		#
#------------------------------	#
RUN apt install -y libopencv-dev python3-opencv

#------------------------------	#
#     INSTALL Boost		#
#------------------------------	#

RUN apt install -y libboost-all-dev

#------------------------------	#
#     Install Ceres		#
#------------------------------	#
RUN apt-get update
RUN apt-get install 
RUN apt-get update && apt-get install -y \
	libgoogle-glog-dev \
	libgflags-dev \
	libatlas-base-dev \
	libeigen3-dev \
	libsuitesparse-dev

RUN apt-get install -y git
RUN mkdir -p /Ceres
RUN /bin/bash -c "cd /Ceres && \
	git clone https://ceres-solver.googlesource.com/ceres-solver && \
	cd ceres-solver && \
	cmake . && \
	make -j8 && \ 
	make -j8 test && \
	make install"





























