FROM ubuntu:20.04 as prod

ENV DEBIAN_FRONTEND noninteractive
RUN apt update && apt install -y --no-install-recommends apt-utils && \
	apt upgrade -y && apt autoremove -y && \
	apt install -y build-essential cmake && \
	rm -rf /var/lib/apt/lists/*

# suppress GTK warnings about accessibility because there's no dbus
# (WARNING **: Couldn't connect to accessibility bus: Failed to connect to socket /tmp/dbus-dw0fOAy4vj: Connection refused)
ENV NO_AT_BRIDGE 1
WORKDIR /home

#------------------------------	#
#     INSTALL OPENCV 4		    #
#------------------------------	#
RUN apt update && apt install -y libopencv-dev && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     INSTALL Boost		        #
#------------------------------	#
RUN apt update && apt install -y libboost-all-dev && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     Install Ceres		        #
#------------------------------	#
RUN apt update && \
	apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev && \
	rm -rf /var/lib/apt/lists/*
RUN apt update && apt install -y git && \
	/bin/bash -c "git clone https://ceres-solver.googlesource.com/ceres-solver && \
	cd ceres-solver && mkdir build && cd build && cmake .. && \
	make && make install && \
	cd /home && rm -rf ceres-solver" && \
	rm -rf /var/lib/apt/lists/*

FROM prod as dev

RUN apt update && apt install -y python3-opencv && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     Doxygen				    #
#------------------------------	#
RUN apt update && apt install -y flex bison && \ 
	/bin/bash -c "git clone https://github.com/doxygen/doxygen.git && \
	cd doxygen && mkdir build && cd build &&\
	cmake -G 'Unix Makefiles' .. && \
	make && \ 
	make install && \
	cd /home && rm -rf doxygen" && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     For development		    #
#------------------------------	#
RUN apt update && apt install -y cppcheck clang-tidy valgrind && \
	rm -rf /var/lib/apt/lists/*