FROM ubuntu:20.04 as prod

ENV DEBIAN_FRONTEND noninteractive
RUN apt update && apt install -y --no-install-recommends apt-utils && \
	apt upgrade -y && apt autoremove -y && \
	apt install -y build-essential cmake

# suppress GTK warnings about accessibility because there's no dbus
# (WARNING **: Couldn't connect to accessibility bus: Failed to connect to socket /tmp/dbus-dw0fOAy4vj: Connection refused)
ENV NO_AT_BRIDGE 1
WORKDIR /home

#------------------------------	#
#     INSTALL OPENCV 4		    #
#------------------------------	#
RUN apt install -y libopencv-dev 

#------------------------------	#
#     INSTALL Boost		        #
#------------------------------	#
RUN apt install -y libboost-all-dev

#------------------------------	#
#     Install Ceres		        #
#------------------------------	#
RUN apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
RUN apt install -y git && \
	/bin/bash -c "git clone https://ceres-solver.googlesource.com/ceres-solver && \
	cd ceres-solver && cmake . && \
	make -j3 && make test && \
	make install && \
	cd /home && rm -rf ceres-solver"

FROM prod as dev

RUN apt install -y python3-opencv

#------------------------------	#
#     Doxygen				    #
#------------------------------	#
RUN apt install -y flex bison && \ 
	/bin/bash -c "git clone https://github.com/doxygen/doxygen.git && \
	cd doxygen && mkdir build && cd build &&\
	cmake -G 'Unix Makefiles' .. && \
	make && \ 
	make install && \
	cd /home && rm -rf doxygen"

#------------------------------	#
#     For development		    #
#------------------------------	#
RUN apt install -y cppcheck clang-tidy valgrind

	




























