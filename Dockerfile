FROM ubuntu:22.04 as prod

ENV DEBIAN_FRONTEND noninteractive
RUN apt update && apt install -y --no-install-recommends apt-utils && \
	apt upgrade -y && apt autoremove -y && \
	apt install -y build-essential cmake wget unzip git && \
	apt install -y python-is-python3 python3-pip && \
	rm -rf /var/lib/apt/lists/*

# suppress GTK warnings about accessibility because there's no dbus
# (WARNING **: Couldn't connect to accessibility bus: Failed to connect to socket /tmp/dbus-dw0fOAy4vj: Connection refused)
ENV NO_AT_BRIDGE 1
WORKDIR /home

#------------------------------	#
#     INSTALL OPENCV 4		    #
#------------------------------	#
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip && \
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip && \
	unzip opencv.zip && unzip opencv_contrib.zip && \
	mkdir -p build && cd build && \
	cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.5/modules ../opencv-4.5.5 && \
	cmake --build . --target install && \
	cd /home && rm opencv.zip && rm opencv_contrib.zip && \ 
	rm -rf opencv-4.5.5 && rm -rf opencv_contrib-4.5.5 && rm -rf build && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     INSTALL Boost		        #
#------------------------------	#
RUN apt update && apt install -y libboost-all-dev && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     Install Ceres		        #
#------------------------------	#
RUN apt update && apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev && \
	rm -rf /var/lib/apt/lists/*
RUN git clone --branch 20240722.0 --single-branch https://github.com/abseil/abseil-cpp.git && \
	cd abseil-cpp && mkdir build && cd build && \
	cmake .. && cmake --build . --target install && \
	cd /home && rm -rf abseil-cpp && \
	rm -rf /var/lib/apt/lists/*
RUN git clone --branch 2.2.0 --single-branch https://github.com/ceres-solver/ceres-solver.git && \
	cd ceres-solver && mkdir build && cd build && cmake .. && \
	make && make install && \
	cd /home && rm -rf ceres-solver && \
	rm -rf /var/lib/apt/lists/*

# Install python requirements for python_utils scripts
RUN --mount=type=bind,source=python_utils/requirements_prod.txt,target=/tmp/requirements.txt \
    python -m pip install --requirement /tmp/requirements.txt && \
	rm -rf /var/lib/apt/lists/*

FROM prod as dev

#------------------------------	#
#     Doxygen				    #
#------------------------------	#
RUN apt update && apt install -y flex bison && \ 
	git clone https://github.com/doxygen/doxygen.git && \
	cd doxygen && mkdir build && cd build &&\
	cmake -G 'Unix Makefiles' .. && \
	make && \ 
	make install && \
	cd /home && rm -rf doxygen && \
	rm -rf /var/lib/apt/lists/*

#------------------------------	#
#     For development		    #
#------------------------------	#
RUN apt update && apt install -y cppcheck clang-tidy valgrind lcov && \
	rm -rf /var/lib/apt/lists/*

# Install python requirements for python_utils scripts
RUN --mount=type=bind,source=python_utils/requirements_dev.txt,target=/tmp/requirements.txt \
    python -m pip install --requirement /tmp/requirements.txt && \
	rm -rf /var/lib/apt/lists/*