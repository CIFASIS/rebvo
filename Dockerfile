FROM ros:indigo-perception
# ubuntu:16.04

WORKDIR /root/

RUN apt-get update && \
    apt-get install -y g++ git cmake gfortran \
    nasm pkg-config \
    unzip \
    wget \
    qt5-qmake \
    libv4l-dev \
    libx11-dev \
    mesa-common-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    zlib1g-dev \
    libpng-dev \
    libjpeg-dev \
    ros-indigo-tf-conversions ros-indigo-rviz vim \
    libgflags-dev libgoogle-glog-dev && \
    rm -rf /var/lib/apt/lists/* && \
    git clone https://github.com/Reference-LAPACK/lapack.git && \
    mkdir -p /root/lapack/build && \
    cd /root/lapack/build && \
    cmake .. -DBUILD_SHARED_LIBS=ON && \
    cmake --build . --target install && \
    cd /root/ && \
    git clone git://git.libav.org/libav.git libav && \
    cd /root/libav/ && \
    ./configure && make && make install && \
    cd /root/ && \
    wget https://github.com/libgd/libgd/releases/download/gd-2.2.5/libgd-2.2.5.tar.gz && \
    tar -xzf libgd-2.2.5.tar.gz && \
    cd /root/libgd-2.2.5 && \
    ./configure && make && make install

COPY ./ /root/rebvo
# WORKDIR /root/rebvo
ENV LD_LIBRARY_PATH /usr/local/lib
ENV QT_X11_NO_MITSHM 1
RUN unzip /root/rebvo/TooN-2.2.zip -d /root/TooN && \
    cd /root/TooN/TooN-2.2 && \
    ./configure && make && make install && \
    cd /root/rebvo/ && \
    qmake && \
    make REBVOFLAGS=-m64 && \
    cd /root/rebvo/ros && \
   /bin/bash -c '. /opt/ros/indigo/setup.bash; catkin_make'

WORKDIR /root/rebvo/app/rebvorun
    #make REBVOFLAGS=-m64  
## TODO: catkin_make  
    

    
    
  
    

    
    

    

