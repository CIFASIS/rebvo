FROM ros:indigo-perception
# ubuntu:16.04

WORKDIR /root/

RUN apt-get update && \
    apt-get install -y g++ cmake gfortran \
    nasm pkg-config \
    qt5-qmake \
    libv4l-dev \
    libx11-dev \
    mesa-common-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    zlib1g-dev \
    libpng-dev \
    libjpeg-dev \
    ros-indigo-tf-conversions \
    libgflags-dev libgoogle-glog-dev && \
    rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH /usr/local/lib
ENV QT_X11_NO_MITSHM 1
ENV CATKIN_WS=/root/catkin_ws REBVO_ROOT=/root/catkin_ws/src/rebvo REBVO_THIRDPARTY=/root/catkin_ws/src/rebvo/thirdparty REBVO_ROS=/root/catkin_ws/src/rebvo/ros
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib/x86_64-linux-gnu/"

COPY ./ $REBVO_ROOT

RUN mkdir -p $REBVO_THIRDPARTY/lapack/build && \
     cd $REBVO_THIRDPARTY/lapack/build && \
     cmake .. -DBUILD_SHARED_LIBS=ON && \
     cmake --build . --target install && \
     cd $REBVO_THIRDPARTY/libav/ && \
     ./configure && make && make install && \
     mkdir -p $REBVO_THIRDPARTY/libgd/build && \
     cd $REBVO_THIRDPARTY/libgd/build && \
     cmake -DBUILD_TEST=1 -DENABLE_JPEG=On -DENABLE_PNG=On .. && make && make install && \
     cd $REBVO_THIRDPARTY/TooN && \
     ./configure && make && make install

WORKDIR $CATKIN_WS
COPY ./scripts/ $CATKIN_WS
RUN ["/bin/bash", "-c", "chmod +x build.sh && chmod +x modify_entrypoint.sh && sync && ./build.sh && ./modify_entrypoint.sh"]
