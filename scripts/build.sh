#!/bin/bash

source /opt/ros/indigo/setup.bash
cd $REBVO_ROOT && \
qmake && make REBVOFLAGS=-m64 && \
cd $CATKIN_WS && \
catkin_make
