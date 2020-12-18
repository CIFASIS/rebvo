#!/bin/bash

source /opt/ros/indigo/setup.bash
cd $REBVO_ROOT && \
qmake && make REBVOFLAGS='-m64 -DSAVE_TIMES=ON' && \
cd $CATKIN_WS && \
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DSAVE_TIMES=ON
