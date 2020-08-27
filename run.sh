XAUTH=/tmp/.docker.xauth; \
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - ; \
docker run -it --rm --net=host --device /dev/dri  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $XAUTH:$XAUTH  -e XAUTHORITY=$XAUTH -e DISPLAY=$DISPLAY -v $1:/dataset rebvo:ros_indigo #rebvo:ubuntu_18.04



