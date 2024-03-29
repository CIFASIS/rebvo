#!/bin/bash
#
# Perform several Rebvo operations in a Docker container.
# 1- Build docker image (compilation)
# 2- Run method in a docker container (dev mode)
# 3- Run method in a docker container (vis mode)

DEV_MODE=0
VIS_MODE=0
BUILD=0
DETACHED=0
MUTUALLY_EXCLUSIVE_OPTS=0
LAUNCH_FILE=rebvo_rosario.launch

# Get full directory name of the script no matter where it is being called from
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function echoUsage()
{
    echo -e "Usage: ./run.sh   -d | -v [detached] [-l LAUNCHFILE] | -b [-h]  \n\
                  \t -d dev mode \n\
                  \t -v vis mode \n\
                  \t\t 'detached' to run in background \n\
                  \t\t -l <NAME_OF_LAUNCHFILE> to run a specific launch file. \n\
                  \t\t\t It must be placed in launch/ \n\
                  \t -b build image \n\
                  \t -h help" >&2
}

function mutually_exclusive_opts() {
  if [ $MUTUALLY_EXCLUSIVE_OPTS -eq 1 ] ; then
    >&2 echo "ERROR: Only one option is allowed"
    echoUsage
    exit 1
  fi
}

unset SOME_OPT
while getopts "hdvl:b" opt; do
    SOME_OPT=1
    case "$opt" in
        h)
            echoUsage
            exit 0
            ;;
        d)  DEV_MODE=1
            mutually_exclusive_opts
            MUTUALLY_EXCLUSIVE_OPTS=1
            ;;
        v)  VIS_MODE=1
            mutually_exclusive_opts
            MUTUALLY_EXCLUSIVE_OPTS=1
            case $2 in
              "detached") DETACHED=1; shift ;;
              -* | "") ;;
              *) echoUsage; exit 1 ;;
            esac
            ;;
        b)  BUILD=1
            mutually_exclusive_opts
            MUTUALLY_EXCLUSIVE_OPTS=1
            ;;
        l)  if [ $VIS_MODE -eq 1 ] ; then
              case $OPTARG in
                -*) echo "ERROR: a path to launchfile must be provided"; echoUsage; exit 1 ;;
                *) LAUNCH_FILE=$OPTARG ;;
              esac
            else
              echoUsage; exit 1
            fi
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

if [ -z "$SOME_OPT" ]; then
  echoUsage
  exit 1
fi

if [ $DEV_MODE -eq 1 ] ; then
  docker run -it --net=host -v $CURRENT_DIR/:/root/catkin_ws/src/rebvo/ \
   rebvo:ros-indigo
fi

if [ $VIS_MODE -eq 1 ] ; then
  if [ $DETACHED -eq 1 ] ; then
    docker run -d --net=host \
       -v $CURRENT_DIR/ros/src/rebvo_ros/config/:/root/catkin_ws/src/rebvo/ros/src/rebvo_ros/config/ \
       -v $CURRENT_DIR/ros/src/rebvo_ros/launch/:/root/catkin_ws/src/rebvo/ros/src/rebvo_ros/launch/ \
       rebvo:ros-indigo \
       roslaunch rebvo $LAUNCH_FILE
  else
    docker run --net=host \
       -v $CURRENT_DIR/ros/src/rebvo_ros/config/:/root/catkin_ws/src/rebvo/ros/src/rebvo_ros/config/ \
       -v $CURRENT_DIR/ros/src/rebvo_ros/launch/:/root/catkin_ws/src/rebvo/ros/src/rebvo_ros/launch/ \
       rebvo:ros-indigo \
       roslaunch rebvo $LAUNCH_FILE
  fi
fi

if [ $BUILD -eq 1 ] ; then
  docker build --rm -t rebvo:ros-indigo $CURRENT_DIR
fi
