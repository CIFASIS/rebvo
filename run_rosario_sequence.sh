#!/bin/bash
#
# Run rosbag play, visualization and save trajectory in a text file.
# It requires a catkin workspace containing pose_listener.

set -eE # Any subsequent commands which fail will cause the shell script to exit immediately
CONTAINER_OUTPUT_FILE=/root/catkin_ws/src/rebvo/rebvo_ros_tray.txt

# Get full directory name of the script no matter where it is being called from
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function echoUsage()
{
    echo -e "Usage: ./run_rosario_sequence.sh [FLAG] ROSBAG\n\
            \t -r run method in a detached docker container \n\
            \t -o path to output file \n\
            \t -h help" >&2
}

RUN_CONTAINER=0
OUTPUT_FILE=$CURRENT_DIR/trajectory_$(date '+%Y%m%d_%H%M%S').txt
while getopts "hro:" opt; do
    case "$opt" in
        h)  echoUsage
            exit 0
            ;;
        r)  RUN_CONTAINER=1
            ;;
        o)  case $OPTARG in
                -*) echo "ERROR: a path to output file must be provided"; echoUsage; exit 1 ;;
                *) OUTPUT_FILE=$OPTARG ;;
            esac
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

shift $((OPTIND -1))
BAG=$1

function cleanup() {
  if [ -n "${CID}" ] ; then
    printf "\e[31m%s %s\e[m\n" "Cleaning"
    docker container stop $CID > /dev/null
    docker cp $CID:$CONTAINER_OUTPUT_FILE $OUTPUT_FILE
    docker logs $CID > $(dirname $OUTPUT_FILE)/$(basename -s .txt $OUTPUT_FILE)_log.txt
    docker rm $CID > /dev/null
    unset CID
  fi
  # rosnode kill -a
}

trap cleanup INT
trap cleanup ERR

function wait_docker() {
    TOPIC=$1
    attempts=0
    max_attempts=30
    output=$(rostopic list $TOPIC 2> /dev/null || :) # force the command to exit successfully (i.e. $? == 0) to avoid trap
    while [ "$attempts" -lt "$max_attempts" ] && [ "$output" != $TOPIC ]; do
        sleep 1
        output=$(rostopic list $TOPIC 2> /dev/null || :)
        attempts=$(( attempts + 1 ))
    done
    if [ "$attempts" -eq "$max_attempts" ] ; then
        echo "ERROR: System seems not to start"
        cleanup
        exit 1
    fi
}

if [ $RUN_CONTAINER -eq 1 ] ; then
    echo "Starting docker container (detached mode)"
    CID=$($CURRENT_DIR/run.sh -v detached)
fi

wait_docker "/rebvo_edgemap" # FIX Hack! Just to be sure docker container is running

roslaunch $CURRENT_DIR/ros/src/rebvo_ros/launch/play_bag_viz.launch \
    config_rviz:=$CURRENT_DIR/ros/src/rebvo_ros/resource/rebvo_rosario.rviz \
    bagfile:=$BAG

cleanup
echo "END"