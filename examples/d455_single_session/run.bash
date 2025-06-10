#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export KIMERA_VIO_ROS2_REALSENSE_DIR=$ROMAN_ROS_WS/src/kimera-vio-ros2-realsense
export OUTPUT_DIR=~/.roman_ros2
export RECORD=false

tmuxp load $SCRIPT_DIR/d455_online.yaml