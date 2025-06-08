#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROS_IP=$(ip route get 192.168.0.1 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export KIMERA_VIO_ROS2_REALSENSE_DIR=$ROMAN_ROS_WS/src/kimera-vio-ros2-realsense

tmuxp load $SCRIPT_DIR/d455_online.yaml