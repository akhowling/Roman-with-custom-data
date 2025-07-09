#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export OUTPUT_DIR=~/.roman_ros2
export RECORD=false

tmuxp load $SCRIPT_DIR/zed2i_online.yaml