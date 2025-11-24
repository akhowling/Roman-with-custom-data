#!/usr/bin/env bash
# Usage: ./run_single_session.sh <bag path> <output directory (optional)>
export ROMAN_BAG=$1
export ROMAN_OUTPUT_DIR=${2:-"~/.roman_ros2"}

if [ ! -d "$ROMAN_BAG" ]; then
    echo "ERROR: Missing positional command line argument 'ROMAN_BAG' or ROMAN_BAG does not exist"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export ROMAN_PRIOR_MAP=""
export ROMAN_LC_CONFIG_DIR=$(realpath $SCRIPT_DIR/../../roman_ros2/cfg/d455)

tmuxp load $SCRIPT_DIR/d455_bag.yaml