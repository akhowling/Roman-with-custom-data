#!/usr/bin/env bash
# Usage: ./run_with_prior_map.sh <bag path> <prior map path> <output directory (optional)>
export ROMAN_BAG=$1
export ROMAN_PRIOR_MAP=$2
export ROMAN_OUTPUT_DIR=${3:-"~/.roman_ros2"}

if [ ! -d "$ROMAN_BAG" ]; then
    echo "ERROR: Missing positional command line argument 'ROMAN_BAG' or ROMAN_BAG does not exist"
    exit
fi

if [ ! -f "$ROMAN_PRIOR_MAP" ]; then
    echo "ERROR: Missing positional command line argument 'ROMAN_PRIOR_MAP' or ROMAN_PRIOR_MAP does not exist"
    exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export ROMAN_LC_CONFIG_DIR=$(realpath $SCRIPT_DIR/../../roman_ros2/cfg/d455_with_prior_map)

tmuxp load $SCRIPT_DIR/d455_bag.yaml