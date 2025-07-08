#!/usr/bin/env bash

# Usage: ./run.bash <output_directory> <12>
# output_directory and 12 are optional
# 12, 1, 2, or "none" can be used to skip mapping sessions.

# Command line args
export OUTPUT_ROOT=${1:-''}
export RUNS=${2:-"12"}

# Internal vars
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"\

export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export RECORD=true


if [ -z "$OUTPUT_ROOT" ]; then
    read -p "Desired output directory: " OUTPUT_ROOT
    export OUTPUT_ROOT=$(eval echo "$OUTPUT_ROOT")
fi
mkdir -p $OUTPUT_ROOT

# Run first session of ROMAN mapping
if [[ "$RUNS" == *"1"* ]]; then
    read -p "Press enter to start first session. "
    export OUTPUT_DIR=$OUTPUT_ROOT/run1
    if [ -d "$OUTPUT_DIR" ]; then
        rm -r $OUTPUT_DIR
    fi
    mkdir -p $OUTPUT_DIR
    tmuxp load $SCRIPT_DIR/../zed2i_single_session/zed2i_online.yaml
else
    echo "Skipping session 1..."
fi

# Run second session of ROMAN mapping
if [[ "$RUNS" == *"2"* ]]; then
    read -p "Press enter to start second session. "
    export OUTPUT_DIR=$OUTPUT_ROOT/run2
    if [ -d "$OUTPUT_DIR" ]; then
        rm -r $OUTPUT_DIR
    fi
    mkdir -p $OUTPUT_DIR
    tmuxp load $SCRIPT_DIR/../zed2i_single_session/zed2i_online.yaml
else
    echo "Skipping session 2..."
fi

mkdir -p $OUTPUT_ROOT/roman/map
cp $OUTPUT_ROOT/run1/roman_map.pkl $OUTPUT_ROOT/roman/map/run1.pkl
cp $OUTPUT_ROOT/run2/roman_map.pkl $OUTPUT_ROOT/roman/map/run2.pkl

$(eval echo "$ROMAN_ENV_ACTIVATE")
python3 $ROMAN_ROS_WS/src/roman/demo/demo.py -p $SCRIPT_DIR/params -o $OUTPUT_ROOT/roman --skip-map --skip-rpgo

python3 $ROMAN_ROS_WS/src/roman/demo/association_vid.py $OUTPUT_ROOT/roman $OUTPUT_ROOT/association_vid.mp4 -r run1 run2 -m
xdg-open $OUTPUT_ROOT/association_vid.mp4