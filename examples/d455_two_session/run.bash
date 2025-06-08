#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROS_IP=$(ip route get 192.168.0.1 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../../)
export KIMERA_VIO_ROS2_REALSENSE_DIR=$ROMAN_ROS_WS/src/kimera-vio-ros2-realsense
export RECORD=true

read -p "Desired output directory: " OUTPUT_ROOT
export OUTPUT_ROOT=$(eval echo "$OUTPUT_ROOT")
mkdir -p $OUTPUT_ROOT

# Run first session of ROMAN mapping
read -p "Press enter to start first session. "
export OUTPUT_DIR=$OUTPUT_ROOT/run1
mkdir -p $OUTPUT_DIR
tmuxp load $SCRIPT_DIR/../d455_single_session/d455_online.yaml

# Run second session of ROMAN mapping
read -p "Press enter to start second session. "
export OUTPUT_DIR=$OUTPUT_ROOT/run2
mkdir -p $OUTPUT_DIR
tmuxp load $SCRIPT_DIR/../d455_single_session/d455_online.yaml

mkdir -p $OUTPUT_ROOT/roman/map
cp $OUTPUT_ROOT/run1/roman_map.pkl $OUTPUT_ROOT/roman/map/run1.pkl
cp $OUTPUT_ROOT/run2/roman_map.pkl $OUTPUT_ROOT/roman/map/run2.pkl

$(eval echo "$ROMAN_ENV_ACTIVATE")
python3 $ROMAN_ROS_WS/src/roman/demo/demo.py -p $SCRIPT_DIR/params -o $OUTPUT_ROOT/roman --skip-map --skip-rpgo

python3 $ROMAN_ROS_WS/src/roman/demo/association_vid.py $OUTPUT_ROOT/roman $OUTPUT_ROOT/association_vid.mp4 -r run1 run2 -m
xdg-open $OUTPUT_ROOT/association_vid.mp4