#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export ROS_IP=$(ip route get 192.168.0.1 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export ROS_MASTER_URI=http://$ROS_IP:11311
export KIMERA_VIO_ROS2_REALSENSE_DIR=$SCRIPT_DIR/../../kimera-vio-ros2-realsense
export ROMAN_ROS_WS=$(realpath $SCRIPT_DIR/../../../)
export RECORD=true

read -p "Desired output directory: " OUTPUT_ROOT
export OUTPUT_ROOT=$(eval echo "$OUTPUT_ROOT")
mkdir -p $OUTPUT_ROOT

# Run first session of ROMAN mapping
read -p "Press enter to start first session. "
export OUTPUT_DIR=$OUTPUT_ROOT/run1
mkdir -p $OUTPUT_DIR
tmuxp load $SCRIPT_DIR/d455_online.yaml

# Run second session of ROMAN mapping
read -p "Press enter to start second session. "
export OUTPUT_DIR=$OUTPUT_ROOT/run2
mkdir -p $OUTPUT_DIR
tmuxp load $SCRIPT_DIR/d455_online.yaml

mkdir -p $OUTPUT_ROOT/roman/map
cp $OUTPUT_ROOT/run1/roman_map.pkl $OUTPUT_ROOT/roman/map/run1.pkl
cp $OUTPUT_ROOT/run2/roman_map.pkl $OUTPUT_ROOT/roman/map/run2.pkl

mkdir $OUTPUT_ROOT/roman/params
read -r -d '' DATA_YAML << EOM
dt: 0.166666666666666667
runs: [run1, run2]
run_env: "RUN"
img_data:
    path: ${OUTPUT_ROOT}/\${RUN}/bag
    topic: /robot/d455/color/image_raw
    camera_info_topic: /robot/d455/color/camera_info
    compressed: False
    compressed_encoding: 'bgr8'
depth_data:
    path: ${OUTPUT_ROOT}/\${RUN}/bag
    topic: /robot/d455/aligned_depth_to_color/image_raw
    camera_info_topic: /robot/d455/color/camera_info
    compressed: False
pose_data:
    type: bag
    path: ${OUTPUT_ROOT}/\${RUN}/bag
    topic: /robot/kimera_vio_ros/odometry
    time_tol: 10.0
    T_camera_flu:
        input_type: "string"
        string: "T_RDFFLU"
EOM
echo "$DATA_YAML" > $OUTPUT_ROOT/roman/params/data.yaml

read -r -d '' SUBMAP_ALIGN_YAML << EOM
method: roman
submap_radius: 1000.0
submap_center_dist: 1000.0
submap_center_time: 1000.0
submap_max_size: 100
sigma: 0.3
epsilon: 0.5
EOM
echo "$SUBMAP_ALIGN_YAML" > $OUTPUT_ROOT/roman/params/submap_align.yaml

$(eval echo "$ROMAN_ENV_ACTIVATE")
python3 $ROMAN_ROS_WS/src/roman/demo/demo.py -p $OUTPUT_ROOT/roman/params -o $OUTPUT_ROOT/roman --skip-map --skip-rpgo

python3 $ROMAN_ROS_WS/src/roman/demo/association_vid.py $OUTPUT_ROOT/roman $OUTPUT_ROOT/association_vid.mp4 -r run1 run2 -i 0 0