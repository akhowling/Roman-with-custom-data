#!/usr/bin/env bash
set -eo pipefail

ROMAN_WS="/home/anuriha/roman_ws"
DOMAIN_ID="88"

# pick one camera (front/left/right/back)
UGV_CAM="front"

conda deactivate >/dev/null 2>&1 || true
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source "${ROMAN_WS}/install/setup.bash"
source "${ROMAN_WS}/venv/bin/activate"
export ROS_DOMAIN_ID="${DOMAIN_ID}"

echo "[relays] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[relays] using UGV camera: ${UGV_CAM}"

pids=()
cleanup() {
  echo
  echo "[relays] stopping..."
  for pid in "${pids[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup INT TERM EXIT

# ---- odom relay (same pattern as UAV) ----
ros2 run topic_tools relay /ugv/odom /robot/odom &
pids+=($!)

# ---- Color CameraInfo ----
python3 "${ROMAN_WS}/fix_camera_info_frame.py" --ros-args \
  -p in_topic:=/ugv/color_front/info \
  -p out_topic:=/robot/d455/color/camera_info \
  -p frame_id:=camera_frame &
pids+=($!) 

# # ---- Depth CameraInfo (dummy: copy color info) ----
python3 "${ROMAN_WS}/fix_camera_info_frame.py" --ros-args \
  -p in_topic:=/ugv/color_${UGV_CAM}/info \
  -p out_topic:=/robot/d455/aligned_depth_to_color/camera_info \
  -p frame_id:=camera_frame &
pids+=($!)

# ---- Color Image ----
python3 "${ROMAN_WS}/fix_image_frame.py" --ros-args \
  -p in_topic:=/ugv/color_front/image \
  -p out_topic:=/robot/d455/color/image_raw \
  -p frame_id:=camera_frame &
pids+=($!)

# # # ---- Dummy depth image from color ----
# python3 "${ROMAN_WS}/dummy_depth.py" --ros-args \
#   -p in_rgb:=/robot/d455/color/image_raw \
#   -p out_depth:=/robot/d455/aligned_depth_to_color/image_raw_dummy \
#   -p frame_id:=camera_frame \
#   -p encoding:=16UC1 &
# pids+=($!)


# # #velodyne 
# # python3 "${ROMAN_WS}/lidar_to_depth.py" &
# # pids+=($!)
# python3 "${ROMAN_WS}/lidar_to_depth.py" --ros-args \
#   -p in_rgb:=/robot/d455/color/image_raw \
#   -p out_depth:=/robot/d455/aligned_depth_to_color/image_raw \
#   -p frame_id:=camera_frame \
#   -p encoding:=16UC1 &
# pids+=($!)


# python3 "${ROMAN_WS}/lidar_to_depth.py" --ros-args \
#   -p cloud_topic:=/ugv/velodyne_points \
#   -p rgb_topic:=/robot/d455/color/image_raw \
#   -p caminfo_topic:=/robot/d455/aligned_depth_to_color/camera_info \
#   -p depth_topic:=/robot/d455/aligned_depth_to_color/image_raw \
#   -p default_depth_mm:=2000 \
#   -p fill_holes:=true \
#   -p dilate_px:=3 \
#   -p dilate_iters:=2 \
#   -p stamp_mode:=rgb &
# pids+=($!)
python3 "${ROMAN_WS}/lidar_to_depth.py" --ros-args \
  -p cloud_topic:=/ugv/velodyne_points \
  -p rgb_topic:=/robot/d455/color/image_raw \
  -p caminfo_topic:=/robot/d455/aligned_depth_to_color/camera_info \
  -p odom_topic:=/robot/odom \
  -p depth_topic:=/robot/d455/aligned_depth_to_color/image_raw \
  -p max_scans:=12 \
  -p max_age_sec:=1.2 \
  -p stride:=4 \
  -p use_motion_comp:=true \
  -p default_depth_mm:=0 \
  -p sync_slop:=0.25 &
pids+=($!)



# ---- Odom -> TF ----
python3 "${ROMAN_WS}/odom_to_tf.py" &
pids+=($!)

# ---- Static TFs ----
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot/base_link camera_frame &
# pids+=($!)

ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 0 -1.5708 robot/base_link camera_frame &
pids+=($!)


ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot/base_link d455_link &
pids+=($!)

# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 ugv_base_link robot/base_link &
# pids+=($!)

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot/base_link ugv_base_link &
pids+=($!)



echo "[relays] all processes started."
wait
