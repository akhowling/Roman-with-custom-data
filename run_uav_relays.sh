#!/usr/bin/env bash
set -eo pipefail  # NOTE: no -u (nounset) because ROS setup can reference unset vars

ROMAN_WS="/home/anuriha/roman_ws"
DOMAIN_ID="88"

# ---- env ----
conda deactivate >/dev/null 2>&1 || true
unset PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source "${ROMAN_WS}/install/setup.bash"
source "${ROMAN_WS}/venv/bin/activate"
export ROS_DOMAIN_ID="${DOMAIN_ID}"

echo "[relays] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[relays] starting relays + TF (Ctrl-C to stop)..."

# ---- processes cleanup ----
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

# ---- relay odom topic so odom_to_tf can subscribe ----
ros2 run topic_tools relay /uav/odom /robot/odom &
pids+=($!)


# ---- CameraInfo relays with frame_id fix ----
python3 "${ROMAN_WS}/fix_camera_info_frame.py" --ros-args \
  -p in_topic:=/uav/color/info \
  -p out_topic:=/robot/d455/color/camera_info \
  -p frame_id:=camera_frame &
pids+=($!)

python3 "${ROMAN_WS}/fix_camera_info_frame.py" --ros-args \
  -p in_topic:=/uav/depth/info \
  -p out_topic:=/robot/d455/aligned_depth_to_color/camera_info \
  -p frame_id:=camera_frame &
pids+=($!)

# ---- Image relays with frame_id fix (THIS prevents ROMAN crash) ----
python3 "${ROMAN_WS}/fix_image_frame.py" --ros-args \
  -p in_topic:=/uav/color/image \
  -p out_topic:=/robot/d455/color/image_raw \
  -p frame_id:=camera_frame &
pids+=($!)

python3 "${ROMAN_WS}/fix_image_frame.py" --ros-args \
  -p in_topic:=/uav/depth/image \
  -p out_topic:=/robot/d455/aligned_depth_to_color/image_raw \
  -p frame_id:=camera_frame &
pids+=($!)

# ---- Publish TF from /robot/odom -> robot/odom->robot/base_link ----
python3 "${ROMAN_WS}/odom_to_tf.py" &
pids+=($!)

# ---- Static TF: base_link -> camera_frame (identity) ----
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot/base_link camera_frame &
pids+=($!)

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot/base_link d455_link &
pids+=($!)

echo "[relays] all processes started."
wait
