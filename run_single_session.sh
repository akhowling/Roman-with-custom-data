#!/usr/bin/env bash
set -euo pipefail

export ROMAN_BAG="$1"
export ROMAN_OUTPUT_DIR="${2:-$HOME/.roman_ros2}"

# Workspace root = directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROMAN_ROS_WS="$SCRIPT_DIR"

# Paths that actually exist in your checkout
export ROMAN_LC_CONFIG_DIR="$(realpath "$ROMAN_ROS_WS/src/roman_ros2/roman_ros2/cfg/d455")"
TMUXP_FILE="$ROMAN_ROS_WS/src/roman_ros2/examples/d455_bag/d455_bag.yaml"

if [ ! -d "$ROMAN_BAG" ]; then
  echo "ERROR: bag path '$ROMAN_BAG' does not exist"
  exit 1
fi
if [ ! -f "$TMUXP_FILE" ]; then
  echo "ERROR: tmuxp yaml not found: $TMUXP_FILE"
  exit 1
fi

tmuxp load "$TMUXP_FILE"
