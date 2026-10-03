#!/usr/bin/env bash
# 02-terminal-lidar-filter — Filtre LiDAR (coupe les 4 secteurs du chassis).
# Terminal 2 : SANS lui la costmap sature (auto-detection du chassis).
# Si "Package laser_filters not found" : sudo apt install -y ros-jazzy-laser-filters
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run --help >/dev/null 2>&1 || { echo "[ERREUR] 'ros2 run' manquant. Lancer : sudo apt install -y ros-jazzy-ros2launch ros-jazzy-ros2run ros-jazzy-ros2action ros-jazzy-ros2lifecycle"; exit 1; }
ros2 run laser_filters scan_to_scan_filter_chain --ros-args \
  --params-file "$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/config/laser_filter_params.yaml" \
  -r scan:=/scan -r scan_filtered:=/scan_filtered
