#!/usr/bin/env bash
# 04-terminal-amcl — Localisation AMCL.
# Terminal 4 : puis 2e onglet 04-b-amcl-activate.sh, puis RViz (Fixed Frame=map) :
# 2D Pose Estimate en (-0.2,-7.4) vers le nord. Verif: ros2 topic echo /localization_ready
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run --help >/dev/null 2>&1 || { echo "[ERREUR] 'ros2 run' manquant. Lancer : sudo apt install -y ros-jazzy-ros2launch ros-jazzy-ros2run ros-jazzy-ros2action ros-jazzy-ros2lifecycle"; exit 1; }
ros2 run nav2_amcl amcl --ros-args \
  --params-file "$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/config/amcl_params.yaml" \
  -p use_sim_time:=true
