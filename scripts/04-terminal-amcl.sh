#!/usr/bin/env bash
# 04-terminal-amcl — Localisation AMCL.
# Terminal 4 : lance AMCL, puis activer le lifecycle :
#   ros2 lifecycle set /amcl configure && ros2 lifecycle set /amcl activate
# RViz obligatoire : Global Options -> Fixed Frame = "map" (le task.rviz des
# organisateurs est en "base_footprint", sinon AMCL ignore le clic initial),
# puis 2D Pose Estimate -> clic a (-0.2,-7.4) + glisser vers le nord (yaw 1.57).
# Verif : ros2 topic echo /localization_ready
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run --help >/dev/null 2>&1 || { echo "[ERREUR] 'ros2 run' manquant. Lancer : sudo apt install -y ros-jazzy-ros2launch ros-jazzy-ros2run ros-jazzy-ros2action ros-jazzy-ros2lifecycle"; exit 1; }
ros2 run nav2_amcl amcl --ros-args \
  --params-file "$(ros2 pkg prefix caytu_nav_bringup)/share/caytu_nav_bringup/config/amcl_params.yaml" \
  -p use_sim_time:=true
