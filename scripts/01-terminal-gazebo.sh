#!/usr/bin/env bash
# 01-terminal-gazebo — Gazebo + RViz + robot + bridges.
# Terminal 1 : lance tout, attendre ~7 s pour la camera.
# Verif : robot a (-0.20,-7.48), goal vert a (-2.29,2.23), TF odom->base_footprint OK.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch --help >/dev/null 2>&1 || { echo "[ERREUR] 'ros2 launch' manquant. Lancer : sudo apt install -y ros-jazzy-ros2launch ros-jazzy-ros2run ros-jazzy-ros2action ros-jazzy-ros2lifecycle"; exit 1; }
ros2 pkg prefix rviz2 >/dev/null 2>&1 || { echo "[ERREUR] package 'rviz2' manquant. Lancer : sudo apt install -y ros-jazzy-rviz2"; exit 1; }
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch parc_robot_bringup task.launch.py use_sim_time:=true
