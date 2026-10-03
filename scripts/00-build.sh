#!/usr/bin/env bash
# 00-build — Prerequis (une fois par session / apres chaque modif config).
# Terminal dedie : build workspace. Appartient a Johny (Jour 3).
set -e
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "[OK] Build termine. Sourcez install/setup.bash dans chaque nouveau terminal."
